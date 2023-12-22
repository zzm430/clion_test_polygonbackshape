//
// Created by zzm on 23-11-15.
//

#include <Emplanner/customPJPO.h>

bool customPJPO::optimizePath(
        std::array<double, 3>& init_state,
        std::array<double, 3>& end_state,
        std::vector<double> path_reference_l,
        bool is_valid_path_reference,
        std::vector<std::pair<double, double>>& boundary,
        std::vector<std::pair<double, double>>& ddl_boundary,
        std::array<double, 5>& w,
        int max_iter,
        std::vector<double>* x,
        std::vector<double>* dx,
        std::vector<double>* ddx
        ){

    auto k_num_knots = boundary.size();
    auto delta_s = DIFF_PTS;

    piecewiseJerkPathAlgorithm piecewiseJerkPathAlgorithm_inst(
            k_num_knots,
            delta_s,
            init_state);
    piecewiseJerkPathAlgorithm_inst.set_use_old_method(true);
    piecewiseJerkPathAlgorithm_inst.set_end_state_ref(
            {100000.0,0.0,0.0},end_state);

    if (end_state[0] != 0 && !is_valid_path_reference) {
        std::vector<double> x_ref(k_num_knots, end_state[0]);
        const double weight_x_ref = 10.0;
        piecewiseJerkPathAlgorithm_inst.set_x_ref(weight_x_ref, std::move(x_ref));
    }

    if (is_valid_path_reference) {
        std::vector<double> weight_x_ref_vec(k_num_knots, w[4]);

        //      const double peak_value = w[0];
        //      const double peak_value_x =
        //          0.5 * static_cast<double>(path_reference_size) * delta_s;
        //      for (size_t i = 0; i < path_reference_size; ++i) {
        //        const double curr_x = static_cast<double>(i) * delta_s;
        //        weight_x_ref_vec[i] =
        //            GaussianWeighting(curr_x, peak_value, peak_value_x);
        //      }
        piecewiseJerkPathAlgorithm_inst.set_x_ref(std::move(weight_x_ref_vec),
                                         std::move(path_reference_l));
    }

    piecewiseJerkPathAlgorithm_inst.set_weight_x(w[0]);
    piecewiseJerkPathAlgorithm_inst.set_weight_dx(w[1]);
    piecewiseJerkPathAlgorithm_inst.set_weight_ddx(w[2]);
    piecewiseJerkPathAlgorithm_inst.set_weight_dddx(w[3]);

    piecewiseJerkPathAlgorithm_inst.set_scale_factor({1, 10.0, 1000.0});  //重要参数

    piecewiseJerkPathAlgorithm_inst.set_x_bounds(boundary);
    piecewiseJerkPathAlgorithm_inst.set_dx_bounds(-1.75, 1.75);
    piecewiseJerkPathAlgorithm_inst.set_ddx_bounds(ddl_boundary);

    const double axis_distance = HEAD_WHEELBASE ;
    const double max_yaw_rate = MAX_TRANSVERSE_ACCELETATION / STEER_GEAR_RATIO / 2.0;
     double jerk_bound = EstimateJerkBoundary(
            std::fmax(init_state[1], CURVE_VEL), axis_distance, max_yaw_rate);

    piecewiseJerkPathAlgorithm_inst.set_dddx_bound(-jerk_bound, jerk_bound);
    bool success = false;
    try {
        success = piecewiseJerkPathAlgorithm_inst.Optimize(max_iter);
    } catch (const std::exception& ex) {
        LOG(ERROR) << "Optimize error: " << ex.what();
        success = false;
    }

    if (!success) {
        return false;
    }

    *x = piecewiseJerkPathAlgorithm_inst.opt_x();
    *dx = piecewiseJerkPathAlgorithm_inst.opt_dx();
    *ddx = piecewiseJerkPathAlgorithm_inst.opt_ddx();

    return true;
}


double customPJPO::GaussianWeighting(double x,
                               double peak_weighting,
                               double peak_weighting_x) {
    double std = 1 / (std::sqrt(2 * M_PI) * peak_weighting);
    double u = peak_weighting_x * std;
    double x_updated = x * std;
    return Gaussian(u, std, x_updated);
}


double customPJPO::Gaussian(double u, double std, double x) {
    return (1.0 / std::sqrt(2 * M_PI * std * std)) *
           std::exp(-(x - u) * (x - u) / (2 * std * std));
}


double customPJPO::EstimateJerkBoundary(double vehicle_speed,
                                  double axis_distance,
                                  double max_yaw_rate) {
    return max_yaw_rate / axis_distance / vehicle_speed;
}