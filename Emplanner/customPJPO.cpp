//
// Created by zzm on 23-11-15.
//

#include <Emplanner/customPJPO.h>


void customPJPO::testPjpo() {
    std::vector<double> opt_l;
    std::vector<double> opt_dl;
    std::vector<double> opt_ddl;

    std::array<double,3> start_state = {
            2,
            0,
            0};
    std::array<double,5> w = {
            10000.0,
            1.0,
            1.0,
            1,
            1000
    };

    std::array<double,3> end_state = {
            0,
            0,
            0
    };

    double start_s = 0.0;
    double end_s = 25;
    double thr = 0;
//    std::vector<double> path_reference_l(40,0);
    std::vector<double> path_reference_l(40);
    for(int i = 0;i < path_reference_l.size();i++){
        if(i >= 0 &&i < 10){
            thr = 0;
        }else if(i >=10 && i <=11){
            thr = 1;
        } else{
            thr = 0;
        }
        path_reference_l[i] = thr;
    }
    double lower_bound = -2;  // 设置每个数据对的下界
    double upper_bound = 4;
//    std::vector<std::pair<double, double>> boundary(40, std::make_pair(-2.0, 4.0));
    std::vector<std::pair<double, double>> boundary(40);
    for (int i = 0; i < boundary.size(); i++) {
        if(i >=0 && i < 10){
            lower_bound = -2;
            upper_bound = 4;
        } else if(i >= 10 && i <=13){
            lower_bound = -1;
            upper_bound = 4;
        }else{
            lower_bound = -2;
            upper_bound = 4;
        }
        boundary[i] = std::make_pair(lower_bound, upper_bound);  // 修改第i个元素的值
    }

    std::vector<std::pair<double, double>> ddl_bounds(40, std::make_pair(0, 0));

    if (optimizePath(start_state,
                     end_state,
                     path_reference_l,
                     true,
                     boundary,
                     ddl_bounds,
                     w,
                     4000,
                     &opt_l,
                     &opt_dl,
                     &opt_ddl)) {
        auto optm_l = opt_l;
       for(auto i : optm_l){
           std::cout << i ;
       }
       std::cout << std::endl;
       std::cout << optm_l.size() << std::endl;
//        auto path_data = reference_line_info_->mutable_path_data();
//        path_data->SetReferenceLine(&ref_line);
//        path_data->SetFrenetPath(std::move(frenet_frame_path));
    }

}



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
    auto delta_s = 0.25;

    piecewiseJerkPathAlgorithm piecewiseJerkPathAlgorithm_inst(
            k_num_knots,
            delta_s,
            init_state);
    piecewiseJerkPathAlgorithm_inst.set_use_old_method(true);
    piecewiseJerkPathAlgorithm_inst.set_end_state_ref(
            {1000.0,0.0,0.0},end_state);

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

    piecewiseJerkPathAlgorithm_inst.set_scale_factor({1000.0, 10.0, 1.0});  //重要参数

    piecewiseJerkPathAlgorithm_inst.set_x_bounds(boundary);
    piecewiseJerkPathAlgorithm_inst.set_dx_bounds(-2, 2);
    piecewiseJerkPathAlgorithm_inst.set_ddx_bounds(ddl_boundary);

    const double axis_distance = 3 ;
    const double max_yaw_rate = 8.5 / 22.4 / 2.0;
    const double jerk_bound = EstimateJerkBoundary(
            std::fmax(init_state[1], 0.3), axis_distance, max_yaw_rate);


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


bool customPJPO::OptimizePathWithKappaMax(
        const std::array<double, 3>& init_state,
        const std::array<double, 3>& end_state,
        const std::vector<double>& path_reference_l,
        const bool is_valid_path_reference,
        std::vector<std::pair<double, double>>& boundary,
        const std::vector<double>& kappa_ref,
        const double kappa_max,
        const std::array<double, 5>& w,
        const int max_iter,
        std::vector<double>* x,
        std::vector<double>* dx,
        std::vector<double>* ddx) {
    /// num of knots
    auto k_num_knots = boundary.size();

    piecewiseJerkPathAlgorithm piecewiseJerkPathAlgorithm_ins(
            k_num_knots, 0.5, init_state);
    piecewiseJerkPathAlgorithm_ins.set_end_state_ref(
            {1000,0.0,0.0},
            end_state
            );

    if (end_state[0] != 0 && !is_valid_path_reference) {
        std::vector<double> x_ref(k_num_knots, end_state[0]);
        const double weight_x_ref = 10.0;
        piecewiseJerkPathAlgorithm_ins.set_x_ref(weight_x_ref, std::move(x_ref));
    }

    if (is_valid_path_reference) {
        std::vector<double> weight_x_ref_vec(k_num_knots, w[4]);
        piecewiseJerkPathAlgorithm_ins.set_x_ref(std::move(weight_x_ref_vec),
                                         path_reference_l);
    }

    piecewiseJerkPathAlgorithm_ins.set_weight_x(w[0]);
    piecewiseJerkPathAlgorithm_ins.set_weight_dx(w[1]);
    piecewiseJerkPathAlgorithm_ins.set_weight_ddx(w[2]);
    piecewiseJerkPathAlgorithm_ins.set_weight_dddx(w[3]);

    piecewiseJerkPathAlgorithm_ins.set_scale_factor({1.0, 1.0, 1.0});

    piecewiseJerkPathAlgorithm_ins.set_x_bounds(boundary);
    piecewiseJerkPathAlgorithm_ins.set_dx_bounds(-1e10, 1e10);
    piecewiseJerkPathAlgorithm_ins.set_ddx_bounds(-1e10, 1e10);
    piecewiseJerkPathAlgorithm_ins.set_dddx_bound(-1e10, 1e10);
    piecewiseJerkPathAlgorithm_ins.set_kappa_max(kappa_max);
    piecewiseJerkPathAlgorithm_ins.set_kappa_ref(kappa_ref);


    bool success = false;
    try {
        success = piecewiseJerkPathAlgorithm_ins.Optimize(max_iter);
    } catch (const std::exception& ex) {
        LOG(ERROR) << "Optimize error: " << ex.what();
        success = false;
    }

    if (!success) {
        return false;
    }


    *x = piecewiseJerkPathAlgorithm_ins.opt_x();
    *dx = piecewiseJerkPathAlgorithm_ins.opt_dx();
    *ddx = piecewiseJerkPathAlgorithm_ins.opt_ddx();
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