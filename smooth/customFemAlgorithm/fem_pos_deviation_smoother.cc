
#include "fem_pos_deviation_smoother.h"
#include "fem_pos_deviation_osqp_interface.h"
#include "fem_pos_deviation_sqp_osqp_interface.h"


namespace smoothalgorithm {
FemPosDeviationSmoother::FemPosDeviationSmoother(){

}

FemPosDeviationSmoother::FemPosDeviationSmoother(const femPosDeviationParam & femParam)
            :fem_pos_deviation_param_(femParam){
}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if ( APPLY_CURVATURE_CONSTRAINT ) {
    return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  } else{
      return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
//
}

bool FemPosDeviationSmoother::QpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    return false;
  }

  FemPosDeviationOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(
          fem_pos_deviation_param_.qp_weigth_fem_pos_deviation);
  solver.set_weight_path_length(fem_pos_deviation_param_.qp_weight_path_length);
  solver.set_weight_ref_deviation(fem_pos_deviation_param_.weight_ref_deviation);

  solver.set_max_iter(fem_pos_deviation_param_.qp_max_iter);
  solver.set_time_limit(fem_pos_deviation_param_.qp_time_limit);
  solver.set_verbose(fem_pos_deviation_param_.verboase);
  solver.set_scaled_termination(fem_pos_deviation_param_.qp_sacled_termination);
  solver.set_warm_start(fem_pos_deviation_param_.qp_warm_start);

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  *opt_x = solver.opt_x();
  *opt_y = solver.opt_y();
  return true;
}

bool FemPosDeviationSmoother::SqpWithOsqp(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (opt_x == nullptr || opt_y == nullptr) {
    //    AERROR << "opt_x or opt_y is nullptr";
    return false;
  }

  FemPosDeviationSqpOsqpInterface solver;

  solver.set_weight_fem_pos_deviation(
          fem_pos_deviation_param_.weight_fem_pos_deviation);
  solver.set_weight_path_length(fem_pos_deviation_param_.weight_path_length);
  solver.set_weight_ref_deviation(fem_pos_deviation_param_.weight_ref_deviation);
  solver.set_weight_curvature_constraint_slack_var(
          fem_pos_deviation_param_.weight_curvature_constrain_slack_var);
  solver.set_curvature_constraint(fem_pos_deviation_param_.apply_curvature_constraint);
  solver.set_sqp_sub_max_iter(fem_pos_deviation_param_.sqp_sub_max_iter);
  solver.set_sqp_ftol(fem_pos_deviation_param_.sqp_ftol);
  solver.set_sqp_pen_max_iter(fem_pos_deviation_param_.sqp_pen_max_iter);
  solver.set_sqp_ctol(fem_pos_deviation_param_.sqp_ctol);
  solver.set_max_iter(fem_pos_deviation_param_.max_iter_fem);
  solver.set_time_limit(fem_pos_deviation_param_.time_limit);
  solver.set_verbose(fem_pos_deviation_param_.verboase);
  solver.set_scaled_termination(fem_pos_deviation_param_.scaled_termination);
  solver.set_warm_start(fem_pos_deviation_param_.warm_start_m);
  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);
  if (!solver.Solve()) {
    return false;
  }

  std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i) {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }
  return true;
}

}  // namespace math
