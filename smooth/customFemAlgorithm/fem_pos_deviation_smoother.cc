
#include "fem_pos_deviation_smoother.h"
#include "fem_pos_deviation_osqp_interface.h"
#include "fem_pos_deviation_sqp_osqp_interface.h"


namespace smoothalgorithm {
FemPosDeviationSmoother::FemPosDeviationSmoother(
    const pnc::data::CfgSmoother& config)
    : config_(config) {}

bool FemPosDeviationSmoother::Solve(
    const std::vector<std::pair<double, double>>& raw_point2d,
    const std::vector<double>& bounds,
    std::vector<double>* opt_x,
    std::vector<double>* opt_y) {
  if (config_.fem_pos.apply_curvature_constraint()) {
    return SqpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
  }
  return QpWithOsqp(raw_point2d, bounds, opt_x, opt_y);
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
      config_.fem_pos.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.fem_pos.weight_path_length());
  solver.set_weight_ref_deviation(config_.fem_pos.weight_ref_deviation());

  solver.set_max_iter(config_.fem_pos.max_iter());
  solver.set_time_limit(config_.fem_pos.time_limit());
  solver.set_verbose(config_.fem_pos.verbose());
  solver.set_scaled_termination(config_.fem_pos.scaled_termination());
  solver.set_warm_start(config_.fem_pos.warm_start());

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
      config_.fem_pos.weight_fem_pos_deviation());
  solver.set_weight_path_length(config_.fem_pos.weight_path_length());
  solver.set_weight_ref_deviation(config_.fem_pos.weight_ref_deviation());
  solver.set_weight_curvature_constraint_slack_var(
      config_.fem_pos.weight_curvature_constraint_slack_var());

  solver.set_curvature_constraint(config_.fem_pos.curvature_constraint());

  solver.set_sqp_sub_max_iter(config_.fem_pos.sqp_sub_max_iter());
  solver.set_sqp_ftol(config_.fem_pos.sqp_ftol());
  solver.set_sqp_pen_max_iter(config_.fem_pos.sqp_pen_max_iter());
  solver.set_sqp_ctol(config_.fem_pos.sqp_ctol());

  solver.set_max_iter(config_.fem_pos.max_iter());
  solver.set_time_limit(config_.fem_pos.time_limit());
  solver.set_verbose(config_.fem_pos.verbose());
  solver.set_scaled_termination(config_.fem_pos.scaled_termination());
  solver.set_warm_start(config_.fem_pos.warm_start());

  solver.set_ref_points(raw_point2d);
  solver.set_bounds_around_refs(bounds);

  if (!solver.Solve()) {
    return false;
  }

  std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

  // TODO(Jinyun): unify output data container
  opt_x->resize(opt_xy.size());
  opt_y->resize(opt_xy.size());
  for (size_t i = 0; i < opt_xy.size(); ++i) {
    (*opt_x)[i] = opt_xy[i].first;
    (*opt_y)[i] = opt_xy[i].second;
  }
  return true;
}

}  // namespace math
