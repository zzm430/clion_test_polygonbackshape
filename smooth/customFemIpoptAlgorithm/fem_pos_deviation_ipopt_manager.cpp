//
// Created by zzm on 24-1-22.
//
#include "fem_pos_deviation_ipopt_manager.h"

namespace smoothalgorithm{

    femPosIpoptSmoother::femPosIpoptSmoother(femPosDeviationIpoptParam femIpoptParam)
                                  :fem_ipopt_param_(femIpoptParam){

    }


    bool femPosIpoptSmoother::NlpWithIpopt(const std::vector<std::pair<double, double>> &raw_point2d,
                                           const std::vector<double> &bounds, std::vector<double> *opt_x,
                                           std::vector<double> *opt_y) {
        if (opt_x == nullptr || opt_y == nullptr) {
            LOG(ERROR) << "opt_x or opt_y is nullptr";
            return false;
        }

        FemPosDeviationIpoptInterface* smoother =
                new FemPosDeviationIpoptInterface(raw_point2d, bounds);

        smoother->set_weight_fem_pos_deviation(fem_ipopt_param_.weight_fem_pos_deviation_ipopt);
        smoother->set_weight_path_length(fem_ipopt_param_.weight_path_length_ipopt);
        smoother->set_weight_ref_deviation(fem_ipopt_param_.weight_ref_deviation_ipopt);
        smoother->set_weight_curvature_constraint_slack_var(
                fem_ipopt_param_.weight_curvature_constraint_slack_var_ipopt);
        smoother->set_curvature_constraint(fem_ipopt_param_.curvature_constraint_ipopt);

        Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

        // Create an instance of the IpoptApplication
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

        app->Options()->SetIntegerValue("print_level",
                                        static_cast<int>(fem_ipopt_param_.print_level_ipopt));
        app->Options()->SetIntegerValue(
                "max_iter", static_cast<int>(fem_ipopt_param_.max_num_of_iterations_ipopt));
        app->Options()->SetIntegerValue(
                "acceptable_iter",
                static_cast<int>(fem_ipopt_param_.acceptable_num_of_iterations_ipopt));
        app->Options()->SetNumericValue("tol", fem_ipopt_param_.tol_ipopt);
        app->Options()->SetNumericValue("acceptable_tol", fem_ipopt_param_.acceptable_tol_ipopt);

        Ipopt::ApplicationReturnStatus status = app->Initialize();
        if (status != Ipopt::Solve_Succeeded) {
            LOG(ERROR)  << "*** Error during initialization!";
            return false;
        }

        status = app->OptimizeTNLP(problem);

        if (status == Ipopt::Solve_Succeeded ||
            status == Ipopt::Solved_To_Acceptable_Level) {
            // Retrieve some statistics about the solve
            Ipopt::Index iter_count = app->Statistics()->IterationCount();
            LOG(ERROR) << "*** The problem solved in " << iter_count << " iterations!";
        } else {
            LOG(ERROR) << "Solver fails with return code: " << static_cast<int>(status);
            return false;
        }
        smoother->get_optimization_results(opt_x, opt_y);
        return true;
    }

}