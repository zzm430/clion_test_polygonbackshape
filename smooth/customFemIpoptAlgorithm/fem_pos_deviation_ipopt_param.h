//
// Created by zzm on 24-1-22.
//

#ifndef POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_PARAM_H
#define POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_PARAM_H
#include <string>

class femPosDeviationIpoptParam{
public:
    femPosDeviationIpoptParam() = default;
    virtual ~femPosDeviationIpoptParam() = default;
public:
    double weight_fem_pos_deviation_ipopt;
    double weight_path_length_ipopt;
    double weight_ref_deviation_ipopt;
    double weight_curvature_constraint_slack_var_ipopt;
    double curvature_constraint_ipopt;
    double print_level_ipopt;
    double max_num_of_iterations_ipopt;
    double acceptable_num_of_iterations_ipopt;
    double tol_ipopt;
    double acceptable_tol_ipopt;
};
#endif //POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_PARAM_H
