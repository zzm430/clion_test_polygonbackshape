//
// Created by zzm on 24-1-9.
//

#ifndef POLYGONBACKSHAPE_FEM_POS_DEVIATION_PARAM_H
#define POLYGONBACKSHAPE_FEM_POS_DEVIATION_PARAM_H
#include <string>

class femPosDeviationParam{

public:
    femPosDeviationParam() = default;
    virtual ~femPosDeviationParam() = default;
public:
    //sqp
    double  weight_fem_pos_deviation;
    double  weight_path_length;
    double  weight_ref_deviation;
    double  weight_curvature_constrain_slack_var;
    bool    apply_curvature_constraint;
    double  sqp_sub_max_iter;
    double  sqp_ftol;
    double  sqp_pen_max_iter;
    double  sqp_ctol;
    double  max_iter_fem;
    double  time_limit;
    bool  verboase;
    bool  scaled_termination;
    bool  warm_start_m;

    //qp
    double qp_weigth_fem_pos_deviation;
    double qp_weight_path_length;
    double qp_weigth_ref_deviation;
    double qp_max_iter;
    double qp_time_limit;
    bool qp_verbose;
    bool qp_sacled_termination;
    double qp_warm_start;

};
#endif //POLYGONBACKSHAPE_FEM_POS_DEVIATION_PARAM_H
