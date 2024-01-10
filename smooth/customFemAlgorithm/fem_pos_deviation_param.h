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

};
#endif //POLYGONBACKSHAPE_FEM_POS_DEVIATION_PARAM_H
