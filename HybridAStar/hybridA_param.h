//
// Created by zzm on 23-12-22.
//

#ifndef POLYGONBACKSHAPE_HYBRIDA_PARAM_H
#define POLYGONBACKSHAPE_HYBRIDA_PARAM_H
#include <string>

class vehicleParam {
public:
    vehicleParam() = default;
    ~vehicleParam() = default;

public:
    double tractor_frontEdge_to_center = 0.0;
    double tractor_backEdge_to_center = 0.0;
    double tractor_wheel_base = 0.0;
    double height = 0.0;
    double width = 0.0;
    double max_frontWheel_SteerAngle_deg = 0.0;
    double max_frontWheel_SteerAngle = 0.0;

};


class coarseSearchParam{
public:
    coarseSearchParam() = default;
    ~coarseSearchParam() = default;

public:
    bool is_show_result_analysis = false;

    int maximum_searched_num = 0.0;
    double maximum_searched_time = 0.0;

    double xy_resolution = 0.0;
    double tractor_phi_resolution = 0.0;
    double xy_2d_resolution = 0.0;
    double ogm_resolution = 0.0;
    int next_node_num = 0.0;
    double step_size = 0.0;
    double delta_t = 0.0;

    double forward_penalty = 0.0;
    double back_penalty = 0.0;
    double gear_switch_penalty = 0.0;
    double steer_penalty = 0.0;
    double steer_change_penalty = 0.0;

    double tractor_phi_resolution_in_deg = 0.0;

};

#endif //POLYGONBACKSHAPE_HYBRIDA_PARAM_H
