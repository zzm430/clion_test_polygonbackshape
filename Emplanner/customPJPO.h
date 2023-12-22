//
// Created by zzm on 23-11-15.
//

#ifndef POLYGONBACKSHAPE_CUSTOMPJPO_H
#define POLYGONBACKSHAPE_CUSTOMPJPO_H
#include <vector>
#include <common/utilpath/path_polygonPoint.h>
#include "Emplanner/piecewise_jerk_path_algorithm.h"
#include <Emplanner/frenet_converter.h>
#include <common/common_param/common_parameters.h>

class customPJPO {
public:
    customPJPO() = default;
    virtual  ~customPJPO()= default;
    bool optimizePath(
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
            );

     double GaussianWeighting( double x,
                               double peak_weighting,
                               double peak_weighting_x);

     double Gaussian(double u, double std, double x);

     double EstimateJerkBoundary(double vehicle_speed,
                                       double axis_distance,
                                       double max_yaw_rate);

};
#endif //POLYGONBACKSHAPE_CUSTOMPJPO_H
