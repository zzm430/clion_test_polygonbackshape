//
// Created by zzm on 24-1-9.
//

#ifndef POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_PARAM_H
#define POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_PARAM_H
#include <string>

class femObstacleParam {
public:
    femObstacleParam() = default;
    virtual ~femObstacleParam() = default;

public:
   double ego_length;
   double ego_width;
   double back_edge_to_center;
   double interpolated_delta_s;
   double reanchoring_trails_num;
   double reanchoring_pos_stddev;
   double reanchoring_length_stddev;
   bool   estimate_bound;
   double default_bound;
   double vehicle_shortest_dimension;
   double collision_decrease_ratio;
};

#endif //POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_PARAM_H
