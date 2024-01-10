//
// Created by zzm on 24-1-9.
//

#ifndef POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_MANAGER_H
#define POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_MANAGER_H
#include "common/utilpath/path_polygonPoint.h"
#include "smooth/customobsFemAlgorithm/fem_obstacle_smooth_algorithm.h"
#include "smooth/customobsFemAlgorithm/fem_obstacle_smooth_param.h"

namespace  smoothalgorithm{
    class femObstacleManager{
    public:
        femObstacleManager() = default;
        virtual ~femObstacleManager() = default;
        femObstacleManager(std::vector<polygonPoint> & originPath);
        void process();
        std::vector<polygonPoint> getSmoothedPath();
    private:
        std::vector<polygonPoint>  origin_path_;
        std::vector<polygonPoint>  storage_smoothed_path_;
    };

}
#endif //POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_MANAGER_H
