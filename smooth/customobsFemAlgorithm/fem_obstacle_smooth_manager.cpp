//
// Created by zzm on 24-1-9.
//
#include "smooth/customobsFemAlgorithm/fem_obstacle_smooth_manager.h"

namespace smoothalgorithm{
    femObstacleManager::femObstacleManager(std::vector<polygonPoint> &originPath):origin_path_(originPath){
    }

    void femObstacleManager::process() {
         std::vector<std::vector<math::Vec2d>>  obstacles_vertices_vec;
        femObstacleParam  femObstacleParamInstance;
        femObstacleParamInstance.ego_length = EGO_LENGTH;
        femObstacleParamInstance.ego_width = EGO_WIDTH;
        femObstacleParamInstance.back_edge_to_center = BACK_EDGE_TO_CENTER;
        femObstacleParamInstance.interpolated_delta_s = INTERPOLATED_DELTA_S;
        femObstacleParamInstance.reanchoring_trails_num = REANCHORING_TRAILS_NUM;
        femObstacleParamInstance.reanchoring_length_stddev = REANCHORING_LENGTH_STDDEV;
        femObstacleParamInstance.estimate_bound = ESTIMATE_BOUND;
        femObstacleParamInstance.default_bound = DEFAULT_BOUND;
        femObstacleParamInstance.vehicle_shortest_dimension = VEHICLE_SHORTEST_DIMENSION;
        femObstacleParamInstance.collision_decrease_ratio = COLLISION_DECREASE_RATIO;
        femObstacleAnchoringSmoother femObstacleAnchoringSmootherInstance(femObstacleParamInstance,
                                                                          obstacles_vertices_vec);
        femObstacleAnchoringSmootherInstance.Smooth(
                origin_path_);
        storage_smoothed_path_ = femObstacleAnchoringSmootherInstance.getStorageSmoothedPts();
#ifdef DEBUG_STATIC_OBSTACLE
        std::ofstream   test_path;
        test_path.open("/home/zzm/Desktop/test_path_figure-main/src/testfemObstaclePath",std::ios::out);
        for(auto i : storage_smoothed_path_){
             test_path << " " << i.x ;
        }
        test_path << std::endl;

        for(auto i : storage_smoothed_path_){
            test_path << " " << i.y;
        }
        test_path << std::endl;
#endif
    }

    std::vector<polygonPoint> femObstacleManager::getSmoothedPath() {
        return storage_smoothed_path_;
    }
}