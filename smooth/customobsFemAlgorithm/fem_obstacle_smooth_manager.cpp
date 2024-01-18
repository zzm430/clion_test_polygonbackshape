//
// Created by zzm on 24-1-9.
//
#include "smooth/customobsFemAlgorithm/fem_obstacle_smooth_manager.h"

namespace smoothalgorithm{
    femObstacleManager::femObstacleManager(std::vector<polygonPoint> &originPath)
                                      :origin_path_(originPath){
        //清除点位信息中点的NAN点位
        common::commonMath::deleteNanPts(origin_path_);
        //对原始路径点的path点位信息进行补充更新
        std::vector<std::pair<double, double>> temp_xypts;
        std::vector<double> headings;
        std::vector<double> accumulated_s;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        for(auto i : origin_path_){
            std::pair<double,double>  tempPt;
            tempPt.first = i.x;
            tempPt.second = i.y;
            temp_xypts.push_back(tempPt);
        }
        auto flag =  computePathProfileInfo::ComputePathProfile(temp_xypts,
                               &headings,
                               &accumulated_s,
                               &kappas,
                               &dkappas);
        for(int i = 0 ;i < origin_path_.size();i++){
              origin_path_[i].set_heading(headings[i]);
              origin_path_[i].set_theta(headings[i]);

        }
    }

    void femObstacleManager::process() {
        std::vector<std::vector<math::Vec2d>>  obstacles_vertices_vec;
        polygonPoint     circle_test(255,476);
        std::vector<polygonPoint>  circle_polygon,circle_polygon1;
        double invertal_dis = 2 * M_PI / 5;
        for( int i = 0;i < 5 ; i++ ){
            polygonPoint temp,temp1;
            temp.x = circle_test.x + 1 * cos(invertal_dis * i);
            temp.y = circle_test.y + 1 * sin(invertal_dis * i);
            circle_polygon.push_back(temp);
        }
        circle_polygon.push_back(circle_polygon[0]);

        std::vector<math::Vec2d>  obstacle_vertices_vec;

#ifdef DEBUG_STATIC_OBSTACLE
        for(auto i : circle_polygon){
            math::Vec2d  tempPt(i.x,i.y);
            obstacle_vertices_vec.push_back(tempPt);
        }
        obstacles_vertices_vec.push_back(obstacle_vertices_vec);
        std::ofstream   test_obstaclem;
        test_obstaclem.open(
                "/home/zzm/Desktop/test_path_figure-main/src/testobstaclem.txt",
                std::ios::out);
        for(auto i : circle_polygon ){
            test_obstaclem << " " << i.x;
        }
        test_obstaclem << std::endl;
        for(auto j : circle_polygon){
            test_obstaclem << " " << j.y ;
        }
        test_obstaclem << std::endl;
#endif
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
        femObstacleAnchoringSmoother femObstacleAnchoringSmootherInstance(
                                                       femObstacleParamInstance,
                                                                          obstacles_vertices_vec);
        femObstacleAnchoringSmootherInstance.Smooth(
                origin_path_);
        storage_smoothed_path_ = femObstacleAnchoringSmootherInstance.getStorageSmoothedPts();
#ifdef DEBUG_STATIC_OBSTACLE
        std::ofstream   test_path;
        test_path.open("/home/zzm/Desktop/test_path_figure-main/src/testfemObstaclePath.txt",
                       std::ios::out);
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