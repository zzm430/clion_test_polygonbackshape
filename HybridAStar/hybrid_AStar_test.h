//
// Created by zzm on 23-12-25.
//
#ifndef POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
#define POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H

#include "HybridAStar/hybrid_AStar_Algorithm.h"
#include "common/common_param/common_parameters.h"
#include "thirdParty/matplotlibcpp.h"
#include "common/plot/debugHybridAStarShow.h"
#include <chrono>
#include <iostream>

namespace searchAlgorithm{
    class hybridAStarTest{
    public:
        hybridAStarTest() = default;
        virtual ~hybridAStarTest() = default;

        void test(){
            // 开始计时
            auto start = std::chrono::high_resolution_clock::now();

            //计算方向向量的夹脚
            polygonPoint  vector_refer(1,0);
            polygonPoint  vector_1(93.74,469.24);
            polygonPoint  vector_2(-163.38,11.961);
            double theta_1 = std::atan2(vector_1.y,vector_1.x);
            double theta_2 = std::atan2(vector_2.y,vector_2.x);
            std::cout << "the theta_1 is : " << theta_1 << std::endl;
            std::cout << "the theta_2 is : " << theta_2 << std::endl;
            double sx = 254.48;
            double sy = 462.85;
            double sphi = 1.37;
            double ex = 249.7;
            double ey = 480.12;
            double ephi = 3.06;
            std::vector<std::vector<math::Vec2d>> obstacles_list;
            HybridAStartResult result;
            math::Vec2d obstacle_vertice_a(1, 12);
            math::Vec2d obstacle_vertice_b(-4.0, 23);
            math::Vec2d obstacle_vertice_c(3,-14);
            math::Vec2d obstacle_vertice_d(10,-3);
            std::vector<math::Vec2d> obstacle,obstacle1;

            std::vector<double> XYbounds_;
            XYbounds_.push_back(225);
            XYbounds_.push_back(275);
            XYbounds_.push_back(460);
            XYbounds_.push_back(510);

            polygonPoint     circle_test(255,480),circle_test1(253,467.5);
            std::vector<polygonPoint>  circle_polygon,circle_polygon1;
            double invertal_dis = 2 * M_PI / 30;
            for( int i = 0;i < 30 ; i++ ){
                polygonPoint temp,temp1;
                temp.x = circle_test.x + 1 * cos(invertal_dis * i);
                temp.y = circle_test.y + 1 * sin(invertal_dis * i);
                temp1.x = circle_test1.x + 2 * cos(invertal_dis * i);
                temp1.y = circle_test1.y + 2 * sin(invertal_dis * i);
                circle_polygon.push_back(temp);
                circle_polygon1.push_back(temp1);
            }
            circle_polygon.push_back(circle_polygon[0]);
            circle_polygon1.push_back(circle_polygon1[0]);
            for(auto i : circle_polygon){
                obstacle.push_back(math::Vec2d(i.x,i.y));
            }
            for(auto i : circle_polygon1){
                obstacle1.push_back(math::Vec2d(i.x,i.y));
            }


#ifdef DEBUG_HYBRIDASTAR
            std::ofstream   searchRegionShow;
            searchRegionShow.open("/home/zzm/Desktop/test_path_figure-main/src/searchRegionShow.txt",std::ios::out);
            searchRegionShow << " " << XYbounds_[0] ;
            searchRegionShow << " " << XYbounds_[1] ;
            searchRegionShow << " " << XYbounds_[1] ;
            searchRegionShow << " " << XYbounds_[0] ;
            searchRegionShow << " " << XYbounds_[0] ;
            searchRegionShow << std::endl;
            searchRegionShow << " " << XYbounds_[2] ;
            searchRegionShow << " " << XYbounds_[2] ;
            searchRegionShow << " " << XYbounds_[3] ;
            searchRegionShow << " " << XYbounds_[3] ;
            searchRegionShow << " " << XYbounds_[2] ;
            searchRegionShow << std::endl;

            std::ofstream   obstacleShow;
            obstacleShow.open("/home/zzm/Desktop/test_path_figure-main/src/obstacleShow.txt",std::ios::out);
            for(auto i : obstacle1){
                obstacleShow << " " << i.x();
            }
            obstacleShow << std::endl;
            for(auto j : obstacle1){
                obstacleShow << " " << j.y();
            }
            obstacleShow << std::endl;
#endif
            obstacles_list.emplace_back(obstacle);
            obstacles_list.emplace_back(obstacle1);
            coarseSearchParam  coarseSearchParamtest;
            vehicleParam   vehicleParamtest;
            vehicleParamtest.tractor_wheel_base = TRACTOR_WHEEL_BASE;
            vehicleParamtest.tractor_frontEdge_to_center = TRACTOR_FRONTEDGE_TO_CENTER;
            vehicleParamtest.tractor_backEdge_to_center = TRACTOR_BACKEDGE_TO_CENTER;
            vehicleParamtest.max_frontWheel_SteerAngle = MAX_FRONTWHEEL_STEERANGLE;
            vehicleParamtest.max_frontWheel_SteerAngle_deg = MAX_FRONTWHEEL_STEERANGLE_DEG;
            vehicleParamtest.width = WIDTH;
            coarseSearchParamtest.is_show_result_analysis = IS_SHOW_RESULT_ANALYSIS;
            coarseSearchParamtest.maximum_searched_num  = MAXIMUM_SEARCHED_NUM;
            coarseSearchParamtest.maximum_searched_time = MAXIMUM_SEARCHED_TIME;
            coarseSearchParamtest.xy_resolution = XY_RESOLUTION;
            coarseSearchParamtest.tractor_phi_resolution = TRACTOR_PHI_RESOLUTION;
            coarseSearchParamtest.xy_2d_resolution = XY_2D_RESOLUTION;
            coarseSearchParamtest.ogm_resolution = OGM_RESOLUTION;
            coarseSearchParamtest.next_node_num = NEXT_NODE_NUM;
            coarseSearchParamtest.step_size = STEP_SIZE;
            coarseSearchParamtest.delta_t = DELTA_T;
            coarseSearchParamtest.forward_penalty = FORWARD_PENALTY;
            coarseSearchParamtest.back_penalty = BACK_PENALTY;
            coarseSearchParamtest.steer_penalty = STEER_PENALTY;
            coarseSearchParamtest.steer_change_penalty = GEAR_SWITHCH_PENALTY;
            coarseSearchParamtest.gear_switch_penalty = STEER_CHANGE_PENALTY;
            coarseSearchParamtest.tractor_phi_resolution_in_deg = TRACTOR_PHI_RESOLUTION_IN_DEG;
            hybridAStarAlgorithm hybridAStarAlgorithmtestM(vehicleParamtest,
                                                           coarseSearchParamtest);
            hybridAStarAlgorithmtestM.Plan(
                    sx,
                    sy,
                    sphi,
                    ex,
                    ey,
                    ephi,
                    XYbounds_,
                    obstacles_list,
                    &result);
            std::cout <<"get result !" <<std::endl;
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            double milliseconds = duration.count();
            std::cout << "代码运行时间: " << milliseconds << " 毫秒" << std::endl;

#ifdef DEBUG_HYBRIDASTAR
            std::ofstream pathResultShow;
            pathResultShow.open("/home/zzm/Desktop/test_path_figure-main/src/pathResultShow.txt",std::ios::out);
            for(size_t i = 0;i < result.x.size();i++){
                pathResultShow << " " << result.x.at(i);
            }
            pathResultShow << std::endl;
            for(size_t i = 0;i < result.y.size();i++){
                pathResultShow << " " << result.y.at(i);
            }
            pathResultShow << std::endl;
#endif
        }

    private:
        std::unique_ptr<hybridAStarAlgorithm> hybrid_test;

    };
}

#endif //POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
