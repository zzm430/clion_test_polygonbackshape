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
            double sx = -10.0;
            double sy = 0.0;
            double sphi = 0.0;
            double ex = -3.0;
            double ey = 13;
            double ephi = 0.0;
            std::vector<std::vector<math::Vec2d>> obstacles_list;
            HybridAStartResult result;
            math::Vec2d obstacle_vertice_a(1, 12);
            math::Vec2d obstacle_vertice_b(-4.0, 23);
            math::Vec2d obstacle_vertice_c(3,-14);
            math::Vec2d obstacle_vertice_d(10,-3);
            std::vector<math::Vec2d> obstacle = {obstacle_vertice_a,
                                                 obstacle_vertice_b,
                                                 obstacle_vertice_c,
                                                 obstacle_vertice_d};

            std::vector<double> XYbounds_;
            XYbounds_.push_back(-50.0);
            XYbounds_.push_back(50.0);
            XYbounds_.push_back(-50.0);
            XYbounds_.push_back(50.0);

            obstacles_list.emplace_back(obstacle);
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
//            plt::ion();
#ifdef DEBUG_HYBRIDASTAR
            plt::figure_size(800, 800);
            plt::grid(false);
            plt::axis("equal");
            plt::title("Search Process");
            KeywordType keywords{{"linewidth", "3"}, {"color", "k"}, {"marker", "D"}};
            std::vector<double> path_x(result.x.size());
            std::vector<double> path_y(result.x.size());
            for(size_t i = 0;i < result.x.size();i++){
                path_x[i] = result.x.at(i);
                path_y[i] = result.y.at(i);
            }
            plt::plot(path_x, path_y,keywords);
            plt::grid(true);
            plt::show();
            plt::save("plot_image.png");
            plt::ioff();
#endif

        }


    private:
        std::unique_ptr<hybridAStarAlgorithm> hybrid_test;

    };
}

#endif //POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
