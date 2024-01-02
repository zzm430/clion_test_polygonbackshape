//
// Created by zzm on 23-12-25.
//

#ifndef POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
#define POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
#include "HybridAStar/hybrid_AStar_Algorithm.h"

namespace searchAlgorithm{
    class hybridAStarTest{
    public:
        hybridAStarTest() = default;
        virtual ~hybridAStarTest() = default;

        void test(){
            double sx = -15.0;
            double sy = 0.0;
            double sphi = 0.0;
            double ex = 15.0;
            double ey = 0.0;
            double ephi = 0.0;
            std::vector<std::vector<math::Vec2d>> obstacles_list;
            HybridAStartResult result;
            math::Vec2d obstacle_vertice_a(1.0, 0.0);
            math::Vec2d obstacle_vertice_b(-1.0, 0.0);
            std::vector<math::Vec2d> obstacle = {obstacle_vertice_a, obstacle_vertice_b};

            std::vector<double> XYbounds_;
            XYbounds_.push_back(-50.0);
            XYbounds_.push_back(50.0);
            XYbounds_.push_back(-50.0);
            XYbounds_.push_back(50.0);

            obstacles_list.emplace_back(obstacle);
            coarseSearchParam  coarseSearchParamtest;
            vehicleParam   vehicleParamtest;
            vehicleParamtest.tractor_wheel_base = 3.5;
            vehicleParamtest.tractor_frontEdge_to_center = 2.5;
            vehicleParamtest.tractor_backEdge_to_center = 1.5;
            vehicleParamtest.max_frontWheel_SteerAngle = 0.52;
            vehicleParamtest.max_frontWheel_SteerAngle_deg = 30;
            vehicleParamtest.width = 4;
            coarseSearchParamtest.is_show_result_analysis = false;
            coarseSearchParamtest.maximum_searched_num = 300000;
            coarseSearchParamtest.maximum_searched_time =20;
            coarseSearchParamtest.xy_resolution = 1.5;
            coarseSearchParamtest.tractor_phi_resolution = 5;
            coarseSearchParamtest.xy_2d_resolution =1;
            coarseSearchParamtest.ogm_resolution = 0.4;
            coarseSearchParamtest.next_node_num = 10;
            coarseSearchParamtest.step_size = 0.5;
            coarseSearchParamtest.delta_t = 0.5;
            coarseSearchParamtest.forward_penalty = 1;
            coarseSearchParamtest.back_penalty = 0.1;
            coarseSearchParamtest.steer_penalty = 2;
            coarseSearchParamtest.steer_change_penalty = 5;
            coarseSearchParamtest.gear_switch_penalty = 2;
            coarseSearchParamtest.tractor_phi_resolution_in_deg = 5;
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
            
        }
    private:
        std::unique_ptr<hybridAStarAlgorithm> hybrid_test;

    };
}
#endif //POLYGONBACKSHAPE_HYBRID_ASTAR_TEST_H
