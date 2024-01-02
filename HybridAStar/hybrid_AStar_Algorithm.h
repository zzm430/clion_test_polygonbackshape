//
// Created by zzm on 23-12-25.
//

#ifndef POLYGONBACKSHAPE_HYBRID_ASTAR_ALGORITHM_H
#define POLYGONBACKSHAPE_HYBRID_ASTAR_ALGORITHM_H
#include <algorithm>
#include <iterator>
#include <memory>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <chrono>

#include "HybridAStar/dubin_path.h"
#include "HybridAStar/grid_search.h"
#include "HybridAStar/node_HybridA.h"
#include "common/math/math_utils.h"
#include "easylogging++.h"
#include "common/common_param/common_parameters.h"

namespace  searchAlgorithm {

    struct HybridAStartResult {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> phi;
        std::vector<double> v;
        std::vector<double> a;
        std::vector<double> steer;
        std::vector<double> accumulated_s;
    };

    class hybridAStarAlgorithm {
    public:
        explicit hybridAStarAlgorithm(
                const vehicleParam &vehicleParam,
                const coarseSearchParam &coarse_search_param);

        virtual ~hybridAStarAlgorithm() = default;
        bool Plan(double sx, double sy, double sphi, double ex, double ey,
                  double ephi, const std::vector<double>& XYbounds,
                  const std::vector<std::vector<math::Vec2d>>&
                  obstacles_vertices_vec,
                  HybridAStartResult* result);

    private:
        bool AnalyticExpansion(std::shared_ptr<nodeTractor> current_node);
        // check collision and validity
        bool ValidityCheck(std::shared_ptr<nodeTractor> node);

        std::shared_ptr<nodeTractor> Next_node_generator(
                std::shared_ptr<nodeTractor> current_node, size_t next_node_index);

        void CalculateNodeCost(std::shared_ptr<nodeTractor> current_node,
                               std::shared_ptr<nodeTractor> next_node);
        double TrajCost(std::shared_ptr<nodeTractor> current_node,
                        std::shared_ptr<nodeTractor> next_node);
        double HoloObstacleHeuristic(std::shared_ptr<nodeTractor> next_node);
        bool GetResult(HybridAStartResult* result);
        bool AnalyticExpansionCheck(const dubinPath& dubinPath_to_end);
        bool ValidityCheckWithAnalysis(std::shared_ptr<nodeTractor> node);
        double CalculateAnalyticPathCost(dubinPath& path);
        std::shared_ptr<nodeTractor> LoadAnalyticPinCS(
                const dubinPath& dubinPath_to_end,
                std::shared_ptr<nodeTractor> current_node,
                double db_path_cost);


    private:

    private:
        vehicleParam vehicle_param_;
        coarseSearchParam coarse_search_param_;
        size_t next_node_num_ = 0;
        double max_steer_angle_ = 0.0;
        double step_size_ = 0.0;
        double xy_grid_resolution_ = 0.0;
        double delta_t_ = 0.0;
        double forward_penalty_ = 0.0;
        double back_penalty_ = 0.0;
        double gear_switch_penalty_ = 0.0;
        double steer_penalty_ = 0.0;
        double steer_change_penalty_ = 0.0;
        double heu_rs_forward_penalty_ = 0.0;
        double heu_rs_back_penalty_ = 0.0;
        double heu_rs_gear_switch_penalty_ = 0.0;
        double heu_rs_steer_penalty_ = 0.0;
        double heu_rs_steer_change_penalty_ = 0.0;
        std::vector<double> XYbounds_;
        std::shared_ptr<nodeTractor> start_node_;
        std::shared_ptr<nodeTractor> end_node_;
        std::shared_ptr<nodeTractor> final_node_;
        std::vector<std::vector<math::LineSegment2d>>
                obstacles_linesegments_vec_;


        double turing_radius_ = 0.0;

        struct cmp {
            bool operator()(const std::pair<std::string, double>& left,
                            const std::pair<std::string, double>& right) const {
                return left.second >= right.second;
            }
        };
        std::priority_queue<std::pair<std::string, double>,
                std::vector<std::pair<std::string, double>>, cmp>
                open_pq_;
        std::unordered_map<std::string, std::shared_ptr<nodeTractor>> open_set_;
        std::unordered_map<std::string, std::shared_ptr<nodeTractor>> close_set_;
        std::unique_ptr<Dubin> dubin_generator_;
        std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;

    };
}
#endif //POLYGONBACKSHAPE_HYBRID_ASTAR_ALGORITHM_H
