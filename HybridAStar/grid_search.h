//
// Created by zzm on 23-12-25.
//

#ifndef POLYGONBACKSHAPE_GRID_SEARCH_H
#define POLYGONBACKSHAPE_GRID_SEARCH_H

#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>

#include "common/math/line_segment_2d.h"
#include "HybridAStar/hybridA_param.h"
#include "easylogging++.h"

namespace searchAlgorithm{
class Node2d {
    public:
        Node2d(const double x, const double y, const double xy_resolution,
               const std::vector<double>& XYbounds) {
            // XYbounds with xmin, xmax, ymin, ymax
            x_ = x;
            y_ = y;
            grid_x_ = static_cast<int>((x - XYbounds[0]) / xy_resolution);
            grid_y_ = static_cast<int>((y - XYbounds[2]) / xy_resolution);
            index_ = ComputeStringIndex(grid_x_, grid_y_);
        }
        Node2d(const int grid_x,
               const int grid_y,
               const std::vector<double>& xy_bounds) {
            grid_x_ = grid_x;
            grid_y_ = grid_y;
            index_ = ComputeStringIndex(grid_x_, grid_y_);
        }

        void SetPathCost(const double path_cost) {
            path_cost_ = path_cost;
            cost_ = path_cost_ + heuristic_;
        }

        void SetHeuristic(const double heuristic) {
            heuristic_ = heuristic;
            cost_ = path_cost_ + heuristic_;
        }

        void SetCost(const double cost) {
            cost_ = cost;
        }
        void SetPreNode(std::shared_ptr<Node2d> pre_node) {
            pre_node_ = pre_node;
        }
        double GetX() const {
            return x_;
        }
        double GetY() const {
            return y_;
        }
        double GetGridX() const {
            return grid_x_;
        }
        double GetGridY() const {
            return grid_y_;
        }
        double GetPathCost() const {
            return path_cost_;
        }
        double GetHeuCost() const {
            return heuristic_;
        }
        double GetCost() const {
            return cost_;
        }
        const std::string& GetIndex() const {
            return index_;
        }
        std::shared_ptr<Node2d> GetPreNode() const {
            return pre_node_;
        }
        static std::string CalcIndex(const double x, const double y,
                                     const double xy_resolution,
                                     const std::vector<double>& XYbounds) {
            // XYbounds with xmin, xmax, ymin, ymax
            int grid_x = static_cast<int>((x - XYbounds[0]) / xy_resolution);
            int grid_y = static_cast<int>((y - XYbounds[2]) / xy_resolution);
            return ComputeStringIndex(grid_x, grid_y);
        }
        bool operator==(const Node2d& right) const {
            return right.GetIndex() == index_;
        }

    private:
        static std::string ComputeStringIndex(int x_grid, int y_grid) {
            return std::to_string(x_grid) + "_" + std::to_string(y_grid);
        }

    private:
        double x_ = 0.0;
        double y_ = 0.0;
        int grid_x_ = 0;
        int grid_y_ = 0;
        double path_cost_ = 0.0;
        double heuristic_ = 0.0;
        double cost_ = 0.0;
        std::string index_;
        std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct GridAStartResult {
        std::vector<double> x;
        std::vector<double> y;
        double path_cost = 0.0;
};


class GridSearch {
    public:
        explicit GridSearch(const coarseSearchParam& coarse_search_param);
        virtual ~GridSearch() = default;
        bool GenerateAStarPath(
                const double sx,
                const double sy,
                const double ex,
                const double ey,
                const std::vector<double>& xy_bounds,
                const std::vector<std::vector<math::LineSegment2d>>& obstacles_linesegments_vec,
                GridAStartResult* result);
        bool GenerateDpMap(const double ex,
                           const double ey,
                           const std::vector<double>& xy_bounds,
                           const std::vector<std::vector<math::LineSegment2d>>&
                           obstacles_linesegments_vec);
        double CheckDpMap(const double sx, const double sy);

        void ShowDpMap();

    private:
        double EuclidDistance(const double x1,
                              const double y1,
                              const double x2,
                              const double y2);
        std::vector<std::shared_ptr<Node2d>> GenerateNextNodes(
                std::shared_ptr<Node2d> node);
        bool CheckConstraints(std::shared_ptr<Node2d> node);
        void LoadGridAStarResult(GridAStartResult* result);

    private:
        double xy_resolution_ = 0.0;
        double half_grid_resolution_ = 0.0;
        std::vector<double> xy_bounds_;
        double max_grid_x_ = 0.0;
        double max_grid_y_ = 0.0;
        std::shared_ptr<Node2d> start_node_;
        std::shared_ptr<Node2d> end_node_;
        std::shared_ptr<Node2d> final_node_;
        std::vector<std::vector<math::LineSegment2d>> obstacles_linesegments_vec_;

        struct cmp {
            bool operator()(const std::pair<std::string, double>& left,
                            const std::pair<std::string, double>& right) const {
                return left.second >= right.second;
            }
        };

        std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
};
}

#endif //POLYGONBACKSHAPE_GRID_SEARCH_H
