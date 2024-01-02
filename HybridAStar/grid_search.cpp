//
// Created by zzm on 23-12-25.
//
#include "HybridAStar/grid_search.h"
#include <fstream>
#include <iostream>

namespace searchAlgorithm{

    GridSearch::GridSearch(const coarseSearchParam& coarse_search_param) {
        xy_resolution_ = coarse_search_param.xy_2d_resolution;
        half_grid_resolution_ = xy_resolution_ / 2;
    }

    double GridSearch::EuclidDistance(const double x1,
                                      const double y1,
                                      const double x2,
                                      const double y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    bool GridSearch::CheckConstraints(std::shared_ptr<Node2d> node) {
        double node_grid_x = node->GetGridX();
        double node_grid_y = node->GetGridY();
        if (node_grid_x > max_grid_x_ || node_grid_x < 0 ||
            node_grid_y > max_grid_y_ || node_grid_y < 0) {
            return false;
        }
        if (obstacles_linesegments_vec_.empty()) {
            return true;
        }
        for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
            for (const math::LineSegment2d& linesegment : obstacle_linesegments) {
                // half_grid_resolution_ is grid resolution
                double x = node->GetGridX() * xy_resolution_ + xy_bounds_[0];
                double y = node->GetGridY() * xy_resolution_ + xy_bounds_[2];
                if (linesegment.DistanceTo({x, y}) <= half_grid_resolution_) {
                    return false;
                }
            }
        }
        return true;
    }


    std::vector<std::shared_ptr<Node2d>> GridSearch::GenerateNextNodes(
            std::shared_ptr<Node2d> current_node) {
        double current_node_x = current_node->GetGridX();
        double current_node_y = current_node->GetGridY();
        double current_node_path_cost = current_node->GetPathCost();
        double diagonal_distance = std::sqrt(2.0);
        std::vector<std::shared_ptr<Node2d>> next_nodes;
        std::shared_ptr<Node2d> up = std::make_shared<Node2d>(
                current_node_x, current_node_y + 1.0, xy_bounds_);
        up->SetPathCost(current_node_path_cost + 1.0);
        std::shared_ptr<Node2d> up_right = std::make_shared<Node2d>(
                current_node_x + 1.0, current_node_y + 1.0, xy_bounds_);
        up_right->SetPathCost(current_node_path_cost + diagonal_distance);
        std::shared_ptr<Node2d> right = std::make_shared<Node2d>(
                current_node_x + 1.0, current_node_y, xy_bounds_);
        right->SetPathCost(current_node_path_cost + 1.0);
        std::shared_ptr<Node2d> down_right = std::make_shared<Node2d>(
                current_node_x + 1.0, current_node_y - 1.0, xy_bounds_);
        down_right->SetPathCost(current_node_path_cost + diagonal_distance);
        std::shared_ptr<Node2d> down = std::make_shared<Node2d>(
                current_node_x, current_node_y - 1.0, xy_bounds_);
        down->SetPathCost(current_node_path_cost + 1.0);
        std::shared_ptr<Node2d> down_left = std::make_shared<Node2d>(
                current_node_x - 1.0, current_node_y - 1.0, xy_bounds_);
        down_left->SetPathCost(current_node_path_cost + diagonal_distance);
        std::shared_ptr<Node2d> left = std::make_shared<Node2d>(
                current_node_x - 1.0, current_node_y, xy_bounds_);
        left->SetPathCost(current_node_path_cost + 1.0);
        std::shared_ptr<Node2d> up_left = std::make_shared<Node2d>(
                current_node_x - 1.0, current_node_y + 1.0, xy_bounds_);
        up_left->SetPathCost(current_node_path_cost + diagonal_distance);

        next_nodes.emplace_back(up);
        next_nodes.emplace_back(up_right);
        next_nodes.emplace_back(right);
        next_nodes.emplace_back(down_right);
        next_nodes.emplace_back(down);
        next_nodes.emplace_back(down_left);
        next_nodes.emplace_back(left);
        next_nodes.emplace_back(up_left);
        return next_nodes;
    }



    bool GridSearch::GenerateAStarPath(
            const double sx,
            const double sy,
            const double ex,
            const double ey,
            const std::vector<double>& xy_bounds,
            const std::vector<std::vector<math::LineSegment2d>>& obstacles_linesegments_vec,
            GridAStartResult* result) {
        std::priority_queue<std::pair<std::string, double>,
                std::vector<std::pair<std::string, double>>,
                cmp>
                open_pq;
        std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
        std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
        xy_bounds_ = xy_bounds;
        std::shared_ptr<Node2d> start_node =
                std::make_shared<Node2d>(sx, sy, xy_resolution_, xy_bounds_);
        std::shared_ptr<Node2d> end_node =
                std::make_shared<Node2d>(ex, ey, xy_resolution_, xy_bounds_);
        std::shared_ptr<Node2d> final_node_ = nullptr;
        obstacles_linesegments_vec_ = obstacles_linesegments_vec;
        open_set.insert(std::make_pair(start_node->GetIndex(), start_node));
        open_pq.push(std::make_pair(start_node->GetIndex(), start_node->GetCost()));

        // Grid a star begins
        size_t explored_node_num = 0;
        while (!open_pq.empty()) {
            std::string current_id = open_pq.top().first;
            open_pq.pop();
            std::shared_ptr<Node2d> current_node = open_set[current_id];
            // Check destination
            if (*(current_node) == *(end_node)) {
                final_node_ = current_node;
                break;
            }
            close_set.emplace(current_node->GetIndex(), current_node);
            std::vector<std::shared_ptr<Node2d>> next_nodes =
                    std::move(GenerateNextNodes(current_node));
            for (auto& next_node : next_nodes) {
                if (!CheckConstraints(next_node)) {
                    continue;
                }
                if (close_set.find(next_node->GetIndex()) != close_set.end()) {
                    continue;
                }
                if (open_set.find(next_node->GetIndex()) == open_set.end()) {
                    ++explored_node_num;
                    next_node->SetHeuristic(EuclidDistance(next_node->GetGridX(),
                                                           next_node->GetGridY(),
                                                           end_node->GetGridX(),
                                                           end_node->GetGridY()));
                    next_node->SetPreNode(current_node);
                    open_set.insert(std::make_pair(next_node->GetIndex(), next_node));
                    open_pq.push(
                            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
                }
            }
        }

        if (final_node_ == nullptr) {
//            SPERROR << "Grid A searching return null ptr(open_set ran out)";
            return false;
        }
        LoadGridAStarResult(result);
//        SPDEBUG << "explored node num is " << explored_node_num;
        return true;
    }



    bool GridSearch::GenerateDpMap(
            const double ex,
            const double ey,
            const std::vector<double>& xy_bounds,
            const std::vector<std::vector<math::LineSegment2d>>& obstacles_linesegments_vec) {
        std::priority_queue<std::pair<std::string, double>,
                std::vector<std::pair<std::string, double>>,
                cmp>
                open_pq;
        std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
        dp_map_ = decltype(dp_map_)();
        xy_bounds_ = xy_bounds;
        // xy_bounds with xmin, xmax, ymin, ymax
        max_grid_y_ = std::round((xy_bounds_[3] - xy_bounds_[2]) / xy_resolution_);
        max_grid_x_ = std::round((xy_bounds_[1] - xy_bounds_[0]) / xy_resolution_);
        std::shared_ptr<Node2d> end_node =
                std::make_shared<Node2d>(ex, ey, xy_resolution_, xy_bounds_);
        obstacles_linesegments_vec_ = obstacles_linesegments_vec;
        open_set.insert(std::make_pair(end_node->GetIndex(), end_node));
        open_pq.push(std::make_pair(end_node->GetIndex(), end_node->GetCost()));

        // Grid a star begins
        size_t explored_node_num = 0;
        while (!open_pq.empty()) {
            const std::string current_id = open_pq.top().first;
            open_pq.pop();
            std::shared_ptr<Node2d> current_node = open_set[current_id];
            dp_map_.insert(std::make_pair(current_node->GetIndex(), current_node));
            std::vector<std::shared_ptr<Node2d>> next_nodes =
                    std::move(GenerateNextNodes(current_node));
            for (auto& next_node : next_nodes) {
                if (!CheckConstraints(next_node)) {
                    continue;
                }
                if (dp_map_.find(next_node->GetIndex()) != dp_map_.end()) {
                    continue;
                }
                if (open_set.find(next_node->GetIndex()) == open_set.end()) {
                    ++explored_node_num;
                    next_node->SetPreNode(current_node);
                    open_set.insert(std::make_pair(next_node->GetIndex(), next_node));
                    open_pq.push(
                            std::make_pair(next_node->GetIndex(), next_node->GetCost()));
                } else {
                    if (open_set[next_node->GetIndex()]->GetCost() > next_node->GetCost()) {
                        open_set[next_node->GetIndex()]->SetCost(next_node->GetCost());
                        open_set[next_node->GetIndex()]->SetPreNode(current_node);
                    }
                }
            }
        }
//        SPDEBUG << "explored node num is " << explored_node_num;
        return true;
    }

    double GridSearch::CheckDpMap(const double sx, const double sy) {
        std::string index = Node2d::CalcIndex(sx, sy, xy_resolution_, xy_bounds_);
        if (dp_map_.find(index) != dp_map_.end()) {
            return dp_map_[index]->GetCost() * xy_resolution_;
        } else {
            return std::numeric_limits<double>::infinity();
        }
    }

    void GridSearch::ShowDpMap() {
        double x_min = xy_bounds_[0];
        double x_max = xy_bounds_[1];
        double y_min = xy_bounds_[2];
        double y_max = xy_bounds_[3];

        std::fstream plan_debug("_dp_map.txt", std::ios::app);
        for (double x = x_min; x < x_max; x += xy_resolution_) {
            for (double y = y_min; y < y_max; y += xy_resolution_) {
                std::string index = Node2d::CalcIndex(x, y, xy_resolution_, xy_bounds_);
                double cost = 0;
                if (dp_map_.find(index) != dp_map_.end()) {
                    cost = dp_map_[index]->GetCost() * xy_resolution_;
                } else {
                    cost = std::numeric_limits<double>::infinity();
                }
                plan_debug << cost << ", ";
            }
            plan_debug << std::endl;
        }
        plan_debug.close();
    }

    void GridSearch::LoadGridAStarResult(GridAStartResult* result) {
        (*result).path_cost = final_node_->GetPathCost() * xy_resolution_;
        std::shared_ptr<Node2d> current_node = final_node_;
        std::vector<double> grid_a_x;
        std::vector<double> grid_a_y;
        while (current_node->GetPreNode() != nullptr) {
            grid_a_x.push_back(current_node->GetGridX() * xy_resolution_ +
                               xy_bounds_[0]);
            grid_a_y.push_back(current_node->GetGridY() * xy_resolution_ +
                               xy_bounds_[2]);
            current_node = current_node->GetPreNode();
        }
        std::reverse(grid_a_x.begin(), grid_a_x.end());
        std::reverse(grid_a_y.begin(), grid_a_y.end());
        (*result).x = std::move(grid_a_x);
        (*result).y = std::move(grid_a_y);
    }
}