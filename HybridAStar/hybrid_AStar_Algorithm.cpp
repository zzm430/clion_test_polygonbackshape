//
// Created by zzm on 23-12-25.
//
#include "HybridAStar/hybrid_AStar_Algorithm.h"


namespace searchAlgorithm{

hybridAStarAlgorithm::hybridAStarAlgorithm(const vehicleParam &vehicle_Param,
                                               const coarseSearchParam &coarse_search_param):
                                               vehicle_param_(vehicle_Param),
                                               coarse_search_param_(coarse_search_param) {
    dubin_generator_ =
            std::make_unique<Dubin>(vehicle_Param, coarse_search_param);
    grid_a_star_heuristic_generator_ =
            std::make_unique<GridSearch>(coarse_search_param);
    next_node_num_ = coarse_search_param.next_node_num;
    max_steer_angle_ = vehicle_Param.max_frontWheel_SteerAngle;
    step_size_ = coarse_search_param.step_size;
    xy_grid_resolution_ =
            coarse_search_param.xy_resolution;
    delta_t_ = coarse_search_param.delta_t;
    forward_penalty_ =
            coarse_search_param.forward_penalty;
    back_penalty_ =
            coarse_search_param.back_penalty;
    gear_switch_penalty_ =
            coarse_search_param.steer_change_penalty;
    steer_penalty_ =
            coarse_search_param.steer_penalty;
    steer_change_penalty_ =
            coarse_search_param.steer_change_penalty;
}

bool hybridAStarAlgorithm::AnalyticExpansion(std::shared_ptr<nodeTractor> current_node) {
    std::vector<dubinPath> all_paths;
    if (!dubin_generator_->AllDPs(current_node, end_node_, &all_paths)) {
        LOG(ERROR) << "Analytic calc error.";
        return false;
    }
    double min_cost = std::numeric_limits<double>::infinity();
    double optimal_path_index = -1;
    for (size_t i = 0; i < all_paths.size(); ++i) {
        if (AnalyticExpansionCheck(all_paths[i])) {
            double current_cost = CalculateAnalyticPathCost(all_paths[i]);
            if (current_cost < min_cost) {
                min_cost = current_cost;
                optimal_path_index = i;
            }
        }
    }
    if (optimal_path_index == -1) {
        return false;
    }

//    // 避免dubin曲线最后的大回环转圈
//    if (min_cost > turing_radius_ * M_PI * 1.5) {
//        return false;
//    }
    auto& optimal_path = all_paths[optimal_path_index];
    final_node_ = LoadAnalyticPinCS(optimal_path, current_node, min_cost);
    return true;
}

std::shared_ptr<nodeTractor> hybridAStarAlgorithm::LoadAnalyticPinCS(
            const dubinPath& dubinPath_to_end,
            std::shared_ptr<nodeTractor> current_node,
            double db_path_cost) {
        std::shared_ptr<nodeTractor> end_node =
                std::shared_ptr<nodeTractor>(new nodeTractor(
                                                             dubinPath_to_end.x,
                                                   dubinPath_to_end.y,
                                                             dubinPath_to_end.phi,
                                                             XYbounds_,
                                                   coarse_search_param_));
        end_node->SetPre(current_node);
        end_node->SetTrajCost(current_node->GetTrajCost() + db_path_cost);
        close_set_.insert(std::make_pair(end_node->GetIndex(), end_node));
        return end_node;
}



bool hybridAStarAlgorithm::AnalyticExpansionCheck(
            const dubinPath& dubinPath_to_end) {
        std::shared_ptr<nodeTractor> node =
                std::shared_ptr<nodeTractor>(new nodeTractor(dubinPath_to_end.x,
                                                             dubinPath_to_end.y,
                                                             dubinPath_to_end.phi,
                                                             XYbounds_,
                                                   coarse_search_param_));
        return ValidityCheckWithAnalysis(node);
}


bool hybridAStarAlgorithm::ValidityCheckWithAnalysis(std::shared_ptr<nodeTractor> node) {
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    bool is_validity = ValidityCheck(node);
    return is_validity;
}


bool hybridAStarAlgorithm::ValidityCheck(std::shared_ptr<nodeTractor> node) {
        CHECK_NOTNULL(node);
        CHECK_GT(node->GetStepSize(), 0);

        if (obstacles_linesegments_vec_.empty()) {
            return true;
        }

        size_t node_step_size = node->GetStepSize();
        const auto& traversed_x = node->GetXs();
        const auto& traversed_y = node->GetYs();
        const auto& traversed_phi = node->GetPhis();

        // The first {x, y, phi} is collision free unless they are start and end
        // configuration of search problem
        //只有起点和终点的node_step_size为1,nextNodeGenerator中都是将父节点先加入节点向量，
        //再放入子节点，至少有两个
        size_t check_start_index = 0;
        if (node_step_size == 1) {
            check_start_index = 0;
        } else {
            check_start_index = 1;
        }

        for (size_t i = check_start_index; i < node_step_size; ++i) {
            //边界检查
            if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
                traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
                return false;
            }
            math::Box2d bounding_box = nodeTractor::GetBoundingBox(
                    vehicle_param_, traversed_x[i], traversed_y[i], traversed_phi[i]);
            for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
                for (const math::LineSegment2d& linesegment :
                        obstacle_linesegments) {
                    if (bounding_box.HasOverlap(linesegment)) {
                        std::cout << "collision start at x: " << linesegment.start().x();
                        std::cout << "collision start at y: " << linesegment.start().y();
                        std::cout << "collision end at x: " << linesegment.end().x();
                        std::cout << "collision end at y: " << linesegment.end().y();
                        return false;
                    }
                }
            }
        }
        return true;
}


double hybridAStarAlgorithm::CalculateAnalyticPathCost(dubinPath& path) {
        double piecewise_cost = 0.0;
        size_t path_segs_size = path.segs_lengths.size();
        for (size_t i = 0; i < path_segs_size; ++i) {
            piecewise_cost += std::abs(path.segs_lengths[i]) * forward_penalty_;
        }

        for (size_t i = 0; i < path_segs_size; ++i) {
            if (path.segs_types[i] != 'S') {
                piecewise_cost += std::abs(max_steer_angle_) * steer_penalty_;
            }
        }

        std::vector<double> steering(path_segs_size, 0.0);
        for (size_t i = 0; i < path_segs_size; ++i) {
            if (path.segs_types[i] == 'L') {
                steering[i] = max_steer_angle_;
            } else if (path.segs_types[i] == 'R') {
                steering[i] = -max_steer_angle_;
            }
        }
        for (size_t i = 1; i < path_segs_size; ++i) {
            piecewise_cost +=
                    std::abs(steering[i] - steering[i - 1]) * steer_change_penalty_;
        }
        return piecewise_cost;
}




std::shared_ptr<nodeTractor> hybridAStarAlgorithm::Next_node_generator(
            std::shared_ptr<nodeTractor> current_node, size_t next_node_index) {
        double steering = 0.0;
        double traveled_distance = 0.0;
        if (next_node_index < static_cast<double>(next_node_num_) / 2) {
            steering =
                    -max_steer_angle_ +
                    (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                    static_cast<double>(next_node_index);
            traveled_distance = step_size_;
        } else {
            size_t index = next_node_index - next_node_num_ / 2;
            steering =
                    -max_steer_angle_ +
                    (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
                    static_cast<double>(index);
            traveled_distance = -step_size_;
        }
        // take above motion primitive to generate a curve driving the car to a
        // different grid
        double arc = std::sqrt(2) * xy_grid_resolution_;
        std::vector<double> intermediate_x;
        std::vector<double> intermediate_y;
        std::vector<double> intermediate_phi;
        double last_x = current_node->GetX();
        double last_y = current_node->GetY();
        double last_phi = current_node->GetPhi();
        intermediate_x.push_back(last_x);
        intermediate_y.push_back(last_y);
        intermediate_phi.push_back(last_phi);
        for (size_t i = 0; i < arc / step_size_; ++i) {
            const double next_x = last_x + traveled_distance * std::cos(last_phi);
            const double next_y = last_y + traveled_distance * std::sin(last_phi);
            const double next_phi = math::NormalizeAngle(
                    last_phi +
                    traveled_distance / vehicle_param_.tractor_wheel_base * std::tan(steering));
            intermediate_x.push_back(next_x);
            intermediate_y.push_back(next_y);
            intermediate_phi.push_back(next_phi);
            last_x = next_x;
            last_y = next_y;
            last_phi = next_phi;
        }
        // check if the vehicle runs outside of XY boundary
        if (intermediate_x.back() > XYbounds_[1] ||
            intermediate_x.back() < XYbounds_[0] ||
            intermediate_y.back() > XYbounds_[3] ||
            intermediate_y.back() < XYbounds_[2]) {
            return nullptr;
        }
        std::shared_ptr<nodeTractor> next_node = std::shared_ptr<nodeTractor>(
                new nodeTractor(intermediate_x,
                                intermediate_y,
                                intermediate_phi,
                                XYbounds_,
                                coarse_search_param_));
        next_node->SetPre(current_node);
        next_node->SetDirec(traveled_distance > 0.0);
        next_node->SetSteer(steering);
        return next_node;
}

void hybridAStarAlgorithm::CalculateNodeCost(std::shared_ptr<nodeTractor> current_node,
                                        std::shared_ptr<nodeTractor> next_node) {
        next_node->SetTrajCost(current_node->GetTrajCost() +
                               TrajCost(current_node, next_node));
        // evaluate heuristic cost
        double optimal_path_cost = 0.0;
        optimal_path_cost += HoloObstacleHeuristic(next_node);
        next_node->SetHeuCost(optimal_path_cost);
}


double hybridAStarAlgorithm::TrajCost(
                                 std::shared_ptr<nodeTractor> current_node,
                                 std::shared_ptr<nodeTractor> next_node) {
        // evaluate cost on the trajectory and add current cost
        double piecewise_cost = 0.0;
        if (next_node->GetDirec()) {
            piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                              step_size_ * forward_penalty_;
        } else {
            piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                              step_size_ * back_penalty_;
        }
        if (current_node->GetDirec() != next_node->GetDirec()) {
            piecewise_cost += gear_switch_penalty_;
        }
        piecewise_cost += steer_penalty_ * std::abs(next_node->GetSteer());
        piecewise_cost += steer_change_penalty_ *
                          std::abs(next_node->GetSteer() - current_node->GetSteer());
        return piecewise_cost;
}

double hybridAStarAlgorithm::HoloObstacleHeuristic(std::shared_ptr<nodeTractor> next_node) {
        return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                            next_node->GetY());
}

bool hybridAStarAlgorithm::GetResult(HybridAStartResult* result) {
        std::shared_ptr<nodeTractor> current_node = final_node_;
        std::vector<double> hybrid_a_x;
        std::vector<double> hybrid_a_y;
        std::vector<double> hybrid_a_phi;
        while (current_node->GetPreNode() != nullptr) {
            std::vector<double> x = current_node->GetXs();
            std::vector<double> y = current_node->GetYs();
            std::vector<double> phi = current_node->GetPhis();
            if (x.empty() || y.empty() || phi.empty()) {
                std::cout << "result size check failed";
                return false;
            }
            if (x.size() != y.size() || x.size() != phi.size()) {
                std::cout << "states sizes are not equal";
                return false;
            }
            std::reverse(x.begin(), x.end());
            std::reverse(y.begin(), y.end());
            std::reverse(phi.begin(), phi.end());
            x.pop_back();
            y.pop_back();
            phi.pop_back();
            hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
            hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
            hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
            current_node = current_node->GetPreNode();
        }

        hybrid_a_x.push_back(current_node->GetX());
        hybrid_a_y.push_back(current_node->GetY());
        hybrid_a_phi.push_back(current_node->GetPhi());

        std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
        std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
        std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
        (*result).x = hybrid_a_x;
        (*result).y = hybrid_a_y;
        (*result).phi = hybrid_a_phi;

        if (result->x.size() != result->y.size() ||
            result->x.size() != result->phi.size()) {
            std::cout << "state sizes not equal, "
                   << "result->x.size(): " << result->x.size() << "result->y.size()"
                   << result->y.size() << "result->phi.size()" << result->phi.size()
                   << "result->v.size()" << result->v.size();
            return false;
        }

//        if (result->a.size() != result->steer.size() ||
//            result->x.size() - result->a.size() != 1) {
//            std::cout << "control sizes not equal or not right";
//            std::cout << " acceleration size: " << result->a.size();
//            std::cout << " steer size: " << result->steer.size();
//            std::cout << " x size: " << result->x.size();
//            return false;
//        }
        return true;
}

bool hybridAStarAlgorithm::Plan(
            double sx,
            double sy,
            double sphi,
            double ex,
            double ey,
            double ephi,
            const std::vector<double>& XYbounds,
            const std::vector<std::vector<math::Vec2d>>& obstacles_vertices_vec,
            HybridAStartResult* result) {
        // clear containers
        open_set_.clear();
        close_set_.clear();
        open_pq_ = decltype(open_pq_)();
        final_node_ = nullptr;

        std::vector<std::vector<math::LineSegment2d>>
                obstacles_linesegments_vec;
        for (const auto& obstacle_vertices : obstacles_vertices_vec) {
            size_t vertices_num = obstacle_vertices.size();
            std::vector<math::LineSegment2d> obstacle_linesegments;
            for (size_t i = 0; i < vertices_num - 1; ++i) {
                math::LineSegment2d line_segment = math::LineSegment2d(
                        obstacle_vertices[i], obstacle_vertices[i + 1]);
                obstacle_linesegments.emplace_back(line_segment);
            }
            obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
        }
        obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

        // load XYbounds
        XYbounds_ = XYbounds;
        // load nodes and obstacles
        start_node_.reset(
                new nodeTractor({sx}, {sy}, {sphi}, XYbounds_, coarse_search_param_));
        end_node_.reset(
                new nodeTractor({ex}, {ey}, {ephi}, XYbounds_, coarse_search_param_));
        if (!ValidityCheck(start_node_)) {
            std::cout << "start_node in collision with obstacles";
            return false;
        }
        if (!ValidityCheck(end_node_)) {
            std::cout << "end_node in collision with obstacles";
            return false;
        }
//      double map_time = Clock::NowInSeconds();
        grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_,
                                                        obstacles_linesegments_vec_);
//       std::cout << "map time " << Clock::NowInSeconds() - map_time;
        // load open set, pq
        open_set_.emplace(start_node_->GetIndex(), start_node_);
        open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

        // Hybrid A* begins
        size_t explored_node_num = 0;
//      double astar_start_time = Clock::NowInSeconds();
        double heuristic_time = 0.0;
        double rs_time = 0.0;
        while (!open_pq_.empty()) {
            // take out the lowest cost neighboring node
            const std::string current_id = open_pq_.top().first;
            open_pq_.pop();
            std::shared_ptr<nodeTractor> current_node = open_set_[current_id];
            // check if an analystic curve could be connected from current
            // configuration to the end configuration without collision. if so, search
            // ends.
//          const double rs_start_time = Clock::NowInSeconds();
            if (AnalyticExpansion(current_node)) {
                break;
            }

//          const double rs_end_time = Clock::NowInSeconds();
//          rs_time += rs_end_time - rs_start_time;
            close_set_.emplace(current_node->GetIndex(), current_node);
            for (size_t i = 0; i < next_node_num_; ++i) {
                std::shared_ptr<nodeTractor> next_node = Next_node_generator(current_node, i);
                // boundary check failure handle
                if (next_node == nullptr) {
                    continue;
                }
                // check if the node is already in the close set
                if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
                    continue;
                }
                // collision check
                if (!ValidityCheck(next_node)) {
                    continue;
                }
                if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
                    explored_node_num++;
//                    const double start_time = Clock::NowInSeconds();
                    CalculateNodeCost(current_node, next_node);
//                    const double end_time = Clock::NowInSeconds();
//                    heuristic_time += end_time - start_time;
                    open_set_.emplace(next_node->GetIndex(), next_node);
                    open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
                }
            }
        }

        if (final_node_ == nullptr) {
            std::cout << "Hybrid A searching return null ptr(open_set ran out)";
            return false;
        }

        if (!GetResult(result)) {
            std::cout << "GetResult failed";
            return false;
        }
        std::cout << "explored node num is " << explored_node_num;
        std::cout << "heuristic time is " << heuristic_time;
        std::cout << "dubin time is " << rs_time;
//        std::cout << "hybrid astar total time is "
//               << Clock::NowInSeconds() - astar_start_time;
        return true;
}


















}