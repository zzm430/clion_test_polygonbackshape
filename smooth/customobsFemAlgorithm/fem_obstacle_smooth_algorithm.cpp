//
// Created by zzm on 24-1-9.
//
#include "fem_obstacle_smooth_algorithm.h"

namespace  smoothalgorithm{

femObstacleAnchoringSmoother::femObstacleAnchoringSmoother(
        const femObstacleParam&  fem_obstacle_param,
        const std::vector<std::vector<math::Vec2d>>&
        obstacles_vertices_vec):fem_obstacle_param_(fem_obstacle_param),
                                obstacles_vertices_vec_(obstacles_vertices_vec){
    ego_length_ = fem_obstacle_param_.ego_length;
    ego_width_ = fem_obstacle_param_.ego_width;
    center_shift_distance_ = ego_length_ /2.0
            - fem_obstacle_param_.back_edge_to_center;
}

bool femObstacleAnchoringSmoother::Smooth(const std::vector<polygonPoint> &pathPts) {

    if (pathPts.size() < 2) {
        LOG(ERROR) << "reference points size smaller than two, smoother early "
                  "returned";
        return false;
    }

    // Set obstacle in form of linesegments
    std::vector<std::vector<math::LineSegment2d>> obstacles_linesegments_vec;
    for (const auto& obstacle_vertices : obstacles_vertices_vec_) {
        size_t vertices_num = obstacle_vertices.size();
        std::vector<math::LineSegment2d> obstacle_linesegments;
        for (size_t i = 0; i + 1 < vertices_num; ++i) {
            math::LineSegment2d line_segment =
                    math::LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
            obstacle_linesegments.emplace_back(line_segment);
        }
        obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
    }
    obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

    // Interpolate the traj
    DiscretizedPath warm_start_path;
    size_t xWS_size = pathPts.size();
    double accumulated_s = 0.0;
    math::Vec2d last_path_point(pathPts[0].x, pathPts[0].y);
    for (size_t i = 0; i < xWS_size; ++i) {
        math::Vec2d cur_path_point(pathPts[i].x, pathPts[i].y);
        accumulated_s += cur_path_point.DistanceTo(last_path_point);
        PathPoint path_point;
        path_point.set_x(pathPts[i].x);
        path_point.set_y(pathPts[i].y);
        path_point.set_theta(pathPts[i].theta());
        path_point.set_s(accumulated_s);
        warm_start_path.push_back(std::move(path_point));
        last_path_point = cur_path_point;
    }

    const double interpolated_delta_s =
            fem_obstacle_param_.interpolated_delta_s;
    std::vector<std::pair<double, double>> interpolated_warm_start_point2ds;
    double path_length = warm_start_path.Length();
    double delta_s = path_length / std::ceil(path_length / interpolated_delta_s);
    path_length += delta_s * 1.0e-6;
    for (double s = 0; s < path_length; s += delta_s) {
        const auto point2d = warm_start_path.Evaluate(s);
        interpolated_warm_start_point2ds.emplace_back(point2d.x(), point2d.y());
    }

    const size_t interpolated_size = interpolated_warm_start_point2ds.size();
    if (interpolated_size < 4) {
        LOG(ERROR)<< "interpolated_warm_start_path smaller than 4, can't enforce "
                  "heading continuity";
        return false;
    } else if (interpolated_size < 6) {
        LOG(DEBUG) << "interpolated_warm_start_path smaller than 4, can't enforce "
                  "initial zero kappa";
        enforce_initial_kappa_ = false;
    } else {
        enforce_initial_kappa_ = true;
    }

    // Adjust heading to ensure heading continuity
    AdjustStartEndHeading(pathPts, &interpolated_warm_start_point2ds);

    // Reset path profile by discrete point heading and curvature estimation
    DiscretizedPath interpolated_warm_start_path;
    if (!SetPathProfile(interpolated_warm_start_point2ds,
                        &interpolated_warm_start_path)) {
        LOG(DEBUG) << "Set path profile fails";
        return false;
    }

    // Generate feasible bounds for each path point
    std::vector<double> bounds;
    if (!GenerateInitialBounds(interpolated_warm_start_path, &bounds)) {
        LOG(ERROR) << "Generate initial bounds failed, path point to close to obstacle";
        return false;
    }

    // Check initial path collision avoidance, if it fails, smoother assumption
    // fails. Try reanchoring
    input_colliding_point_index_.clear();
    if (!CheckCollisionAvoidance(interpolated_warm_start_path,
                                 &input_colliding_point_index_)) {
        LOG(ERROR) << "Interpolated input path points colliding with obstacle";
        // if (!ReAnchoring(colliding_point_index, &interpolated_warm_start_path)) {
        //   AERROR << "Fail to reanchor colliding input path points";
        //   return false;
        // }
    }

    // Smooth path to have smoothed x, y, phi, kappa and s
    DiscretizedPath smoothed_path_points;
    if (!SmoothPath(interpolated_warm_start_path, bounds,
                    &smoothed_path_points)) {
        return false;
    }
    for(auto i : smoothed_path_points){
        polygonPoint tempPt;
        tempPt.x = i.x();
        tempPt.y = i.y();
        storage_smoothed_pts_.push_back(tempPt);
    }
    return true;
}

std::vector<polygonPoint>  femObstacleAnchoringSmoother::getStorageSmoothedPts(){
    return storage_smoothed_pts_;
}

void femObstacleAnchoringSmoother::AdjustStartEndHeading(
            const std::vector<polygonPoint>& pathPts,
            std::vector<std::pair<double, double>>* const point2d) {
        // Sanity check
        CHECK_NOTNULL(point2d);
        CHECK_GT(point2d->size(), 3);

        // Set initial heading and bounds
        auto size_pts = pathPts.size();
        const double initial_heading = pathPts[0].heading();
        const double end_heading = pathPts[size_pts-1].heading() ;

        // Adjust the point position to have heading by finite element difference of
        // the point and the other point equal to the given warm start initial or end
        // heading
        const double first_to_second_dx = point2d->at(1).first - point2d->at(0).first;
        const double first_to_second_dy =
                point2d->at(1).second - point2d->at(0).second;
        const double first_to_second_s =
                std::sqrt(first_to_second_dx * first_to_second_dx +
                          first_to_second_dy * first_to_second_dy);
        math::Vec2d first_point(point2d->at(0).first, point2d->at(0).second);
        math::Vec2d initial_vec(first_to_second_s, 0);
        initial_vec.SelfRotate(gear_ ? initial_heading
                                     : math::NormalizeAngle(initial_heading + M_PI));
        initial_vec += first_point;
        point2d->at(1) = std::make_pair(initial_vec.x(), initial_vec.y());

        const size_t path_size = point2d->size();
        const double second_last_to_last_dx =
                point2d->at(path_size - 1).first - point2d->at(path_size - 2).first;
        const double second_last_to_last_dy =
                point2d->at(path_size - 1).second - point2d->at(path_size - 2).second;
        const double second_last_to_last_s =
                std::sqrt(second_last_to_last_dx * second_last_to_last_dx +
                          second_last_to_last_dy * second_last_to_last_dy);
        math::Vec2d last_point(point2d->at(path_size - 1).first,
                         point2d->at(path_size - 1).second);
        math::Vec2d end_vec(second_last_to_last_s, 0);
        end_vec.SelfRotate(gear_ ? math::NormalizeAngle(end_heading + M_PI) : end_heading);
        end_vec += last_point;
        point2d->at(path_size - 2) = std::make_pair(end_vec.x(), end_vec.y());
}

bool femObstacleAnchoringSmoother::ReAnchoring(
            const std::vector<size_t>& colliding_point_index,
            DiscretizedPath* path_points) {
        CHECK_NOTNULL(path_points);
        if (colliding_point_index.empty()) {
            LOG(DEBUG) << "no point needs to be re-anchored";
            return true;
        }
        CHECK_GT(path_points->size(),
                 *(std::max_element(colliding_point_index.begin(),
                                    colliding_point_index.end())));

        for (const auto index : colliding_point_index) {
            if (index == 0 || index == path_points->size() - 1) {
                LOG(ERROR) << "Initial and end points collision avoid condition failed.";
                return false;
            }
            if (index == 1 || index == path_points->size() - 2) {
                LOG(ERROR) << "second to last point or second point pos reanchored. Heading "
                          "discontinuity might "
                          "happen";
            }
        }

        // TODO(Jinyun): move to confs
        const size_t reanchoring_trails_num = static_cast<size_t>(
                fem_obstacle_param_.reanchoring_trails_num);
        const double reanchoring_pos_stddev =
                fem_obstacle_param_.reanchoring_pos_stddev;
        const double reanchoring_length_stddev =
                fem_obstacle_param_.reanchoring_length_stddev;
        std::random_device rd;
        std::default_random_engine gen = std::default_random_engine(rd());
        std::normal_distribution<> pos_dis{0, reanchoring_pos_stddev};
        std::normal_distribution<> length_dis{0, reanchoring_length_stddev};

        for (const auto index : colliding_point_index) {
            bool reanchoring_success = false;
            for (size_t i = 0; i < reanchoring_trails_num; ++i) {
                // Get ego box for collision check on collision point index
                bool is_colliding = false;
                for (size_t j = index - 1; j < index + 2; ++j) {
                    const double heading =
                            gear_ ? path_points->at(j).theta()
                                  : math::NormalizeAngle(path_points->at(j).theta() + M_PI);
                    math::Box2d ego_box({path_points->at(j).x() +
                                   center_shift_distance_ * std::cos(heading),
                                   path_points->at(j).y() +
                                   center_shift_distance_ * std::sin(heading)},
                                  heading, ego_length_, ego_width_);
                    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
                        for (const math::LineSegment2d& linesegment : obstacle_linesegments) {
                            if (ego_box.HasOverlap(linesegment)) {
                                is_colliding = true;
                                break;
                            }
                        }
                        if (is_colliding) {
                            break;
                        }
                    }
                    if (is_colliding) {
                        break;
                    }
                }

                if (is_colliding) {
                    // Adjust the point by randomly move around the original points
                    if (index == 1) {
                        const double adjust_theta = path_points->at(index - 1).theta();
                        const double delta_s = std::abs(path_points->at(index).s() -
                                                        path_points->at(index - 1).s());
                        double rand_dev = math::Clamp(length_dis(gen), 0.8, -0.8);
                        double adjusted_delta_s = delta_s * (1.0 + rand_dev);
                        path_points->at(index).set_x(path_points->at(index - 1).x() +
                                                     adjusted_delta_s *
                                                     std::cos(adjust_theta));
                        path_points->at(index).set_y(path_points->at(index - 1).y() +
                                                     adjusted_delta_s *
                                                     std::sin(adjust_theta));
                    } else if (index == path_points->size() - 2) {
                        const double adjust_theta =
                                math::NormalizeAngle(path_points->at(index + 1).theta() + M_PI);
                        const double delta_s = std::abs(path_points->at(index + 1).s() -
                                                        path_points->at(index).s());
                        double rand_dev = math::Clamp(length_dis(gen), 0.8, -0.8);
                        double adjusted_delta_s = delta_s * (1.0 + rand_dev);
                        path_points->at(index).set_x(path_points->at(index + 1).x() +
                                                     adjusted_delta_s *
                                                     std::cos(adjust_theta));
                        path_points->at(index).set_y(path_points->at(index + 1).y() +
                                                     adjusted_delta_s *
                                                     std::sin(adjust_theta));
                    } else {
                        double rand_dev_x =
                                math::Clamp(pos_dis(gen), 2.0 * reanchoring_pos_stddev,
                                                    -2.0 * reanchoring_pos_stddev);
                        double rand_dev_y =
                                math::Clamp(pos_dis(gen), 2.0 * reanchoring_pos_stddev,
                                                    -2.0 * reanchoring_pos_stddev);
                        path_points->at(index).set_x(path_points->at(index).x() + rand_dev_x);
                        path_points->at(index).set_y(path_points->at(index).y() + rand_dev_y);
                    }

                    // Adjust heading accordingly
                    // TODO(Jinyun): refactor into math module
                    // current point heading adjustment
                    for (size_t i = index - 1; i < index + 2; ++i) {
                        path_points->at(i).set_theta(CalcHeadings(*path_points, i));
                    }
                } else {
                    reanchoring_success = true;
                    break;
                }
            }

            if (!reanchoring_success) {
                LOG(ERROR) << "interpolated points at index " << index
                       << "can't be successfully reanchored";
                return false;
            }
        }
        return true;
}

bool femObstacleAnchoringSmoother::GenerateInitialBounds(
            const DiscretizedPath& path_points, std::vector<double>* initial_bounds) {
        CHECK_NOTNULL(initial_bounds);
        initial_bounds->clear();

        const bool estimate_bound =
                fem_obstacle_param_.estimate_bound;
        const double default_bound =
                fem_obstacle_param_.default_bound;
        const double vehicle_shortest_dimension =
                fem_obstacle_param_.vehicle_shortest_dimension;
        const double kEpislon = 1e-8;

        if (!estimate_bound) {
            std::vector<double> default_bounds(path_points.size(), default_bound);
            *initial_bounds = std::move(default_bounds);
            return true;
        }

        // TODO(Jinyun): refine obstacle formulation and speed it up
        for (const auto& path_point : path_points) {
            double min_bound = std::numeric_limits<double>::infinity();
            for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
                for (const math::LineSegment2d& linesegment : obstacle_linesegments) {
                    min_bound =
                            std::min(min_bound,
                                     linesegment.DistanceTo({path_point.x(), path_point.y()}));
                }
            }
            min_bound -= vehicle_shortest_dimension;
            min_bound = min_bound < kEpislon ? 0.0 : min_bound;
            initial_bounds->push_back(min_bound);
        }
        return true;
}

bool femObstacleAnchoringSmoother::SmoothPath(
            const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
            DiscretizedPath* smoothed_path_points) {
        std::vector<std::pair<double, double>> raw_point2d;
        std::vector<double> flexible_bounds;
        for (const auto& path_point : raw_path_points) {
            raw_point2d.emplace_back(path_point.x(), path_point.y());
        }
        flexible_bounds = bounds;

      femPosDeviationParam  femPosDeviationParamInstance;
      femPosDeviationParamInstance.weight_fem_pos_deviation = WEIGHT_FEM_POS_DEVIATION;
      femPosDeviationParamInstance.weight_path_length = WEIGHT_PATH_LEGNTH;
      femPosDeviationParamInstance.weight_ref_deviation = WEIGHT_REF_DEVIATION;
      femPosDeviationParamInstance.weight_curvature_constrain_slack_var = WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR;
      femPosDeviationParamInstance.apply_curvature_constraint = APPLY_CURVATURE_CONSTRAINT;
      femPosDeviationParamInstance.sqp_sub_max_iter = SQP_SUB_MAX_ITER;
      femPosDeviationParamInstance.sqp_ftol = SQP_FTOL;
      femPosDeviationParamInstance.sqp_pen_max_iter = SQP_PEN_MAX_ITER;
      femPosDeviationParamInstance.sqp_ctol = SQP_CTOL;
      femPosDeviationParamInstance.max_iter_fem = MAX_ITER_FEM;
      femPosDeviationParamInstance.time_limit = TIME_LIMIT;
      femPosDeviationParamInstance.verboase = VERBOSE;
      femPosDeviationParamInstance.scaled_termination = SCALED_TERMINATION;
      femPosDeviationParamInstance.warm_start_m = WARM_START_M;
      FemPosDeviationSmoother fem_pos_smoother(femPosDeviationParamInstance);

        // TODO(Jinyun): move to confs
        const size_t max_iteration_num = 50;

        bool is_collision_free = false;
        std::vector<size_t> colliding_point_index;
        std::vector<std::pair<double, double>> smoothed_point2d;
        size_t counter = 0;

        while (!is_collision_free) {
            if (counter > max_iteration_num) {
                LOG(ERROR) << "Maximum iteration reached, path smoother early stops";
                return true;
            }

            AdjustPathBounds(colliding_point_index, &flexible_bounds);

            std::vector<double> opt_x;
            std::vector<double> opt_y;
            if (!fem_pos_smoother.Solve(raw_point2d, flexible_bounds, &opt_x, &opt_y)) {
                LOG(ERROR) << "Smoothing path fails";
                return false;
            }

            if (opt_x.size() < 2 || opt_y.size() < 2) {
                LOG(ERROR) << "Return by fem_pos_smoother is wrong. Size smaller than 2 ";
                return false;
            }

            CHECK_EQ(opt_x.size(), opt_y.size());

            size_t point_size = opt_x.size();
            smoothed_point2d.clear();
            for (size_t i = 0; i < point_size; ++i) {
                smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
            }

            if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
                LOG(ERROR) << "Set path profile fails";
                return false;
            }

            is_collision_free =
                    CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);

            LOG(DEBUG) << "loop iteration number is " << counter;
            ++counter;
        }
        return true;
}

bool femObstacleAnchoringSmoother::CheckCollisionAvoidance(
            const DiscretizedPath& path_points,
            std::vector<size_t>* colliding_point_index) {
        CHECK_NOTNULL(colliding_point_index);

        colliding_point_index->clear();
        size_t path_points_size = path_points.size();
        for (size_t i = 0; i < path_points_size; ++i) {
            // Skip checking collision for thoese points colliding originally
            bool skip_checking = false;
            for (const auto index : input_colliding_point_index_) {
                if (i == index) {
                    skip_checking = true;
                    break;
                }
            }
            if (skip_checking) {
                continue;
            }

            const double heading = gear_
                                   ? path_points[i].theta()
                                   : math::NormalizeAngle(path_points[i].theta() + M_PI);
            math::Box2d ego_box(
                    {path_points[i].x() + center_shift_distance_ * std::cos(heading),
                     path_points[i].y() + center_shift_distance_ * std::sin(heading)},
                    heading, ego_length_, ego_width_);

            bool is_colliding = false;
            for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
                for (const math::LineSegment2d& linesegment : obstacle_linesegments) {
                    if (ego_box.HasOverlap(linesegment)) {
                        colliding_point_index->push_back(i);
                        LOG(DEBUG) << "point at " << i << "collied with LineSegment "<< std::endl;
                        is_colliding = true;
                        break;
                    }
                }
                if (is_colliding) {
                    break;
                }
            }
        }

        if (!colliding_point_index->empty()) {
            return false;
        }
        return true;
}

void femObstacleAnchoringSmoother::AdjustPathBounds(
            const std::vector<size_t>& colliding_point_index,
            std::vector<double>* bounds) {
        CHECK_NOTNULL(bounds);

        const double collision_decrease_ratio =
                fem_obstacle_param_.collision_decrease_ratio;

        for (const auto index : colliding_point_index) {
            bounds->at(index) *= collision_decrease_ratio;
        }

        // Anchor the end points to enforce the initial end end heading continuity and
        // zero kappa
        bounds->at(0) = 0.0;
        bounds->at(1) = 0.0;
        bounds->at(bounds->size() - 1) = 0.0;
        bounds->at(bounds->size() - 2) = 0.0;
        if (enforce_initial_kappa_) {
            bounds->at(2) = 0.0;
        }
}


bool femObstacleAnchoringSmoother::SetPathProfile(
            const std::vector<std::pair<double, double>>& point2d,
            DiscretizedPath* raw_path_points) {
        CHECK_NOTNULL(raw_path_points);
        raw_path_points->clear();
        // Compute path profile
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulated_s;
        if (!computePathProfileInfo::ComputePathProfile(
                point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
            return false;
        }
        CHECK_EQ(point2d.size(), headings.size());
        CHECK_EQ(point2d.size(), kappas.size());
        CHECK_EQ(point2d.size(), dkappas.size());
        CHECK_EQ(point2d.size(), accumulated_s.size());

        // Load into path point
        size_t points_size = point2d.size();
        for (size_t i = 0; i < points_size; ++i) {
            PathPoint path_point;
            path_point.set_x(point2d[i].first);
            path_point.set_y(point2d[i].second);
            path_point.set_theta(headings[i]);
            path_point.set_s(accumulated_s[i]);
            path_point.set_kappa(kappas[i]);
            path_point.set_dkappa(dkappas[i]);
            raw_path_points->push_back(std::move(path_point));
        }
        return true;
}

double femObstacleAnchoringSmoother::CalcHeadings(
            const DiscretizedPath& path_points, const size_t index) {
        CHECK_GT(path_points.size(), 2);
        double dx = 0.0;
        double dy = 0.0;
        if (index == 0) {
            dx = path_points[index + 1].x() - path_points[index].x();
            dy = path_points[index + 1].y() - path_points[index].y();
        } else if (index == path_points.size() - 1) {
            dx = path_points[index].x() - path_points[index - 1].x();
            dy = path_points[index].y() - path_points[index - 1].y();
        } else {
            dx = 0.5 * (path_points[index + 1].x() - path_points[index - 1].x());
            dy = 0.5 * (path_points[index + 1].y() - path_points[index - 1].y());
        }
        return std::atan2(dy, dx);
}

}