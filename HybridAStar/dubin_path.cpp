//
// Created by zzm on 23-12-22.
//
#include "dubin_path.h"

namespace searchAlgorithm{

namespace {
/**
 * Floating point modulus suitable for rings
 *
 * fmod doesn't behave correctly for angular quantities, this function does
 */
        double fmodr(double x, double y) {
            return x - y * std::floor(x / y);
        }

        double mod2pi(double theta) {
            return fmodr(theta, 2 * M_PI);
        }
}  // namespace

Dubin::Dubin(const vehicleParam& vehicle_param,
            const coarseSearchParam& coarse_search_param)
            : vehicle_param_(vehicle_param)
            , planner_open_space_config_(coarse_search_param) {
        double wheelbase = vehicle_param_.tractor_wheel_base;
        turing_radius_ =
                    wheelbase / std::tan(vehicle_param_.max_frontWheel_SteerAngle);
        step_size_ = coarse_search_param.step_size;
}


bool Dubin::AllDPs(const std::shared_ptr<nodeTractor> start_node,
                   const std::shared_ptr<nodeTractor> end_node,
                   std::vector<dubinPath>* all_paths) {
    all_paths->clear();
    start_x_ = start_node->GetX();
    start_y_ = start_node->GetY();
    start_phi_ = start_node->GetPhi();
    DubinsIntermediateResults inter_result =
            dubins_intermediate_results(start_node, end_node, turing_radius_);

    std::vector<double> normalized_seg_lengths(3, 0.0);
    DubinsPath path;
    path.qi[0] = start_x_;
    path.qi[1] = start_y_;
    path.qi[2] = start_phi_;
    path.rho = turing_radius_;

    // LSL
    if (dubins_LSL(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::LSL;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'L', 'S', 'L'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }

    // RSR
    if (dubins_RSR(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::RSR;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'R', 'S', 'R'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }

    // LSR
    if (dubins_LSR(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::LSR;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'L', 'S', 'R'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }

    // RSL
    if (dubins_RSL(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::RSL;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'R', 'S', 'L'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }

    // RLR
    if (dubins_RLR(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::RLR;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'R', 'L', 'R'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }

    // LRL
    if (dubins_LRL(inter_result, &normalized_seg_lengths)) {
        // construct path
        path.param[0] = normalized_seg_lengths[0];
        path.param[1] = normalized_seg_lengths[1];
        path.param[2] = normalized_seg_lengths[2];
        path.type = DubinsPathType::LRL;
        // construct discrete path
        all_paths->emplace_back();
        auto& discret_path = all_paths->back();
        discret_path.total_length = dubins_path_length(&path);
        discret_path.segs_lengths = {
                normalized_seg_lengths[0] * turing_radius_,
                normalized_seg_lengths[1] * turing_radius_,
                normalized_seg_lengths[2] * turing_radius_,
        };
        discret_path.segs_types = {'L', 'R', 'L'};
        dubins_path_sample_many(&path, step_size_, discret_path);
    }
    return true;
}



bool Dubin::ShortestPath(const std::shared_ptr<nodeTractor> start_node,
                             const std::shared_ptr<nodeTractor> end_node,
                             dubinPath* optimal_path,
                             double* optimal_cost) {
        start_x_ = start_node->GetX();
        start_y_ = start_node->GetY();
        start_phi_ = start_node->GetPhi();
        DubinsIntermediateResults inter_result =
                dubins_intermediate_results(start_node, end_node, turing_radius_);

        std::vector<double> normalized_seg_lengths(3, 0.0);
        DubinsPath optimal_inter_path;
        DubinsPath path;
        path.qi[0] = start_x_;
        path.qi[1] = start_y_;
        path.qi[2] = start_phi_;
        path.rho = turing_radius_;

        (*optimal_path).total_length = 10e9;
        // LSL
        if (dubins_LSL(inter_result, &normalized_seg_lengths)) {
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::LSL;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;
                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'L', 'S', 'L'};
            }
        }

        // RSR
        if (dubins_RSR(inter_result, &normalized_seg_lengths)) {
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::RSR;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;
                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'R', 'S', 'R'};
            }
        }

        // LSR
        if (dubins_LSR(inter_result, &normalized_seg_lengths)) {
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::LSR;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;

                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'L', 'S', 'R'};
            }
        }

        // RSL
        if (dubins_RSL(inter_result, &normalized_seg_lengths)) {
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::RSL;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;

                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'R', 'S', 'L'};
            }
        }

        // RLR
        if (dubins_RLR(inter_result, &normalized_seg_lengths)) {
            // construct path
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::RLR;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;

                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'R', 'L', 'R'};
            }
        }

        // LRL
        if (dubins_LRL(inter_result, &normalized_seg_lengths)) {
            // construct path
            path.param[0] = normalized_seg_lengths[0];
            path.param[1] = normalized_seg_lengths[1];
            path.param[2] = normalized_seg_lengths[2];
            path.type = DubinsPathType::LRL;
            double length = dubins_path_length(&path);
            if (length < (*optimal_path).total_length) {
                optimal_inter_path = path;

                (*optimal_path).total_length = length;
                (*optimal_path).segs_lengths = {
                        normalized_seg_lengths[0] * turing_radius_,
                        normalized_seg_lengths[1] * turing_radius_,
                        normalized_seg_lengths[2] * turing_radius_,
                };
                (*optimal_path).segs_types = {'L', 'R', 'L'};
            }
        }
        // if (optimal_path->total_length > 10e8) {
        //   return false;
        // }
        dubins_path_sample_many(&optimal_inter_path, step_size_, *optimal_path);
        *optimal_cost = optimal_path->total_length;
        return true;
}



double Dubin::ShortestPathLength(const std::shared_ptr<nodeTractor> start_node,
                                 const std::shared_ptr<nodeTractor> end_node) {
        start_x_ = start_node->GetX();
        start_y_ = start_node->GetY();
        start_phi_ = start_node->GetPhi();
        DubinsIntermediateResults inter_result =
                dubins_intermediate_results(start_node, end_node, turing_radius_);

        std::vector<double> normalized_seg_lengths(3, 0.0);
        double min_length = 10e9;
        double tmp_length = 0.0;
        // LSL
        if (dubins_LSL(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }

        // RSR
        if (dubins_RSR(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }

        // LSR
        if (dubins_LSR(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }

        // RSL
        if (dubins_RSL(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }

        // RLR
        if (dubins_RLR(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }

        // LRL
        if (dubins_LRL(inter_result, &normalized_seg_lengths)) {
            tmp_length = normalized_seg_lengths[0] + normalized_seg_lengths[1] +
                         normalized_seg_lengths[2];
            if (tmp_length < min_length) {
                min_length = tmp_length;
            }
        }
        return min_length * turing_radius_;
}



double Dubin::dubins_path_length(DubinsPath* path) {
        double length = 0.;
        length += path->param[0];
        length += path->param[1];
        length += path->param[2];
        length = length * path->rho;
        return length;
}


void Dubin::dubins_segment(double t,
                               double qi[3],
                               double qt[3],
                               SegmentType type) {
        double st = std::sin(qi[2]);
        double ct = std::cos(qi[2]);
        if (type == L_SEG) {
            qt[0] = +std::sin(qi[2] + t) - st;
            qt[1] = -std::cos(qi[2] + t) + ct;
            qt[2] = t;
        } else if (type == R_SEG) {
            qt[0] = -std::sin(qi[2] - t) + st;
            qt[1] = +std::cos(qi[2] - t) - ct;
            qt[2] = -t;
        } else if (type == S_SEG) {
            qt[0] = ct * t;
            qt[1] = st * t;
            qt[2] = 0.0;
        }
        qt[0] += qi[0];
        qt[1] += qi[1];
        qt[2] += qi[2];
}



void Dubin::dubins_path_sample(DubinsPath* path, double t, double q[3]) {
        /* tprime is the normalised variant of the parameter t */
        double tprime = t / path->rho;
        double qi[3]; /* The translated initial configuration */
        double q1[3]; /* end-of segment 1 */
        double q2[3]; /* end-of segment 2 */
        const SegmentType* types = DIRDATA[static_cast<int>(path->type)];
        double p1, p2;

        if (t < 0 || t > dubins_path_length(path)) {
//            SPDEBUG << "Dubins t out of range : " << t;
            return;
        }

        /* initial configuration */
        qi[0] = 0.0;
        qi[1] = 0.0;
        qi[2] = path->qi[2];

        /* generate the target configuration */
        p1 = path->param[0];
        p2 = path->param[1];
        dubins_segment(p1, qi, q1, types[0]);
        dubins_segment(p2, q1, q2, types[1]);
        if (tprime < p1) {
            dubins_segment(tprime, qi, q, types[0]);
        } else if (tprime < (p1 + p2)) {
            dubins_segment(tprime - p1, q1, q, types[1]);
        } else {
            dubins_segment(tprime - p1 - p2, q2, q, types[2]);
        }

        /* scale the target configuration, translate back to the original starting
         * point */
        q[0] = q[0] * path->rho + path->qi[0];
        q[1] = q[1] * path->rho + path->qi[1];
        q[2] = mod2pi(q[2]);
}

int Dubin::dubins_path_sample_many(DubinsPath* path,
                                       double stepSize,
                                       dubinPath& discret_path) {
        double q[3];
        double x = 0.0;
        double length = dubins_path_length(path);
        while (x < length) {
            dubins_path_sample(path, x, q);
            discret_path.x.push_back(q[0]);
            discret_path.y.push_back(q[1]);
            discret_path.phi.push_back(q[2]);
            discret_path.gear.push_back(true);
            x += stepSize;
        }
        return 0;
}

DubinsIntermediateResults Dubin::dubins_intermediate_results(
            const std::shared_ptr<nodeTractor> start_node,
            const std::shared_ptr<nodeTractor> end_node,
            double rho) {
    double dx, dy, D, d, theta, alpha, beta;
//        CHECK_GT(rho, 0);
    double start_x = start_node->GetX();
    double start_y = start_node->GetY();
    double start_phi = start_node->GetPhi();
    double end_x = end_node->GetX();
    double end_y = end_node->GetY();
    double end_phi = end_node->GetPhi();
    dx = end_x - start_x;
    dy = end_y - start_y;
    D = std::sqrt(dx * dx + dy * dy);
    d = D / rho;
    theta = 0;

    /* test required to prevent domain errors if dx=0 and dy=0 */
    if (d > 0) {
        theta = mod2pi(std::atan2(dy, dx));
    }
    alpha = mod2pi(start_phi - theta);
    beta = mod2pi(end_phi - theta);

    DubinsIntermediateResults in;
    in.alpha = alpha;
    in.beta = beta;
    in.d = d;
    in.sa = std::sin(alpha);
    in.sb = std::sin(beta);
    in.ca = std::cos(alpha);
    in.cb = std::cos(beta);
    in.c_ab = std::cos(alpha - beta);
    in.d_sq = d * d;

    return in;
}

bool Dubin::dubins_LSL(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double tmp0, tmp1, p_sq;
        tmp0 = in.d + in.sa - in.sb;
        p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sa - in.sb));
        if (p_sq >= 0) {
            tmp1 = std::atan2((in.cb - in.ca), tmp0);
            (*seg_lengths)[0] = mod2pi(tmp1 - in.alpha);
            (*seg_lengths)[1] = std::sqrt(p_sq);
            (*seg_lengths)[2] = mod2pi(in.beta - tmp1);
            return true;
        }
        return false;
}

bool Dubin::dubins_RSR(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double tmp0 = in.d - in.sa + in.sb;
        double p_sq = 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sb - in.sa));
        if (p_sq >= 0) {
            double tmp1 = std::atan2((in.ca - in.cb), tmp0);
            (*seg_lengths)[0] = mod2pi(in.alpha - tmp1);
            (*seg_lengths)[1] = std::sqrt(p_sq);
            (*seg_lengths)[2] = mod2pi(tmp1 - in.beta);
            return true;
        }
        return false;
}

bool Dubin::dubins_LSR(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double p_sq = -2 + (in.d_sq) + (2 * in.c_ab) + (2 * in.d * (in.sa + in.sb));
        if (p_sq >= 0) {
            double p = std::sqrt(p_sq);
            double tmp0 = std::atan2((-in.ca - in.cb), (in.d + in.sa + in.sb)) -
                          std::atan2(-2.0, p);
            (*seg_lengths)[0] = mod2pi(tmp0 - in.alpha);
            (*seg_lengths)[1] = p;
            (*seg_lengths)[2] = mod2pi(tmp0 - mod2pi(in.beta));
            return true;
        }
        return false;
}

bool Dubin::dubins_RSL(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double p_sq = -2 + in.d_sq + (2 * in.c_ab) - (2 * in.d * (in.sa + in.sb));
        if (p_sq >= 0) {
            double p = std::sqrt(p_sq);
            double tmp0 = std::atan2((in.ca + in.cb), (in.d - in.sa - in.sb)) -
                          std::atan2(2.0, p);
            (*seg_lengths)[0] = mod2pi(in.alpha - tmp0);
            (*seg_lengths)[1] = p;
            (*seg_lengths)[2] = mod2pi(in.beta - tmp0);
            return true;
        }
        return false;
}

bool Dubin::dubins_RLR(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double tmp0 = (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sa - in.sb)) / 8.;
        double phi = std::atan2(in.ca - in.cb, in.d - in.sa + in.sb);
        if (fabs(tmp0) <= 1) {
            double p = mod2pi((2 * M_PI) - acos(tmp0));
            double t = mod2pi(in.alpha - phi + mod2pi(p / 2.));
            (*seg_lengths)[0] = t;
            (*seg_lengths)[1] = p;
            (*seg_lengths)[2] = mod2pi(in.alpha - in.beta - t + mod2pi(p));
            return true;
        }
        return false;
}

bool Dubin::dubins_LRL(const DubinsIntermediateResults& in,
                           std::vector<double>* seg_lengths) {
        double tmp0 = (6. - in.d_sq + 2 * in.c_ab + 2 * in.d * (in.sb - in.sa)) / 8.;
        double phi = std::atan2(in.ca - in.cb, in.d + in.sa - in.sb);
        if (fabs(tmp0) <= 1) {
            double p = mod2pi(2 * M_PI - acos(tmp0));
            double t = mod2pi(-in.alpha - phi + p / 2.);
            (*seg_lengths)[0] = t;
            (*seg_lengths)[1] = p;
            (*seg_lengths)[2] = mod2pi(mod2pi(in.beta) - in.alpha - t + mod2pi(p));
            return true;
        }
        return false;
}


}





