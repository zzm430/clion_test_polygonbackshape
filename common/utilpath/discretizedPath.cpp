//
// Created by zzm on 23-12-5.
//

#include "common/utilpath/discretizedPath.h"

PathPoint::PathPoint()
        : x_(0.0)
        , y_(0.0)
        , z_(0.0)
        , s_(0.0)
        , theta_(0.0)
        , heading_(0.0)
        , kappa_(0.0)
        , dkappa_(0.0)
        , ddkappa_(0.0) {}

PathPoint::PathPoint(double x,
                     double y,
                     double z,
                     double s,
                     double theta,
                     double heading,
                     double kappa,
                     double dkappa,
                     double ddkappa)
        : x_(x)
        , y_(y)
        , z_(z)
        , s_(s)
        , theta_(theta)
        , heading_(heading)
        , kappa_(kappa)
        , dkappa_(dkappa)
        , ddkappa_(ddkappa) {}

PathPoint::PathPoint( polygonPoint& traj_pt)
        : x_(traj_pt.x)
        , y_(traj_pt.y)
        , z_(traj_pt.z)
        , s_(traj_pt.s())
        , theta_(traj_pt.theta())
        , heading_(traj_pt.theta())
        , kappa_(traj_pt.kappa())
        , dkappa_(traj_pt.dkappa())
        , ddkappa_(traj_pt.ddkappa()) {}

bool PathPoint::operator==(const PathPoint& other) const {
    if (x_ != other.x()) return false;
    if (y_ != other.y()) return false;
    if (z_ != other.z()) return false;
    if (s_ != other.s()) return false;
    if (theta_ != other.theta()) return false;
    if (heading_ != other.heading()) return false;
    if (kappa_ != other.kappa()) return false;
    if (dkappa_ != other.dkappa()) return false;
    return ddkappa_ == other.ddkappa();
}

void PathPoint::TransVehToUtm(double vehicle_x,
                              double vehicle_y,
                              double vehicle_yaw) {
    double x_offset = sin(vehicle_yaw) * x_ + cos(vehicle_yaw) * y_;
    double y_offset = -cos(vehicle_yaw) * x_ + sin(vehicle_yaw) * y_;
    x_ = vehicle_x + x_offset;
    y_ = vehicle_y + y_offset;
    theta_ = math::Algebra::normalizeAngle(theta_ + vehicle_yaw - M_PI_2);
    heading_ = math::Algebra::normalizeAngle(heading_ + vehicle_yaw - M_PI_2);
}

void PathPoint::TransUtmToVeh(double vehicle_x,
                              double vehicle_y,
                              double vehicle_yaw) {
    auto x_offset = x_ - vehicle_x;
    auto y_offset = y_ - vehicle_y;
    x_ = sin(vehicle_yaw) * x_offset - cos(vehicle_yaw) * y_offset;
    y_ = cos(vehicle_yaw) * x_offset + sin(vehicle_yaw) * y_offset;
    theta_ = math::Algebra::normalizeAngle(theta_ - vehicle_yaw + M_PI_2);
    heading_ = math::Algebra::normalizeAngle(heading_ - vehicle_yaw + M_PI_2);
}

void PathPoint::TransBetweenCoord(double x, double y, double theta) {
    //  data::Point::TransBetweenCoord(x, y, theta);
    x_ += x;
    y_ += y;
    theta_ = math::Algebra::normalizeAngle(theta_ + theta);
    heading_ = math::Algebra::normalizeAngle(heading_ + theta);
}

double PathPoint::distance(double x, double y) {
    return math::Algebra::distance(x_, y_, x, y);
}

PathPoint PathPoint::FindProjectionPt(const PathPoint& p0,
                                            const PathPoint& p1,
                                            double x,
                                            double y) {
    double v0x = x - p0.x_;
    double v0y = y - p0.y_;
    double v1x = p1.x_ - p0.x_;
    double v1y = p1.y_ - p0.y_;
    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;
    double delta_s = dot / v1_norm / v1_norm;
    return InterpPtByLinearS(p0, p1, delta_s);
}

PathPoint PathPoint::InterpPtByLinearS(const PathPoint& p0,
                                             const PathPoint& p1,
                                             double s) {
    PathPoint pt;
    pt.x_ = math::Algebra::lerp(p0.x_, p1.x_, s);
    pt.y_ = math::Algebra::lerp(p0.y_, p1.y_, s);
    pt.z_ = math::Algebra::lerp(p0.z_, p1.z_, s);
    pt.set_theta(math::Algebra::slerp(p0.theta(), p1.theta(), s));
    pt.set_heading(math::Algebra::slerp(p0.heading(), p1.heading(), s));
    pt.set_kappa(math::Algebra::lerp(p0.kappa(), p1.kappa(), s));
    pt.set_dkappa(math::Algebra::lerp(p0.dkappa(), p1.dkappa(), s));
    pt.set_ddkappa(math::Algebra::lerp(p0.ddkappa(), p1.ddkappa(), s));
    pt.set_s(math::Algebra::lerp(p0.s(), p1.s(), s));
    return pt;
}


//将大地转为sl参考坐标点
void DiscretizedPath::FromRawPts(
        const std::vector<polygonPoint>& raw_pts) {
    this->resize(raw_pts.size());
    for (size_t pt_idx = 0; pt_idx < raw_pts.size(); ++pt_idx) {
        auto& raw_pt = raw_pts.at(pt_idx);
        auto& private_pt = this->at(pt_idx);
        private_pt.x_ = raw_pt.x;
        private_pt.y_ = raw_pt.y;
        private_pt.z_ = raw_pt.z;
        if (pt_idx == 0) {
            private_pt.set_s(0.0);
        } else {
            auto& prev_x = this->at(pt_idx - 1).x_;
            auto& prev_y = this->at(pt_idx - 1).y_;
            auto prev_s = this->at(pt_idx - 1).s();
            private_pt.set_s(prev_s +
                             math::Algebra::distance(
                                     prev_x, prev_y, private_pt.x_, private_pt.y_));
        }
        private_pt.set_theta(math::Algebra::normalizeAngle(raw_pt.heading()));
        private_pt.set_heading(math::Algebra::normalizeAngle(raw_pt.heading()));
        private_pt.set_kappa(raw_pt.kappa());
    }
}



//void DiscretizedPath::FromPosPoints(
//        const std::vector<hd_map::common::PosPoint>& raw_points,
//        const std::vector<double>& headings) {
//    this->resize(raw_points.size());
//    hd_map::common::PointENU prev_point = raw_points.front().point;
//    double acc_s = 0.0;
//    for (std::size_t index = 0; index < raw_points.size(); index++) {
//        auto& raw_point = raw_points.at(index);
//        auto& private_point = this->at(index);
//        private_point.x_ = raw_point.point.x;
//        private_point.y_ = raw_point.point.y;
//        private_point.z_ = raw_point.point.z;
//        private_point.set_theta(math::Algebra::normalizeAngle(headings.at(index)));
//        private_point.set_heading(
//                math::Algebra::normalizeAngle(headings.at(index)));
//        private_point.set_kappa(0.0);
//        private_point.set_s(acc_s + math::Algebra::distance(prev_point.x,
//                                                            prev_point.y,
//                                                            private_point.x_,
//                                                            private_point.y_));
//        acc_s = private_point.s();
//        prev_point.x = private_point.x_;
//        prev_point.y = private_point.y_;
//    }
//}


bool DiscretizedPath::SLToXY(double s, double l, polygonPoint& xy_point) {
    const auto ref_pt = GetPathPtFromS(s);
    const double angle = ref_pt.theta();
    xy_point.x = ref_pt.x() - std::sin(angle) * l;
    xy_point.y = ref_pt.y() + std::cos(angle) * l;
    return true;
}


//data::Point2D DiscretizedPath::SLToXY(double s,
//                                      double l,
//                                      data::PathPoint& ref_pt) {
//    data::Point2D xy_point;
//    const double angle = ref_pt.theta();
//    xy_point.x_ = ref_pt.x_ - std::sin(angle) * l;
//    xy_point.y_ = ref_pt.y_ + std::cos(angle) * l;
//    return xy_point;
//}


PathPoint DiscretizedPath::GetPathPtFromS(double s) const {
//    LOG_ASSERT(s >= front().s() - 1e-3 && s <= back().s() + 1e-3);
    auto comp = [](const PathPoint& pt, double s) { return pt.s() < s; };
    auto it_lower = std::lower_bound(begin(), end(), s, comp);
    if (it_lower == begin()) {
        return front();
    } else if (it_lower == end()) {
        return back();
    } else {
        auto l = it_lower - 1;
        auto u = it_lower;
        auto r = (s - l->s()) / (u->s() - l->s());
        return PathPoint::InterpPtByLinearS(*l, *u, r);
    }
}

//size_t DiscretizedPath::GetPathPtIdxFromS(double s) {
//    LOG_ASSERT(s >= front().s() - 1e-3 && s <= back().s() + 1e-3);
//    auto comp = [](const PathPoint& pt, double s) { return pt.s() < s; };
//    auto it_lower = std::lower_bound(begin(), end(), s, comp);
//    if (it_lower == begin()) {
//        return 0;
//    } else if (it_lower == end()) {
//        return size() - 1;
//    } else {
//        auto back_s = std::fabs((it_lower - 1)->s() - s);
//        auto front_s = std::fabs(it_lower->s() - s);
//
//        return back_s > front_s ? std::distance(begin(), it_lower)
//                                : std::distance(begin(), it_lower) - 1;
//    }
//}
//
//PathPoint DiscretizedPath::GetPathPtFromS(double s, size_t& idx) {
//    LOG_ASSERT(s >= front().s() - 1e-3 && s <= back().s() + 1e-3);
//    auto comp = [](const PathPoint& pt, double s) { return pt.s() < s; };
//    auto it_lower = std::lower_bound(begin(), end(), s, comp);
//    if (it_lower == begin()) {
//        idx = 0;
//        return front();
//    } else if (it_lower == end()) {
//        idx = size() - 1;
//        return back();
//    } else {
//        auto l = it_lower - 1;
//        auto u = it_lower;
//        auto r = (s - l->s()) / (u->s() - l->s());
//        idx = std::distance(begin(), u) -
//              (std::fabs(l->s() - s) > std::fabs(u->s() - s) ? 0 : 1);
//        return PathPoint::InterpPtByLinearS(*l, *u, r);
//    }
//}
//
//void DiscretizedPath::TransVehToUtm(double vehicle_x,
//                                    double vehicle_y,
//                                    double vehicle_yaw) {
//    for (auto& pt : *this) {
//        pt.TransVehToUtm(vehicle_x, vehicle_y, vehicle_yaw);
//    }
//}
//
//void DiscretizedPath::TransUtmToVeh(double vehicle_x,
//                                    double vehicle_y,
//                                    double vehicle_yaw) {
//    for (auto& pt : *this) {
//        pt.TransUtmToVeh(vehicle_x, vehicle_y, vehicle_yaw);
//    }
//}
//
//void DiscretizedPath::TransBetweenCoord(double x,
//                                        double y,
//                                        double theta_offset) {
//    for (auto& pt : *this) {
//        pt.TransBetweenCoord(x, y, theta_offset);
//    }
//}
//
//
//DiscretizedPath::DiscretizedPath(const std::vector<PathPoint>& path_points)
//        : math::DiscreteMatchHelper<PathPoint>(path_points) {}

double DiscretizedPath::Length() const {
    if (empty()) {
        return 0.0;
    }
    return back().s() - front().s();
}

//PathPoint DiscretizedPath::Evaluate(double path_s) const {
//    CHECK(!empty());
//    auto it_lower = QueryLowerBound(path_s);
//    if (it_lower == begin()) {
//        return front();
//    }
//    if (it_lower == end()) {
//        return back();
//    }
//    return math::InterpolateUsingLinearApproximation(
//            *(it_lower - 1), *it_lower, path_s);
//}
//
//std::vector<PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
//        double path_s) const {
//    auto func = [](const PathPoint& tp, double path_s) {
//        return tp.s() < path_s;
//    };
//    return std::lower_bound(begin(), end(), path_s, func);
//}
//
//PathPoint DiscretizedPath::EvaluateReverse(double path_s) const {
//    CHECK(!empty());
//    auto it_upper = QueryUpperBound(path_s);
//    if (it_upper == begin()) {
//        return front();
//    }
//    if (it_upper == end()) {
//        return back();
//    }
//    return math::InterpolateUsingLinearApproximation(
//            *(it_upper - 1), *it_upper, path_s);
//}
//
//std::vector<PathPoint>::const_iterator DiscretizedPath::QueryUpperBound(
//        double path_s) const {
//    auto func = [](double path_s, const PathPoint& tp) {
//        return tp.s() < path_s;
//    };
//    return std::upper_bound(begin(), end(), path_s, func);
//}
//
//bool DiscretizedPath::MatchEgo(double x, double y, double theta) {
//    ego_match_ = Match(x, y, theta);
//    return ego_match_.IsAvailable();
//}
//
//bool DiscretizedPath::IsAvailable() const {
//    return !empty();
//}
//
//
//bool DiscretizedPath::GetSLBoundary(const math::Box2d& box,
//                                    SLBoundary* sl_boundary) const {
//    if (box.GetAllCorners().empty()) return false;
//
//    return GetSLBoundary(box.GetAllCorners(), sl_boundary);
//}
//
//
//bool DiscretizedPath::GetMutableSLBoundary(math::Box2d& box,
//                                           data::SLBoundary* sl_boundary) {
//    if (box.GetAllCorners().empty()) return false;
//
//    return GetSLBoundary(box.GetAllCorners(), sl_boundary);
//}
//
//bool DiscretizedPath::GetSLBoundary(const math::Polygon2d& polygon,
//                                    SLBoundary* sl_boundary) const {
//    if (polygon.GetAllVertices().empty()) return false;
//
//    return GetSLBoundary(polygon.GetAllVertices(), sl_boundary);
//}
//bool DiscretizedPath::GetSLBoundary(const std::vector<math::Vec2d>& vec_s,
//                                    SLBoundary* sl_boundary) const {
//    std::vector<Point2D> pts;
//    pts.reserve(vec_s.size());
//    for (auto& vec : vec_s) {
//        pts.emplace_back(vec.x(), vec.y());
//    }
//    return GetSLBoundary(pts, sl_boundary);
//}
//bool DiscretizedPath::GetSLBoundary(const std::vector<Point2D>& pts,
//                                    SLBoundary* sl_boundary) const {
//    double start_s(std::numeric_limits<double>::max());
//    double end_s(std::numeric_limits<double>::lowest());
//    double start_l(std::numeric_limits<double>::max());
//    double end_l(std::numeric_limits<double>::lowest());
//    for (const auto& point : pts) {
//        SLPoint sl_point;
//        if (!XYToSL(point, &sl_point)) return false;
//
//        start_s = std::fmin(start_s, sl_point.s());
//        end_s = std::fmax(end_s, sl_point.s());
//        start_l = std::fmin(start_l, sl_point.l());
//        end_l = std::fmax(end_l, sl_point.l());
//    }
//
//    sl_boundary->set_start_s(start_s);
//    sl_boundary->set_end_s(end_s);
//    sl_boundary->set_start_l(start_l);
//    sl_boundary->set_end_l(end_l);
//    return true;
//}

bool DiscretizedPath::GetSlBoundary(const std::vector<polygonPoint>& polygon,
                   SLBoundary* slBoundary) const{
    double start_s(std::numeric_limits<double>::max());
    double end_s(std::numeric_limits<double>::lowest());
    double start_l(std::numeric_limits<double>::max());
    double end_l(std::numeric_limits<double>::lowest());

    std::vector<slPoint>  temp_slPts;
    for (const auto& point : polygon) {
        slPoint sl_point;
        if (!XYToSL(math::Vec2d(point.x,point.y), &sl_point)) return false;

        temp_slPts.push_back(sl_point);
        start_s = std::fmin(start_s, sl_point.s());
        end_s = std::fmax(end_s, sl_point.s());
        start_l = std::fmin(start_l, sl_point.l());
        end_l = std::fmax(end_l, sl_point.l());
    }

    slBoundary->set_start_s(start_s);
    slBoundary->set_end_s(end_s);
    slBoundary->set_start_l(start_l);
    slBoundary->set_end_l(end_l);
    slBoundary->set_boundary_pts(temp_slPts);

    return true;
}



bool DiscretizedPath::XYToSL(const math::Vec2d& xy_point, slPoint* sl_point) const {
    double s = 0.0, l = 0.0;
    auto ok = GetProjection(xy_point, &s, &l);
    if (ok) {
        sl_point->set_l(l);
        sl_point->set_s(s);
    }
    return ok;
}


bool DiscretizedPath::GetProjection(const math::Vec2d& point,
                                    double* accumulate_s,
                                    double* lateral) const {
    double min_distance = 0.0;
    int min_index = 0;
    return GetProjection(math::Vec2d(point.x(), point.y()),
                         accumulate_s,
                         lateral,
                         &min_distance,
                         &min_index);
}


//计算一个点距离一些路径点组成的线段集最近的距离带方向，左正右负
//用于计算s的值和l的值
bool DiscretizedPath::GetProjection(const math::Vec2d& point,
                                    double* accumulate_s,
                                    double* lateral,
                                    double* min_distance,
                                    int* min_index) const {
    if (empty()) return false;
    if (accumulate_s == nullptr || lateral == nullptr ||
        min_distance == nullptr || min_index == nullptr) {
        return false;
    }

   // CHECK(size() >= 2);

    *min_distance = std::numeric_limits<double>::infinity();
    std::vector<math::LineSegment2d> segments;
    for (int i = 0; i + 1 < size(); ++i) {
        math::LineSegment2d line_segment(math::Vec2d(at(i).x(), at(i).y()),
        math::Vec2d(at(i + 1).x(), at(i + 1).y()));
        segments.emplace_back(line_segment);
        double distance = line_segment.DistanceSquareTo(point);
        if (distance < *min_distance) {
            *min_index = i;
            *min_distance = distance;
        }
    }

    *min_distance = std::sqrt(*min_distance);
    const auto& nearest_seg = segments[*min_index];
    auto prod = nearest_seg.ProductOntoUnit(point);
    auto proj = nearest_seg.ProjectOntoUnit(point);

    if (*min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (*min_index == size() - 2) {
        *accumulate_s = at(*min_index).s() + std::max(0.0, proj);
        if (proj > 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = at(*min_index).s() +
                        std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;
}