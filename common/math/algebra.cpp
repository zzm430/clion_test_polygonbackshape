//
// Created by zzm on 23-12-4.
//

#include "algebra.h"

namespace math {
    double Algebra::WrapAngle(double angle) {
        double new_angle = std::fmod(angle, M_PI * 2.0);
        return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
    }

    double Algebra::CrossProd(const Vec2d& start_point,
                              const Vec2d& end_point_1,
                              const Vec2d& end_point_2) {
        return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
    }

    double Algebra::InnerProd(const Vec2d& start_point,
                              const Vec2d& end_point_1,
                              const Vec2d& end_point_2) {
        return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
    }

    double Algebra::CrossProd(double x0, double y0, double x1, double y1) {
        return x0 * y1 - x1 * y0;
    }

    double Algebra::InnerProd(double x0, double y0, double x1, double y1) {
        return x0 * x1 + y0 * y1;
    }

    double Algebra::distance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    double Algebra::distance(const math::Vec2d& a0, const math::Vec2d& a1) {
        return std::sqrt((a0.x() - a1.x()) * (a0.x() - a1.x()) +
                         (a0.y() - a1.y()) * (a0.y() - a1.y()));
    }

    double Algebra::normalizeAngle(double a) {
        double a_n = std::fmod(a, 2.0 * M_PI);
        if (a_n < 0.0) {
            a_n += (2.0 * M_PI);
        }
        return a_n;
    }

    double Algebra::normalizeAngleToSignField(double angle) {
        double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (a < 0.0) {
            a += (2.0 * M_PI);
        }
        return a - M_PI;
    }

    double Algebra::angleDiff(double from, double to) {
        return normalizeAngleToSignField(to - from);
    }

    double Algebra::differentAngle(double a0, double a1) {
        double d;
        auto a0_n = normalizeAngle(a0);
        auto a1_n = normalizeAngle(a1);
        if (a1_n > a0_n && a1_n - a0_n <= M_PI) {
            d = a1_n - a0_n;
        } else if (a1_n > a0_n && a1_n - a0_n > M_PI) {
            d = (a1_n - a0_n) - 2 * M_PI;
        } else if (a0_n > a1_n && a0_n - a1_n <= M_PI) {
            d = a1_n - a0_n;
        } else if (a0_n > a1_n && a0_n - a1_n > M_PI) {
            d = 2 * M_PI - (a0_n - a1_n);
        } else {
            d = std::abs(a0_n - a1_n);
        }
        return d;
    }

    double Algebra::lerp(double x0, double x1, double t) {
        double x = x0 + t * (x1 - x0);
        return x;
    }

    double Algebra::slerp(double a0, double a1, double t) {
        double a0_n = normalizeAngle(a0);
        double a1_n = normalizeAngle(a1);
        double d = differentAngle(a0_n, a1_n);

        double a = a0_n + d * t;
        return normalizeAngle(a);
    }

    double Algebra::slerp(double a0, double t0, double a1, double t1, double t) {
        if (std::abs(t1 - t0) <= 1e-3) {
            return normalizeAngle(a0);
        }
        const double a0_n = normalizeAngle(a0);
        const double a1_n = normalizeAngle(a1);
        double d = a1_n - a0_n;
        if (d > M_PI) {
            d = d - 2 * M_PI;
        } else if (d < -M_PI) {
            d = d + 2 * M_PI;
        }

        const double r = (t - t0) / (t1 - t0);
        const double a = a0_n + d * r;
        return normalizeAngle(a);
    }

    void Algebra::findCircleCenterPtWithThreePts(double x0,
                                                 double y0,
                                                 double x1,
                                                 double y1,
                                                 double x2,
                                                 double y2,
                                                 double& cx,
                                                 double& cy) {
        auto a0 = 2.0 * (x1 - x0);
        auto b0 = 2.0 * (y1 - y0);
        auto c0 = std::pow(x1, 2.0) + std::pow(y1, 2.0) - std::pow(x0, 2.0) -
                  std::pow(y0, 2.0);
        auto a1 = 2.0 * (x2 - x1);
        auto b1 = 2.0 * (y2 - y1);
        auto c1 = std::pow(x2, 2.0) + std::pow(y2, 2.0) - std::pow(x1, 2.0) -
                  std::pow(y1, 2.0);

        if (std::abs((a0 * b1) - (a1 * b0)) <= DBL_EPSILON) {
            cx = DBL_MAX;
            cy = DBL_MAX;
        } else {
            cx = ((c0 * b1) - (c1 * b0)) / ((a0 * b1) - (a1 * b0));
            cy = ((a0 * c1) - (a1 * c0)) / ((a0 * b1) - (a1 * b0));
        }
    }

    double Algebra::calAngleVectorToXAxel(double x, double y) {
        if (std::abs(y) <= DBL_EPSILON && std::abs(x) <= DBL_EPSILON) {
            return 0.0;
        }
        double angle;
        if (std::abs(x) <= DBL_EPSILON) {
            if (y > 0.0) {
                angle = M_PI_2;
            } else {
                angle = M_PI_2 * 3.0;
            }
        } else if (x > 0.0) {
            angle = std::atan(y / x);
        } else {
            angle = M_PI + std::atan(y / x);
        }
        return normalizeAngle(angle);
    }

    double Algebra::projectPtToLine(const polygonPoint& pt,
                                    const polygonPoint& line) {
        return pt.x * line.x + pt.y * line.y;
    }

    void Algebra::projectContourToLine(
            const std::vector<polygonPoint>& contour,
            const polygonPoint& line,
            double& left,
            double& right) {
        left = DBL_MAX;
        right = -DBL_MAX;
        for (auto& pt : contour) {
            double pt_project = projectPtToLine(pt, line);
            if (pt_project <= left) {
                left = pt_project;
            }

            if (pt_project >= right) {
                right = pt_project;
            }
        }
    }

    void Algebra::projectContourToLine(
            const std::vector<polygonPoint>& contour,
            const polygonPoint& line,
            double& left_dis,
            double& right_dis,
            polygonPoint& left_pt,
            polygonPoint& right_pt) {
        left_dis = DBL_MAX;
        right_dis = -DBL_MAX;
        left_pt = polygonPoint(DBL_MAX, DBL_MAX);
        right_pt = polygonPoint(DBL_MAX, DBL_MAX);
        for (auto& pt : contour) {
            double pt_project = projectPtToLine(pt, line);
            if (pt_project <= left_dis) {
                left_dis = pt_project;
                left_pt = pt;
            }

            if (pt_project >= right_dis) {
                right_dis = pt_project;
                right_pt = pt;
            }
        }
    }

    void Algebra::projectContourToLine(
            const std::vector<polygonPoint>& contour,
            const polygonPoint& line,
            double& near_dis,
            double& far_dis,
            size_t& near_idx,
            size_t& far_idx) {
        near_dis = DBL_MAX;
        far_dis = -DBL_MAX;
        near_idx = SIZE_MAX;
        far_idx = SIZE_MAX;
        for (size_t pt_idx = 0; pt_idx < contour.size(); ++pt_idx) {
            auto& pt = contour.at(pt_idx);
            double pt_project = projectPtToLine(pt, line);
            if (pt_project <= near_dis) {
                near_dis = pt_project;
                near_idx = pt_idx;
            }

            if (pt_project >= far_dis) {
                far_dis = pt_project;
                far_idx = pt_idx;
            }
        }
    }

    bool Algebra::onSegment(const polygonPoint& p,
                            const polygonPoint& q,
                            const polygonPoint& r) {
        return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
    }

    double Algebra::modVector(const polygonPoint& vec) {
        return std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
    }

    polygonPoint Algebra::normalizeVector(const polygonPoint& vec) {
        auto len = modVector(vec);
//        LOG_ASSERT(len > DBL_EPSILON);
        return polygonPoint(vec.x / len, vec.y / len);
    }

    polygonPoint Algebra::vertNormVector(const polygonPoint& vec) {
        polygonPoint line(vec.y, -vec.x);
        return normalizeVector(line);
    }

    polygonPoint Algebra::footOfPtToSegment(
            const polygonPoint& pt,
            const polygonPoint& line,
            const polygonPoint& seg_spt) {
        if (modVector(line) <= DBL_EPSILON) return seg_spt;
        polygonPoint pt_vec(pt.x - seg_spt.x, pt.y - seg_spt.y);
        if (modVector(pt_vec) <= DBL_EPSILON) return seg_spt;

        auto seg_normalize = normalizeVector(line);
        auto dot_value = line.x * pt_vec.x + line.y * pt_vec.y;
        auto foot_vec_norm = dot_value / modVector(line);
        polygonPoint foot_vec{foot_vec_norm * seg_normalize.x,
                                     foot_vec_norm * seg_normalize.y};
        return polygonPoint(seg_spt.x + foot_vec.x, seg_spt.y + foot_vec.y);
    }

    int Algebra::orientation(const polygonPoint& p,
                             const polygonPoint& q,
                             const polygonPoint& r) {
        if (distance(p.x, p.y, q.x, q.y) < DBL_EPSILON) return 0;
        if (distance(p.x, p.y, r.x, r.y) < DBL_EPSILON) return 0;
        if (distance(q.x, q.y, r.x, r.y) < DBL_EPSILON) return 0;
        auto val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (std::abs(val) <= DBL_EPSILON) return 0;  // co-linear
        return (val > 0.0) ? 1 : 2;                  // clockwise or counterclockwise
    }

    bool Algebra::isTwoContourCrash(
            const std::vector<polygonPoint>& contour1,
            const std::vector<polygonPoint>& contour2) {
        if (contour1.size() < 3 || contour2.size() < 3) {
            return false;
        }

        bool is_crash = true;

        for (size_t idx = 0; idx < contour1.size() - 1; ++idx) {
            polygonPoint line(contour1[idx + 1].x - contour1[idx].x,
                                     contour1[idx + 1].y - contour1[idx].y);
            polygonPoint normal_line = vertNormVector(line);

            double seg_left_1, seg_right_1, seg_left_2, seg_right_2;
            projectContourToLine(contour1, normal_line, seg_left_1, seg_right_1);
            projectContourToLine(contour2, normal_line, seg_left_2, seg_right_2);

            if (seg_left_1 > seg_right_2 || seg_left_2 > seg_right_1) {
                is_crash = false;
                break;
            } else {
                continue;
            }
        }

        if (!is_crash) {
            return false;
        }

        for (size_t idx = 0; idx < contour2.size() - 1; ++idx) {
            polygonPoint line(contour2[idx + 1].x - contour2[idx].x,
                                     contour2[idx + 1].y - contour2[idx].y);
            polygonPoint normal_line = vertNormVector(line);

            double seg_left_1, seg_right_1, seg_left_2, seg_right_2;
            projectContourToLine(contour1, normal_line, seg_left_1, seg_right_1);
            projectContourToLine(contour2, normal_line, seg_left_2, seg_right_2);

            if (seg_left_1 > seg_right_2 || seg_left_2 > seg_right_1) {
                is_crash = false;
                break;
            } else {
                continue;
            }
        }

        return is_crash;
    }

    bool Algebra::isContourConvex(const std::vector<polygonPoint>& contour) {
        if (contour.size() < 3) return false;

        int orient = 0;
        for (size_t pt_idx = 0; pt_idx < contour.size(); ++pt_idx) {
            auto& p = contour.at(pt_idx);

            auto q_idx = pt_idx + 1;
            q_idx = q_idx > contour.size() - 1 ? q_idx - contour.size() : q_idx;
            auto& q = contour.at(q_idx);

            auto r_idx = pt_idx + 2;
            r_idx = r_idx > contour.size() - 1 ? r_idx - contour.size() : r_idx;
            auto& r = contour.at(r_idx);

            int o = orientation(p, q, r);

            if (o == 0) continue;
            if (orient == 0) orient = o;
            if (orient != o) return false;
        }
        return true;
    }

    double Algebra::calMinDisPtToSegment(const polygonPoint& pt,
                                         const polygonPoint& seg_spt,
                                         const polygonPoint& seg_ept) {
        polygonPoint segment(seg_ept.x - seg_spt.x, seg_ept.y - seg_spt.y);
        auto seq_len = modVector(segment);
        polygonPoint pt_vec(pt.x - seg_spt.x, pt.y - seg_spt.y);
        auto pt_vec_len = modVector(pt_vec);
        if (seq_len <= DBL_EPSILON && pt_vec_len <= DBL_EPSILON) return 0.0;
        auto foot = footOfPtToSegment(pt, segment, seg_spt);
        if (onSegment(seg_spt, foot, seg_ept)) {
            return distance(pt.x, pt.y, foot.x, foot.y);
        } else {
            return std::min(distance(pt.x, pt.y, seg_spt.x, seg_spt.y),
                            distance(pt.x, pt.y, seg_ept.x, seg_ept.y));
        }
    }

    double Algebra::calMinDisPtToContour(
            const polygonPoint& pt,
            const std::vector<polygonPoint>& contour) {
        if (contour.empty()) return DBL_MAX;
        if (contour.size() == 1)
            return distance(pt.x, pt.y, contour[0].x, contour[0].y);
        if (contour.size() == 2)
            return calMinDisPtToSegment(pt, contour[0], contour[1]);
        auto min_dis_pt_idx = SIZE_MAX;
        auto min_dis = DBL_MAX;
        for (size_t pt_idx = 0; pt_idx < contour.size(); ++pt_idx) {
            auto rt_dis =
                    distance(pt.x, pt.y, contour[pt_idx].x, contour[pt_idx].y);
            if (rt_dis < min_dis) {
                min_dis = rt_dis;
                min_dis_pt_idx = pt_idx;
            }
        }
        size_t p_idx, q_idx, r_idx;
        if (min_dis_pt_idx == 0) {
            p_idx = contour.size() - 1;
            q_idx = 0;
            r_idx = 1;
        } else if (min_dis_pt_idx == contour.size() - 1) {
            p_idx = contour.size() - 2;
            q_idx = contour.size() - 1;
            r_idx = 0;
        } else {
            p_idx = min_dis_pt_idx - 1;
            q_idx = min_dis_pt_idx;
            r_idx = min_dis_pt_idx + 1;
        }

        return std::min(calMinDisPtToSegment(pt, contour[p_idx], contour[q_idx]),
                        calMinDisPtToSegment(pt, contour[q_idx], contour[r_idx]));
    }

    double Algebra::calMinDisOfTwoContour(
            const std::vector<polygonPoint>& contour1,
            const std::vector<polygonPoint>& contour2) {
        if (contour1.size() < 3 || contour2.size() < 3) return DBL_MAX;
        if (!isContourConvex(contour1) || !isContourConvex(contour2)) return DBL_MAX;
        if (isTwoContourCrash(contour1, contour2)) return -1;

        auto min_dis = DBL_MAX;
        for (auto& pt : contour1) {
            auto rt_dis = calMinDisPtToContour(pt, contour2);
            if (rt_dis < min_dis) min_dis = rt_dis;
        }

        for (auto& pt : contour2) {
            auto rt_dis = calMinDisPtToContour(pt, contour1);
            if (rt_dis < min_dis) min_dis = rt_dis;
        }

        return min_dis;
    }

}  // namespace math