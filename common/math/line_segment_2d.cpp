//
// Created by zzm on 23-12-5.
//

#include "common/math/line_segment_2d.h"


namespace math{

        LineSegment2d::LineSegment2d(const Vec2d &start, const Vec2d &end)
                : start_(start), end_(end) {
            const double dx = end_.x() - start_.x();
            const double dy = end_.y() - start_.y();
            length_ = std::hypot(dx, dy);
            unit_direction_ =
                    (length_ <= kMathEpsilons ? Vec2d(0, 0)
                                              : Vec2d(dx / length_, dy / length_));
            heading_ = unit_direction_.Angle();
        }

        LineSegment2d &LineSegment2d::Extend(double length) {
            if (length > 0.0) {
                end_.set_x(end_.x() + length * std::cos(heading()));
                end_.set_y(end_.y() + length * std::sin(heading()));
            } else {
                start_.set_x(start_.x() + length * std::cos(heading()));
                start_.set_y(start_.y() + length * std::sin(heading()));
            }
            const double dx = end_.x() - start_.x();
            const double dy = end_.y() - start_.y();
            length_ = std::hypot(dx, dy);
            return *this;
        }

        const Vec2d &LineSegment2d::start() const {
            return start_;
        }

        const Vec2d &LineSegment2d::end() const {
            return end_;
        }

        const Vec2d &LineSegment2d::unit_direction() const {
            return unit_direction_;
        }

        Vec2d LineSegment2d::center() const {
            return (start_ + end_) / 2.0;
        }

        Vec2d LineSegment2d::rotate(const double angle) {
            Vec2d diff_vec = end_ - start_;
            diff_vec.SelfRotate(angle);
            return start_ + diff_vec;
        }

        double LineSegment2d::heading() const {
            return heading_;
        }

        double LineSegment2d::cos_heading() const {
            return unit_direction_.x();
        }

        double LineSegment2d::sin_heading() const {
            return unit_direction_.y();
        }

        double LineSegment2d::length() const {
            return length_;
        }

        double LineSegment2d::length_sqr() const {
            return length_ * length_;
        }

        double LineSegment2d::DistanceTo(const Vec2d &point) const {
            if (length_ <= kMathEpsilons) {
                return point.DistanceTo(start_);
            }
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
            if (proj <= 0.0) {
                return hypot(x0, y0);
            }
            if (proj >= length_) {
                return point.DistanceTo(end_);
            }
            return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
        }

        double LineSegment2d::StraightSignedDistanceTo(const Vec2d &point) const {
//            LOG_ASSERT(length_ > kMathEpsilons);
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            return x0 * unit_direction_.y() - y0 * unit_direction_.x();
        }

        double LineSegment2d::StraightAbsDistanceTo(const Vec2d &point) const {
            return std::abs(StraightSignedDistanceTo(point));
        }

        double LineSegment2d::DistanceTo(const Vec2d &point,
                                         Vec2d *const nearest_pt) const {
//            CHECK_NOTNULL(nearest_pt);
            if (length_ <= kMathEpsilons) {
                *nearest_pt = start_;
                return point.DistanceTo(start_);
            }
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
            if (proj < 0.0) {
                *nearest_pt = start_;
                return hypot(x0, y0);
            }
            if (proj > length_) {
                *nearest_pt = end_;
                return point.DistanceTo(end_);
            }
            *nearest_pt = start_ + unit_direction_ * proj;
            return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
        }

        double LineSegment2d::DistanceSquareTo(const Vec2d &point) const {
            if (length_ <= kMathEpsilons) {
                return point.DistanceSquareTo(start_);
            }
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
            if (proj <= 0.0) {
                return Algebra::Square(x0) + Algebra::Square(y0);
            }
            if (proj >= length_) {
                return point.DistanceSquareTo(end_);
            }
            return Algebra::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
        }

        double LineSegment2d::DistanceSquareTo(const Vec2d &point,
                                               Vec2d *const nearest_pt) const {
//            CHECK_NOTNULL(nearest_pt);
            if (length_ <= kMathEpsilons) {
                *nearest_pt = start_;
                return point.DistanceSquareTo(start_);
            }
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
            if (proj <= 0.0) {
                *nearest_pt = start_;
                return Algebra::Square(x0) + Algebra::Square(y0);
            }
            if (proj >= length_) {
                *nearest_pt = end_;
                return point.DistanceSquareTo(end_);
            }
            *nearest_pt = start_ + unit_direction_ * proj;
            return Algebra::Square(x0 * unit_direction_.y() - y0 * unit_direction_.x());
        }

        bool LineSegment2d::IsPointIn(const Vec2d &point) const {
            if (length_ <= kMathEpsilons) {
                return std::abs(point.x() - start_.x()) <= kMathEpsilons &&
                       std::abs(point.y() - start_.y()) <= kMathEpsilons;
            }
            const double prod = Algebra::CrossProd(point, start_, end_);
            if (std::abs(prod) > kMathEpsilons) {
                return false;
            }
            return IsWithin(point.x(), start_.x(), end_.x()) &&
                   IsWithin(point.y(), start_.y(), end_.y());
        }

        double LineSegment2d::ProjectOntoUnit(const Vec2d &point) const {
            return unit_direction_.InnerProd(point - start_);
        }

        double LineSegment2d::ProductOntoUnit(const Vec2d &point) const {
            return unit_direction_.CrossProd(point - start_);
        }

        bool LineSegment2d::HasIntersect(const LineSegment2d &other_segment) const {
            Vec2d point;
            return GetIntersect(other_segment, &point);
        }

        bool LineSegment2d::GetIntersect(const LineSegment2d &other_segment,
                                         Vec2d *const point) const {
//            CHECK_NOTNULL(point);
            if (IsPointIn(other_segment.start())) {
                *point = other_segment.start();
                return true;
            }
            if (IsPointIn(other_segment.end())) {
                *point = other_segment.end();
                return true;
            }
            if (other_segment.IsPointIn(start_)) {
                *point = start_;
                return true;
            }
            if (other_segment.IsPointIn(end_)) {
                *point = end_;
                return true;
            }
            if (length_ <= kMathEpsilons || other_segment.length() <= kMathEpsilons) {
                return false;
            }
            const double cc1 = Algebra::CrossProd(start_, end_, other_segment.start());
            const double cc2 = Algebra::CrossProd(start_, end_, other_segment.end());
            if (cc1 * cc2 >= -kMathEpsilons) {
                return false;
            }
            const double cc3 =
                    Algebra::CrossProd(other_segment.start(), other_segment.end(), start_);
            const double cc4 =
                    Algebra::CrossProd(other_segment.start(), other_segment.end(), end_);
            if (cc3 * cc4 >= -kMathEpsilons) {
                return false;
            }
            const double ratio = cc4 / (cc4 - cc3);
            *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                           start_.y() * ratio + end_.y() * (1.0 - ratio));
            return true;
        }

// return distance with perpendicular foot point.
        double LineSegment2d::GetPerpendicularFoot(const Vec2d &point,
                                                   Vec2d *const foot_point) const {
//            CHECK_NOTNULL(foot_point);
            if (length_ <= kMathEpsilons) {
                *foot_point = start_;
                return point.DistanceTo(start_);
            }
            const double x0 = point.x() - start_.x();
            const double y0 = point.y() - start_.y();
            const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
            *foot_point = start_ + unit_direction_ * proj;
            return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
        }

        bool LineSegment2d::IsWithin(double val, double bound1, double bound2) const {
            if (bound1 > bound2) {
                std::swap(bound1, bound2);
            }
            return val >= bound1 - kMathEpsilons && val <= bound2 + kMathEpsilons;
        }

        bool LineSegment2d::IsParallelTo(LineSegment2d &other_segment) {
            //    const double cc1 = Algebra::CrossProd(start_, end_,
            //    other_segment.start());
            //    const double cc2 = Algebra::CrossProd(start_, end_,
            //    other_segment.end());
            //
            //    const double cc3 =
            //        Algebra::CrossProd(other_segment.start(), other_segment.end(),
            //        start_);
            //    const double cc4 =
            //        Algebra::CrossProd(other_segment.start(), other_segment.end(),
            //        end_);
            //
            //    if(cc1 <= 1e-3 && cc2 <= 1e-3 && cc3 <= 1e-3 && cc4 <= 1e-3) {
            //      return true;
            //    }
            //    return false;
            if (std::fabs(Algebra::normalizeAngle(
                    math::Algebra::normalizeAngle(heading()) -
                    math::Algebra::normalizeAngle(other_segment.heading()))) < 1e-3) {
                return true;
            }
            if (std::fabs(
                    std::fabs(math::Algebra::normalizeAngle(heading()) -
                              math::Algebra::normalizeAngle(other_segment.heading())) -
                    M_PI) < 1e-3) {
                return true;
            }

            return false;
        }

        bool LineSegment2d::IsColinear(LineSegment2d &other_segment) {
            const double cc1 = Algebra::CrossProd(start_, end_, other_segment.start());
            const double cc2 = Algebra::CrossProd(start_, end_, other_segment.end());
            return fabs(cc1) < 1e-3 && fabs(cc2) < 1e-3;
        }

}  // namespace math

