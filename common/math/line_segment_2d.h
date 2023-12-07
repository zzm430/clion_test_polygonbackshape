//
// Created by zzm on 23-12-5.
//

#ifndef POLYGONBACKSHAPE_LINE_SEGMENT_2D_H
#define POLYGONBACKSHAPE_LINE_SEGMENT_2D_H

#include <cmath>
#include "common/math/vec_2d.h"
#include "common/math/algebra.h"

namespace math{

    class LineSegment2d {
    public:
        LineSegment2d(const Vec2d &start, const Vec2d &end);

        LineSegment2d() = default;

        LineSegment2d &Extend(double length);

        const Vec2d &start() const;

        const Vec2d &end() const;

        const Vec2d &unit_direction() const;

        Vec2d center() const;

        Vec2d rotate(const double angle);

        double heading() const;

        double cos_heading() const;

        double sin_heading() const;

        double length() const;

        double length_sqr() const;

        double DistanceTo(const Vec2d &point) const;

        double StraightSignedDistanceTo(const Vec2d &point) const;

        double StraightAbsDistanceTo(const Vec2d &point) const;

        double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

        double DistanceSquareTo(const Vec2d &point) const;

        double DistanceSquareTo(const Vec2d &point, Vec2d *const nearest_pt) const;

        bool IsPointIn(const Vec2d &point) const;

        double ProjectOntoUnit(const Vec2d &point) const;

        double ProductOntoUnit(const Vec2d &point) const;

        bool HasIntersect(const LineSegment2d &other_segment) const;

        bool GetIntersect(const LineSegment2d &other_segment,
                          Vec2d *const point) const;

        // return distance with perpendicular foot point.
        double GetPerpendicularFoot(const Vec2d &point,
                                    Vec2d *const foot_point) const;

        bool IsWithin(double val, double bound1, double bound2) const;

        bool IsParallelTo(LineSegment2d &other_segment);

        bool IsColinear(LineSegment2d &other_segment);

    private:
        Vec2d start_;
        Vec2d end_;
        Vec2d unit_direction_;
        double heading_ = 0.;
        double length_ = 0.;
    };

}  //namespace math

#endif //POLYGONBACKSHAPE_LINE_SEGMENT_2D_H
