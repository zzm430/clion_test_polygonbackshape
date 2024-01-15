//
// Created by zzm on 23-12-5.
//

#ifndef POLYGONBACKSHAPE_DISCRETIZEDPATH_H
#define POLYGONBACKSHAPE_DISCRETIZEDPATH_H

#include <algorithm>
#include "common/utilpath/discrete_match_helper.h"
#include "common/utilpath/path_polygonPoint.h"
#include "common/math/line_segment_2d.h"
#include "common/utilpath/pt_slboundary.h"
#include "common/math/linear_interpolation.h"
#include "common/utilpath/path_point.h"
class DiscretizedPath : public DiscreteMatchHelper<PathPoint> {
public:
    DiscretizedPath() = default;

//    explicit DiscretizedPath(std::vector<polygonPoint>& path_points);

    double Length() const;
//
    PathPoint Evaluate(double path_s) const;
//
//    data::PathPoint EvaluateReverse(double path_s) const;
//
//    const PathMatch& ego_match() const {
//        return ego_match_;
//    }
//
//    PathMatch& mutable_ego_match() {
//        return ego_match_;
//    }
//
//    bool MatchEgo(double x, double y, double theta);
//    bool IsAvailable() const;
//

    void FromRawPts(const std::vector<polygonPoint>& raw_pts);
//    void FromPosPoints(const std::vector<hd_map::common::PosPoint>& raw_points,
//                       const std::vector<double>& headings);
//
    bool SLToXY(double s, double l, polygonPoint& xy_point);
//
//    data::Point2D SLToXY(double s, double l, data::PathPoint& ref_pt);
//
    bool XYToSL(const math::Vec2d& xy_point, slPoint* sl_point) const;

    bool GetProjection(const math::Vec2d& point,
                       double* accumulate_s,
                       double* lateral) const;
    bool GetProjection(const math::Vec2d& point,
                       double* accumulate_s,
                       double* lateral,
                       double* min_distance,
                       int* min_index) const;
    PathPoint GetPathPtFromS(double s) const;
//
//    size_t GetPathPtIdxFromS(double s);
//
//    PathPoint GetPathPtFromS(double s, size_t& idx);
//
//
//    void TransVehToUtm(double vehicle_x, double vehicle_y, double vehicle_yaw);
//
//    void TransUtmToVeh(double vehicle_x, double vehicle_y, double vehicle_yaw);
//
//    void TransBetweenCoord(double x, double y, double theta_offset);

//    bool GetSLBoundary(const math::Box2d& box, SLBoundary* sl_boundary) const;
//    bool GetMutableSLBoundary(math::Box2d& box, data::SLBoundary* sl_boundary);
//    bool GetSLBoundary(const math::Polygon2d& polygon,
//                       SLBoundary* sl_boundary) const;
//    bool GetSLBoundary(const std::vector<math::Vec2d>& vec_s,
//                       SLBoundary* sl_boundary) const;
//
//    bool GetSLBoundary(const std::vector<Point2D>& pts,
//                       SLBoundary* sl_boundary) const;
      bool GetSlBoundary(const std::vector<polygonPoint>& polygon,
                         SLBoundary* slBoundary) const;
protected:
    std::vector<PathPoint>::const_iterator QueryLowerBound(
            double path_s) const;
//    std::vector<data::PathPoint>::const_iterator QueryUpperBound(
//            double path_s) const;
private:

};



#endif //POLYGONBACKSHAPE_DISCRETIZEDPATH_H
