//
// Created by zzm on 23-12-5.
//

#ifndef POLYGONBACKSHAPE_DISCRETIZEDPATH_H
#define POLYGONBACKSHAPE_DISCRETIZEDPATH_H

#include <algorithm>
#include "common/utilpath/discrete_match_helper.h"
#include "common/utilpath/path_polygonPoint.h"
#include "common/math/line_segment_2d.h"

struct PathPoint   {
    PathPoint();
    explicit PathPoint(  polygonPoint& traj_pt);
    double s() const {
        return s_;
    }

    double x() const{
        return  x_;
    }

    double y() const {
        return  y_;
    }

    double z() const{
        return  z_;
    }

    double kappa() const {
        return kappa_;
    }

    double theta() const {
        return theta_;
    }
    double dkappa() const {
        return dkappa_;
    }
    double ddkappa() const {
        return ddkappa_;
    }
    double heading() const {
        return heading_;
    }
    void set_heading(double heading) {
        heading_ = heading;
    }
    void set_s(double s) {
        s_ = s;
    }
    void set_theta(double theta) {
        theta_ = theta;
    }
    void set_kappa(double kappa) {
        kappa_ = kappa;
    }
    void set_dkappa(double dkappa) {
        dkappa_ = dkappa;
    }
    void set_ddkappa(double ddkappa) {
        ddkappa_ = ddkappa;
    }

    PathPoint(double x,
              double y,
              double z,
              double s,
              double theta,
              double heading,
              double kappa,
              double dkappa,
              double ddkappa);

    PathPoint(const PathPoint& pt) = default;

    bool operator==(const PathPoint& other) const;

    void TransVehToUtm(double vehicle_x,
                       double vehicle_y,
                       double vehicle_yaw) ;

    void TransUtmToVeh(double vehicle_x,
                       double vehicle_y,
                       double vehicle_yaw) ;

    void TransBetweenCoord(double x, double y, double theta);

    double distance(double x, double y);

    static PathPoint FindProjectionPt(const PathPoint& p0,
                                            const PathPoint& p1,
                                            double x,
                                            double y);

    static  PathPoint InterpPtByLinearS(const PathPoint& p0,
                                             const PathPoint& p1,
                                             double s);

public:
    double x_;
    double y_;
    double z_;

private:
    double s_;
    double theta_;       /// 速度方向的角度
    double heading_;     /// 车体的角度
    double kappa_;
    double dkappa_;
    double ddkappa_;


private:

};




class DiscretizedPath : public DiscreteMatchHelper<PathPoint> {
public:
    DiscretizedPath() = default;

//    explicit DiscretizedPath(std::vector<polygonPoint>& path_points);

//    double Length() const;
//
//    data::PathPoint Evaluate(double path_s) const;
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
//    bool XYToSL(const Point2D& xy_point, SLPoint* sl_point) const;
//
//    bool GetProjection(const Point2D& point,
//                       double* accumulate_s,
//                       double* lateral) const;
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

protected:
//    std::vector<data::PathPoint>::const_iterator QueryLowerBound(
//            double path_s) const;
//    std::vector<data::PathPoint>::const_iterator QueryUpperBound(
//            double path_s) const;

private:


};



#endif //POLYGONBACKSHAPE_DISCRETIZEDPATH_H
