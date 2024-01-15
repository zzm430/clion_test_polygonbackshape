//
// Created by zzm on 24-1-10.
//

#ifndef POLYGONBACKSHAPE_PATH_POINT_H
#define POLYGONBACKSHAPE_PATH_POINT_H
#include "common/utilpath/path_polygonPoint.h"

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

    void set_x(double x){
        x_ = x;
    }

    void set_y(double y){
        y_ = y;
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


#endif //POLYGONBACKSHAPE_PATH_POINT_H
