//
// Created by zzm on 23-12-5.
//

#ifndef POLYGONBACKSHAPE_POINT_H
#define POLYGONBACKSHAPE_POINT_H
#include <cmath>
#include <vector>

struct Point1D {
    double x_;

    explicit Point1D(double x = 0.);
    double x() const {
        return x_;
    }
    bool operator==(const Point1D &other) const;

private:

};

struct Point2D : public Point1D {
    double y_;

    explicit Point2D(double x = 0., double y = 0.);
    double y() const {
        return y_;
    }
    bool operator==(const Point2D &other) const;

    virtual void TransVehToUtm(double vehicle_x,
                               double vehicle_y,
                               double vehicle_yaw);

    virtual void TransUtmToVeh(double vehicle_x,
                               double vehicle_y,
                               double vehicle_yaw);

    virtual void TransBetweenCoord(double x, double y, double theta);

private:

};

struct Point3D : public Point2D {
    double z_;

    explicit Point3D(double x = 0., double y = 0., double z = 0.);

    bool operator==(const Point3D &other) const;
    double z() const {
        return z_;
    }

private:

};

typedef Point3D Point;


#endif //POLYGONBACKSHAPE_POINT_H
