//
// Created by zzm on 23-12-5.
//
#include "common/utilpath/point.h"

Point1D::Point1D(double x) : x_(x) {}

bool Point1D::operator==(const Point1D &other) const {
    return x_ == other.x_;
}

Point2D::Point2D(double x, double y) : Point1D(x), y_(y) {}

bool Point2D::operator==(const Point2D &other) const {
    if (!Point1D::operator==(other)) return false;
    return y_ == other.y_;
}

void Point2D::TransVehToUtm(double vehicle_x,
                            double vehicle_y,
                            double vehicle_yaw) {
    double x_offset = sin(vehicle_yaw) * x_ + cos(vehicle_yaw) * y_;
    double y_offset = -cos(vehicle_yaw) * x_ + sin(vehicle_yaw) * y_;
    x_ = vehicle_x + x_offset;
    y_ = vehicle_y + y_offset;
}

void Point2D::TransUtmToVeh(double vehicle_x,
                            double vehicle_y,
                            double vehicle_yaw) {
    auto x_offset = x_ - vehicle_x;
    auto y_offset = y_ - vehicle_y;
    x_ = sin(vehicle_yaw) * x_offset - cos(vehicle_yaw) * y_offset;
    y_ = cos(vehicle_yaw) * x_offset + sin(vehicle_yaw) * y_offset;
}

void Point2D::TransBetweenCoord(double x, double y, double theta) {
    auto x_temp = x_;
    auto y_temp = y_;
    x_ = x_temp * std::cos(theta) - y_temp * sin(theta) + x;
    y_ = y_temp * std::cos(theta) + x_temp * sin(theta) + y;
}

Point3D::Point3D(double x, double y, double z) : Point2D(x, y), z_(z) {}

bool Point3D::operator==(const Point3D &other) const {
    if (!Point2D::operator==(other)) return false;
    return z_ == other.z_;
}