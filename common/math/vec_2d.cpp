//
// Created by zzm on 23-12-4.
//

#include "common/math/vec_2d.h"

namespace math {

    Vec2d::Vec2d(double x, double y) : x_(x), y_(y) {}

    Vec2d::Vec2d() : Vec2d(0., 0.) {}

    Vec2d Vec2d::CreateUnitVec2d(double angle) {
        return Vec2d(std::cos(angle), std::sin(angle));
    }

    double Vec2d::x() const {
        return x_;
    }

    double Vec2d::y() const {
        return y_;
    }

    void Vec2d::set_x(double x) {
        x_ = x;
    }

    void Vec2d::set_y(double y) {
        y_ = y;
    }

    double Vec2d::Length() const {
        return std::hypot(x_, y_);
    }

    double Vec2d::LengthSquare() const {
        return x_ * x_ + y_ * y_;
    }

    double Vec2d::Angle() const {
        return std::atan2(y_, x_);
    }

    void Vec2d::Normalize() {
        double l = Length();
        if (l > kMathEpsilons) {
            x_ /= l;
            y_ /= l;
        }
    }

    double Vec2d::DistanceTo(const Vec2d &other) const {
        return std::hypot(x_ - other.x_, y_ - other.y_);
    }

    double Vec2d::DistanceSquareTo(const Vec2d &other) const {
        double dx = x_ - other.x_;
        double dy = y_ - other.y_;
        return dx * dx + dy * dy;
    }

    double Vec2d::CrossProd(const Vec2d &other) const {
        return x_ * other.y() - y_ * other.x();
    }

    double Vec2d::InnerProd(const Vec2d &other) const {
        return x_ * other.x() + y_ * other.y();
    }

    Vec2d Vec2d::rotate(double angle) const {
        return Vec2d(x_ * cos(angle) - y_ * sin(angle),
                     x_ * sin(angle) + y_ * cos(angle));
    }

    Vec2d Vec2d::Extend(double theta, double length) const {
        return Vec2d(x() + length * std::cos(theta), y() + length * std::sin(theta));
    }

    void Vec2d::SelfRotate(double angle) {
        double tmp_x = x_;
        x_ = x_ * cos(angle) - y_ * sin(angle);
        y_ = tmp_x * sin(angle) + y_ * cos(angle);
    }

    Vec2d Vec2d::operator+(const Vec2d &other) const {
        return Vec2d(x_ + other.x(), y_ + other.y());
    }

    Vec2d Vec2d::operator-(const Vec2d &other) const {
        return Vec2d(x_ - other.x(), y_ - other.y());
    }

    Vec2d Vec2d::operator*(double ratio) const {
        return Vec2d(x_ * ratio, y_ * ratio);
    }

    Vec2d Vec2d::operator/(double ratio) const {
//        LOG_ASSERT(std::abs(ratio) > kMathEpsilons);
        return Vec2d(x_ / ratio, y_ / ratio);
    }

    Vec2d &Vec2d::operator+=(const Vec2d &other) {
        x_ += other.x();
        y_ += other.y();
        return *this;
    }

    Vec2d &Vec2d::operator-=(const Vec2d &other) {
        x_ -= other.x();
        y_ -= other.y();
        return *this;
    }

    Vec2d &Vec2d::operator*=(double ratio) {
        x_ *= ratio;
        y_ *= ratio;
        return *this;
    }

    Vec2d &Vec2d::operator/=(double ratio) {
//        CHECK_GT(std::abs(ratio), kMathEpsilons);
        x_ /= ratio;
        y_ /= ratio;
        return *this;
    }

    bool Vec2d::operator==(const Vec2d &other) const {
        return (std::abs(x_ - other.x()) < kMathEpsilons &&
                std::abs(y_ - other.y()) < kMathEpsilons);
    }

    bool Vec2d::operator<(const Vec2d &rhs) const {
        if (fabs(rhs.x() - x_) < 1e-3) {
            return y_ > rhs.y();
        } else {
            return x_ < rhs.y();
        }
    }

}  // namespace math