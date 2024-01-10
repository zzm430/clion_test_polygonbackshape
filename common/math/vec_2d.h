//
// Created by zzm on 23-12-4.
//

#ifndef POLYGONBACKSHAPE_VEC_2D_H
#define POLYGONBACKSHAPE_VEC_2D_H
#include <cmath>

//   Vec2d::Vec2d(double x, double y)：构造函数，用来初始化二维向量对象的x、y坐标值。
//   Vec2d::Vec2d()：默认构造函数，将二维向量对象的x、y坐标值初始化为0。
//   Vec2d Vec2d::CreateUnitVec2d(double angle)：根据给定的角度创建一个单位向量。
//   double Vec2d::x() const：获取向量的x坐标值。
//   double Vec2d::y() const：获取向量的y坐标值。
//   void Vec2d::set_x(double x)：设置向量的x坐标值。
//   void Vec2d::set_y(double y)：设置向量的y坐标值。
//   double Vec2d::Length() const：计算向量的长度。
//   double Vec2d::LengthSquare() const：计算向量长度的平方。
//   double Vec2d::Angle() const：计算向量的角度。
//   void Vec2d::Normalize()：将向量归一化（即长度变为1）。
//   double Vec2d::DistanceTo(const Vec2d &other) const：计算到另一个向量的距离。
//   double Vec2d::DistanceSquareTo(const Vec2d &other) const：计算到另一个向量的距离的平方。
//   double Vec2d::CrossProd(const Vec2d &other) const：计算向量的叉乘。
//   double Vec2d::InnerProd(const Vec2d &other) const：计算向量的点乘。
//   Vec2d Vec2d::rotate(double angle) const：对向量进行旋转。
//   Vec2d Vec2d::Extend(double theta, double length) const：在指定方向上延伸指定长度的向量。
//   void Vec2d::SelfRotate(double angle)：对向量本身进行旋转。
//   Vec2d Vec2d::operator+(const Vec2d &other) const：重载加法操作符。
//   Vec2d Vec2d::operator-(const Vec2d &other) const：重载减法操作符。
//   Vec2d Vec2d::operator*(double ratio) const：重载乘法操作符。
//   Vec2d Vec2d::operator/(double ratio) const：重载除法操作符。
//   Vec2d &Vec2d::operator+=(const Vec2d &other)：重载加等操作符。
//   Vec2d &Vec2d::operator-=(const Vec2d &other)：重载减等操作符。
//   Vec2d &Vec2d::operator*=(double ratio)：重载乘等操作符。
//   Vec2d &Vec2d::operator/=(double ratio)：重载除等操作符。
//   bool Vec2d::operator==(const Vec2d &other) const：重载相等操作符。
//   bool Vec2d::operator<(const Vec2d &rhs) const：重载小于操作符。

namespace math {

    constexpr double kMathEpsilons = 1e-7;

    class Vec2d {
    public:
        Vec2d(double x, double y);

        Vec2d();

        static Vec2d CreateUnitVec2d(double angle);

        double x() const;

        double y() const;

        void set_x(double x);

        void set_y(double y);

        double Length() const;

        double LengthSquare() const;

        double Angle() const;

        void Normalize();

        double DistanceTo(const Vec2d &other) const;

        double DistanceSquareTo(const Vec2d &other) const;

        double CrossProd(const Vec2d &other) const;

        double InnerProd(const Vec2d &other) const;

        Vec2d rotate(double angle) const;

        Vec2d Extend(double theta, double length) const;

        void SelfRotate(double angle);

        Vec2d operator+(const Vec2d &other) const;

        Vec2d operator-(const Vec2d &other) const;

        Vec2d operator*(double ratio) const;

        Vec2d operator/(double ratio) const;

        Vec2d &operator+=(const Vec2d &other);

        Vec2d &operator-=(const Vec2d &other);

        Vec2d &operator*=(double ratio);

        Vec2d &operator/=(double ratio);

        bool operator==(const Vec2d &other) const;

        bool operator<(const Vec2d &rhs) const;

    protected:
        double x_ = 0.;
        double y_ = 0.;

    };

    static Vec2d operator*(double ratio, const Vec2d &vec) {
        return vec * ratio;
    }

}//namespace math

#endif //POLYGONBACKSHAPE_VEC_2D_H
