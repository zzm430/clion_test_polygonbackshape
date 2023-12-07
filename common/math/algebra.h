//
// Created by zzm on 23-12-4.
//

#ifndef POLYGONBACKSHAPE_ALGEBRA_H
#define POLYGONBACKSHAPE_ALGEBRA_H
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>
#include "common/utilpath/path_polygonPoint.h"
#include "vec_2d.h"

// WrapAngle(double angle)：将角度值规范化到 [0, 2π) 的范围内。
// CrossProd(const Vec2d& start_point, const Vec2d& end_point_1, const Vec2d& end_point_2)：计算两个向量的叉积。
// InnerProd(const Vec2d& start_point, const Vec2d& end_point_1, const Vec2d& end_point_2)：计算两个向量的点积。
// distance(double x1, double y1, double x2, double y2)：计算两点之间的欧氏距离。
// normalizeAngle(double a)：将角度规范化到 [0, 2π) 的范围内。
// differentAngle(double a0, double a1)：计算两个角度之间的最小差值。
// lerp(double x0, double x1, double t)：线性插值。
// slerp(double a0, double a1, double t)：球面线性插值。
// findCircleCenterPtWithThreePts(double x0, double y0, double x1, double y1, double x2, double y2, double& cx, double& cy)：通过三个点求圆的中心。
// calAngleVectorToXAxel(double x, double y)：计算向量与 x 轴正方向的夹角。
// projectPtToLine(const polygonPoint& pt, const polygonPoint& line)：将点投影到直线上。
// isTwoContourCrash(const std::vector<polygonPoint>& contour1, const std::vector<polygonPoint>& contour2)：判断两个多边形是否相交。
// isContourConvex(const std::vector<polygonPoint>& contour)：判断多边形是否为凸多边形。
// calMinDisPtToSegment(const polygonPoint& pt, const polygonPoint& seg_spt, const polygonPoint& seg_ept)：计算点到线段的最短距离。
// calMinDisPtToContour(const polygonPoint& pt, const std::vector<polygonPoint>& contour)：计算点到多边形的最短距离。
// calMinDisOfTwoContour(const std::vector<polygonPoint>& contour1, const std::vector<polygonPoint>& contour2)：计算两个多边形的最短距离。

namespace math {
    struct Algebra {
        static double WrapAngle(double angle);

        static double CrossProd(const Vec2d &start_point,
                                const Vec2d &end_point_1,
                                const Vec2d &end_point_2);

        static double InnerProd(const Vec2d &start_point,
                                const Vec2d &end_point_1,
                                const Vec2d &end_point_2);

        static double CrossProd(double x0, double y0, double x1, double y1);

        static double InnerProd(double x0, double y0, double x1, double y1);

        template<typename T>
        static T Square(const T value);

        static double distance(double x1, double y1, double x2, double y2);

        static double distance(const math::Vec2d &a0, const math::Vec2d &a1);

        static double normalizeAngle(double a);

        static double normalizeAngleToSignField(double angle);

        static double angleDiff(double from, double to);

        static double differentAngle(double a0, double a1);

        template<typename T>

        static T lerp(const T &x0, double t0, const T &x1, double t1, double t);

        static double lerp(double x0, double x1, double t);

        static double slerp(double a0, double a1, double t);

        static double slerp(double a0, double t0, double a1, double t1, double t);

        static void findCircleCenterPtWithThreePts(double x0,
                                                   double y0,
                                                   double x1,
                                                   double y1,
                                                   double x2,
                                                   double y2,
                                                   double &cx,
                                                   double &cy);

        static double calAngleVectorToXAxel(double x, double y);

        static double projectPtToLine(const polygonPoint &pt,
                                      const polygonPoint &line);

        static void projectContourToLine(
                const std::vector<polygonPoint> &contour,
                const polygonPoint &line,
                double &left,
                double &right);

        static void projectContourToLine(
                const std::vector<polygonPoint> &contour,
                const polygonPoint &line,
                double &left_dis,
                double &right_dis,
                polygonPoint &left_pt,
                polygonPoint &right_pt);

        static void projectContourToLine(
                const std::vector<polygonPoint> &contour,
                const polygonPoint &line,
                double &near_dis,
                double &far_dis,
                size_t &near_idx,
                size_t &far_idx);

        static bool onSegment(const polygonPoint&p,
                              const polygonPoint &q,
                              const polygonPoint &r);

        static double modVector(const polygonPoint&vec);

        static polygonPoint normalizeVector(const polygonPoint &vec);

        static polygonPoint vertNormVector(const polygonPoint &vec);

        static polygonPoint footOfPtToSegment(
                const polygonPoint &pt,
                const polygonPoint &line,
                const polygonPoint &seg_spt);

        static int orientation(const polygonPoint &p,
                               const polygonPoint &q,
                               const polygonPoint &r);

        static bool isTwoContourCrash(
                const std::vector<polygonPoint> &contour1,
                const std::vector<polygonPoint> &contour2);

        static bool isContourConvex(const std::vector<polygonPoint> &contour);

        static double calMinDisPtToSegment(const polygonPoint &pt,
                                           const polygonPoint &seg_spt,
                                           const polygonPoint &seg_ept);

        static double calMinDisPtToContour(
                const polygonPoint &pt,
                const std::vector<polygonPoint> &contour);

        static double calMinDisOfTwoContour(
                const std::vector<polygonPoint> &contour1,
                const std::vector<polygonPoint> &contour2);
    };

    template <typename T>
    T Algebra::Square(const T value) {
        return value * value;
    }

    template <typename T>
    T Algebra::lerp(const T& x0, double t0, const T& x1, double t1, double t) {
        if (std::abs(t1 - t0) <= 1.0e-6) {
//            LOG(WARNING) << "input time difference is too small";
            return x0;
        }
        double r = (t - t0) / (t1 - t0);
        const T x = x0 + r * (x1 - x0);
        return x;
    }
}


#endif //POLYGONBACKSHAPE_ALGEBRA_H
