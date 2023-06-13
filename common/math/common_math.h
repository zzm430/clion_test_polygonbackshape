//
// Created by zzm on 2023/6/1.
//
#ifndef POLYGONBACKSHAPE_COMMON_MATH_H
#define POLYGONBACKSHAPE_COMMON_MATH_H
#include <vector>
#include <Planning/path_common.h>
#include <Planning/path_polygonplan.h>
#include <math.h>
#include <algorithm>
#include <float.h>
namespace common{
    class  commonMath{
    public:
//1.计算x范围内最小值
//2.计算y范围内最小值
//3.计算凹凸多边形内哪些点是凹点、凸点
//4.计算凹凸多边形内每个顶点的角度是多大(未实现)
//4.计算两点之间叉积
//5.计算两向量方向是否一致
//6.计算两条线段一条沿着另外一条移动固定距离d
//7.已知多边形中一个点，求其前一个点和后一个点按照逆时针
//8.已知一条线段，求其延长一边固定距离后的线段（未实现）
//9.已知一条线段，求其延长两边固定距离后的线段（未实现）
//10.对一个多边形的顶点进行排序按照逆时针左下角点为起始点(未实现)
//11.针对给定的多边形的点位信息，找到x最小的点作为起始点，开始逆时针排序存储
//12.针对给定点，插入到多边形的指定边上，前提是插入的点在多边形的边上


//1.计算两点之间的距离

        commonMath() = default;
        virtual ~commonMath() = default;

        static  double get_point_min_x( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> x_value;
                for(auto i : points){
                    x_value.push_back(i.x);
                }
                min_value = *std::min_element(x_value.begin(),x_value.end());


                return min_value;
        }

        static  double get_point_min_y( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> y_value;
                for(auto i : points){
                    y_value.push_back(i.y);
                }
                min_value = *std::min_element(y_value.begin(),y_value.end());

                return min_value;
        }
        //计算两个向量的叉积
        static double cross(const Point &v1, const Point &v2){
            return v1.x * v2.y - v1.y * v2.x;
        }
        //判断这三个点构成的三角形是否是凸三角形
        static bool  isConvex(const Point& p, const Point & prev, const Point & next){
            Point v1(prev.x - p.x, prev.y - p.y);
            Point v2(next.x - p.x,next.y-p.y);
            return cross(v1,v2) > 0.0;
        }
        //计算得到凹凸多边形的凸点或凹点
        static  std::vector<Point> IsHollow_vec(std::vector<Point> curveLoopPoints){
            //假设法向量判断凹凸性，检测多边形上上是否有凸点，每个顶点转向都应该一致，不一致则为凹点
            //假设给出的点是顺时针，则选择的是给出的是凸点，逆时针给出的是凹点
            std::vector<Point> hollowPoints;
            auto num = curveLoopPoints.size();
            Point p,prev,next;
            for(int i = 0; i < num ;i++){
                p = curveLoopPoints[i];
                prev = curveLoopPoints[(i + num -1) % num];
                next = curveLoopPoints[(i +1) % num];
                if(isConvex(p,prev,next)){
                    hollowPoints.push_back(p);
                }
            }
            return hollowPoints;
        }
        //计算一条线段沿着另外一条线段的方向移动固定的距离distance
        static std::vector<Point>   computeLineTranslationPoints(std::vector<Point> initialPoints,
                                                            std::vector<Point> directionPoints,
                                                            double distance){
               long double Bx = directionPoints[1].x - directionPoints[0].x;
               long double By = directionPoints[1].y - directionPoints[0].y;
               long double length_B_squared = Bx * Bx + By * By;
               long double u_Bx = Bx / sqrt(length_B_squared);
               long double u_By = By / sqrt(length_B_squared);
               long double p1x_new = initialPoints[0].x + distance * u_Bx;
               long double p1y_new = initialPoints[0].y + distance * u_By;
               long double p2x_new = initialPoints[1].x + distance * u_Bx;
               long double p2y_new = initialPoints[1].y + distance * u_By;
                Point p1,p2;
                p1.x = p1x_new;
                p1.y = p1y_new;
                p2.x = p2x_new;
                p2.y = p2y_new;
                std::vector<Point>  result;
                result.push_back(p1);
                result.push_back(p2);
                return result;
        }

        //对于给定点，求出给定点的前向点和后向点
        static std::vector<Point>  computeForwardAndBackPoints(std::vector<Point> polyPoints,
                                                               Point topPoint){
            //要求polyPoints中没有重复点，是按照逆时针给的
            polyPoints.pop_back(); //因为最后一个点是起点，所以删除
            int num =  polyPoints.size();
            std::vector<Point> storagePoints;
            Point p, prev,next;
            for(int i = 0; i < num ;i++){
                p = polyPoints[i];
                prev = polyPoints[(i + num -1) % num];
                next = polyPoints[(i +1) % num];
                if(p == topPoint){
                    storagePoints.push_back(prev);
                    storagePoints.push_back(next);
                    return storagePoints;
                }
            }
        }

        //针对给定的多边形的点位信息，找到x最小的点作为起始点，开始逆时针排序存储
        static std::vector<Point> updatePolygonPointsSequence(std::vector<Point> polyPoints){
            polyPoints.pop_back(); //删除最后一个用于闭环的点(起点重复的点)
            Point min_point;
            min_point.x = DBL_MAX;
            min_point.y = DBL_MAX;
            int number;
            std::vector<Point>  storage_new_points;
            auto num = polyPoints.size();
            for(int i = 0 ;i < num;i++){
                if(polyPoints[i].x < min_point.x){
                    min_point.x = polyPoints[i].x;
                    min_point.y = polyPoints[i].y;
                    number = i;
                }
            }
            Point temp,next;
            storage_new_points.push_back(min_point);
            for(int i = 1; i <=num-1;i++){
                 next = polyPoints[(number+1)%num];
                 storage_new_points.push_back(next);
                 number = number + 1;
            }
            return storage_new_points;
        }
        //计算两向量之间的点积
        static double dot_product(Point line_1,
                           Point line_2){
            return line_1.x * line_2.x + line_1.y * line_2.y;
        }
        //计算向量的模
        static double magnitude(const double &x,const double &y){
            return  sqrt(x * x + y * y);
        }
        //计算两向量之间的夹角
        static double computeTwoLineAngle(Point line_1,
                                          Point line_2){
               double dot = dot_product(line_1,line_2);
               double mag_a = magnitude(line_1.x,line_1.y);
               double mag_b = magnitude(line_2.x,line_2.y);
               return acos(dot / (mag_a * mag_b));
        }
        static double  distanceTwoPolygonPoints(aiforce::Route_Planning::polygonPoint  point_1,
                                                aiforce::Route_Planning::polygonPoint point_2){
                 double  result;
                 result = sqrt((point_2.x - point_1.x) * (point_2.x - point_1.x) *
                                       (point_2.y - point_1.y) * (point_2.y - point_1.y));
            return  result;
        }
    };
}
#endif //POLYGONBACKSHAPE_COMMON_MATH_H
