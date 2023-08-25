//
// Created by zzm on 2023/6/1.
//
#ifndef POLYGONBACKSHAPE_COMMON_MATH_H
#define POLYGONBACKSHAPE_COMMON_MATH_H
#include <vector>
#include <Planning/path_common.h>
#include <Planning/path_polygonplan.h>
#include <Planning/path_interface.h>
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
//13.给出一个三角形，求三角形对应的最短边，及其对角的顶点
//14.给定一个点和一个多边形(闭环)的点位，更新多边形的存储顺序，第一个点距离该点距离最近

//1.计算两点之间的距离
//2.找到线段上固定端点固定距离的点
//3.找到线段上固定端点的延长线上固定距离的点
//4.在线段之间插入factor-1个点，等间隔插入
//5.计算点到线段所在直线距离
//6.计算点p在线段AB上的投影点

//基础数学运算
//1.取得余数
//2.计算两点之间欧式距离
//3.计算向量的heading
//4.计算一个点是在线段AB所在直线的左侧还是右侧
//5.根据两点构造一个向量
        commonMath() = default;
        virtual ~commonMath() = default;

        //1.计算x范围内最小值
        static  double get_point_min_x( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> x_value;
                for(auto i : points){
                    x_value.push_back(i.x);
                }
                min_value = *std::min_element(x_value.begin(),x_value.end());


                return min_value;
        }
        //2.计算y范围内最小值
        static  double get_point_min_y( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> y_value;
                for(auto i : points){
                    y_value.push_back(i.y);
                }
                min_value = *std::min_element(y_value.begin(),y_value.end());

                return min_value;
        }


        //判断这三个点构成的三角形是否是凸三角形
        static bool  isConvex(const Point& p, const Point & prev, const Point & next){
            Point v1(prev.x - p.x, prev.y - p.y);
            Point v2(next.x - p.x,next.y-p.y);
            return cross(v1,v2) > 0.0;
        }

        //3.计算得到凹凸多边形的凸点或凹点
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

        //4.计算两个向量的叉积
        static double cross(const Point &v1, const Point &v2){
            return v1.x * v2.y - v1.y * v2.x;
        }

        static double cross(aiforce::Route_Planning::polygonPoint A,
                            aiforce::Route_Planning::polygonPoint B){
            return A.x * B.y - A.y * B.x;
        }

        //6.计算一条线段沿着另外一条线段的方向移动固定的距离distance
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

        static std::vector<aiforce::Route_Planning::polygonPoint>   computeLineTranslationPoints(
                                                std::vector<aiforce::Route_Planning::polygonPoint> initialPoints,
                                                std::vector<aiforce::Route_Planning::polygonPoint> directionPoints,
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
            aiforce::Route_Planning::polygonPoint p1,p2;
            p1.x = p1x_new;
            p1.y = p1y_new;
            p2.x = p2x_new;
            p2.y = p2y_new;
            std::vector<aiforce::Route_Planning::polygonPoint>  result;
            result.push_back(p1);
            result.push_back(p2);
            return result;
        }

        //7.对于给定点，求出给定点的前向点和后向点
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
                if(fabs(p.x -topPoint.x)< 2.5 &&
                   fabs(p.y -topPoint.y) < 0.8){
                    storagePoints.push_back(prev);
                    storagePoints.push_back(next);
                    return storagePoints;
                }
            }
        }

        //7.对于给定点，求出给定点的前向点和后向点
        static std::vector<aiforce::Route_Planning::polygonPoint>  computeForwardAndBackPoints(
                                        std::vector<aiforce::Route_Planning::polygonPoint> polyPoints,
                                        aiforce::Route_Planning::polygonPoint topPoint){
            //要求polyPoints中没有重复点
            int num =  polyPoints.size();
            std::vector<aiforce::Route_Planning::polygonPoint> storagePoints;
            aiforce::Route_Planning::polygonPoint p, prev,next;
            for(int i = 0; i < num ;i++){
                p = polyPoints[i];
                prev = polyPoints[(i + num -1) % num];
                next = polyPoints[(i +1) % num];
                if(fabs(p.x -topPoint.x)< 0.01 &&
                   fabs(p.y -topPoint.y) < 0.01){
                    storagePoints.push_back(prev);
                    storagePoints.push_back(next);
                    return storagePoints;
                }
            }
        }

        //11.针对给定的多边形的点位信息，找到x最小的点作为起始点，开始逆时针排序存储
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
        //13.给出一个三角形，删除重复点，按照逆时针返回
//        static std::vector<aiforce::Route_Planning::polygonPoint> computeOrderedPts(
//                std::vector<aiforce::Route_Planning::polygonPoint> points){
//               //默认给出的三角形是闭环的
//               int num = points.size();
//               double min_distance = DBL_MAX;
//
//               return storage_pts;
//        }
        //14.给定一个点和一个多边形(闭环)的点位，更新多边形的存储顺序，第一个点距离该点距离最近
        static std::vector<aiforce::Route_Planning::polygonPoint>  updatePolySequenceOrdered(
                aiforce::Route_Planning::polygonPoint point_A,
                std::vector<aiforce::Route_Planning::polygonPoint>  pts){
            double min_dis = DBL_MAX;
            aiforce::Route_Planning::polygonPoint min_pt;
            int ordered_number;
            for(int it = 0; it  < pts.size();it++){
                double temp_dis = distance2(pts[it],point_A);
                if(temp_dis < min_dis){
                    min_dis = temp_dis;
                    min_pt.x = pts[it].x;
                    min_pt.y = pts[it].y;
                    ordered_number = it;
                }
            }
            int num = pts.size();
            int temp_m = ordered_number;
            std::vector<aiforce::Route_Planning::polygonPoint> stro_pts;
            for(int i = 0;i < num-1;i++){
                stro_pts.push_back(pts[ordered_number % num]);
                ordered_number +=1;
            }
            stro_pts.push_back(pts[temp_m % num]);
            return stro_pts;
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
        static double computeTwoLineAngle(aiforce::Route_Planning::polygonPoint line_1,
                                          aiforce::Route_Planning::polygonPoint line_2){
            double dot = dot_product(line_1,line_2);
            double mag_a = magnitude(line_1.x,line_1.y);
            double mag_b = magnitude(line_2.x,line_2.y);
            return acos(dot / (mag_a * mag_b));
        }
        //输出角度范围为-180 到 180 度
        static double computeTwolineAngleDu(
                aiforce::Route_Planning::polygonPoint a,
                aiforce::Route_Planning::polygonPoint b){
            float dotProduct = a.x * b.x + a.y * b.y;
            float magnitude = std::sqrt(a.x * a.x + a.y * a.y) * std::sqrt(b.x * b.x + b.y * b.y);
            float cosTheta = dotProduct / magnitude;
            float theta = std::acos(cosTheta) * 180.0 / M_PI;
            float crossProduct = a.x * b.y - a.y * b.x;
            if (crossProduct < 0) {
                theta = -theta;
            }
            return theta;
        }
        //1.计算两点之间的距离
        static double  distanceTwoPolygonPoints(aiforce::Route_Planning::polygonPoint  point_1,
                                                aiforce::Route_Planning::polygonPoint point_2){
                 double  result;
                 result = sqrt((point_2.x - point_1.x) * (point_2.x - point_1.x) +
                                       (point_2.y - point_1.y) * (point_2.y - point_1.y));
            return  result;
        }
        //2.找到线段上固定端点固定距离的点
        static aiforce::Route_Planning::polygonPoint findPointOnSegment(
                aiforce::Route_Planning::polygonPoint p1,
                aiforce::Route_Planning::polygonPoint p2,
                double dist,
                bool isP1Fixed){
            //计算线段长度
            double segmentLength = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
            // 计算方向向量
            double dx,dy;
//            if(isP1Fixed){
                 dx = (p2.x - p1.x) / segmentLength;
                 dy = (p2.y - p1.y) / segmentLength;
//            }else{
//                 dx = (p1.x - p2.x) /segmentLength;
//                 dy = (p1.y - p2.y) /segmentLength;
//            }
            // 确定起始点
            Point startPoint = isP1Fixed ? p1 : p2;
            aiforce::Route_Planning::polygonPoint endPoint;
            if(isP1Fixed){
                endPoint.x = startPoint.x + dx * dist;
                endPoint.y = startPoint.y + dy * dist;
            }else{
                endPoint.x = startPoint.x - dx * dist;
                endPoint.y = startPoint.y - dy * dist;
            }
            endPoint.heading = atan2(dy,dx);
            return endPoint;
        }
        //3.找到线段上固定端点的延长线上固定距离的点
        static std::vector<pathInterface::pathPoint>  findPointExtendSegment(
                aiforce::Route_Planning::polygonPoint p1,
                aiforce::Route_Planning::polygonPoint p2,
                double dist,
                bool isP1Fixed,
                double count){
            //计算线段长度
            double segmentLength = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
            // 计算方向向量
            double dx = (p2.x - p1.x) / segmentLength;
            double dy = (p2.y - p1.y) / segmentLength;
            // 确定起始点
            Point startPoint = isP1Fixed ? p1 : p2;
            Point endPoint = isP1Fixed  ? p2 : p1;
            pathInterface::pathPoint tans_endPoint;
            std::vector<pathInterface::pathPoint>  storage_points;
            if(isP1Fixed){
                for(int i = 1;i <= count;i++){
                    tans_endPoint.x = endPoint.x + dx * dist * i;
                    tans_endPoint.y = endPoint.y + dy * dist * i;
                    storage_points.push_back(tans_endPoint);
                }
            }else{
                for(int i = 1;i <= count;i++){
                    tans_endPoint.x = endPoint.x - dx * dist * i;
                    tans_endPoint.y = endPoint.y - dy * dist * i;
                    storage_points.push_back(tans_endPoint);
                }
            }
           return storage_points;
        }
        //4.在线段之间插入factor-1个点，等间隔插入
        static std::vector<Point> densify(const std::vector<aiforce::Route_Planning::polygonPoint>& points,
                                                      int factor) {
            std::vector<Point> densified_points;
            for (size_t i = 0; i < points.size() - 1; ++i) {
                densified_points.push_back(points[i]);
                for (int j = 1; j < factor; ++j) {
                    double t = static_cast<double>(j) / factor;
                    Point interpolated_point;
                    interpolated_point.x = points[i].x + t * (points[i + 1].x - points[i].x);
                    interpolated_point.y = points[i].y + t * (points[i + 1].y - points[i].y);
                    densified_points.push_back(interpolated_point);
                }
            }
            densified_points.push_back(points.back());
            return densified_points;
        }
        //5.计算点到线段所在直线距离
        static double distanceToLineSegment(double x1, double y1,
                                              double x2,double y2,
                                              double x,double y){
            double distanceAB =
                    std::sqrt(std::pow(x2 - x1,2) + std::pow(y2 - y1,2));
            double distancePToAB =
                    std::abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - x1 * y2) / distanceAB;
            return distanceAB;
        }
        //6.计算点p在线段AB上的投影点
        static aiforce::Route_Planning::polygonPoint  projectPointOnSegment(
                aiforce::Route_Planning::polygonPoint p,
                aiforce::Route_Planning::polygonPoint a,
                aiforce::Route_Planning::polygonPoint b){
            double  segmentLength =
                    std::sqrt(std::pow(b.x - a.x,2) + std::pow(b.y - a.y,2));
            if(segmentLength == 0){
                return p;
            }
            //计算向量ab的单位向量
            double vx = (b.x - a.x) / segmentLength;
            double vy = (b.y - a.y) / segmentLength;

            //计算向量ap
            double dx = p.x - a.x;
            double dy = p.y - a.y;

            double projectionLength = dx * vx + dy * vy;

            //如果投影长度小于等于0,则点p在线段ab之外，返回点a
            if(projectionLength <= 0){
                LOG(INFO) << "projection length <= 0";
                return a;
            }
            //如果投影長度大于等于线段长度，泽点ｐ在线段ａｂ之外，返回点ｂ
            if(projectionLength >= segmentLength){
                LOG(INFO) << "projection length >= length ab";
                return b;
            }
            aiforce::Route_Planning::polygonPoint projectionPoint;
            projectionPoint.x = a.x + projectionLength * vx;
            projectionPoint.y = a.y + projectionLength * vy;

            return  projectionPoint;
        }

        // 7.计算点p在线段AB上的垂足点
        static aiforce::Route_Planning::polygonPoint  computeFootPoint(
               aiforce::Route_Planning::polygonPoint P,
               aiforce::Route_Planning::polygonPoint A,
               aiforce::Route_Planning::polygonPoint B){
           aiforce::Route_Planning::polygonPoint AP = {P.x - A.x, P.y - A.y};
           aiforce::Route_Planning::polygonPoint AB = {B.x - A.x, B.y - A.y};

           double dotProduct = AP.x * AB.x + AP.y * AB.y;
           double abSquared = AB.x * AB.x + AB.y * AB.y;

           double t = dotProduct/abSquared;

           aiforce::Route_Planning::polygonPoint footPoint;
           footPoint.x = A.x + t * AB.x;
           footPoint.y = A.y + t * AB.y;
           return footPoint;
        }

        //基础数学运算
        //1.取得余数
       static double  doubleMod(double x, double y){
            double q  = x / y;
            long int q_number = static_cast<long int>(q);
            return (x - static_cast<double>(q_number) * y) / y;
        }
        //2.计算两点之间欧式距离
        static double distance2(aiforce::Route_Planning::polygonPoint a,
                                aiforce::Route_Planning::polygonPoint b){
            double distance;
            distance =   std::sqrt(std::pow(b.x - a.x,2) + std::pow(b.y - a.y,2));
            return  distance;
        }
        static  double distance2(Point a,
                                aiforce::Route_Planning::polygonPoint b){
            double distance;
            distance =   std::sqrt(std::pow(b.x - a.x,2) + std::pow(b.y - a.y,2));
            return  distance;
        }
        //3.计算向量的heading
        static double calculateHeading(aiforce::Route_Planning::polygonPoint vector_p){
            double heading = std::atan2(vector_p.y,vector_p.x);
            return heading;
        }
        //4.计算一个点是在线段AB所在直线的左侧还是右侧,左侧返回1,右侧返回-1,直线上返回0
        static int pointLocation(aiforce::Route_Planning::polygonPoint v1,
                                 aiforce::Route_Planning::polygonPoint v2){
            double result = cross(v1,v2);
            if(result > 0){
                return 1;  //点在直线左侧
            }else if(result < 0){
                return  -1; //点在直线右侧
            }else {
                return 0;
            }
        }
        //5.根据两点构造一个向量
         static  aiforce::Route_Planning::polygonPoint  construceVector(
                aiforce::Route_Planning::polygonPoint A,
                aiforce::Route_Planning::polygonPoint B){
            aiforce::Route_Planning::polygonPoint vector_C;
            vector_C.x = A.x - B.x;
            vector_C.y = A.y - B.y;
            return vector_C;
        }
        //6.判断一个点是否在线段AB上
        static bool isPointOnSegment(
                aiforce::Route_Planning::polygonPoint A,
                aiforce::Route_Planning::polygonPoint B,
                aiforce::Route_Planning::polygonPoint P){
            // 计算向量AP、BP和AB的长度
            double AP_length = distance2(A, P);
            double BP_length = distance2(B, P);
            double AB_length = distance2(A, B);
            // 如果AP + BP = AB，则表明点P在线段AB上
            if (fabs(AP_length + BP_length - AB_length) < 1e-6) {
                return true;
            }
            return false;
        }


    };
}
#endif //POLYGONBACKSHAPE_COMMON_MATH_H
