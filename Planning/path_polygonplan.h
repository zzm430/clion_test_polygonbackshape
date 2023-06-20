#ifndef PATH_POLYGONPLAN_H
#define PATH_POLYGONPLAN_H

#include <vector>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <Planning/path_interface.h>
#include <Geometry/reeds_shepp.h>
#include "path_baseplan.h"
#include "common/common_parameters.h"

namespace aiforce{
namespace Route_Planning
{
    typedef double coordinate_type;
    typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
    typedef boost::geometry::model::polygon<point> polygon;
    typedef boost::geometry::model::linestring<point> linestring_type;
    typedef boost::geometry::model::segment<point> Segment;

    class  polygonPoint: public Point{
    public:
        polygonPoint() = default;
        virtual  ~polygonPoint(){};

        bool operator==(const polygonPoint& other) const{
            return (fabs(x - other.x) < 0.1 &&
                    fabs(y -other.y) < 0.1);
        }

        // 定义小于运算符
        bool operator<(const polygonPoint& other) const {
            if (x != other.x) {
                return x < other.x;
            } else {
                return y < other.y;
            }
        }
    private:
        int origin_polygon_point_index_;   //对应多边形起始点的索引号
    public:
        bool entrance_ = false;                    //回字形入口点
    };

    struct polyPointHash{
        size_t operator()(const polygonPoint& s) const{
            std::hash<long double> h;
            return h(s.x) ^ h(s.y);
        }
    };

 class pathPolygonPlan  {
 public:
     pathPolygonPlan(){}
     virtual ~pathPolygonPlan(){}

//     virtual void initiate();
    struct lineKb{
         double k;           //斜率
         double b;           //截距
         double count;       //记录有多少个点在这个线段的延长线上
         double distance;    //两点之间间隔
         double index;       //对应多边形的顶点的序号
         std::vector<polygonPoint> points; //初始的线段点位
     };
     class lineOrignPolygon{
     public:
         lineOrignPolygon() = default;
         virtual  ~lineOrignPolygon() = default;
         double k;        //斜率
         double b;        //截距
         double i;        //线段起点
         double j;        //线段终点
     };
     struct ridgeKeypoint{
         double start_dis;      //关键点起始转弯距离
         double end_dis;        //关键点末尾转弯距离设置
         int    ridge_index;    //关键点位于第几垄
         int    keypoint_index; //关键点所在垄的序号
         int    numbers;        //关键点所在垄总有多少点
         Point   start_curve_point;   //起始点
         Point   end_curve_point;     //弯道结束点
     };

 private:

 public:
     virtual void computeNarrowPolygons(std::vector<Point> & points); //计算缩小后的轮廓点信息
     virtual void computePolygonsAndLineNode(Point node);             //计算多边形与线段的交点
     virtual void computePolygonsAndLineNode(std::vector<Point> linePoints); //计算多边形与线段的交点
     virtual void computeEveryKbLine();                               //计算kbline
     virtual void computebackShapeKeypoints();                        //计算回字形关键点信息
     virtual void judgePointPosition(int i,                           //判断点位信息
                                     boost::geometry::model::multi_polygon<polygon> result);
     virtual void computeOriginPolygonKbline(std::vector<Point> &points);
     virtual void findSuitableEntrance(std::vector<Point> points );
     virtual void updatePolygonPointsSequence1();                      //更新多边形顶点存储的顺序
     virtual void updatePolygonPointsIncrease();                       //向多边形中增加点
     virtual std::vector<polygonPoint> anewStoragePolygonPoints(std::vector<polygonPoint> points,
                                                                          polygonPoint given_point);  //给定点距离最近对多边形重新排序存储
     virtual std::vector<polygonPoint>  deleteRepeatPolyPts(std::vector<polygonPoint> points);   //删除多边形中的重复点并返回
     virtual void  filteredBackShapeKeyPoints();  //删除关键点中的重复点
     virtual void  computeKeypointsRelativeInfo();  //计算后续用于弯道处理信息
     virtual std::vector<pathInterface::pathPoint> computeRidgeRoutingpts(int ridge_index); //根据垄号计算routingpath信息


     const std::vector<std::vector<polygonPoint>> getNarrowPolygonPoints() const; //得到缩小后多边形点位信息
     const polygonPoint  getMinPolygonCentroid() const;               //得到最小多边形的质心
     const int  countNarrowPolygonNumbers() const;                    //得到缩小后的多边形数目
     const std::vector<polygonPoint> getPolygonAndLineNodes() const;  //得到多边形和线段交点
     const std::vector<Point> getLineOriginEntrance() const;          //得到原始的回字形入口
     const std::vector<std::vector<polygonPoint>> getBackShapeKeyPoints() const;   //得到回字形关键点信息
     const std::vector<std::vector<polygonPoint>> getFilteredBackShapeKeyPoints() const;     //得到过滤后的关键点信息
     //     const std::vector<std::vector<Point>> getUpdatedPolygonPointsSequence() const;   //得到更新后排序的多边形
     const std::vector<std::vector<polygonPoint>> getInsertedPolygonPointsIncrease() const;  //得到包含回字形入口的多边形
     const std::vector<std::vector<polygonPoint>> getMiddlePointsPolygon() const;             //得到关键点之前的中间结果
     static bool pointOnSegment(polygonPoint p, polygonPoint a, polygonPoint b);              //判断点p是否在线段ab上
     static std::vector<polygonPoint> insertPointToPolygon(polygonPoint insertPoint,          //将点插入到多边形上，前提是该点在这个多边形上
                                              std::vector<polygonPoint> polygonPoints);
 private:
     std::vector<std::vector<polygonPoint>>   storageNarrowPolygonPoints_;                   //存储缩小的多边形点位
     std::vector<std::vector<polygonPoint>>   insertedPtToNarrowPolygon_;                    //插入交点到内缩多边形们中
     std::vector<std::vector<polygonPoint>>   middle_points_polygon_;          //将增点后的多边形点位开始排序处理，第一个点是入口点
//     std::vector<std::vector<Point>>   updatedPolygonPointsSequence_;        //按照给定点为起始点存储多边形,这里都不包含重复点
     std::vector<boost::geometry::model::multi_polygon<polygon>>     storageNarrowPolygonPoints2_; //同上
     polygonPoint min_polygon_centroid_;                             //缩小的最小多边形的质心
     int  count_narrow_polygon_numbers_ = 0;                         //统计一共获取多少个缩小的多边形
     std::vector<polygonPoint>   polygonIntersectionPoints_ ;        //线段与多边形们的交点(回字形的进入方向)
     std::vector<lineKb>  k_b_data_;                                 //记录线段定义
     std::unordered_map<int,lineOrignPolygon> lineOriginPolygonInfo_;//记录多边形对应的顶点的各种信息
     std::vector<Point>  line_origin_entrance_;                      //记录原始的回字形入口:[1]为多边形顶点
     std::vector<std::vector<polygonPoint>>  backShape_keypoints_;    //回字形的关键点位信息[第几垄][对应的关键点位们]
     std::vector<std::vector<polygonPoint>>  filtered_backshape_keypoints_;  //过滤后的关键点信息
     std::unordered_map<polygonPoint,ridgeKeypoint,polyPointHash>  backshape_keypts_info_; //获取关键点的映射信息
     std::vector<pathInterface::pathPoint>      ridge_routing_points_;           //每垄的routing信息

 };
}
}

#endif // PATH_POLYGONPLAN_H
