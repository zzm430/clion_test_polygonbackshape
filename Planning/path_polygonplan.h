#ifndef PATH_POLYGONPLAN_H
#define PATH_POLYGONPLAN_H

#include <vector>
#include <cmath>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include "path_baseplan.h"
#include "common/common_parameters.h"
namespace aiforce{
namespace Route_Planning
{
    typedef double coordinate_type;
    typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
    typedef boost::geometry::model::polygon<point> polygon;
    typedef boost::geometry::model::linestring<point> linestring_type;

    class  polygonPoint: public Point{
    public:
        polygonPoint() = default;
        virtual  ~polygonPoint() = default;

    private:
        int origin_polygon_point_index_;   //对应多边形起始点的索引号

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
 private:

 public:
     virtual void computeNarrowPolygons(std::vector<Point> & points); //计算缩小后的轮廓点信息
     virtual void computePolygonsAndLineNode(Point node);             //计算多边形与线段的交点
     virtual void computePolygonsAndLineNode(std::vector<Point> linePoints); //计算多边形与线段的交点
     virtual void computeEveryKbLine();                               //计算kbline
     virtual void judgePointPosition(int i,                           //判断点位信息
                                     boost::geometry::model::multi_polygon<polygon> result);
     virtual void computeOriginPolygonKbline(std::vector<Point> &points);
     virtual void findSuitableEntrance(std::vector<Point> points );
     const std::vector<std::vector<polygonPoint>> getNarrowPolygonPoints() const; //得到缩小后多边形点位信息
     const polygonPoint  getMinPolygonCentroid() const;               //得到最小多边形的质心
     const int  countNarrowPolygonNumbers() const;                    //得到缩小后的多边形数目
     const std::vector<polygonPoint> getPolygonAndLineNodes() const;  //得到多边形和线段交点
     const std::vector<Point> getLineOriginEntrance() const;          //得到原始的回字形入口

 private:
     std::vector<std::vector<polygonPoint>>   storageNarrowPolygonPoints_; //存储缩小的多边形点位
     std::vector<boost::geometry::model::multi_polygon<polygon>>     storageNarrowPolygonPoints2_; //同上
     polygonPoint min_polygon_centroid_;                             //缩小的最小多边形的质心
     int  count_narrow_polygon_numbers_ = 0;                         //统计一共获取多少个缩小的多边形
     std::vector<polygonPoint>   polygonIntersectionPoints_ ;        //线段与多边形们的交点(回字形的进入方向)
     std::vector<lineKb>  k_b_data_;                                 //记录线段定义
     std::unordered_map<int,lineOrignPolygon> lineOriginPolygonInfo_;//记录多边形对应的顶点的各种信息
     std::vector<Point>  line_origin_entrance_;                      //记录原始的回字形入口:[1]为多边形顶点

 };
}
}

#endif // PATH_POLYGONPLAN_H
