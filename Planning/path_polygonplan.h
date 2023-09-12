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
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/intersections.h>
#include <common/utilpath/path_interface.h>
#include <Geometry/reeds_shepp.h>
#include "common/common_param/common_parameters.h"
#include <common/plot/plotter.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>
#include <numeric>
#include <list>
#include <boost/shared_ptr.hpp>
#include <common/print/print.h>
#include <unordered_set>
#include <unordered_map>
#include "common/utilpath/path_polygonPoint.h"
#include "Geometry/cornerTuring_location.h"
#include "Geometry/newCornerTuring_location.h"
#include "Geometry/cornerTuring_TishNail_Algorithm.h"

namespace aiforce{
namespace Route_Planning
{
    typedef double coordinate_type;
    typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
    typedef boost::geometry::model::polygon<point> polygon;
    typedef boost::geometry::model::linestring<point> linestring_type;
    typedef boost::geometry::model::segment<point> Segment;

    struct Kernel : public CGAL::Cartesian<double> {};

    typedef Kernel::Point_2                           Point_2;
    typedef Kernel::Line_2                            Line_2;
    typedef Kernel::Segment_2                         Segment_2;
    typedef CGAL::Polygon_2<Kernel>                   Polygon_2;

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
    typedef K::FT                             cgal_FT ;
    typedef K::Point_2                        cgal_Point ;
    typedef CGAL::Polygon_2<K>                cgal_Polygon_2 ;
    typedef CGAL::Straight_skeleton_2<K>      cgal_Ss ;
    typedef boost::shared_ptr<cgal_Ss>        cgal_SsPtr ;
    typedef K::Segment_2                      cgal_Segment_2;
    typedef Kernel::Bounded_side              cgal_Bounded_side;
    typedef CGAL::Convex_hull_traits_adapter_2<K,
            CGAL::Pointer_property_map<cgal_Point>::type > Convex_hull_traits_2;

    typedef boost::shared_ptr<cgal_Polygon_2> cgal_PolygonPtr ;
    typedef std::vector<cgal_PolygonPtr>      cgal_PolygonPtrVector ;

    typedef CGAL::Partition_traits_2<K>                         Traits;
    typedef Traits::Point_2                               partition_Point_2;
    typedef Traits::Polygon_2                             partition_Polygon_2;      // a polygon of indices
    typedef std::list<partition_Polygon_2>                partition_Polygon_list;
    //半边数据结构
    typedef CGAL::HalfedgeDS_default<K> cgal_HalfedgeDS;


    enum class lastPolyIdentify:uint8_t {
       POLY_FOUR_AND_THREE = 0,  //四边形内嵌三角形
       POLY_FOUR_AND_FOUR,       //四边形内嵌四边形
       POLY_FOUR,                //四边形
       POLY_FIVE                 //五边形
    };

    enum class cgalLastPolyIdentify:uint8_t {
        POLY_NONE = 0,      //标志着入口线段们与最后一笼相交
        POLY_ONLY_ONE = 1,  //标志着入口线段们未与最后一笼相交
        POLY_LEAVE  = 2,    //标志着入口线段们至少有两笼未相交
        POLY_LESS_THR       //入口交点小于最后阈值内无交点
    };

 class pathPolygonPlan  {
 public:
     pathPolygonPlan(){}
     virtual ~pathPolygonPlan(){}

//     virtual void initiate();
    struct lineKb{
         double count;       //记录有多少个点在这个线段的延长线上
         double distance;    //两点之间间隔
         double index;       //对应多边形的顶点的序号
         Point  fixedPt;     //线段初始的位置
         double dx;          //方向向量x分量
         double dy;          //方向向量y分量
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
     virtual void dealWithLastSeveralPolygons();   //根据最后几垄的情况分类处理
     virtual void computeLastRidgePoints4and3();     //提供用于处理最后几垄的关键点4边行内嵌3角形信息
     virtual void computeLastRidgeKeyPoints4and3(int count_flag);  //计算得到最后几垄的关键点4边行内嵌3角形信息
     virtual void computeLastRidgePoints4and4();     //提供用于处理最后的4边形内嵌4边形信息
     virtual void computeLastRidgeSituation();       //处理最后几垄信息
     virtual void computeLastRidgeKeyPoints4and4(int count_flag);   //计算得到最后几垄的关键点4边行内嵌4边行信息
     virtual void computeLastRidgePoints4();         //处理最后一垄是四边行的情况
     virtual void computeLastRidgeKeyPoints4();      //处理最后一垄是四边形的关键点
     virtual void computeLastRidgeRoutingFour(std::vector<pathInterface::pathPoint> &storageAllPath,
                                                       int ridge_index);  //处理最后一垄是四边形的path
     virtual  void   computeLastRidgeRoutingFourAndFour(std::vector<pathInterface::pathPoint> &storageAllPath,
                                                        int ridge_index);
     virtual void  computeKeypointsHeading();        //计算最后一笼点位的heading
     virtual void computeLastRidgeRoutingFourAndThree(std::vector<pathInterface::pathPoint> &storageAllPath,
                                                      int ridge_index); //计算最后一笼属于四边形内嵌三角形的情况path生成



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
     static  std::vector<polygonPoint> insertPointToPolygon(polygonPoint insertPoint,          //将点插入到多边形上，前提是该点在这个多边形上
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

 private:                                                              //用于4边形内嵌3角形
     std::vector<std::vector<polygonPoint>>   last_several_polygons_;  //最后几垄特殊处理
     lastPolyIdentify       last_polys_type_;                          //最后几垄的类别
     int  record_poly_index_;                                          //记录特定内缩四边形的索引
     std::vector<polygonPoint>      last_ridge_AC_pts_;       //用于最后一笼的AC点位信息
     std::vector<polygonPoint>      last_ridge_BC_pts_;       //用于最后一笼的BC点位信息
     std::vector<polygonPoint>      move_last_ridge_AC_pts_;  //平移后的最后一笼的AC点位信息
     std::vector<polygonPoint>      move_last_ridge_BC_pts_;  //平移后的最后一笼的BC点位信息
     std::vector<polygonPoint>      line_long_AB_;            //三角形对应的最长边
     std::vector<polygonPoint>      move_pts_tangle_line_1_;  //线段的交点1
     std::vector<polygonPoint>      move_pts_tangle_line_2_;  //线段的交点2

 private:                                                     //用于4边形内嵌4边形
     std::vector<polygonPoint>       line_rec_short_BA_;      //矩形短边BA上的关键点
     std::vector<polygonPoint>       line_rec_short_CD_;      //矩形短边CD上的关键点
     std::vector<polygonPoint>       line_rec_short_AB_;      //矩形角点信息
     std::vector<polygonPoint>       line_rec_short_DC_;      //矩形角点信息

 private:                                                    //用于4边行
     std::vector<polygonPoint>       line_middle_;           //四边形中线

 private:                                                    //用于四边形内嵌四边形
     std::vector<polygonPoint>  move_pts_line_1_;            //线段的交点1
     std::vector<polygonPoint>  move_pts_line_2_;            //线段的交点2
     std::vector<polygonPoint>  move_pts_line_B_1_;            //线段的交点1 B
     std::vector<polygonPoint>  move_pts_line_B_2_;            //线段的交点2 B
     std::vector<std::vector<polygonPoint>> move_pts_line_B_all_1_; //所有的线段的交点1 B
     std::vector<std::vector<polygonPoint>> move_pts_line_B_all_2_; //所有的线段的交点2 B

 public:
     void cgalNarrowPolygons(std::vector<Point> &points);
     void computeLastRidgeInnerPoints( std::vector<polygonPoint> & points);
     void cgalUpdatePolygonPointsINcrease();
     void cgalUpatePolygonPointsSequence();
     void cgalComputebackShapeKeypoints();
     void cgalComputeAKeyptsMapping();        //只针对回字形的关键点位的映射处理
     void cgalComputeParallelLinesHeading(std::vector<polygonPoint> & lastRidgePts);  //计算平行线的点位的heading
     void judgeIncreaseskeleton(std::vector<polygonPoint> &  inner_polypts );
     std::vector<std::vector<polygonPoint>>  cgalGetBackShapeKeyPoints();
     std::vector<polygonPoint> cgalGetBackShapeSkeletonPts();
     void cgalIncludeLastskeletonMap();
     std::vector<pathInterface::pathPoint> cgalComputeRidgeRoutingpts(int ridge_index);  //根据垄号获取routing
     void cgalComputeLastRidgeRoutingParallelLines(std::vector<pathInterface::pathPoint> &storageAllPath);
     void computeEntranceLines(std::vector<Point> &points);
     void judgePolysSample(int* record_spilt_index,bool&  have_spilt_poly);
     void spiltPolyTo2(
             bool have_spilt_poly,
             std::vector<polygonPoint>  &spilt_origin_poly,
             polygon& spilt_ori,
             double&  buffer_distance,
             std::vector<std::vector<polygonPoint>>&  storage_spilt_first_polys,
             std::vector<std::vector<polygonPoint>>& storage_spilt_second_polys,
             point&                            spilt_first_poly_center);
     void computeLeaveSituation(int last_ordered_poly_index);
     void computeLeaveSituation();
     void computeLeaveSituation(std::vector<polygonPoint> & origin_poly);                       //将B中的分裂多边形按照平行线处理
     void cgalComputeRidgeKeyPointsLeave();               //计算A剩余未相交的按照弓字型处理的关键点
     void cgalComputeBRidgeKeyPointsLeave();              //计算B剩余未相交的按照弓字型处理的关键点
     void processSpiltPolys();           //将分裂多边形分开存储,挑选出A结构的多边形，Bs结构的多边形
 private:
     std::vector<std::vector<polygonPoint>>   cgalPolypts_;         //内缩多边形的存储
     std::vector<std::vector<polygonPoint>>   cgalandboostPolypts_; //cgal和boost混合的内缩多边形存储
     std::vector<std::vector<polygonPoint>>   bufferspiltPolys1_;   //buffer裂变出的第一个多边形
     std::vector<std::vector<polygonPoint>>   bufferspiltPolys2_;   //buffer裂变出的第二个多边形

     std::vector<std::vector<std::vector<polygonPoint>>>   bufferspiltPolysAandBs_; //分裂多边形A和Bs的存储
     std::vector<int>      storage_order_polys_distance_;   //依次存储距离入口的远近多边形的序号

     int cgal_narrow_size_;
     std::vector<polygonPoint>        lastPoly_innerpts_;    //从顶点到直骨架最内部的关键点
     std::vector<polygonPoint>        entrance_lines_;      //回字形的入口线段信息
     std::vector<polygonPoint>        entrance_pts_;        //回字形的入口点位信息
     std::vector<std::vector<polygonPoint>>      cgalIncreaseptPolypts_;  //增加入口点的内缩多边形
     std::vector<std::vector<polygonPoint>>      cgalSequencedPolypts_;   //增加入口点并已入口点为起点存储的内缩多边形
     std::vector<std::vector<polygonPoint>>      cgalbackShape_keypoints_; //回字形的关键点位信息[第几垄][对应的关键点位们]
     std::map<polygonPoint,std::vector<polygonPoint>>    cgalPtMaping_; //直骨架点位映射
     int mode_choose_ = 0;                                      //分裂多边形分裂 mode_choose_ = 1 ,2
     cgalLastPolyIdentify             cgalLastPolyType_;
     int find_entrance_pts_size_ = 0;                        //入口点数
     bool set_flag_about_buffer_spilt_polys_ = false;                //设置是否启用B的平行线相关代码
     bool flag_increase_last_skeleton_ = false;              //判断是否添加最后内部直骨架路径
     std::vector<polygonPoint>  storage_keypts_inner_skeleton_;  //存储最后的内部直骨架路径
     std::unordered_map<polygonPoint,std::vector<polygonPoint>,polyPointHash> last_inner_skeleton_keypts_; //获取关键点的映射信息
 };
}
}
#endif // PATH_POLYGONPLAN_H
