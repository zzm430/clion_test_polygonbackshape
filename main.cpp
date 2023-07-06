#include <iostream>
#include "PreProcess/getMapData.h"
#include "Planning/path_common.h"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <string>
#include "easylogging++.h"
#include "common/math/common_math.h"
#include "Planning/path_polygonplan.h"
#include "Geometry/innerRect.h"
namespace bg = boost::geometry;



INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
    //获取到原始点位信息
    std::cout << "the argc number is :" << argc  << std::endl;
    std::cout << "the argv frist is :" << *argv <<  std::endl;
    std::string loadPath;

    el::Configurations defaultConf;
    defaultConf.setToDefault();
    getMapData  getMapPoints;
    if(argc == 1){
        getMapPoints.loadMapOuter();
    }else{
        loadPath = argv[1];
        std::cout << "the argv is :" << argv[1]<<std::endl;
        getMapPoints.loadMapOuter(loadPath);
    }
    getMapPoints.updatepolygonSequence();
    std::vector<Point> originalPoints = getMapPoints.getMapUpdatedOuter();

    std::vector<Point>  narrowingPolygonPoints;

    unsigned long long  value_x = common::commonMath::get_point_min_x(originalPoints);
    unsigned long long  value_y = common::commonMath::get_point_min_y(originalPoints);

    for(auto i : originalPoints){
        Point  tempPoint;
        tempPoint.x =  (i.x - value_x) ;
        tempPoint.y =  (i.y - value_y) ;
        narrowingPolygonPoints.push_back(tempPoint);
    }
    std::ofstream  origin_polygon;
    origin_polygon.open("/home/zzm/Desktop/test_path_figure-main/src/origin_polygon.txt",std::ios::out);
    for(auto it : narrowingPolygonPoints){
        origin_polygon << " " << it.x;
    }
    origin_polygon << std::endl;
    for(auto it : narrowingPolygonPoints){
        origin_polygon << " " << it.y;
    }
    origin_polygon << std::endl;
    origin_polygon.close();

    aiforce::Route_Planning::pathPolygonPlan   instance_pathPolygonPlan;
    instance_pathPolygonPlan.computeNarrowPolygons(narrowingPolygonPoints);

    //temp point;
    Point temp_point;
    temp_point.x = (narrowingPolygonPoints[0].x + narrowingPolygonPoints[1].x)/2;
    temp_point.y = (narrowingPolygonPoints[0].y + narrowingPolygonPoints[1].y)/2;
//    instance_pathPolygonPlan.computePolygonsAndLineNode(temp_point);
    auto points = instance_pathPolygonPlan.getNarrowPolygonPoints();
    std::ofstream  polyps;
    polyps.open("/home/zzm/Desktop/test_path_figure-main/src/narrow_points.txt",std::ios::out);
    for(auto it : points ){
       for(auto j : it){
           polyps << " " << j.x ;
       }
    }
    polyps << std::endl;
    for(auto it : points){
        for(auto j : it){
            polyps << " " << j.y;
        }
    }
    polyps << std::endl;
    polyps.close();

    //找到顶点和对应的kbline
    instance_pathPolygonPlan.findSuitableEntrance(narrowingPolygonPoints);

    auto temp_line = instance_pathPolygonPlan.getLineOriginEntrance();

    //得到对应的边[0]:最小的内缩点 [1]:多边形顶点
    std::vector<Point>  storageForwardAndBack =
            common::commonMath::computeForwardAndBackPoints(narrowingPolygonPoints,
                                                            temp_line[1]);
    //another_line direction
    std::vector<Point>  another_line;
#ifdef  JUDGE_CLOCKWISE
        another_line.push_back(temp_line[1]);
        another_line.push_back(storageForwardAndBack[1]);
#else
        another_line.push_back(temp_line[1]);
        another_line.push_back(storageForwardAndBack[0]);
#endif
    //计算平移后的向量
    //计算需要平移的距离
    Point vector_1((storageForwardAndBack[0].x - temp_line[1].x),
                    storageForwardAndBack[0].y - temp_line[1].y);
    Point vector_2((storageForwardAndBack[1].x - temp_line[1].x),
                   (storageForwardAndBack[1].y - temp_line[1].y));

    double move_angle = common::commonMath::computeTwoLineAngle(vector_1,vector_2);
    double move_distance ;
    if(fabs(move_angle) < PI/2){
        move_distance = RIDGE_WIDTH_LENGTH/sin(move_angle);
    }else{
        move_distance = RIDGE_WIDTH_LENGTH;
    }
    LOG(INFO) << "chosen point angle is :" << move_angle * (180 /PI);
    std::vector<Point> real_line =
            common::commonMath::computeLineTranslationPoints(temp_line,
                                                             another_line,
                                                             move_distance);
    std::ofstream  realline;
    realline.open("/home/zzm/Desktop/test_path_figure-main/src/realline.txt",std::ios::out);
    for(auto i : real_line){
        realline << " " << i.x;
    }
    realline << std::endl;
    for(auto j : real_line){
        realline << " " << j.y;
    }
    realline << std::endl;
    realline.close();
    //将计算得到的回字形入口线段延长一段距离
    //auto real_line_extend = common::commonMath::extendLineLength(real_line,0.5);
    //计算平移后的向量和内缩多边形们的交点
    instance_pathPolygonPlan.computePolygonsAndLineNode(real_line);
    auto nodes = instance_pathPolygonPlan.getPolygonAndLineNodes();

    //根据生成的交点和内缩多边形们生成回字形关键点
    std::ofstream interNodes;
    interNodes.open("/home/zzm/Desktop/test_path_figure-main/src/inter_nodes.txt",std::ios::out);
    for(auto node : nodes){
        interNodes << " " << node.x;
    }
    interNodes << std::endl;
    for(auto node : nodes){
        interNodes << " " << node.y;
    }
    interNodes << std::endl;
    interNodes.close();

    instance_pathPolygonPlan.updatePolygonPointsIncrease();
    instance_pathPolygonPlan.updatePolygonPointsSequence1();
    auto m = instance_pathPolygonPlan.getMiddlePointsPolygon();
    std::ofstream  increaseNodes;
    increaseNodes.open("/home/zzm/Desktop/test_path_figure-main/src/increaseNodes.txt",std::ios::out);
    for(auto it : m ){
        for(auto j : it){
            increaseNodes << " " << j.x ;
        }
    }
    increaseNodes << std::endl;
    for(auto it : m){
        for(auto j : it){
            increaseNodes << " " << j.y;
        }
    }
    increaseNodes << std::endl;
    increaseNodes.close();

    instance_pathPolygonPlan.computebackShapeKeypoints();
    instance_pathPolygonPlan.filteredBackShapeKeyPoints();
    instance_pathPolygonPlan.computeKeypointsRelativeInfo();

    auto keypoints_m = instance_pathPolygonPlan.getFilteredBackShapeKeyPoints();

    std::ofstream keypoints;
    keypoints.open("/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt",std::ios::out);
    int all_size = keypoints_m.size();
    for(auto i  = 0;i < all_size;i++){
         for(auto j : keypoints_m[i]){
             keypoints << " " << j.x;
         }
    }
    keypoints << std::endl;
    for(auto i = 0;i < all_size;i++){
        for(auto j : keypoints_m[i]){
            keypoints << " " << j.y;
        }
    }
    keypoints << std::endl;
    keypoints.close();
//    for(auto it : keypoints_m){
//        for(auto j : it){
//            keypoints << " " << j.x;
//        }
//    }
//    keypoints << std::endl;
//    for(auto it : keypoints_m){
//        for(auto j : it){
//            keypoints << " " << j.y;
//        }
//    }
//    keypoints << std::endl;
//    keypoints.close();

   LOG(INFO) << "the program can enter here !";
    //获取routing信息
    std::ofstream  show_ridge_path;
    show_ridge_path.open("/home/zzm/Desktop/reeds_shepp-master/RS_Lib/show_ridge_path12.txt",
                         std::ios::out);
    auto ridges = keypoints_m.size();
    std::vector<pathInterface::pathPoint> routing_pts;
    std::vector<std::vector<pathInterface::pathPoint>>  all_path;
    for(auto i  = 0;i < ridges;i++){
         routing_pts =
                instance_pathPolygonPlan.computeRidgeRoutingpts(i);
        for (int i = 0; i < (int)routing_pts.size(); i++) {
                show_ridge_path << routing_pts[i].x << " " << routing_pts[i].y  << std::endl;
            }
         all_path.push_back(routing_pts);
    }
    show_ridge_path.close();
    std::ofstream  routing_ps;
    routing_ps.open("/home/zzm/Desktop/reeds_shepp-master/RS_Lib/show_ridge_path11.txt",std::ios::out);
    for(auto i : all_path){
        for(auto it : i){
            routing_ps << " " << it.x;
        }
    }
    routing_ps << std::endl;
    for(auto i : all_path){
        for(auto j : i){
            routing_ps << " " << j.y;
        }
    }
    routing_ps << std::endl;
    routing_ps.close();

    LOG(INFO) << "happy ending!" << std::endl;
    return 0;
}
