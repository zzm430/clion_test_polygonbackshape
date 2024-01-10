#include <iostream>
#include "PreProcess/getMapData.h"
#include "common/utilpath/path_common.h"
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <string>
#include "easylogging++.h"
#include "common/math/common_math.h"
#include "Planning/path_polygonplan.h"
#include "Decision/curveDecisionType.h"
//#include "Geometry/innerRect.h"
#include "Emplanner/customPJPO.h"

//#include "Geometry/cornerTuring_location.h"
#include "Geometry/newCornerTuring_location.h"

#include "Emplanner/curve_static_obstacles_manager.h"
#include "HybridAStar/hybrid_AStar_test.h"

namespace bg = boost::geometry;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
    searchAlgorithm::hybridAStarTest hybridAStarTest1;
    hybridAStarTest1.test();
    std::cout <<" adfdsf!" << std::endl;

    return 0;
    //获取到原始点位信息
    std::cout << "the argc number is :" << argc  << std::endl;
    std::cout << "the argv frist is :" << *argv <<  std::endl;
    std::string loadPath;

    el::Configurations defaultConf;
    defaultConf.setToDefault();
    getMapData  getMapPoints;
    if(argc == 1){
        getMapPoints.loadMapOuter();
    } else {
        loadPath = argv[1];
        std::cout << "the argv is :" << argv[1]<<std::endl;
        getMapPoints.loadMapOuter(loadPath);
    }
    getMapPoints.updatepolygonSequence();
    std::vector<Point> originalPoints = getMapPoints.getMapUpdatedOuter();

    std::vector<Point>  narrowingPolygonPoints;

    double  value_x = common::commonMath::get_point_min_x(originalPoints);
    double  value_y = common::commonMath::get_point_min_y(originalPoints);

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
    instance_pathPolygonPlan.initialize();
    instance_pathPolygonPlan.cgalNarrowPolygons(narrowingPolygonPoints);

    Point temp_point;
    temp_point.x = (narrowingPolygonPoints[0].x + narrowingPolygonPoints[1].x)/2;
    temp_point.y = (narrowingPolygonPoints[0].y + narrowingPolygonPoints[1].y)/2;


    instance_pathPolygonPlan.cgalUpdatePolygonPointsINcrease();
    instance_pathPolygonPlan.cgalUpatePolygonPointsSequence();
    instance_pathPolygonPlan.cgalComputebackShapeKeypoints();


    auto keypoints_m = instance_pathPolygonPlan.cgalGetBackShapeKeyPoints();
    LOG(INFO) << "the all ridge number is : " << keypoints_m.size();
    std::ofstream keypoints;
    keypoints.open("/home/zzm/Desktop/test_path_figure-main/src/keypoints.txt",std::ios::out);
    int all_size = keypoints_m.size();
    int m ;
    m = 0;
    auto m_skeleton =  instance_pathPolygonPlan.cgalGetBackShapeSkeletonPts();

    for(auto i  = m  ;i < all_size ;i++){
         for(auto j : keypoints_m[i]){
             keypoints << " " << j.x;
         }
    }
    if(!m_skeleton.empty()){
        for(auto i  = 0  ;i < m_skeleton.size() ;i++){
            keypoints << " " << m_skeleton[i].x;
        }
    }
    keypoints << std::endl;
    for(auto i = m ;i < all_size ;i++){
        for(auto j : keypoints_m[i]){
            keypoints << " " << j.y;
        }
    }
    if(!m_skeleton.empty()){
        for(auto f  = 0  ;f < m_skeleton.size() ;f++){
            keypoints << " " << m_skeleton[f].y;
        }
        all_size += 1;
    }
    keypoints << std::endl;
    keypoints.close();

    LOG(INFO) << "the program can enter here !";

    //获取cgal版本的routing信息
    std::ofstream  cgal_show_ridge_path;
    cgal_show_ridge_path.open("/home/zzm/Desktop/test_path_figure-main/src/cgal_show_ridge_path.txt",
                         std::ios::out);
    std::vector<pathInterface::pathPoint> cgal_routing_pts;
    std::vector<std::vector<pathInterface::pathPoint>>  cgal_all_path;
    for(auto i  = 0  ; i < all_size ;i++){
        cgal_routing_pts =
                instance_pathPolygonPlan.cgalComputeRidgeRoutingpts(i);
        for (int i = 0; i < (int)cgal_routing_pts.size(); i++) {
            cgal_show_ridge_path << cgal_routing_pts[i].x << " " << cgal_routing_pts[i].y  << std::endl;
        }
        cgal_all_path.push_back(cgal_routing_pts);
    }
    LOG(INFO) << "the program can enter here　2 !";
    return 0;

    //获取routing信息
    std::ofstream  show_ridge_path;
    show_ridge_path.open("/home/zzm/Desktop/reeds_shepp-master/RS_Lib/show_ridge_path12.txt",
                         std::ios::out);
    auto ridges = keypoints_m.size();
    std::vector<pathInterface::pathPoint> routing_pts;
    std::vector<std::vector<pathInterface::pathPoint>>  all_path;
    for(auto i  = 0; i< ridges ;i++){
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
