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

#include "smooth/customFemIpoptAlgorithm/fem_pos_deviation_ipopt_manager.h"
#include "smooth/customFemIpoptAlgorithm/fem_pos_deviation_ipopt_param.h"
#include "common/common_param/common_parameters.h"
#include "common/plot/computePathProfile.h"

#include <fstream>
#include <vector>

namespace bg = boost::geometry;

INITIALIZE_EASYLOGGINGPP

int main(int argc, char **argv) {
//    searchAlgorithm::hybridAStarTest hybridAStarTest1;
//    hybridAStarTest1.test();
//    std::cout <<" adfdsf!" << std::endl;
//
//    return 0;
//测试fem_ipopt算法
//    femPosDeviationIpoptParam femPosDeviationIpoptParamINs;
//    femPosDeviationIpoptParamINs.weight_fem_pos_deviation_ipopt  = WEIGHT_FEM_POS_DEVIATION_IPOPT;
//    femPosDeviationIpoptParamINs.weight_path_length_ipopt = WEIGHT_PATH_LENGTH_IPOPT;
//    femPosDeviationIpoptParamINs.weight_ref_deviation_ipopt = WEIGHT_REF_DEVIATION_IPOPT;
//    femPosDeviationIpoptParamINs.weight_curvature_constraint_slack_var_ipopt = WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR_IPOPT;
//    femPosDeviationIpoptParamINs.curvature_constraint_ipopt = CURVATURE_CONSTRAINT_IPOPT ;
//    femPosDeviationIpoptParamINs.print_level_ipopt = PRINT_LEVEL_IPOPT;
//    femPosDeviationIpoptParamINs.max_num_of_iterations_ipopt = MAX_NUM_OF_ITERATIONS_IPOPT;
//    femPosDeviationIpoptParamINs.acceptable_num_of_iterations_ipopt = ACCEEPTABLE_NUM_OF_ITERATIONS_IPOPT;
//    femPosDeviationIpoptParamINs.tol_ipopt = TOL_IPOPT;
//    femPosDeviationIpoptParamINs.acceptable_tol_ipopt = ACCEPTABLE_TOL_IPOPT;
//    smoothalgorithm::femPosIpoptSmoother test(femPosDeviationIpoptParamINs);
//    polygonPoint  pt1(0,0),pt2(15,0),pt3(7.5,7.5);
//    std::vector<polygonPoint>   vec_pts1,vec_pts2;
//    vec_pts1.push_back(pt1);
//    vec_pts1.push_back(pt3);
//    vec_pts2.push_back(pt3);
//    vec_pts2.push_back(pt2);
//    auto ins_pts1 = common::commonMath::densify2(vec_pts1,20);
//    auto ins_pts2 = common::commonMath::densify2(vec_pts2,20);
//    std::vector<std::pair<double,double>> storage_origin_pts;
//    for(auto i : ins_pts1){
//        storage_origin_pts.push_back({i.x,i.y});
//    }
//    for(auto j : ins_pts2){
//        storage_origin_pts.push_back({j.x,j.y});
//    }
//    auto pts_size = ins_pts1.size();
//    std::vector<double> bounds;
//    bounds.push_back(0);
//    for(int i = 1;i < pts_size ;i++){
//        bounds.push_back(10);
//    }
//
//    //读取fem文件中的test文本文件
//    std::ifstream   fileTemp("/home/zzm/Desktop/test_path_figure-main/src/testfemObstaclePath.txt");
//    std::vector<std::vector<double>>  strage_m;
//    std::string line;
//    while(std::getline(fileTemp,line)){
//        std::istringstream  iss(line);
//        double value;
//        std::vector<double> temp;
//        while(iss >> value){
//             temp.push_back(value);
//        }
//        strage_m.push_back(temp);
//    }
//    storage_origin_pts.clear();
//    for(auto i = 0;i < strage_m[0].size();i++){
//        storage_origin_pts.push_back({strage_m[0][i],strage_m[1][i]});
//    }
//
//
//
//    std::vector<double>  opt_x;
//    std::vector<double>  opt_y;
//    test.NlpWithIpopt(storage_origin_pts,bounds,&opt_x,&opt_y);
//
//    //输出曲率信息
//    std::vector<double> headings;
//    std::vector<double> accumulated_s;
//    std::vector<double> kappas;
//    std::vector<double> dkappas;
//    std::vector<std::pair<double,double>>  storage_transed_pts;
//    for(int i  = 0;i < opt_x.size(); i++){
//        storage_transed_pts.push_back({opt_x[i],opt_y[i]});
//    }
//    computePathProfileInfo::ComputePathProfile(
//            storage_transed_pts,
//            &headings,
//            &accumulated_s,
//            &kappas,
//            &dkappas);
//
//    //输出path结果
//    std::ofstream   testfemipopt;
//    testfemipopt.open("/home/zzm/Desktop/test_path_figure-main/src/testfemipopt.txt",std::ios::out);
//    for(auto i : opt_x){
//        testfemipopt << " " << i;
//    }
//    testfemipopt << std::endl;
//    for(auto j : opt_y){
//        testfemipopt << " " << j;
//    }
//    testfemipopt << std::endl;
//
//    //显示原始path
//    std::ofstream   testoriginPath;
//    testoriginPath.open("/home/zzm/Desktop/test_path_figure-main/src/testoriginPath.txt",std::ios::out);
//    testoriginPath << " " << pt1.x << " " << pt3.x << " " << pt2.x << std::endl;
//    testoriginPath << " " << pt1.y << " " << pt3.y << " " << pt2.y << std::endl;
//
//    std::ofstream  testcurveInfo;
//    testcurveInfo.open("/home/zzm/Desktop/test_path_figure-main/src/testcurveInfo.txt",std::ios::out);
//    for(auto i : kappas){
//        testcurveInfo << " " << i;
//    }
//    testcurveInfo << std::endl;
//
//    return 0;

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
