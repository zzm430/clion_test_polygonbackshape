#include <iostream>
#include "getMapData.h"
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

namespace bg = boost::geometry;



INITIALIZE_EASYLOGGINGPP

int main() {
    //获取到原始点位信息
    el::Configurations defaultConf;
    defaultConf.setToDefault();
    getMapData  getMapPoints;
    getMapPoints.loadMapOuter();
    std::vector<Point> originalPoints = getMapPoints.getMapOuter();

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
    another_line.push_back(temp_line[1]);
    another_line.push_back(storageForwardAndBack[0]);

    //计算平移后的向量
    std::vector<Point> real_line =
            common::commonMath::computeLineTranslationPoints(temp_line,
                                                             another_line,
                                                             RIDGE_WIDTH_LENGTH);

    //计算平移后的向量和内缩多边形们的交点
    instance_pathPolygonPlan.computePolygonsAndLineNode(real_line);


    std::ofstream interNodes;
    interNodes.open("/home/zzm/Desktop/test_path_figure-main/src/inter_nodes.txt",std::ios::out);
    auto nodes = instance_pathPolygonPlan.getPolygonAndLineNodes();
    for(auto node : nodes){
        interNodes << " " << node.x;
    }
    interNodes << std::endl;
    for(auto node : nodes){
        interNodes << " " << node.y;
    }
    interNodes << std::endl;
    interNodes.close();



//    // Declare strategies
//     double buffer_distance = -50;
//   for(auto i = 1;i  <= 16; i++ ) {
//       const int points_per_circle = 0;
//       buffer_distance = i * -4;
//       boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
//       boost::geometry::strategy::buffer::join_round join_strategy(0.01);
////       boost::geometry::strategy::buffer::join_round join_strategy;
//       boost::geometry::strategy::buffer::end_round end_strategy;
//       boost::geometry::strategy::buffer::point_circle circle_strategy;
//       boost::geometry::strategy::buffer::side_straight side_strategy;
//
//       // Declare output
//       boost::geometry::model::multi_polygon<polygon> result;
//
//       boost::geometry::model::multi_polygon<polygon> mpol;
////
//
////    boost::geometry::read_wkt("MULTIPOLYGON(((0,0,0 150,30 150,60 100,80 100,80 170,200 150,200 0, 0,0)))", mpol);
//
//       boost::geometry::read_wkt("MULTIPOLYGON(((20 270,40 270,40 230,60 230,100 300,140 220,200 190,320 70,300 50,150 50,60 140,20 100,20 270)))",mpol);
//       // Create the buffer of a multi polygon
//       boost::geometry::buffer(mpol, result,
//                               distance_strategy, side_strategy,
//                               join_strategy, end_strategy, circle_strategy);
//       point centroid;
//       boost::geometry::centroid(mpol, centroid);
//      std::cout << "the center  is :" << boost::geometry::dsv(centroid) << std::endl;
//      point  centroid_souxiao;
//      boost::geometry::centroid(result,centroid_souxiao);
//      std::cout << "the center suoxiao is :" << boost::geometry::dsv(centroid_souxiao) << std::endl;
//
//       std::ofstream polygon_reduce;
//       std::string temp1 = "/home/zzm/Desktop/test_path_figure-main/src/polygon_reduce" + std::to_string(i) + ".txt";
//       std::string temp2 = "/home/zzm/Desktop/test_path_figure-main/src/polygon_origin" + std::to_string(i) + ".txt";
//       polygon_reduce.open(temp1, std::ios::out);
//       int number_a = 0, number_b = 0;
//       for (auto it = result.begin(); it != result.end(); it++) {
//           for (auto j = it->outer().begin(); j != it->outer().end(); j++) {
//               polygon_reduce << " " << (*j).x();
//               number_a += 1;
//           }
//       }
//       std::cout << std::endl;
//       std::cout << "the bian li ci shu is :" << i << " " << "the ridge is :" << number_a;
//       polygon_reduce << std::endl;
//       for (auto it = result.begin(); it != result.end(); it++) {
//           for (auto j = it->outer().begin(); j != it->outer().end(); j++) {
//               polygon_reduce << " " << (*j).y();
//           }
//       }
//       polygon_reduce << std::endl;
//
//      if(i==1) {
//          std::ofstream polygon_origin;
//          polygon_origin.open(temp2, std::ios::out);
//          for (auto it = mpol.begin(); it != mpol.end(); it++) {
//              for (auto j = it->outer().begin(); j != it->outer().end(); j++) {
//                  polygon_origin << " " << (*j).x();
//              }
//
//          }
//          polygon_origin << std::endl;
//          for (auto it = mpol.begin(); it != mpol.end(); it++) {
//              for (auto j = it->outer().begin(); j != it->outer().end(); j++) {
//                  polygon_origin << " " << (*j).y();
//              }
//
//          }
//          polygon_origin << std::endl;
//      }
//   }
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
