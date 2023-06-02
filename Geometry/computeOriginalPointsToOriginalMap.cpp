//
// Created by zzm on 2023/4/24.
//

#include "computeOriginalPointsToOriginalMap.h"

computeOriginalMap::computeOriginalMap(){

}
computeOriginalMap::~computeOriginalMap(){

}

void  computeOriginalMap::process(std::vector<Point> points){
    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> bg_point;
    typedef boost::geometry::model::polygon<bg_point> polygon;
    typedef boost::geometry::model::d2::point_xy<double> xy;
    boost::geometry::model::linestring<xy> linexy;
    polygon poly;
    for(int i=0;i < points.size();i++){
        bg_point temp_point(points[i].x,points[i].y);
        boost::geometry::append(poly.outer(),temp_point);
    }

    polygon hull;
    boost::geometry::convex_hull(poly, hull);
    for(auto && i: hull.outer()){
        xy   temp_point(boost::geometry::get<0>(i),boost::geometry::get<1>(i));
        boost::geometry::append(linexy,temp_point);
    }
    // simplify the polygon using Douglas-Peucker algorithm with a tolerance of 0.5 units
    boost::geometry::model::linestring<xy> simplifiedxy;
    boost::geometry::simplify(linexy, simplifiedxy, 30);

    for (std::size_t i = 0; i < simplifiedxy.size(); i++) {
        Point  temp_point;
        temp_point.x = boost::geometry::get<0>(simplifiedxy[i]);
        temp_point.y = boost::geometry::get<1>(simplifiedxy[i]);
        storagePoints_.push_back(temp_point);
    }

}

std::vector<Point>  computeOriginalMap::getOriginalMap(){
    return storagePoints_;
}
