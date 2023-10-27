//
// Created by zzm on 2023/10/27.
//
#include "curveCurvatureCalculate.h"

curveCurvatureCalculate::curveCurvatureCalculate(const std::vector<polygonPoint> originPath){
    //计算给出的路径从第二个点到倒数第二个路径点的曲率
    for(int i = 0 ;i <= originPath.size()-3;i++){
        Point1_2 p1( originPath[i].x,  originPath[i].y);
        Point1_2 p2(originPath[i+1].x, originPath[i+1].y );
        Point1_2 p3(  originPath[i+2].x,   originPath[i+2].y);
        //先判断3个点是否共线
        bool isCollinear = CGAL::collinear(p1, p2, p3);
        if(!isCollinear){
            Circle1_2 circle(p1, p2, p3);
            // 获取圆心和半径
            Point1_2 center = circle.center();
            auto squared_radius = circle.squared_radius();
            double radius = std::sqrt(CGAL::to_double(squared_radius));
            pathPtsR_.push_back(radius);
            double k = 1/radius;
            pathPtsCurvature_.push_back(k);
            std::cout << "the radius is : " << radius << " " << to_double(center.x()) << " " << to_double(center.y()) << std::endl;
        }
    }
}

std::vector<double> curveCurvatureCalculate::getPathPtsCurvature(){
    return  pathPtsCurvature_;
}

std::vector<double> curveCurvatureCalculate::getPathPtsR(){
    return pathPtsR_;
}
