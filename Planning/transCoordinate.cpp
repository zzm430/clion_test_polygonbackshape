//
// Created by zzm on 2023/4/24.
//

#include "transCoordinate.h"

transCoordinate::transCoordinate() {

}

transCoordinate::~transCoordinate() {

}

//其中第一个坐标放入的是坐标原点的基
void  transCoordinate::computeRelativePoints( std::vector<Point> & points){
    double x_min,x_max,y_min,y_max;
    std::vector<double> x_value,y_value;
    for(auto i : points){
        x_value.push_back(i.x);
        y_value.push_back(i.y);
    }
    x_min = *std::min_element(x_value.begin(),x_value.end());
    x_max = *std::max_element(x_value.begin(),x_value.end());
    y_min = *std::min_element(y_value.begin(),y_value.end());
    y_max = *std::max_element(y_value.begin(),y_value.end());

    Point firstPoint;
    firstPoint.x = x_min;
    firstPoint.y = y_min;
    relativePoints_.push_back(firstPoint);
    for(auto i : points){
        Point   tempP;
        tempP.x = i.x - x_min;
        tempP.y = i.y - y_min;
        relativePoints_.push_back(tempP);
    }

}

//约定第一个点是坐标基点
 void transCoordinate::computeOriginalPoints(std::vector<Point> & points){
    double first_x,first_y;
    first_x = points[0].x;
    first_y = points[0].y;

    for(int i = 1 ; i<points.size();i++){
        Point tempPoint;
        tempPoint.x = points[i].x + first_x;
        tempPoint.y = points[i].y + first_y;
        OriginalPoints_.push_back(tempPoint);
    }
}
