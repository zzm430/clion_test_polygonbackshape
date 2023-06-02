//
// Created by zzm on 2023/4/24.
//

#ifndef UNTITLED6_COMPUTEORIGINALPOINTSTOORIGINALMAP_H
#define UNTITLED6_COMPUTEORIGINALPOINTSTOORIGINALMAP_H
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/adapted/boost_tuple.hpp>
#include "Planning/path_common.h"
class computeOriginalMap{
public:
    computeOriginalMap();
    ~computeOriginalMap();
    void  process(std::vector<Point> points);
    std::vector<Point>  getOriginalMap();
private:
    std::vector<Point>   storagePoints_;  //存储过滤后的坐标点
};
#endif //UNTITLED6_COMPUTEORIGINALPOINTSTOORIGINALMAP_H
