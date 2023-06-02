//
// Created by zzm on 2023/5/18.
//

#ifndef MAXRECTANGLE_OPENCV_H
#define MAXRECTANGLE_OPENCV_H
#include <easylogging++.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include "Geometry/innerRect.h"
#include "common/common_parameters.h"

using namespace std;

class maxRectangle{
public:
    maxRectangle() = default;
    virtual ~maxRectangle();
    void  process(std::vector<cv::Point> & initialPoints);
    void  createOpencvPng(std::vector<cv::Point> points);

    std::vector<cv::Point2f>  getMaxRecPoints();
    unsigned long long  getXGCSMin();
    unsigned long long  getYGCSMin();
    std::vector<cv::Point>  getCoordinateNarrowingPoints();
    double get_point_min_x( std::vector<cv::Point>& points );
    double get_point_min_y( std::vector<cv::Point>& points );
private:
    std::vector<cv::Point2f>  storageRecPoints_;                      //存储生成的矩形坐标点
    std::vector<cv::Point>  coordinateNarrowing_;                     //获取缩小后坐标系的点
    unsigned long long x_GCS_min_;                                    //用于坐标转换的基x
    unsigned long long y_GCS_min_;                                    //用于坐标转换的基y
};

#endif //MAXRECTANGLE_OPENCV_H
