//
// Created by zzm on 2023/4/25.
//

#ifndef UNTITLED4_ROTATIONMATRIX_TRANSLATE_H
#define UNTITLED4_ROTATIONMATRIX_TRANSLATE_H
#include <vector>
#include <math.h>
#include <cmath>
#include <iostream>
#include <easylogging++.h>
#include "Planning/path_common.h"
using namespace std;


class rotationMatrixAndTranslate{
public:
    rotationMatrixAndTranslate();
    ~rotationMatrixAndTranslate();
    void processInverse(std::vector<Point> & initial_points);                 //矩阵的还原
    void process(std::vector<Point> & initial_points);                        //矩阵旋转加平移
    std::vector<Point>  getRotationMartixTranslatePoints();
    std::vector<Point>  getOriginalMartixTranslatePoints();
    std::vector<Point>  postProcess(std::vector<Point> & initial_points);     //矩阵坐标点按照逆时针进行排序
private:
    std::vector<Point>  rotationedMartixTranslatePoints_;                     //矩阵坐标点的旋转
    std::vector<Point>  originalMartixTranslatePoints_;                       //矩阵坐标点的还原
private:
    double angle_;                                                            //矩阵旋转的角度
    Point  translate_point_;                                                  //矩阵平移量
public:
    std::vector<Point> processOtherInverse(std::vector<Point> & initial_points); //GCS坐标点的获取
};
#endif //UNTITLED4_ROTATIONMATRIX_TRANSLATE_H
