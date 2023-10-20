//
// Created by zzm on 2023/10/19.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
#include <common/utilpath/path_polygonPoint.h>
#include "common/common_param/common_parameters.h"
#include <iostream>

enum  class PARTTYPE :uint8_t{
    REQUIRED_PART_1  = 0,      //仅仅需要第一部分路径
    REQUIRED_PART_1_2 = 1,     //需要第一、二部分路径
    REQUIRED_PART_1_2_3 = 2    //需要第一、二、三部分路径
};

class cornerTuringFTCPACVAlgorithm{
public:
    cornerTuringFTCPACVAlgorithm() =default;
    virtual ~cornerTuringFTCPACVAlgorithm() = default;
    cornerTuringFTCPACVAlgorithm(double angleInt);
    void computeLimitPtInPart2() ;
    void computeTheAnglesForFTCPACV();
    void computePath();
    void computePathAboutPart1(double angleStart,double angleEnd);
    void computePathAboutPart2(double angleStart,double angleEnd);
    void computePathAboutPart3(double angleStart,double angleEnd);
    void reprojectionPts(
            std::vector<polygonPoint> & storageCurvePathPart1,
            std::vector<polygonPoint> & storageCurvePathPart2,
            std::vector<polygonPoint> & storageCurvePathPart3,
            std::vector<polygonPoint> & storageCurvePath);    // 映射到局部坐标系中
    std::vector<polygonPoint> getPathAboutPart1();
    std::vector<polygonPoint> getPathAboutPart2();
    std::vector<polygonPoint> getPathAboutPart3();
    std::vector<polygonPoint> getPathAboutAll();
    void SelectTheRequiredParts();
private:
    double angleInt_;
    double dFL_;
    polygonPoint pLIM_;
    double dLIMCC_;
    double angleInt2_;
    double angleP1_;
    double dCCYP1_;
    double angleP2_;
    double dCCX_;
    double angleStart_;
    double angleEnd_;
    double F2_;
    PARTTYPE parttype_;
    std::vector<polygonPoint>  storageCurvePath_;
    std::vector<polygonPoint>  storageCurvePathPart1_;
    std::vector<polygonPoint>  storageCurvePathPart2_;
    std::vector<polygonPoint>  storageCurvePathPart3_;

};

#endif //POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
