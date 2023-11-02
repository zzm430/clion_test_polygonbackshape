//
// Created by zzm on 2023/10/19.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
#include <common/utilpath/path_polygonPoint.h>
#include "common/common_param/common_parameters.h"
#include <iostream>
#include "common/math/common_math.h"

enum  class PARTTYPE :uint8_t{
    REQUIRED_PART_1  = 0,      //仅仅需要第一部分路径
    REQUIRED_PART_1_2 = 1,     //需要第一、二部分路径
    REQUIRED_PART_1_2_3 = 2    //需要第一、二、三部分路径
};

class cornerTuringFTCPACVAlgorithm{
public:
    cornerTuringFTCPACVAlgorithm() =default;
    virtual ~cornerTuringFTCPACVAlgorithm() = default;
    cornerTuringFTCPACVAlgorithm(
                            const  double angleInt,
                            const polygonPoint referencePt,
                            const double arriveLineHeading,
                            const polygonPoint pathStartPt,
                            const polygonPoint pathEndPt,
                            const std::vector<polygonPoint> arriveLine,
                            const std::vector<polygonPoint> leaveLine);
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
            std::vector<polygonPoint> & storageCurvePath);    // 映射到局部坐标系中、全局坐标系下
    void reprojectionGlobalPts(std::vector<polygonPoint> & pts);
    std::vector<polygonPoint> getPathAboutPart1();
    std::vector<polygonPoint> getPathAboutPart2();
    std::vector<polygonPoint> getPathAboutPart3();
    std::vector<polygonPoint> getPathAboutCurve();
    std::vector<polygonPoint> getPathAboutALL();   //返回的是直道+弯道+弯道前进后腿属性的所有点的信息
    void SelectTheRequiredParts();
    std::vector<polygonPoint>  getPathStraight1();
    std::vector<polygonPoint>  getPathStraight2();
    void computePathAboutStraightLine();
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
    double arriveLineHeading_;
    PARTTYPE parttype_;
    polygonPoint referencePt_;
    std::vector<polygonPoint>  storageCurvePath_;    //只包含弯道
    std::vector<polygonPoint>  storageCurvePathPart1_;
    std::vector<polygonPoint>  storageCurvePathPart2_;
    std::vector<polygonPoint>  storageCurvePathPart3_;
    std::vector<polygonPoint>  storaveCurvePathAll_;  //包含所有的弯道的点位和直道的信息
    //两条直道相关参数
private:
    polygonPoint  pathStartPt_;  //全局坐标系下
    polygonPoint  pathEndPt_;    //全局坐标系下
    std::vector<polygonPoint>  pathStraight1_;   //直道倒车部分1
    std::vector<polygonPoint>  pathStraight2_;   //直道倒车部分2
    polygonPoint  curveStartPt_;  //FT-CPA-CV算法弯道起始点统一Part1\Part12\Part123
    polygonPoint  curveEndPt_;    //FT-CPA-CV算法弯道结束点统一Part1\Part12\Part123

    //更新弯道起始点和结束点
private:
   std::vector<polygonPoint>  arriveLine_;
   std::vector<polygonPoint>  leaveLine_;



};

#endif //POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ALGORITHM_H
