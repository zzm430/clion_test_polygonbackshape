//
// Created by zzm on 23-9-26.
//

#ifndef POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
#define POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
#include <iostream>
#include "Planning/normalMatrix_Translate.h"
#include "Geometry/cornerTuring_Implement_Radius.h"
#include "Geometry/cornerTuring_C_CPA_Algorithm.h"
#include "Geometry/cornerTuring_FT_CPA_FS_Algorithm.h"
#include "Geometry/cornerTuring_location.h"
#include "Geometry/cornerTuring_FT_CPA_CC_Algorithm.h"
#include "Geometry/cornerTuring_FT_CPA_CV_ABptLocation.h"
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_typedef.h"
#include "common/common_param/common_parameters.h"
#include "common/plot/tractorPolygonShow.h"
#include "Geometry/cornerTuring_FT_CPA_CV_Algorithm.h"
#include "common/print/tractorPolyPrint2.h"
#include "common/plot/curveCurvatureCalculate.h"
#include "Decision/curveDecisionType.h"


class curveDecisionManager {
public:
    curveDecisionManager() = default;
    virtual ~curveDecisionManager() = default;
//    curveDecisionManager(double angleTwoline);
    curveDecisionManager(double ridgeNumber,
                         double ptIndex,
                         std::vector<polygonPoint> arriveLine,
                         std::vector<polygonPoint> leaveLine,
                         std::vector<std::vector<polygonPoint>>& cgalbackShape_keypoints,
                         std::unordered_map<polygonPoint,std::vector<polygonPoint>,polyPointHash>& backshape_fishnail_curve_path);
    void processCurvePath();
    void processCurveType();
    void processBorderlessFishNail();
    void processBorderlessFishNail(polygonPoint curvePt);
    void processCCPA();
    void processFTCPACC();
    void processFTCPACV();
    void processCCCURVE();
    void processREEDSHEPP();
    double getArriveLineHeading();
    void changeState(CurveDecision  state);

private:
    CurveDecision  curveType_ = CurveDecision::IDLE;
    double angleIntManager_;                        //此角度不可用于算法中的angleInt
    double ridgeNumber_;
    double ptIndex_;
    double arriveLineHeading_;
    std::vector<polygonPoint> arriveLine_;
    std::vector<polygonPoint> leaveLine_;
    std::vector<std::vector<polygonPoint>>&      cgalbackShape_keypoints_; //回字形的关键点位信息[第几垄][对应的关键点位们]
    std::unordered_map<polygonPoint,std::vector<polygonPoint>,polyPointHash>& backshape_fishnail_curve_path_;  //关键点映射到fishnail路径点
};

#endif //POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
