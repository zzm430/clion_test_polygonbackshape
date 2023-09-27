//
// Created by zzm on 23-9-26.
//

#ifndef POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
#define POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
#include <iostream>
#include "Planning/normalMatrix_Translate.h"
#include "Geometry/cornerTuring_Implement_Radius.h"
#include "Geometry/cornerTuring_C_CPA_Algorithm.h"
#include "Geometry/cornerTuring_TishNail_Algorithm.h"
#include "Geometry/cornerTuring_location.h"
#include "common/utilpath/path_polygonPoint.h"
#include "Planning/path_polygonplan.h"

enum  class CurveDecision : uint8_t {
    IDLE = 0,
    BORDERLESS_FISHNAIL = 1,          //无边界鱼尾
    CONVEX_CORNER = 2,                //凸角
    CONCAVE_CORNER = 3,               //凹角
    CC_CURVE = 4 ,                    //连续弯道
    REED_SHEPP = 5,                   //
    FT_CPA_CC  = 6                    //最外层替代无限制鱼尾弯道
};

class curveDecisionManager {
public:
    curveDecisionManager() = default;
    virtual ~curveDecisionManager() = default;
    curveDecisionManager(double angleTwoline);
    void setCurvePara( double angleInt,
                       double ridgeNumber,
                       double ptIndex,
                       std::vector<polygonPoint> arriveLine,
                       std::vector<polygonPoint> leaveLine,
                       aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processCurvePath(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processCurveType();
    void processBorderlessFishNail(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processCCPA(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processFTCPACC(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processCCCURVE(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void processREEDSHEPP(aiforce::Route_Planning::pathPolygonPlan & path_frame);
    void changeState(CurveDecision  state);

private:
    CurveDecision  curveType_ = CurveDecision::IDLE;
    double angleInt_;
    double ridgeNumber_;
    double ptIndex_;
    std::vector<polygonPoint> arriveLine_;
    std::vector<polygonPoint> leaveLine_,
};

#endif //POLYGONBACKSHAPE_CURVEDECISIONMANAGER_H
