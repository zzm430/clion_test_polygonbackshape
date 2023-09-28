//
// Created by zzm on 23-9-22.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_C_CPA_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_C_CPA_ALGORITHM_H

#include <common/utilpath/path_polygonPoint.h>
#include "common/common_param/common_parameters.h"


class cornerTuringCCPAAlgorithm{
public:
    cornerTuringCCPAAlgorithm() = default;
    ~cornerTuringCCPAAlgorithm() = default;
    cornerTuringCCPAAlgorithm(
            double angleInt,
            double Rsw,
            double Nswath,
            bool arriveAndLeaveAngleType,
            polygonPoint fieldCornerPt,
            polygonPoint referencePt,
            double arriveLineHeading);
    void calculateAngleCC2();
    void calculateCirclesCenter();
    void calculateNewFieldBorder();
    void calculatePath();
    void correctForCornerOrientation(std::vector<polygonPoint> & vecPts);
    void reprojectionCCA(std::vector<polygonPoint> & pts);
    std::vector<polygonPoint>  getAllPath();
    polygonPoint  getCircleC1Center();
    polygonPoint  getCircleC2Center();
    polygonPoint  getCircleC3Center();
    polygonPoint  getCircleCVCenter();
private:

    double angleInt_;
    double angleOC_;
    double angleC_;
    double Rsw_;
    double angleCC2_;
    double Nswath_;                       //垄号，从内往外数
    polygonPoint   circleC1_center_;
    polygonPoint   circleC2_center_;
    polygonPoint   circleC3_center_;
    polygonPoint   circleCV_center_;
    bool   arriveAndLeaveAngleType_;       //true为凹角 ,false为凸角
    polygonPoint   fieldCornerPt_;         //映射的原始坐标点
    polygonPoint   referencePt_;           //用于从局部更新到全局的基准点
    double   arriveLineHeading_;
    std::vector<polygonPoint>  storage_allPath_;  //存储世界坐标系下path信息
};





#endif //POLYGONBACKSHAPE_CORNERTURING_C_CPA_ALGORITHM_H
