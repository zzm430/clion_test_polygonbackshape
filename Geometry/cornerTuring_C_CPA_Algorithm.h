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
            bool arriveAndLeaveAngleType);
    void calculateAngleCC2();
    void calculateCirclesCenter();
    void calculateNewFieldBorder();
    void calculatePath();
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

};





#endif //POLYGONBACKSHAPE_CORNERTURING_C_CPA_ALGORITHM_H
