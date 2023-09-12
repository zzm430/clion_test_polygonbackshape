//
// Created by zzm on 23-9-11.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#include <vector>
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/print/normaPrint.h"


class cornerTuringTishNail{
public:
    cornerTuringTishNail() = default;
    virtual ~cornerTuringTishNail();
    void computeCircleCenterPt(
            polygonPoint A,
            polygonPoint B,
            double angleInt,
            double RC2);
    void cornerTuringPath( polygonPoint A,
                           polygonPoint B,
                           double RC2);

private:
    std::vector<polygonPoint>   storage_circle_center_;  //存储c1、c2、c3的圆心

};
#endif //POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H

