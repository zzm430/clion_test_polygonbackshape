//
// Created by zzm on 23-10-10.
//
//FT-CPA-CC算法,适用于凹角连续弯道
#ifndef POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CC_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CC_ALGORITHM_H

#include <iostream>
#include <common/utilpath/path_polygonPoint.h>
#include "common/common_param/common_parameters.h"
#include "common/math/common_math.h"
#include "Geometry/cornerTuring_location.h"

class cornerTuringFTCPACCAlgorithm {
public:
    cornerTuringFTCPACCAlgorithm() = default;
    ~cornerTuringFTCPACCAlgorithm() = default;
    cornerTuringFTCPACCAlgorithm(std::vector<polygonPoint> arriveLine,
                                 std::vector<polygonPoint> leaveLine);
    std::vector<polygonPoint>  getPath();                  //此path返回的是全局坐标下的path
    void reprojectionFTCPACC(std::vector<polygonPoint> & pts);

private:
    double angleInt_;
    std::vector<polygonPoint>  arriveLine_; //arrive线段
    std::vector<polygonPoint>  leaveLine_;  //leave线段
    double arriveLineHeading_;
    polygonPoint referenctPt_;
    std::vector<polygonPoint>  storageAllPath_;  //存储路径点
    int   F2_;
};

#endif //POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CC_ALGORITHM_H
