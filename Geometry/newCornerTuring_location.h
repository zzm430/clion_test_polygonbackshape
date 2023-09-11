//
// Created by zzm on 2023/9/6.
//

#ifndef POLYGONBACKSHAPE_NEWCORNERTURING_LOCATION_H
#define POLYGONBACKSHAPE_NEWCORNERTURING_LOCATION_H
#include <boost/geometry/algorithms/intersection.hpp>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include "Planning/path_polygonplan.h"
#include "common/math/common_math.h"
#include "common/print/filePrint.h"
#include "common/print/filePrint2.h"
#include "common/designPattern/singleton.hpp"

typedef boost::geometry::model::d2::point_xy<double> pointm;
typedef boost::geometry::model::segment<pointm> segmentm;

enum  class curveAngleType :uint8_t{
    ANGLE_ONE  = 0,
    ANGLE_TWO = 1,
    ANGLE_THREE = 2,
    ANGLE_FOUR = 3
};

class newCornerTuringLocation{
public:
    newCornerTuringLocation() = default;
    newCornerTuringLocation(
            std::vector<polygonPoint> arriveLine,
    std::vector<polygonPoint> leaveLine);
    void computeLpAandLpB(
            std::vector<polygonPoint> arriveLine,
            std::vector<polygonPoint> leaveLine);
    void computeBPt(    std::vector<polygonPoint>& arriveLine,
                        std::vector<polygonPoint>& leaveLine);
    void computeAPt(    std::vector<polygonPoint>& arriveLine,
                        std::vector<polygonPoint>& leaveLine);
    void computeAPt2(    std::vector<polygonPoint>& arriveLine,
                        std::vector<polygonPoint>& leaveLine);
    void computeBPt2(std::vector<polygonPoint>& arriveLine,
                                              std::vector<polygonPoint>& leaveLine);
    virtual  ~newCornerTuringLocation();
    polygonPoint  getCurveStartAPt();
    polygonPoint  getCurveEndBPt();

private:
    curveAngleType angleType_;
    double angleInt_;
    polygonPoint  startA_;
    polygonPoint  endB_;
};
#endif //POLYGONBACKSHAPE_NEWCORNERTURING_LOCATION_H



