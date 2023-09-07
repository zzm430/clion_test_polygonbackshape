//
// Created by zzm on 2023/9/6.
//

#ifndef POLYGONBACKSHAPE_NEWCORNERTURING_LOCATION_H
#define POLYGONBACKSHAPE_NEWCORNERTURING_LOCATION_H
#include "Planning/path_polygonplan.h"
#include "common/math/common_math.h"
#include <boost/geometry/algorithms/intersection.hpp>
#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>

typedef boost::geometry::model::d2::point_xy<double> pointm;
typedef boost::geometry::model::segment<pointm> segmentm;

enum  class curveAngleType :uint8_t{
    ANGLE_ACUTE  = 0,
    ANGLE_OBTUSE = 1
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



