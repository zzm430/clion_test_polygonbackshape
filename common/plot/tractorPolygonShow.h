//
// Created by zzm on 2023/10/13.
//

#ifndef POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
#define POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
#include <iostream>
#include <common/utilpath/path_polygonPoint.h>
#include <common/common_param/common_parameters.h>
#include <common/math/common_math.h>

class tractorPolygonShow {
public:
    tractorPolygonShow() = default;
    virtual  ~tractorPolygonShow() = default;
    tractorPolygonShow(int Index, double arriveLineHeading,std::vector<polygonPoint> orginCurvePathPts);
    void computeLocalTractorPolygonPts();
    void transferTractorPolygonPts();
    std::vector<polygonPoint> getTractorPolygonHeadPts();
    std::vector<polygonPoint> getTractorPolygonTailPts();
public:
    std::vector<polygonPoint>   orginCurvePathPts1_;
    std::vector<polygonPoint>   orginCurvePathPts2_;
private:
    int  index_;
    double    arriveLineHeading_;                   //旋转方向
    std::vector<polygonPoint>  tractorPolyHeadPts_;  //拖拉机头部点位
    std::vector<polygonPoint>  tractorPolyTailPts_;  //拖拉机尾部点位
    std::vector<polygonPoint>  localPolyPts_;        //
    std::vector<polygonPoint>  orginCurvePathPts_;  //原始弯道点位信息

};

#endif //POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
