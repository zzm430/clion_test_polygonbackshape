//
// Created by zzm on 2023/10/13.
//

#ifndef POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
#define POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H

#include <iostream>
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/math/common_math.h"

class tractorPolygonShow {
public:
    tractorPolygonShow() = default;
    virtual  ~tractorPolygonShow() = default;
    tractorPolygonShow(int Index, std::vector<polygonPoint> orginCurvePathPts);
    void computeLocalTractorPolygonPts();
    void transferTractorPolygonPts();
    std::vector<polygonPoint> getTractorPolygonHeadPts();
    std::vector<polygonPoint> getTractorPolygonTailPts();

private:
    int  index_;
    std::vector<polygonPoint>  tractorPolyHeadPts_;  //拖拉机头部点位
    std::vector<polygonPoint>  tractorPolyTailPts_;  //拖拉机尾部点位
    std::vector<polygonPoint>  localPolyPts_;        //
    std::vector<polygonPoint>  orginCurvePathPts_;  //原始弯道点位信息，全局坐标系下

};

#endif //POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
