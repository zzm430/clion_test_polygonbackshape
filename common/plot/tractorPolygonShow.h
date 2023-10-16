//
// Created by zzm on 2023/10/13.
//

#ifndef POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
#define POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
#include <iostream>
#include <common/utilpath/path_polygonPoint.h>
#include <common/common_param/common_parameters.h>

class tractorPolygonShow {
public:
    tractorPolygonShow() = default;
    virtual  ~tractorPolygonShow() = default;
    tractorPolygonShow(polygonPoint referencePt, double arriveLineHeading);
    void computeLocalTractorPolygonPts();
    void transferTractorPolygonPts();
    std::vector<polygonPoint> getTractorPolygonHeadPts();
    std::vector<polygonPoint> getTractorPolygonTailPts();
private:
    polygonPoint referencePt_;                      //当对局部坐标原点
    double    arriveLineHeading_;                   //旋转方向
    std::vector<polygonPoint>  tractorPolyHeadPts_;  //拖拉机头部点位
    std::vector<polygonPoint>  tractorPolyTailPts_;  //拖拉机尾部点位
    std::vector<polygonPoint>  localPolyPts_;        //

};

#endif //POLYGONBACKSHAPE_TRACTORPOLYGONSHOW_H
