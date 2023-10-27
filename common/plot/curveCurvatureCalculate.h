//
// Created by zzm on 2023/10/27.
//

#ifndef POLYGONBACKSHAPE_CURVECURVATURECALCULATE_H
#define POLYGONBACKSHAPE_CURVECURVATURECALCULATE_H
#include <iostream>
#include <vector>
#include <common/utilpath/path_polygonPoint.h>
#include <common/common_param/common_typedef.h>

class  curveCurvatureCalculate{
public:
    curveCurvatureCalculate() = default;
    virtual ~curveCurvatureCalculate() = default;
    curveCurvatureCalculate(const std::vector<polygonPoint> originPath);
    std::vector<double> getPathPtsCurvature();
    std::vector<double> getPathPtsR();
private:
    std::vector<double>  pathPtsCurvature_;   //路径曲率集
    std::vector<double>  pathPtsR_;           //路径转弯半径集

};
#endif //POLYGONBACKSHAPE_CURVECURVATURECALCULATE_H
