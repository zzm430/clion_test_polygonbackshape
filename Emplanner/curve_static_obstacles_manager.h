//
// Created by zzm on 23-11-27.
//

#ifndef POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H
#define POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H

#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/common_param/common_typedef.h"
#include "common/math/common_math.h"
#include "smooth/customFemAlgorithm/fem_pos_smooth_manager.h"
#include "common/utilpath/path_slPoint.h"
#include "Emplanner/frenet_converter.h"
#include "common/utilpath/discretizedPath.h"
#include "Emplanner/customPJPO.h"

class curveStaticObstaclesManager{
public:
    curveStaticObstaclesManager() = default;
    virtual ~curveStaticObstaclesManager() = default;

    curveStaticObstaclesManager(std::vector<std::vector<polygonPoint>>  & originPath,
                                std::vector<std::vector<polygonPoint>> &polygonPts);


private:
    std::vector<std::vector<polygonPoint>>  three_custom_referencePts_;  //三段定制参考线

};
#endif //POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H
