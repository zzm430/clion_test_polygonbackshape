//
// Created by zzm on 23-11-28.
//

#ifndef POLYGONBACKSHAPE_PATH_ANCHORPOINT_H
#define POLYGONBACKSHAPE_PATH_ANCHORPOINT_H
#include "common/utilpath/path_polygonPoint.h"

struct AnchorPoint{
    polygonPoint path_point_;
    double lateral_bound_ = 0.0;
    double longitudinal_bound_ = 0.0;

    //enforce smoother to strictly follow this reference point
    bool enforced = false;
};


#endif //POLYGONBACKSHAPE_PATH_ANCHORPOINT_H
