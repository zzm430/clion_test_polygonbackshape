//
// Created by zzm on 23-12-1.
//

#ifndef POLYGONBACKSHAPE_PATH_SLPOINT_H
#define POLYGONBACKSHAPE_PATH_SLPOINT_H
#include "common/utilpath/path_polygonPoint.h"

class slPoint : public polygonPoint{
public:
    slPoint() = default;
    virtual ~slPoint() = default;

    double s_;
    double l_;
};
#endif //POLYGONBACKSHAPE_PATH_SLPOINT_H
