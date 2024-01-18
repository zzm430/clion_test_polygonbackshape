//
// Created by zzm on 24-1-16.
//

#ifndef POLYGONBACKSHAPE_TRAILERCENTERPTCALCULATE_H
#define POLYGONBACKSHAPE_TRAILERCENTERPTCALCULATE_H

#include <iostream>
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/math/common_math.h"

class trailerCenterPtCalculate {
public:
    trailerCenterPtCalculate() = default;
    virtual ~trailerCenterPtCalculate() = default;
    trailerCenterPtCalculate(const polygonPoint & tractor_center_pt);
    void transTractorPtToTrailerPt();
    polygonPoint  getTrailerCenterPt();
private:
    polygonPoint   tractor_center_pt_;
    polygonPoint   trailer_center_pt_;
};

#endif //POLYGONBACKSHAPE_TRAILERCENTERPTCALCULATE_H
