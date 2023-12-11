//
// Created by zzm on 23-12-1.
//

#ifndef POLYGONBACKSHAPE_PATH_SLPOINT_H
#define POLYGONBACKSHAPE_PATH_SLPOINT_H
#include "common/utilpath/path_polygonPoint.h"

class slPoint : public polygonPoint{
public:
    slPoint() : s_(0.0), l_(0.0){};
    slPoint(double s, double l) : s_(s), l_(l){};

    virtual ~slPoint() = default;

    double s() const {
        return s_;
    }
    double l() const {
        return l_;
    }

    void set_s(double s) {
        s_ = s;
    }

    void set_l(double l) {
        l_ = l;
    }

    double s_;
    double l_;
};
#endif //POLYGONBACKSHAPE_PATH_SLPOINT_H
