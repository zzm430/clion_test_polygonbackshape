//
// Created by zzm on 23-12-8.
//

#ifndef POLYGONBACKSHAPE_PT_SLBOUNDARY_H
#define POLYGONBACKSHAPE_PT_SLBOUNDARY_H

#include "common/utilpath/path_slPoint.h"

class SLBoundary {
public:
    SLBoundary() = default;

    double start_s() const {
        return start_s_;
    }

    double end_s() const {
        return end_s_;
    }

    double start_l() const {
        return start_l_;
    }

    double end_l() const {
        return end_l_;
    }

    void set_start_s(double start_s) {
        start_s_ = start_s;
    }
    void set_end_s(double end_s) {
        end_s_ = end_s;
    }
    void set_start_l(double start_l) {
        start_l_ = start_l;
    }
    void set_end_l(double end_l) {
        end_l_ = end_l;
    }

    const std::vector<slPoint>& boundary_pts() const {
        return boundary_pts_;
    }

    std::vector<slPoint>& mutable_boundary_pts() {
        return boundary_pts_;
    }

    void set_boundary_pts(const std::vector<slPoint>& boundary_pts) {
        boundary_pts_ = boundary_pts;
    }

private:
    double start_s_;
    double end_s_;
    double start_l_;
    double end_l_;
    std::vector<slPoint> boundary_pts_;

};

#endif //POLYGONBACKSHAPE_PT_SLBOUNDARY_H
