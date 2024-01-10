//
// Created by zzm on 24-1-9.
//
#include "linear_interpolation.h"

namespace math{

    double slerp(const double a0, const double t0, const double a1, const double t1,
                 const double t) {
        if (std::abs(t1 - t0) <= kMathEpsilons) {
            LOG(DEBUG) << "input time difference is too small";
            return NormalizeAngle(a0);
        }
        const double a0_n = NormalizeAngle(a0);
        const double a1_n = NormalizeAngle(a1);
        double d = a1_n - a0_n;
        if (d > M_PI) {
            d = d - 2 * M_PI;
        } else if (d < -M_PI) {
            d = d + 2 * M_PI;
        }

        const double r = (t - t0) / (t1 - t0);
        const double a = a0_n + d * r;
        return NormalizeAngle(a);
    }

    slPoint InterpolateUsingLinearApproximation(const slPoint &p0,
                                                const slPoint &p1, const double w) {
        CHECK_GE(w, 0.0);

        slPoint p;
        p.set_s((1 - w) * p0.s() + w * p1.s());
        p.set_l((1 - w) * p0.l() + w * p1.l());
        return p;
    }

    PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                                  const PathPoint &p1,
                                                  const double s) {
        double s0 = p0.s();
        double s1 = p1.s();

        PathPoint path_point;
        double weight = (s - s0) / (s1 - s0);
        double x = (1 - weight) * p0.x() + weight * p1.x();
        double y = (1 - weight) * p0.y() + weight * p1.y();
        double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
        double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
        double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
        double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
        path_point.set_x(x);
        path_point.set_y(y);
        path_point.set_theta(theta);
        path_point.set_kappa(kappa);
        path_point.set_dkappa(dkappa);
        path_point.set_ddkappa(ddkappa);
        path_point.set_s(s);
        return path_point;
    }

}