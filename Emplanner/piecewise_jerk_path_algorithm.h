//
// Created by zzm on 23-11-16.
//

#ifndef POLYGONBACKSHAPE_PIECEWISE_JERK_PATH_ALGORITHM_H
#define POLYGONBACKSHAPE_PIECEWISE_JERK_PATH_ALGORITHM_H
#include <utility>
#include <vector>

#include "piecewise_jerk_algorithm.h"

class piecewiseJerkPathAlgorithm : public piecewiseJerkAlgorithm {
public:
    piecewiseJerkPathAlgorithm(const size_t num_of_knots,
                             const double delta_s,
                             const std::array<double, 3>& x_init);

    ~piecewiseJerkPathAlgorithm() override = default;

    void set_kappa_max(double kappa_max) {
        kappa_max_ = kappa_max;
    }
    void set_kappa_ref(const std::vector<double>& kappa_ref) {
        kappa_ref_ = kappa_ref;
    }
    void set_kappa_ref(std::vector<double>&& kappa_ref) {
        kappa_ref_ = std::move(kappa_ref);
    }
    void set_use_old_method(bool use_old) {
        use_old_method_ = use_old;
    }

protected:
    void CalculateKernel(std::vector<c_float>* P_data,
                         std::vector<c_int>* P_indices,
                         std::vector<c_int>* P_indptr) override;

    void CalculateOffset(std::vector<c_float>* q) override;

    void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                   std::vector<c_int>* A_indices,
                                   std::vector<c_int>* A_indptr,
                                   std::vector<c_float>* lower_bounds,
                                   std::vector<c_float>* upper_bounds) override;


private:
    double kappa_max_;
    std::vector<double> kappa_ref_;
    bool use_old_method_ = false;

};

#endif //POLYGONBACKSHAPE_PIECEWISE_JERK_PATH_ALGORITHM_H

