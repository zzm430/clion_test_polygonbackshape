//
// Created by zzm on 23-11-15.
//

#ifndef POLYGONBACKSHAPE_PIECEWISE_JERK_ALGORITHM_H
#define POLYGONBACKSHAPE_PIECEWISE_JERK_ALGORITHM_H
#include <tuple>
#include <utility>
#include <vector>
#include "osqp/osqp.h"
#include "easylogging++.h"

class piecewiseJerkAlgorithm {
public:
    piecewiseJerkAlgorithm(size_t num_of_knots,
                         double delta_s,
                         std::array<double, 3> x_init);

    virtual ~piecewiseJerkAlgorithm() = default;


    OSQPData* FormulateProblem();


    bool Optimize(int max_iter);

    virtual OSQPSettings* SolverDefaultSettings();


    void set_x_bounds(std::vector<std::pair<double, double>> x_bounds);

    void set_dx_bounds(std::vector<std::pair<double, double>> dx_bounds);

    void set_ddx_bounds(std::vector<std::pair<double, double>> ddx_bounds);

    void set_x_bounds(double x_lower_bound, double x_upper_bound);

    void set_dx_bounds(double dx_lower_bound, double dx_upper_bound);

    void set_ddx_bounds(double ddx_lower_bound, double ddx_upper_bound);
    void set_dddx_bound(double dddx_bound);

    void set_dddx_bound(double dddx_lower_bound, double dddx_upper_bound);
    void set_weight_x(double weight_x);

    void set_weight_dx(double weight_dx);

    void set_weight_ddx(double weight_ddx);

    void set_weight_dddx(double weight_dddx);


    void set_scale_factor(const std::array<double, 3>& scale_factor);

    void set_x_ref(double weight_x_ref, std::vector<double> x_ref);

    void set_x_ref(std::vector<double> weight_x_ref_vec,
                   std::vector<double> x_ref);

    void set_end_state_ref(const std::array<double, 3>& weight_end_state,
                           const std::array<double, 3>& end_state_ref);

    const std::vector<double>& opt_x() const;

    const std::vector<double>& opt_dx() const;

    const std::vector<double>& opt_ddx() const;

protected:
    void FreeData(OSQPData* data) {
        delete[] data->q;
        delete[] data->l;
        delete[] data->u;

        delete[] data->P->i;
        delete[] data->P->p;
        delete[] data->P->x;

        delete[] data->A->i;
        delete[] data->A->p;
        delete[] data->A->x;

        delete data;
    };


    virtual void CalculateAffineConstraint(std::vector<c_float>* A_data,
                                           std::vector<c_int>* A_indices,
                                           std::vector<c_int>* A_indptr,
                                           std::vector<c_float>* lower_bounds,
                                           std::vector<c_float>* upper_bounds);
    template <typename T>
    T* CopyData(const std::vector<T>& vec);
    virtual void CalculateKernel(std::vector<c_float>* P_data,
                                 std::vector<c_int>* P_indices,
                                 std::vector<c_int>* P_indptr) = 0;

    virtual void CalculateOffset(std::vector<c_float>* q) = 0;

    size_t num_of_knots_;
    std::array<double, 3> x_init_;
    std::array<double, 3> scale_factor_ = {{1.0, 1.0, 1.0}};
    std::vector<std::pair<double, double>> x_bounds_;
    std::vector<std::pair<double, double>> dx_bounds_;
    std::vector<std::pair<double, double>> ddx_bounds_;
    std::pair<double, double> dddx_bound_;


protected:
    double weight_x_;
    double weight_dx_;
    double weight_ddx_;
    double weight_dddx_;

    double delta_s_;

    bool has_x_ref_;
    double weight_x_ref_;

    std::vector<double> x_ref_;
    std::vector<double> weight_x_ref_vec_;

    bool has_end_state_ref_;
    std::array<double, 3> weight_end_state_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> end_state_ref_;

    // output
    std::vector<double> x_;
    std::vector<double> dx_;
    std::vector<double> ddx_;


};

#endif //POLYGONBACKSHAPE_PIECEWISE_JERK_ALGORITHM_H
