//
// Created by zzm on 23-11-15.
//

#include "piecewise_jerk_algorithm.h"


piecewiseJerkAlgorithm::piecewiseJerkAlgorithm(const size_t num_of_knots,
                                           double delta_s,
                                           const std::array<double, 3> x_init)
        : num_of_knots_(num_of_knots), delta_s_(delta_s), x_init_(x_init) {
    x_bounds_.resize(num_of_knots_, std::make_pair(-1.0e10, 1.0e10));
    dx_bounds_.resize(num_of_knots_, std::make_pair(-1.0e10, 1.0e10));
    ddx_bounds_.resize(num_of_knots_, std::make_pair(-1.0e10, 1.0e10));
    weight_x_ref_vec_.resize(num_of_knots_, 0.0);
}

OSQPData* piecewiseJerkAlgorithm::FormulateProblem() {
    // calculate kernel
    std::vector<c_float> P_data;
    std::vector<c_int> P_indices;
    std::vector<c_int> P_indptr;
    CalculateKernel(&P_data, &P_indices, &P_indptr);

    // calculate affine constraints
    std::vector<c_float> A_data;
    std::vector<c_int> A_indices;
    std::vector<c_int> A_indptr;
    std::vector<c_float> lower_bounds;
    std::vector<c_float> upper_bounds;
    CalculateAffineConstraint(
            &A_data, &A_indices, &A_indptr, &lower_bounds, &upper_bounds);

    // calculate offset
    std::vector<c_float> q;
    CalculateOffset(&q);

    auto* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));

    size_t kernel_dim = 3 * num_of_knots_;
    size_t num_affine_constraint = lower_bounds.size();

    data->n = static_cast<c_int>(kernel_dim);
    data->m = static_cast<c_int>(num_affine_constraint);
    data->P = csc_matrix(static_cast<c_int>(kernel_dim),
                         static_cast<c_int>(kernel_dim),
                         static_cast<c_int>(P_data.size()),
                         CopyData(P_data),
                         CopyData(P_indices),
                         CopyData(P_indptr));
    data->q = CopyData(q);
    data->A = csc_matrix(static_cast<c_int>(num_affine_constraint),
                         static_cast<c_int>(kernel_dim),
                         static_cast<c_int>(A_data.size()),
                         CopyData(A_data),
                         CopyData(A_indices),
                         CopyData(A_indptr));
    data->l = CopyData(lower_bounds);
    data->u = CopyData(upper_bounds);
    return data;
}

bool piecewiseJerkAlgorithm::Optimize(const int max_iter) {
    OSQPData* data = FormulateProblem();

    OSQPSettings* settings = SolverDefaultSettings();
    settings->max_iter = max_iter;

    OSQPWorkspace* osqp_work = nullptr;
    osqp_work = osqp_setup(data, settings);
    // osqp_setup(&osqp_work, data, settings);

    osqp_solve(osqp_work);
    if (!osqp_work) {
        LOG(ERROR) << "osqp_work nullptr";
        osqp_cleanup(osqp_work);
        FreeData(data);
        c_free(settings);
        return false;
    }

    auto status = osqp_work->info->status_val;

    if (status < 0 || (status != 1 && status != 2)) {
        LOG(ERROR) << "failed optimization status:\t" << osqp_work->info->status;
        osqp_cleanup(osqp_work);
        FreeData(data);
        c_free(settings);
        return false;
    } else if (osqp_work->solution == nullptr) {
        LOG(ERROR) << "The solution from OSQP is nullptr";
        osqp_cleanup(osqp_work);
        FreeData(data);
        c_free(settings);
        return false;
    }

    // extract primal results
    x_.resize(num_of_knots_);
    dx_.resize(num_of_knots_);
    ddx_.resize(num_of_knots_);
    for (size_t i = 0; i < num_of_knots_; ++i) {
        x_.at(i) = osqp_work->solution->x[i] / scale_factor_[0];
        dx_.at(i) = osqp_work->solution->x[i + num_of_knots_] / scale_factor_[1];
        ddx_.at(i) =
                osqp_work->solution->x[i + 2 * num_of_knots_] / scale_factor_[2];
    }

    // Cleanup
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    return true;
}


OSQPSettings* piecewiseJerkAlgorithm::SolverDefaultSettings() {
    // Define Solver default settings
    auto* settings =
            reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
    osqp_set_default_settings(settings);
    settings->polish = true;
    settings->verbose = false;
    settings->scaled_termination = true;
    settings->time_limit = 0.10;
    return settings;
}



void piecewiseJerkAlgorithm::set_x_bounds(
        std::vector<std::pair<double, double>> x_bounds) {
    CHECK_EQ(x_bounds.size(), num_of_knots_);
    x_bounds_ = std::move(x_bounds);
}

void piecewiseJerkAlgorithm::set_dx_bounds(
        std::vector<std::pair<double, double>> dx_bounds) {
    CHECK_EQ(dx_bounds.size(), num_of_knots_);
    dx_bounds_ = std::move(dx_bounds);
}

void piecewiseJerkAlgorithm::set_ddx_bounds(
        std::vector<std::pair<double, double>> ddx_bounds) {
    CHECK_EQ(ddx_bounds.size(), num_of_knots_);
    ddx_bounds_ = std::move(ddx_bounds);
}

void piecewiseJerkAlgorithm::set_x_bounds(double x_lower_bound,
                                        double x_upper_bound) {
    for (auto& x : x_bounds_) {
        x.first = x_lower_bound;
        x.second = x_upper_bound;
    }
}

void piecewiseJerkAlgorithm::set_dx_bounds(double dx_lower_bound,
                                         double dx_upper_bound) {
    for (auto& x : dx_bounds_) {
        x.first = dx_lower_bound;
        x.second = dx_upper_bound;
    }
}

void piecewiseJerkAlgorithm::set_ddx_bounds(double ddx_lower_bound,
                                          double ddx_upper_bound) {
    for (auto& x : ddx_bounds_) {
        x.first = ddx_lower_bound;
        x.second = ddx_upper_bound;
    }
}
void piecewiseJerkAlgorithm::set_dddx_bound(double dddx_bound) {
    set_dddx_bound(-dddx_bound, dddx_bound);
}

void piecewiseJerkAlgorithm::set_dddx_bound(double dddx_lower_bound,
                                          double dddx_upper_bound) {
    dddx_bound_.first = dddx_lower_bound;
    dddx_bound_.second = dddx_upper_bound;
}

void piecewiseJerkAlgorithm::set_weight_x(double weight_x) {
    weight_x_ = weight_x;
}

void piecewiseJerkAlgorithm::set_weight_dx(double weight_dx) {
    weight_dx_ = weight_dx;
}

void piecewiseJerkAlgorithm::set_weight_ddx(double weight_ddx) {
    weight_ddx_ = weight_ddx;
}

void piecewiseJerkAlgorithm::set_weight_dddx(double weight_dddx) {
    weight_dddx_ = weight_dddx;
}

void piecewiseJerkAlgorithm::set_scale_factor(
        const std::array<double, 3>& scale_factor) {
    scale_factor_ = scale_factor;
}

void piecewiseJerkAlgorithm::set_x_ref(double weight_x_ref,
                                     std::vector<double> x_ref) {
    CHECK_EQ(x_ref.size(), num_of_knots_);
    weight_x_ref_ = weight_x_ref;
    // set uniform weighting
    weight_x_ref_vec_ = std::vector<double>(num_of_knots_, weight_x_ref);
    x_ref_ = std::move(x_ref);
    has_x_ref_ = true;
}

void piecewiseJerkAlgorithm::set_x_ref(std::vector<double> weight_x_ref_vec,
                                     std::vector<double> x_ref) {
    CHECK_EQ(x_ref.size(), num_of_knots_);
    CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);
    // set piecewise weighting
    weight_x_ref_vec_ = std::move(weight_x_ref_vec);
    x_ref_ = std::move(x_ref);
    has_x_ref_ = true;
}

void piecewiseJerkAlgorithm::set_end_state_ref(
        const std::array<double, 3>& weight_end_state,
        const std::array<double, 3>& end_state_ref) {
    weight_end_state_ = weight_end_state;
    end_state_ref_ = end_state_ref;
    has_end_state_ref_ = true;
}

const std::vector<double>& piecewiseJerkAlgorithm::opt_x() const {
    return x_;
}

const std::vector<double>& piecewiseJerkAlgorithm::opt_dx() const {
    return dx_;
}

const std::vector<double>& piecewiseJerkAlgorithm::opt_ddx() const {
    return ddx_;
}

void piecewiseJerkAlgorithm::CalculateAffineConstraint(
        std::vector<c_float>* A_data,
        std::vector<c_int>* A_indices,
        std::vector<c_int>* A_indptr,
        std::vector<c_float>* lower_bounds,
        std::vector<c_float>* upper_bounds) {
    // 3N params bounds on x, x', x''
    // 3(N-1) constraints on x, x', x''
    // 3 constraints on x_init_
    const auto n = static_cast<size_t>(num_of_knots_);
    const size_t num_of_variables = 3 * n;
    const size_t num_of_constraints = num_of_variables + 3 * (n - 1) + 3;
    lower_bounds->resize(num_of_constraints);
    upper_bounds->resize(num_of_constraints);

    std::vector<std::vector<std::pair<c_int, c_float>>> variables(
            num_of_variables);

    size_t constraint_index = 0;
    // set x, x', x'' bounds
    for (int i = 0; i < num_of_variables; ++i) {
        if (i < n) {
            variables[i].emplace_back(constraint_index, 1.0);
            lower_bounds->at(constraint_index) =
                    x_bounds_[i].first * scale_factor_[0];
            upper_bounds->at(constraint_index) =
                    x_bounds_[i].second * scale_factor_[0];
        } else if (i < 2 * n) {
            variables[i].emplace_back(constraint_index, 1.0);

            lower_bounds->at(constraint_index) =
                    dx_bounds_[i - n].first * scale_factor_[1];
            upper_bounds->at(constraint_index) =
                    dx_bounds_[i - n].second * scale_factor_[1];
        } else {
            variables[i].emplace_back(constraint_index, 1.0);
            lower_bounds->at(constraint_index) =
                    ddx_bounds_[i - 2 * n].first * scale_factor_[2];
            upper_bounds->at(constraint_index) =
                    ddx_bounds_[i - 2 * n].second * scale_factor_[2];
        }

//        LOG_ASSERT(upper_bounds->at(constraint_index) >=
//                   lower_bounds->at(constraint_index));
        ++constraint_index;
    }
    CHECK_EQ(constraint_index, num_of_variables);

    // x(i->i+1)''' = (x(i+1)'' - x(i)'') / delta_s
    for (int i = 0; i + 1 < n; ++i) {
        variables[2 * n + i].emplace_back(constraint_index, -1.0);
        variables[2 * n + i + 1].emplace_back(constraint_index, 1.0);
        lower_bounds->at(constraint_index) =
                dddx_bound_.first * delta_s_ * scale_factor_[2];
        upper_bounds->at(constraint_index) =
                dddx_bound_.second * delta_s_ * scale_factor_[2];

//        LOG_ASSERT(upper_bounds->at(constraint_index) >=
//                   lower_bounds->at(constraint_index));
        ++constraint_index;
    }

    // x(i+1)' - x(i)' - 0.5 * delta_s * x(i)'' - 0.5 * delta_s * x(i+1)'' = 0
    for (int i = 0; i + 1 < n; ++i) {
        variables[n + i].emplace_back(constraint_index, -1.0 * scale_factor_[2]);
        variables[n + i + 1].emplace_back(constraint_index, 1.0 * scale_factor_[2]);
        variables[2 * n + i].emplace_back(constraint_index,
                                          -0.5 * delta_s_ * scale_factor_[1]);
        variables[2 * n + i + 1].emplace_back(constraint_index,
                                              -0.5 * delta_s_ * scale_factor_[1]);
        lower_bounds->at(constraint_index) = 0.0;
        upper_bounds->at(constraint_index) = 0.0;

//        LOG_ASSERT(upper_bounds->at(constraint_index) >=
//                   lower_bounds->at(constraint_index));
        ++constraint_index;
    }

    // x(i+1) - x(i) - delta_s * x(i)'
    // - 1/3 * delta_s^2 * x(i)'' - 1/6 * delta_s^2 * x(i+1)''
    auto delta_s_sq_ = delta_s_ * delta_s_;
    for (int i = 0; i + 1 < n; ++i) {
        variables[i].emplace_back(constraint_index,
                                  -1.0 * scale_factor_[1] * scale_factor_[2]);
        variables[i + 1].emplace_back(constraint_index,
                                      1.0 * scale_factor_[1] * scale_factor_[2]);
        variables[n + i].emplace_back(
                constraint_index, -delta_s_ * scale_factor_[0] * scale_factor_[2]);
        variables[2 * n + i].emplace_back(
                constraint_index,
                -delta_s_sq_ / 3.0 * scale_factor_[0] * scale_factor_[1]);
        variables[2 * n + i + 1].emplace_back(
                constraint_index,
                -delta_s_sq_ / 6.0 * scale_factor_[0] * scale_factor_[1]);

        lower_bounds->at(constraint_index) = 0.0;
        upper_bounds->at(constraint_index) = 0.0;

//        LOG_ASSERT(upper_bounds->at(constraint_index) >=
//                   lower_bounds->at(constraint_index));
        ++constraint_index;
    }

    // constrain on x_init
    variables[0].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
    upper_bounds->at(constraint_index) = x_init_[0] * scale_factor_[0];
    ++constraint_index;

    variables[n].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
    upper_bounds->at(constraint_index) = x_init_[1] * scale_factor_[1];
    ++constraint_index;

    variables[2 * n].emplace_back(constraint_index, 1.0);
    lower_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
    upper_bounds->at(constraint_index) = x_init_[2] * scale_factor_[2];
    ++constraint_index;

    CHECK_EQ(constraint_index, num_of_constraints);

    int ind_p = 0;
    for (int i = 0; i < num_of_variables; ++i) {
        A_indptr->push_back(ind_p);
        for (const auto& variable_nz : variables[i]) {
            // coefficient
            A_data->push_back(variable_nz.second);

            // constraint index
            A_indices->push_back(variable_nz.first);
            ++ind_p;
        }
    }
    // We indeed need this line because of
    // https://github.com/oxfordcontrol/osqp/blob/master/src/cs.c#L255
    A_indptr->push_back(ind_p);
}



template <typename T>
T* piecewiseJerkAlgorithm::CopyData(const std::vector<T>& vec) {
    T* data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
}