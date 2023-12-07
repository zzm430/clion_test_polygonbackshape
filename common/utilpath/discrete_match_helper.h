//
// Created by zzm on 23-12-4.
//

#ifndef POLYGONBACKSHAPE_DISCRETE_MATCH_HELPER_H
#define POLYGONBACKSHAPE_DISCRETE_MATCH_HELPER_H

#include "common/math/algebra.h"

template <class PtType>
struct DiscreteMatch {
    PtType foot_point_;
    double deviation_d_;
    double deviation_y_;
    size_t upper_idx_;
    size_t lower_idx_;
    size_t front_idx_;
    double min_s_;
    double max_s_;

    DiscreteMatch()
            : foot_point_()
            , deviation_d_(DBL_MAX)
            , deviation_y_(DBL_MAX)
            , upper_idx_(SIZE_MAX)
            , lower_idx_(SIZE_MAX)
            , front_idx_(SIZE_MAX)
            , min_s_(-DBL_MAX)
            , max_s_(DBL_MAX){};

    bool IsAvailable() const {
        return upper_idx_ != SIZE_MAX && lower_idx_ != SIZE_MAX &&
               front_idx_ != SIZE_MAX && foot_point_.s() >= min_s_ &&
               foot_point_.s() <= max_s_;
    };

    void Reset() {
        deviation_d_ = DBL_MAX;
        deviation_y_ = DBL_MAX;
        upper_idx_ = SIZE_MAX;
        lower_idx_ = SIZE_MAX;
        front_idx_ = SIZE_MAX;
        min_s_ = -DBL_MAX;
        max_s_ = DBL_MAX;
    };

private:

};


template <class PtType>
struct DiscreteMatchHelper : public std::vector<PtType> {
    DiscreteMatchHelper() = default;
    explicit DiscreteMatchHelper<PtType>(const std::vector<PtType>& pts)
            : std::vector<PtType>(pts) {}

    DiscreteMatch<PtType> Match(PtType& point) {
        return Match(point.x_, point.y_, point.theta());
    };

    DiscreteMatch<PtType> Match(double x, double y, double theta) {
        DiscreteMatch<PtType> match;
        if (this->empty()) {
            match.Reset();
            return match;
        }

        auto min_dis = DBL_MAX;
        size_t near_idx_ = SIZE_MAX;
        for (size_t idx = 0; idx < this->size(); ++idx) {
            double dis = this->at(idx).distance(x, y);
            if (dis < min_dis) {
                min_dis = dis;
                near_idx_ = idx;
            }
        }
        if (near_idx_ == SIZE_MAX) {
            match.Reset();
            return match;
        }
        match.min_s_ = this->front().s() - 0.1;
        match.max_s_ = this->back().s() + 0.1;
        near_idx_ = near_idx_ >= this->size() ? this->size() - 1 : near_idx_;
        match.lower_idx_ = (near_idx_ == 0) ? 0 : near_idx_ - 1;
        match.upper_idx_ =
                (near_idx_ == this->size() - 1) ? near_idx_ : near_idx_ + 1;

        if (match.lower_idx_ == match.upper_idx_) {
            match.front_idx_ = match.upper_idx_;
            match.foot_point_ = this->at(match.lower_idx_);
            match.deviation_d_ = match.foot_point_.distance(x, y);
            match.deviation_y_ =
                    math::Algebra::differentAngle(theta, match.foot_point_.theta());
        } else {
            match.foot_point_ = PtType::FindProjectionPt(
                    this->at(match.lower_idx_), this->at(match.upper_idx_), x, y);
            match.deviation_y_ =
                    math::Algebra::differentAngle(theta, match.foot_point_.theta());

            for (size_t idx = 0; idx < this->size(); ++idx) {
                auto& point = this->at(idx);
                if (point.s() >= match.foot_point_.s() + 0.25) {
                    match.front_idx_ = idx;
                    break;
                }
            }

            match.front_idx_ = match.front_idx_ >= this->size() ? this->size() - 1
                                                                : match.front_idx_;

            double v0x = x - this->at(match.lower_idx_).x_;
            double v0y = y - this->at(match.lower_idx_).y_;
            double v1x =
                    this->at(match.upper_idx_).x_ - this->at(match.lower_idx_).x_;
            double v1y =
                    this->at(match.upper_idx_).y_ - this->at(match.lower_idx_).y_;

            auto min_x = std::min(this->at(match.lower_idx_).x_,
                                  this->at(match.upper_idx_).x_);
            auto max_x = std::max(this->at(match.lower_idx_).x_,
                                  this->at(match.upper_idx_).x_);
            auto min_y = std::min(this->at(match.lower_idx_).y_,
                                  this->at(match.upper_idx_).y_);
            auto max_y = std::max(this->at(match.lower_idx_).y_,
                                  this->at(match.upper_idx_).y_);
            /// 区分match的点（垂足）在线段内还是线段外
            if (match.lower_idx_ == 0) {
                if (match.foot_point_.x_ >= min_x && match.foot_point_.x_ <= max_x &&
                    match.foot_point_.y_ >= min_y && match.foot_point_.y_ <= max_y) {
                    if (v1x * v0y - v1y * v0x > 0) {
                        match.deviation_d_ = -match.foot_point_.distance(x, y);
                    } else if (std::abs(v1x * v0y - v1y * v0x - 0.0) < DBL_EPSILON) {
                        match.deviation_d_ = 0.0;
                    } else {
                        match.deviation_d_ = match.foot_point_.distance(x, y);
                    }
                } else {
                    if (v1x * v0y - v1y * v0x > 0) {
                        match.deviation_d_ = -this->front().distance(x, y);
                    } else if (std::abs(v1x * v0y - v1y * v0x - 0.0) < DBL_EPSILON) {
                        match.deviation_d_ = this->front().distance(x, y);
                    } else {
                        match.deviation_d_ = this->front().distance(x, y);
                    }
                }
            } else if (match.upper_idx_ + 1 == this->size()) {
                if (match.foot_point_.x_ >= min_x && match.foot_point_.x_ <= max_x &&
                    match.foot_point_.y_ >= min_y && match.foot_point_.y_ <= max_y) {
                    if (v1x * v0y - v1y * v0x > 0) {
                        match.deviation_d_ = -match.foot_point_.distance(x, y);
                    } else if (std::abs(v1x * v0y - v1y * v0x - 0.0) < DBL_EPSILON) {
                        match.deviation_d_ = 0.0;
                    } else {
                        match.deviation_d_ = match.foot_point_.distance(x, y);
                    }
                } else {
                    if (v1x * v0y - v1y * v0x > 0) {
                        match.deviation_d_ = -this->back().distance(x, y);
                    } else if (std::abs(v1x * v0y - v1y * v0x - 0.0) < DBL_EPSILON) {
                        match.deviation_d_ = this->back().distance(x, y);
                    } else {
                        match.deviation_d_ = this->back().distance(x, y);
                    }
                }
            } else {
                if (v1x * v0y - v1y * v0x > 0) {
                    match.deviation_d_ = -match.foot_point_.distance(x, y);
                } else if (std::abs(v1x * v0y - v1y * v0x - 0.0) < DBL_EPSILON) {
                    match.deviation_d_ = 0.0;
                } else {
                    match.deviation_d_ = match.foot_point_.distance(x, y);
                }
            }
        }

        return match;
    };

private:

};




#endif //POLYGONBACKSHAPE_DISCRETE_MATCH_HELPER_H
