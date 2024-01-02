//
// Created by zzm on 23-12-23.
//
#include "node_HybridA.h"
#include "easylogging++.h"
namespace searchAlgorithm {
    nodeTractor::nodeTractor(double x, double y, double phi) {
        x_ = x;
        y_ = y;
        phi_ = phi;
    }

    nodeTractor::nodeTractor(double x, double y, double phi,
                   const std::vector<double>& XYbounds,
                             const coarseSearchParam& coarse_search_param) {
        CHECK_EQ(XYbounds.size(), 4)
                << "XYbounds size is not 4, but" << XYbounds.size();

        x_ = x;
        y_ = y;
        phi_ = phi;

        x_grid_ = static_cast<int>(
                (x_ - XYbounds[0]) /
                        coarse_search_param.xy_resolution);
        y_grid_ = static_cast<int>(
                (y_ - XYbounds[2]) /
                        coarse_search_param.xy_resolution);
        phi_grid_ = static_cast<int>(
                (phi_ - (-M_PI)) /
                        coarse_search_param.xy_resolution);

        traversed_x_.push_back(x);
        traversed_y_.push_back(y);
        traversed_phi_.push_back(phi);

        index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
    }


    nodeTractor::nodeTractor(const std::vector<double>& traversed_x,
                   const std::vector<double>& traversed_y,
                   const std::vector<double>& traversed_phi,
                   const std::vector<double>& XYbounds,
                   const coarseSearchParam& coarse_search_param) {
        CHECK_EQ(XYbounds.size(), 4)
            << "XYbounds size is not 4, but" << XYbounds.size();
        CHECK_EQ(traversed_x.size(), traversed_y.size());
        CHECK_EQ(traversed_x.size(), traversed_phi.size());

        x_ = traversed_x.back();
        y_ = traversed_y.back();
        phi_ = traversed_phi.back();

        // XYbounds in xmin, xmax, ymin, ymax
        x_grid_ = static_cast<int>(
                (x_ - XYbounds[0]) /
                        coarse_search_param.xy_resolution);
        y_grid_ = static_cast<int>(
                (y_ - XYbounds[2]) /
                        coarse_search_param.xy_resolution);
        phi_grid_ = static_cast<int>(
                (phi_ - (-M_PI)) /
                        coarse_search_param.xy_resolution);

        traversed_x_ = traversed_x;
        traversed_y_ = traversed_y;
        traversed_phi_ = traversed_phi;

        index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
        step_size_ = traversed_x.size();
    }

    math::Box2d nodeTractor::GetBoundingBox( const vehicleParam& vehicle_param_,
                                 const double x, const double y, const double phi) {
        double tractor_length = vehicle_param_.tractor_frontEdge_to_center +
                                vehicle_param_.tractor_backEdge_to_center;
        double tractor_width = vehicle_param_.width;
        double shift_distance =
                tractor_length / 2.0 - vehicle_param_.tractor_backEdge_to_center;
        math::Box2d ego_box(
                {x + shift_distance * std::cos(phi), y + shift_distance * std::sin(phi)},
                phi, tractor_length, tractor_width);
        return ego_box;
    }

    bool nodeTractor::operator==(const nodeTractor& right) const {
        return right.GetIndex() == index_;
    }

    std::string nodeTractor::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
//        return absl::StrCat(x_grid, "_", y_grid, "_", phi_grid);
         return "";
    }



}