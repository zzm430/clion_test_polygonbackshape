//
// Created by zzm on 23-11-24.
//

#ifndef POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
#define POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
#include <utility>
#include <vector>
#include <algorithm>
#include <fstream>
#include "common/plot/tractorPolygonShow.h"
#include "common/common_param/common_parameters.h"
#include "common/utilpath/path_polygonPoint.h"
#include "common/utilpath/path_anchorPoint.h"
#include "smooth/customFemAlgorithm/fem_pos_deviation_smoother.h"
#include "common/plot/computePathProfile.h"

namespace  smoothalgorithm{

    class femSmoothManager{
    public:
        femSmoothManager() = default;
        virtual ~femSmoothManager() = default;

        femSmoothManager(
               const std::vector<std::pair<double,double>>  xy_pts);

        void initiate();
        bool GenerateRefPointProfile(
                const std::vector<std::pair<double, double>>& xy_points);


        bool setPathProfile(
                const std::vector<std::pair<double,double>>& point2d,
                std::vector<polygonPoint>& raw_path_points);

        void normalizePoints(std::vector<std::pair<double, double>>* xy_points);
        void deNormalizePoints(std::vector<std::pair<double, double>>* xy_points);

        std::vector<double>  getRefOptx();
        std::vector<double>  getRefOpty();
        std::vector<polygonPoint> get_smoothed_pts();
    private:
        std::vector<double> optx_;
        std::vector<double> opty_;
        std::vector<std::pair<double,double>> xy_pts_;
        double x_ref_;
        double y_ref_;
        std::vector<polygonPoint> last_smoothed_pts_;

    };

}


#endif //POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
