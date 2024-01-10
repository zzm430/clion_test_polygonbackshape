//
// Created by zzm on 23-12-27.
//

#ifndef POLYGONBACKSHAPE_DEBUGHYBRIDASTARSHOW_H
#define POLYGONBACKSHAPE_DEBUGHYBRIDASTARSHOW_H
#include "thirdParty/matplotlibcpp.h"
#include <Eigen/Dense>

namespace plt = matplotlibcpp;

namespace searchAlgorithm{
    using KeywordType = std::map<std::string, std::string>;
    class  debugHelpShow{
    public:
        debugHelpShow() = default;
        virtual ~debugHelpShow() = default;

        static void PlotArrayLine(
                const std::vector<std::array<std::pair<double, double>, 2>> data,
                const KeywordType& keywords) {
            for (size_t i = 0; i < data.size(); ++i) {
                std::vector<double> x(2);
                std::vector<double> y(2);
                x[0] = data[i][0].first;
                y[0] = data[i][0].second;
                x[1] = data[i][1].first;
                y[1] = data[i][1].second;
                plt::plot(x, y, keywords);
            }
        }

        static void ShowPath(const std::vector<Eigen::Vector2d>& path,
                             const KeywordType& keywords) {
            size_t size = path.size();
            std::vector<double> x(size);
            std::vector<double> y(size);
            for (size_t i = 0; i < size; i++) {
                x[i] = path[i][0];
                y[i] = path[i][1];
            }
            plt::plot(x, y, keywords);
        }




    };
}
#endif //POLYGONBACKSHAPE_DEBUGHYBRIDASTARSHOW_H
