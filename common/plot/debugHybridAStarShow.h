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

    };
}
#endif //POLYGONBACKSHAPE_DEBUGHYBRIDASTARSHOW_H
