//
// Created by zzm on 23-11-29.
//

#ifndef POLYGONBACKSHAPE_COMPUTEPATHPROFILE_H
#define POLYGONBACKSHAPE_COMPUTEPATHPROFILE_H
#include <iostream>
#include <utility>
#include <vector>
#include <math.h>

#include "common/plot/computePathProfile.h"

class computePathProfileInfo {
public:
    computePathProfileInfo() = delete;

    static bool ComputePathProfile(
            const std::vector<std::pair<double, double>> &xy_points,
            std::vector<double> *headings,
            std::vector<double> *accumulated_s,
            std::vector<double> *kappas,
            std::vector<double> *dkappas);
};

#endif //POLYGONBACKSHAPE_COMPUTEPATHPROFILE_H
