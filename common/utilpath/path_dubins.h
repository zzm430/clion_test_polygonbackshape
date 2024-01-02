//
// Created by zzm on 23-12-25.
//

#ifndef POLYGONBACKSHAPE_PATH_DUBINS_H
#define POLYGONBACKSHAPE_PATH_DUBINS_H
#include <vector>

struct dubinPath{
    std::vector<double> segs_lengths;
    std::vector<char> segs_types;
    double total_length = 0.0;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;
    //前进、后退flag
    std::vector<bool> gear;
};

#endif //POLYGONBACKSHAPE_PATH_DUBINS_H
