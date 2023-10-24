//
// Created by zzm on 2023/10/24.
//

#ifndef POLYGONBACKSHAPE_TRACTORPOLYPRINT2_H
#define POLYGONBACKSHAPE_TRACTORPOLYPRINT2_H
#include <string>
#include <iostream>
#include <fstream>
#include "common/utilpath/path_polygonPoint.h"
class tractorPolyPrint2 {
public:
    tractorPolyPrint2() = default;
    virtual ~tractorPolyPrint2() = default;
    tractorPolyPrint2(const std::string filename){
        file_.open(filename, std::ios::in | std::ios::out );
        if (!file_) {
            std::cout << "tractorPoly File does not exist. Creating file..." << std::endl;
            file_.open(filename, std::ios::out);
        }
    }
    void writePts(std::vector<polygonPoint> ptsA,std::vector<polygonPoint> ptsB){
        for(auto it : ptsA){
            file_ << it.x << " ";
        }
        for(auto im :ptsB){
            file_ << im.x << " ";
        }
        file_ << std::endl;
        for(auto it : ptsA){
            file_ << it.y << " ";
        }
        for(auto im :ptsB){
            file_ << im.y << " ";
        }
        file_ << std::endl;
    }

private:
    std::fstream  file_;

};
#endif //POLYGONBACKSHAPE_TRACTORPOLYPRINT2_H
