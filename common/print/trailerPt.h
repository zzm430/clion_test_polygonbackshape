//
// Created by zzm on 24-1-16.
//

#ifndef POLYGONBACKSHAPE_TRAILERPT_H
#define POLYGONBACKSHAPE_TRAILERPT_H
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "common/utilpath/path_polygonPoint.h"

class trailerPt {
public:
    trailerPt() = default;
    virtual  ~trailerPt() = default;

    trailerPt(const std::string filename){
        file_.open(filename,std::ios::in | std::ios::out);
        if(!file_){
            std::cout << "boxshow File does not exist. Creating file..." << std::endl;
            file_.open(filename, std::ios::out);
        }
    }

    void writePts(polygonPoint ordered_pt){
        file_ << ordered_pt.x  << " ";
        file_ << ordered_pt.y  << " ";
        file_ << std::endl;
    }

private:
    std::fstream file_;
};
#endif //POLYGONBACKSHAPE_TRAILERPT_H
