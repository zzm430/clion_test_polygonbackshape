//
// Created by zzm on 23-12-19.
//

#ifndef POLYGONBACKSHAPE_TRACTORPOLYPRINTCURVE_H
#define POLYGONBACKSHAPE_TRACTORPOLYPRINTCURVE_H
#include <string>
#include <iostream>
#include <fstream>
#include "common/utilpath/path_polygonPoint.h"

class tractorPolyCurve {
public:
    tractorPolyCurve() = default;
    virtual ~tractorPolyCurve() = default;
    tractorPolyCurve(const std::string filename){
        file_.open(filename,std::ios::in | std::ios::out);
        if(!file_){
            std::cout << "tractor curve does not exist.create file..."
                     << std::endl;
            file_.open(filename,std::ios::out);
        }
    }
    void writePts(std::vector<double> kappas){
        for(auto i : kappas){
            file_ << " " << i;
        }
        file_ << std::endl;
    }
private:
    std::fstream file_;
};

#endif //POLYGONBACKSHAPE_TRACTORPOLYPRINTCURVE_H
