//
// Created by zzm on 23-12-20.
//

#ifndef POLYGONBACKSHAPE_TRACTORPJPOPATHPRINT_H
#define POLYGONBACKSHAPE_TRACTORPJPOPATHPRINT_H
#include <string>
#include <iostream>
#include <fstream>
#include "common/utilpath/path_polygonPoint.h"

class tractorPolyPathPrint{
public:
    tractorPolyPathPrint() = default;
    virtual  ~tractorPolyPathPrint() = default;
    tractorPolyPathPrint(const std::string filename){
        file_.open(filename,std::ios::in | std::ios::out);
        if(!file_){
            std::cout << "tractor polypath does not exist.create file..."
                      << std::endl;
            file_.open(filename,std::ios::out);
        }
    }

    void writePts(std::vector<polygonPoint> storage_path){
        for(auto i : storage_path){
            file_ << " " << i.x;
        }
        file_ << std::endl;
        for(auto j : storage_path){
            file_ << " " << j.y;
        }
        file_ << std::endl;
    }
private:
    std::fstream file_;

};

#endif //POLYGONBACKSHAPE_TRACTORPJPOPATHPRINT_H
