//
// Created by zzm on 24-1-11.
//

#ifndef POLYGONBACKSHAPE_BOXPRINT_H
#define POLYGONBACKSHAPE_BOXPRINT_H
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include "common/math/vec_2d.h"

class boxPrint {
public:
    boxPrint() = default;
    virtual  ~boxPrint() = default;

    boxPrint(const std::string filename){
        file_.open(filename,std::ios::in | std::ios::out);
        if(!file_){
            std::cout << "boxshow File does not exist. Creating file..." << std::endl;
            file_.open(filename, std::ios::out);
        }
    }

    void writePts(std::vector<math::Vec2d> pts){
        for(auto it : pts){
            file_ << it.x() << " ";
        }

        file_ << std::endl;
        for(auto it : pts){
            file_ << it.y() << " ";
        }

        file_ << std::endl;
    }

private:
    std::fstream file_;
};
#endif //POLYGONBACKSHAPE_BOXPRINT_H
