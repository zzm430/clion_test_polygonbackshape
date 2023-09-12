//
// Created by zzm on 23-9-12.
//

#ifndef POLYGONBACKSHAPE_NORMAPRINT_H
#define POLYGONBACKSHAPE_NORMAPRINT_H
#include <string>
#include <iostream>
#include <fstream>
#include "common/utilpath/path_polygonPoint.h"
class normalPrint{
public:
    normalPrint() = default;
    virtual  ~normalPrint() = default;
    normalPrint(const std::string& filename) {
        file_.open(filename, std::ios::in | std::ios::out );
        if (!file_) {
            std::cout << "File does not exist. Creating file..." << std::endl;
            file_.open(filename, std::ios::out);
        }
    }

    void writePts(std::vector<polygonPoint> pts) {
        for(auto it : pts){
            file_ << it.x << " ";
        }
        file_ <<std::endl;
        for(auto  it : pts){
            file_ << it.y << " ";
        }
        file_ << std::endl;
    }

    void closeFile() {
        file_.close();
    }


private:
    std::fstream file_;

};
#endif //POLYGONBACKSHAPE_NORMAPRINT_H
