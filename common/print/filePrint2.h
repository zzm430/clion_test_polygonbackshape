//
// Created by zzm on 2023/9/7.
//

#ifndef POLYGONBACKSHAPE_FILEPRIN_H
#define POLYGONBACKSHAPE_FILEPRIN_H

#include <iostream>
#include <fstream>
#include "common/utilpath/path_polygonPoint.h"
class filePrint2 {
public:
    filePrint2(const std::string& filename) {
        file_.open(filename, std::ios::in | std::ios::out );
        if (!file_) {
            std::cout << "File does not exist. Creating file..." << std::endl;
            file_.open(filename, std::ios::out);
        }
    }
    void writePt(polygonPoint pt) {
        file_ << pt.x << " " << pt.y << std::endl;
    }
    void closeFile() {
        file_.close();
    }
public:
    std::fstream file_;
};

#endif //POLYGONBACKSHAPE_FILEPRIN_H
