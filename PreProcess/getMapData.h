#ifndef  GETMAPDATA_H
#define  GETMAPDATA_H
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Planning/path_common.h"
#include "easylogging++.h"
class getMapData{
public:
    getMapData();
    ~getMapData();
    void loadMapOuter();
    std::vector<Point> getMapOuter();
private:
    std::vector<Point>  pointsOuter_;

};
#endif
