#ifndef  GETMAPDATA_H
#define  GETMAPDATA_H
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "Planning/path_common.h"
#include "easylogging++.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/strategies.hpp>
#include <boost/geometry/algorithms/correct.hpp>

typedef boost::geometry::model::d2::point_xy<double>  point_map;
typedef boost::geometry::model::polygon<point_map>  polygon_map;

class getMapData{
public:
    getMapData();
    ~getMapData();
    void loadMapOuter();
    void loadMapOuter(std::string & loadPath);
    std::vector<Point> getMapOuter();
    void updatepolygonSequence();
    std::vector<Point> getMapUpdatedOuter();
private:
    std::vector<Point>  pointsOuter_;
    std::vector<Point>  updatedPointOuter_;   //更新后的原始多边形点
};
#endif
