//
// Created by zzm on 23-9-11.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#include <vector>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/print/normaPrint.h"
#include <algorithm>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
typedef boost::geometry::model::d2::point_xy<double> pointBoost;
typedef boost::geometry::model::polygon<pointBoost> polygonBoost;

class cornerTuringTishNail{
public:
    cornerTuringTishNail() = default;
    virtual ~cornerTuringTishNail();
    cornerTuringTishNail(
            polygonPoint A,
            polygonPoint B,
            double angleInt,
            double& RC2,
            double F1,
            double F2);
    void cornerTuringPath( polygonPoint A,
                           polygonPoint B,
                           double RC2,
                           double F3);
    std::vector<polygonPoint>   getFishNailC1path();
    std::vector<polygonPoint>   getFishNailC2path();
    std::vector<polygonPoint>   getFishNailC3path();
private:
    std::vector<polygonPoint>   storage_circle_center_;   //存储c1、c2、c3的圆心
    std::vector<polygonPoint>  C1path_;
    std::vector<polygonPoint>  C2path_;
    std::vector<polygonPoint>  C3path_;

};
#endif //POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H

