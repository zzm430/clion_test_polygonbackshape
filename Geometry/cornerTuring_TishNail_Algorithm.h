//
// Created by zzm on 23-9-11.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#define POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H
#include <vector>
#include <algorithm>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Exact_circular_kernel_2.h>
#include <boost/geometry.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/print/normaPrint.h"
#include "common/math/common_math.h"

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
            double F2,
            double F3);
    void cornerTuringPath( polygonPoint A,
                           polygonPoint B,
                           double RC2,
                           double F3);
    polygonPoint  computeCircleInterSectPt(
            polygonPoint pt1,
            polygonPoint pt2,
            const double& R1,
            double& R2);
    std::vector<polygonPoint> computeInterceptPath(std::vector<polygonPoint> & path,
                                                                          polygonPoint  order_pt);
    void computeCircleCenter( polygonPoint A,
                              polygonPoint B,
                               double angleInt,
                               double& RC2,
                               double F1,
                               double F2);
    void computeCircleC2Radius( polygonPoint A,
                              polygonPoint B,
                              double angleInt,
                              double& RC2,
                              double F1,
                              double F2);
    void chooseOptimalpath(  polygonPoint A,
                             polygonPoint B,
                             double angleInt,
                             double& RC2,
                             double F1,
                             double F2,
                              double F3);
    double computeLengthCostPath();
    std::vector<polygonPoint>   getFishNailC1path();
    std::vector<polygonPoint>   getFishNailC2path();
    std::vector<polygonPoint>   getFishNailC3path();
    std::vector<polygonPoint>   getC1path();
    std::vector<polygonPoint>   getC2path();
    std::vector<polygonPoint>   getC3path();
private:
    std::vector<polygonPoint>   storage_circle_center_;   //存储c1、c2、c3的圆心
    std::vector<polygonPoint>  C1path_;                   //原始path
    std::vector<polygonPoint>  C2path_;
    std::vector<polygonPoint>  C3path_;
    std::vector<polygonPoint>  fishNailC1path_;                   //鱼尾path
    std::vector<polygonPoint>  fishNailC2path_;
    std::vector<polygonPoint>  fishNailC3path_;


};
#endif //POLYGONBACKSHAPE_CORNERTURING_TISHNAIL_ALGORITHM_H

