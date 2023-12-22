//
// Created by zzm on 23-11-27.
//

#ifndef POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H
#define POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H

#include "common/utilpath/path_polygonPoint.h"
#include "common/common_param/common_parameters.h"
#include "common/common_param/common_typedef.h"
#include "common/math/common_math.h"
#include "smooth/customFemAlgorithm/fem_pos_smooth_manager.h"
#include "common/utilpath/path_slPoint.h"
#include "Emplanner/frenet_converter.h"
#include "common/utilpath/discretizedPath.h"
#include "common/print/tractorPolyPrintCurve.h"
#include "Emplanner/customPJPO.h"
#include <cmath>
#include <algorithm>
#include "common/print/tractorPJPOPathPrint.h"
#include "common/print/tractorPolyReferencePath.h"

class curveStaticObstaclesManager{
public:
    curveStaticObstaclesManager() = default;
    virtual ~curveStaticObstaclesManager() = default;

    curveStaticObstaclesManager(std::vector<std::vector<polygonPoint>>  & originPath,
                                std::vector<std::vector<polygonPoint>> &polygonPts);

    bool curveGeneratePathFromDiscretePts(
            const DiscretizedPath& anchor_pts,
            DiscretizedPath& pathProfile);

    void computeOriginReferenceLinePts(  const polygon& obstacle_polygon,
                                         const polygonPoint&  centroid_pt,
                                         const std::vector<polygonPoint> & path_line,
                                         std::vector<polygonPoint> & reference_pts1,
                                         std::vector<polygonPoint> & reference_pts2,
                                         std::vector<polygonPoint> & reference_pts3,
                                         std::vector<polygonPoint> & reference_allpts);
    void computeOriginReferenceCenterLinePts(
                                         const polygon& obstacle_polygon,
                                         const polygonPoint&  centroid_pt,
                                         const std::vector<polygonPoint> & path_line,
                                         std::vector<polygonPoint> & reference_pts1,
                                         std::vector<polygonPoint> & reference_pts2,
                                         std::vector<polygonPoint> & reference_pts3,
                                         std::vector<polygonPoint> & reference_allpts);

    void computeObstacleCrashCheck(
                          const  std::vector<std::vector<polygonPoint>> &polygonPts,
                          const  std::vector<polygonPoint> & path_line,
                          std::vector<polygon>& obstacle_polygon,
                          std::vector<polygonPoint> & centroid_pt);

    std::vector<polygonPoint> computeReferenceLine(
            const polygon& obstacle_polygon,
            const polygonPoint & centroid_pt,
            const std::vector<polygonPoint> & path_line);

    void transOriginPathToDiscretizedPath(
            const std::vector<polygonPoint> & origin_path,
            DiscretizedPath & anchor_path);

    void pjpoInitialize(
            const std::vector<polygonPoint> & consider_static_obstacles_pts,
            DiscretizedPath & pathProfile);

    void pjpodealSLBoundary(
            const  DiscretizedPath & pathProfile,
            const  std::vector<polygonPoint> orderedPolygon,
            std::vector<std::pair<double, double>> * boundary);

    std::vector<polygonPoint>  getCurvePath(const polygonPoint orderedPt);

    void dealPJPO(
            const std::vector<polygonPoint> & consider_static_obstacles_pts,
            const polygon & obstacle_polygon,
            const polygonPoint & centroid_pt,
            const std::vector<polygonPoint> &path_line);

private:
    std::vector<std::vector<polygonPoint>>  three_custom_referencePts_;  //三段定制参考线
    std::unordered_map<polygonPoint,std::vector<polygonPoint>,polyPointHash>  storage_last_path_;  //最终path输出

};
#endif //POLYGONBACKSHAPE_CURVE_STATIC_OBSTACLES_MANAGER_H
