//
// Created by zzm on 24-1-9.
//

#ifndef POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_ALGORITHM_H
#define POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_ALGORITHM_H
#include <random>
#include <unordered_set>
#include "common/designPattern/singleton.hpp"
#include "common/plot/trailerCenterPtCalculate.h"
#include "Eigen/Eigen"
#include "common/math/box2d.h"
#include "common/math/line_segment_2d.h"
#include "common/math/vec_2d.h"
#include "common/utilpath/discretizedPath.h"
#include "smooth/customobsFemAlgorithm/fem_obstacle_smooth_param.h"
#include "smooth/customFemAlgorithm/fem_pos_deviation_smoother.h"
#include "common/math/linear_interpolation.h"
#include "common/utilpath/path_polygonPoint.h"
#include "easylogging++.h"
#include "common/plot/computePathProfile.h"
#include "common/print/boxPrint.h"
#include "common/print/trailerPt.h"
#include "common/print/trailerBoxPrint.h"

namespace  smoothalgorithm{
   class femObstacleAnchoringSmoother{
   public:
       femObstacleAnchoringSmoother() = default;
       virtual ~femObstacleAnchoringSmoother() = default;

       femObstacleAnchoringSmoother(
               const femObstacleParam&  fem_obstacle_param,
               const std::vector<std::vector<math::Vec2d>>&
               obstacles_vertices_vec);
       bool Smooth(const std::vector<polygonPoint> &pathPts);

       std::vector<polygonPoint> getStorageSmoothedPts();



   private:
       void AdjustStartEndHeading(
               const std::vector<polygonPoint>& pathPts,
               std::vector<std::pair<double, double>>* const point2d);

       bool ReAnchoring(const std::vector<size_t>& colliding_point_index,
                        DiscretizedPath* path_points);

       bool GenerateInitialBounds(const DiscretizedPath& path_points,
                                  std::vector<double>* initial_bounds);

       bool SmoothPath(const DiscretizedPath& raw_path_points,
                       const std::vector<double>& bounds,
                       DiscretizedPath* smoothed_path_points);

       bool CheckCollisionAvoidance(const DiscretizedPath& path_points,
                                    std::vector<size_t>* colliding_point_index,
                                    size_t counter);

       void AdjustPathBounds(const std::vector<size_t>& colliding_point_index,
                             std::vector<double>* bounds);

       void AdjustPathPtsPosition(const std::vector<size_t> & colliding_point_index,
                                  DiscretizedPath& path_points,
                                  DiscretizedPath& transed_path_points) ;

       bool SetPathProfile(const std::vector<std::pair<double, double>>& point2d,
                           DiscretizedPath* raw_path_points);

       bool CheckGear(const Eigen::MatrixXd& xWS);

       double CalcHeadings(const DiscretizedPath& path_points, const size_t index);

       void trailerCrashChecker(polygonPoint tractor_center_pt,
                                const size_t i,
                                std::vector<size_t>& colliding_point_index);
   private:
       // vehicle_param
       double ego_length_ = 0.0;
       double ego_width_  = 0.0;
       double center_shift_distance_ = 0.0;

       std::vector<std::vector<math::LineSegment2d>>
               obstacles_linesegments_vec_;
       std::vector<size_t> input_colliding_point_index_;
       bool enforce_initial_kappa_ = true;
       // gear DRIVE as true and gear REVERSE as false
       bool gear_ = false;
       femObstacleParam  fem_obstacle_param_;
       const std::vector<std::vector<math::Vec2d>>
               obstacles_vertices_vec_;
       std::vector<polygonPoint>  storage_smoothed_pts_;
   };
}


#endif //POLYGONBACKSHAPE_FEM_OBSTACLE_SMOOTH_ALGORITHM_H
