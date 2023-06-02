#ifndef PATH_RECTANGLEPLAN_H
#define PATH_RECTANGLEPLAN_H

#include "path_baseplan.h"
#include "Geometry/innerRect.h"
namespace aiforce {
namespace Route_Planning
{
   class pathRectanglePlan : public  pathBasePlan{
   public:
       pathRectanglePlan(){}
       virtual ~pathRectanglePlan(){}


       virtual void initiate(int use_get_curve_point,
                                  double distance) override;

       virtual void process(std::vector<Point>& points) override;

       virtual  void processCurve() override;

       virtual  void processComputeRidges() override;

       virtual  void processComputeTraversalNumbers() override;

       virtual  std::vector<Point> densify(const std::vector<Point>& points,
                                           int factor) override;
       virtual  std::vector<pathInterface::pathPoint> processComputeRidgePath(int ridge_index) override;

       virtual  std::unordered_map<int,std::vector<pathBasePlan::curvePoint>>  getCurvePoints() override;

       virtual  void diff1PointToJudgeDirection(Point p1,Point p2,
                                                std::vector<pathInterface::pathPoint> &path,
                                                int ridge_index) override;
       virtual  void processTwoPointsDifference(Point &point1,
                                                Point& point2,
                                                std::vector<pathInterface::pathPoint> &path,
                                                int ridge_number) override;
       virtual std::vector<Point>  getDiff3Points(Point&m,Point&n)  override;
   public:
       double  remainder_;                                                   //短边是垄宽倍数的余数
       std::vector<Point>   show_key_points_only_;                           //仅做关键点显示使用处理过的
       std::vector<Point>   original_show_key_points_;                       //大地坐标系下的关键点位信息
       std::vector<Point>   transferd_points_;                               //矩形的坐标点信息
       std::vector<Point>   first_road_;                                     //第一垄对应的关键点
       std::vector<pathInterface::pathPoint>  first_road_points_;            //第一垄生成的地图点位
       double deviation_number_;                                             //矩形几字形横移距离
       double deviation_number_L_;                                           //矩形L字型横移距离
   public:
       std::vector<Point> keyPointsCoordinateoffset(std::vector<Point>& points);
       std::vector<Point> getPointsWithFixedDistance(Point & p1,
                                                     Point & p2,
                                                     double distance);
       std::vector<Point>  getShowKeyPoints();
       void processDiff3Points(Point&m,
                               Point&n,
                               std::vector<pathInterface::pathPoint> & path,
                               int ridge_number);
       void processIncludeFirstRidgeKeyPoints(std::vector<Point>& points);
       std::vector<pathInterface::pathPoint>  processJiziShapeFirstRidge();
       std::vector<Point>  getOriginalShowKeyPoints();
       std::vector<pathInterface::pathPoint>  processFirstRidge();
       void processDecideBackShapeType();
       double   double_mod(double x, double y);                             //double取余
   };


}
}

#endif // PATH_RECTANGLEPLAN_H
