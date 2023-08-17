//data   /05/09
#ifndef PATH_BASEPLAN_H
#define PATH_BASEPLAN_H

#include <iostream>
#include <math.h>
#include <easylogging++.h>
#include <vector>
#include <unordered_map>
//#include "planning_common.h"
#include "Planning/path_interface.h"
#include "Planning/path_common.h"
#include "common/common_parameters.h"

namespace aiforce {
namespace Route_Planning
{
     class pathBasePlan{
     public:
         pathBasePlan(){}
         virtual ~pathBasePlan(){}
         struct curvePoint{
             double x;
             double y;
             double heading;
      };

     //矩形回字形分类
     enum  class baskShapeType :uint8_t{
          L_SHAPE_AND_BACKSHAPE   = 0 ,   //L型第1垄 + 回字形
          FONT_SHAPE_AND_BACKSHAPE = 1,   //几字型第1垄 + 回字形
          NORMAL_BACKSHAPE = 2,           //正常回字形
          TURN_RIGHT_BACKSHAPE = 3,       //右拐回字形
          TURN_LEFT_BACKSHAPE             //左拐回字形
     };

     virtual void initiate(int use_get_curve_point,
                           double distance) = 0 ;                         //参数初始化
     virtual void process(std::vector<Point>& points) = 0 ;               //关键点获取

     virtual  void processCurve() = 0 ;                                   //获取关键点生成弯道关键点，并做个映射

     virtual  void  processComputeTraversalNumbers()  = 0;                //计算遍历次数

     virtual  void processComputeRidges()  = 0;                           //计算需要的垄数

     virtual  int  getRidges();                                          //获取垄数

     virtual  std::vector<Point> densify(const std::vector<Point>& points, int factor)  = 0;

     virtual  std::vector<pathInterface::pathPoint> processComputeRidgePath(int ridge_index)  = 0;

     virtual  std::vector<Point>  getKeyPoints() ;                       //获取相对坐标系下的关键点信息



     virtual  std::unordered_map<int,std::vector<pathBasePlan::curvePoint>>  getCurvePoints() =0 ;

     virtual  void diff1PointToJudgeDirection(Point p1,Point p2,
                                              std::vector<pathInterface::pathPoint> &path,
                                              int ridge_index)  = 0;
     virtual  void processTwoPointsDifference(Point &point1,
                                              Point& point2,
                                              std::vector<pathInterface::pathPoint> &path,
                                              int ridge_number)  = 0;
     virtual  std::vector<Point>  getDiff3Points(Point&m,Point&n) = 0;

     public:
        std::vector<Point>   key_points_;                 //关键点
        int      ridge_numbers_;                          //垄数
        std::unordered_map<int,std::vector<curvePoint>>   key_curve_points_; //关键点与弯道关键点的映射,这里将起始点和终点排除
        int      traversals_numbers_;                     //设定的遍历次数
        double   bow_spacing_setting_;                    //垄宽间距设置
        int      use_get_curve_point_;                    //用于得到弯道的点
        double   back_off_interval_dis_;                  //车辆后退间隔设置
        int      judge_increase_points_direction_;        //当需要前进后退转换时需要考虑的8个小循环规则如下
        /*
         * ----------------------------------------------------------
         * | 0   | x不变     | y变大     |
         * | 1   | x不变     | y减小     |
         * | 2   | x变小     | y不变     |
         * | 3   | x变大     | y不变     |
         * | 4   | x不变     | y减小     |
         * | 5   | x不变     | y变大     |
         * | 6   | x变大     | y不变     |
         * | 7   | x变小     | y不变     |
         * ----------------------------------------------------------
        */
        baskShapeType    backShapeMode_;                    //回字形规划方式
        bool    backShapeFlag_;                             //决定从外侧添加1垄还是从内部添加1垄 false = 从外部添加
        bool    useGCSPoints_;                              //判断是否将其转换为大地坐标系下的坐标点 FALSE = 不转   TRUE = 转
     };
}
}


#endif // PATH_BASEPLAN_H
