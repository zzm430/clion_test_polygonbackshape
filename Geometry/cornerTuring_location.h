//
// Created by zzm on 2023/9/5.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_LOCATION_H
#define POLYGONBACKSHAPE_CORNERTURING_LOCATION_H
#include "Planning/path_polygonplan.h"
#include "common/math/common_math.h"
enum  class angleType :uint8_t{
    FIRST_QUADRANT  = 0,   //0 ~ 0.5M_PI
    SECOND_QUADRANT = 1,   //0.5M_PI ~ M_PI
    THIRD_QUADRANT = 2,    //M_PI ~ 1.5M_PI
    FOURTH_QUADRANT = 3    //1.5M_PI ~ 2M_PI
};

enum  class cornerPosition :uint8_t{
    FRONT_RIGHT_WORKING_AREA  = 0,   //右前方
    FRONT_LEFT_WORKING_AREA = 1,     //左前方
    REAR_RIGHT_WORKING_AREA = 2,     //右后方
    REAR_LEFT_WORKING_AREA = 3      //左后方
};

class cornerAngleInfoMap{
public:
    cornerPosition   lpA_;   //lpA的类别
    cornerPosition   lpB_;   //lpB的类别
    double           WLA_;   //WLA的值
    double           WLB_;   //WLB的值
    double           F1_;    //F1为影响因子
    polygonPoint     A_;     //lpA
    polygonPoint     B_;     //lpB
};


class cornerTuringLocation{
public:
      cornerTuringLocation() = default;
      cornerTuringLocation(
              std::vector<polygonPoint> arriveLine,
              std::vector<polygonPoint> leaveLine);
      virtual  ~cornerTuringLocation();
      void decideLpAandLpB();
      void calculatePointsAandBForCurve();
      polygonPoint   getCurveStartPtA();
      polygonPoint   getCurveendPtB();
      double getCurveAngleInt();
      double getCurveCCPAAngleInt();
      double getCurveaboutF1();
      double getCurveaboutF2();
      double getCurveaboutF3();
      double getCurveArrriveLineHeading();
      double getCurveArrriveLineHeading2();

private:
    angleType  angleType_;                   //angleInt的类型
    double angleInt_;                        //向量的夹脚angleInt,用于鱼尾算法
    double CCPAAngleInt_;                    //用于CCPA算法(angleInt)
    cornerAngleInfoMap cornerAngleInfoMap_;  //弯道中间信息
    polygonPoint   A_;                       //弯道起始点，以交点作为坐标原点
    polygonPoint   B_;                       //弯道终点，以交点作为坐标原点
    double         F1_;
    double         F2_;
    double         F3_;
    double         arriveLineHeading_;         //以x轴为正方向
    double         arriveLineHeading2_;        //以正比为正方向


};

#endif //POLYGONBACKSHAPE_CORNERTURING_LOCATION_H
