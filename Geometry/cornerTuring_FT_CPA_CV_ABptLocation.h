//
// Created by zzm on 2023/10/11.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ABPTLOCATION_H
#define POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ABPTLOCATION_H
#include <common/utilpath/path_polygonPoint.h>
#include "common/common_param/common_parameters.h"

enum class ricPositionRobot :uint8_t {
    FRONT_RIGHT_ROBOT = 0,   //机器人右前方
    FRONT_LEFT_ROBOT = 1,    //机器人左前方
    REAR_RIGHT_ROBOT = 2,    //机器人右后方
    REAR_LEFT_ROBOT = 3    //机器人左后方
};

enum class ricPositionIm :uint8_t {
    FRONT_RIGHT_IMPLEMENT = 0, //农具右前方
    FRONT_LEFT_IMPLEMENT = 1,  //农具左前方
    REAR_RIGHT_IMPLEMENT = 2,   //农具右后方
    REAR_LEFT_IMPLEMENT = 3     //农具左后方
};

class cornerAngleInfo{
public:
    double WLA_;      //WLA的值
    double WLB_;      //WLB的值
    double F1_;       //F1影响因子
    polygonPoint LpA_robot_;     //LPA-robot
    polygonPoint LpB_robot_;     //LPB-robot
    polygonPoint LpA_im_;        //LPA-im
    polygonPoint LpB_im_;        //LPB-im
    polygonPoint LpA_WORKAREA_;  //LPA-WORKAREA
    polygonPoint LpB_WORKAREA_;  //LPB-WORKAREA
    double WLA_FS_;   //free space 使用
    double WLB_FS_;   //free space 使用
    double F1_WORKAREA_;        //F1_WORKAREA
};


class turingFtcpacvLocation{
public:
    turingFtcpacvLocation() = default;
    virtual ~turingFtcpacvLocation() = default;

    turingFtcpacvLocation(
            std::vector<polygonPoint> arriveLine,
            std::vector<polygonPoint> leaveLine,
            double ridgeNumber);
    void calculatePointsAandBForCurve();
    polygonPoint   getCurveStartPtA();
    polygonPoint   getCurveendPtB();
    double getCurveAngleInt();
private:
    std::vector<polygonPoint> arriveLine_;
    std::vector<polygonPoint> leaveLine_;
//  ricPositionRobot ricPositionRobot_;
//  ricPositionIm    ricPositionIm_;
    cornerAngleInfo  cornerAngleInfo_;
    double angleInt_;
    double Nlp_;
    double Nap_;
    polygonPoint A_;            //FT-CPA-CV 起始点
    polygonPoint B_;            //FT-CPA-CV 结束点
    polygonPoint A_robot_;      //根据机器人确定的A点
    polygonPoint B_robot_;
    polygonPoint A_im_;          //根据工具确定的A点
    polygonPoint B_im_;
    polygonPoint A_WORKAREA_;    //根据工作区域确定的A点
    polygonPoint B_WORKAREA_;
};
#endif //POLYGONBACKSHAPE_CORNERTURING_FT_CPA_CV_ABPTLOCATION_H
