//
// Created by zzm on 2023/10/11.
//
#include "Geometry/cornerTuring_FT_CPA_CV_ABptLocation.h"
#include "common/math/common_math.h"
turingFtcpacvLocation::turingFtcpacvLocation(
             std::vector<polygonPoint> arriveLine,
              std::vector<polygonPoint> leaveLine,
              double ridgeNumber):arriveLine_(arriveLine),
                                  leaveLine_(leaveLine),
                                  Nlp_(ridgeNumber),
                                  Nap_(ridgeNumber){
    //arriveLine描述起始向量
    //leaveLine描述结束向量
    polygonPoint vector_1,vector_2,reference_vector;
    vector_1.x = arriveLine_[1].x - arriveLine_[0].x;
    vector_1.y = arriveLine_[1].y - arriveLine_[0].y;
    vector_2.x = leaveLine_[1].x - leaveLine_[0].x;
    vector_2.y = leaveLine_[1].y - leaveLine_[0].y;
    reference_vector.x = 0;
    reference_vector.y = 1;

    //计算arriveline与x轴正向的夹角
    polygonPoint  refer_x_vector;
    refer_x_vector.x = 1;
    refer_x_vector.y = 0;

    double angle_x = common::commonMath::computeTwolineAngleDu(vector_1,refer_x_vector);
    if(angle_x < 0){
        angle_x += 360;
    }
    //此方向为顺时针
    arriveLineHeading_ = angle_x;

    double fish_angle_temp_1 = common::commonMath::computeTwolineAngleDu(vector_1,reference_vector);
    double fish_angle_temp_2 = common::commonMath::computeTwolineAngleDu(vector_2,reference_vector);
    double fishAngleInt;
    double fish_diff_angle = fish_angle_temp_1 * M_PI / 180 - fish_angle_temp_2 * M_PI / 180;
    if(fish_diff_angle > 0){
        fishAngleInt = fish_diff_angle;
    }else{
        fishAngleInt = fish_diff_angle + 2 * M_PI;
    }

    double angle_diff = fishAngleInt * 180 / M_PI;

    if(angle_diff >0 && angle_diff < 90){
        polygonPoint temp_lpA_robot,temp_lpB_robot,temp_lpA_im,temp_lpB_im;
        temp_lpA_robot.x =  DCRF ;
        temp_lpA_robot.y = - 0.5 * WROBOT;
        temp_lpB_robot.x = - DCRR;
        temp_lpB_robot.y = - 0.5 * WROBOT;
        temp_lpA_im.x = - DCRI;
        temp_lpA_im.y = -0.5 * WIM + OFFIM;
        temp_lpB_im.x = -DCRI-LIM ;
        temp_lpB_im.y = -0.5 * WIM + OFFIM;
        cornerAngleInfo_.LpA_robot_ = temp_lpA_robot;
        cornerAngleInfo_.LpA_im_ = temp_lpA_im;
        cornerAngleInfo_.LpB_robot_ = temp_lpB_robot;
        cornerAngleInfo_.LpB_im_ = temp_lpB_im;
        cornerAngleInfo_.WLA_ =  (-0.5 * WIM + OFFIM) - Nlp_ * WWORK;
        cornerAngleInfo_.WLB_ = (-0.5 * WIM + OFFIM) - Nap_ * WWORK;
        cornerAngleInfo_.F1_ = -1;
        //free space 使用
        cornerAngleInfo_.WLA_FS_ = 0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.WLB_FS_ = (-0.5 * WWORK + OFFWORK)
                                   + cos(angleInt_) * cos(angleInt_) * WWORK;
        cornerAngleInfo_.F1_WORKAREA_ = 1;
        polygonPoint  tempA,tempB;
        tempA.x = - DCRI - DWA - LWORK;
        tempA.y = 0.5 * WWORK + OFFWORK;
        tempB.x = - DCRI - DWA ;
        tempB.y = 0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.LpA_WORKAREA_ = tempA;
        cornerAngleInfo_.LpB_WORKAREA_ = tempB;
        LOG(INFO) << "FT-CPA-CV's angleInt is in FIRST_QUADRANT";
    } else if(angle_diff >=90  && angle_diff < 180){
        polygonPoint temp_lpA_robot,temp_lpB_robot,temp_lpA_im,temp_lpB_im;
        temp_lpA_robot.x = DCRF;
        temp_lpA_robot.y = 0.5 * WROBOT;
        temp_lpB_robot.x = -DCRR ;
        temp_lpB_robot.y = 0.5 * WROBOT;
        temp_lpA_im.x = -DCRI;
        temp_lpA_im.y = 0.5 * WIM + OFFIM;
        temp_lpB_im.x = -DCRI - LIM;
        temp_lpB_im.y = 0.5 * WIM + OFFIM;
        cornerAngleInfo_.LpA_robot_ = temp_lpA_robot;
        cornerAngleInfo_.LpB_robot_ = temp_lpB_robot;
        cornerAngleInfo_.LpA_im_ = temp_lpA_im;
        cornerAngleInfo_.LpB_im_ = temp_lpB_im;
        cornerAngleInfo_.WLA_ = (-0.5 * WIM + OFFIM) - Nlp_ * WWORK;
        cornerAngleInfo_.WLB_ = (-0.5 * WIM + OFFIM) - Nap_ * WWORK;
        cornerAngleInfo_.F1_ = 1;
        //free space使用
        cornerAngleInfo_.WLA_FS_ = 0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.WLB_FS_ = (-0.5 * WWORK + OFFWORK);
        cornerAngleInfo_.F1_WORKAREA_ = -1;
        polygonPoint  tempA,tempB;
        tempA.x = - DCRI - DWA - LWORK;
        tempA.y = -0.5 * WWORK + OFFWORK;
        tempB.x = - DCRI - DWA ;
        tempB.y = -0.5 * WWORK + OFFWORK ;
        cornerAngleInfo_.LpA_WORKAREA_ = tempA;
        cornerAngleInfo_.LpB_WORKAREA_ = tempB;
        LOG(INFO) << "FT-CPA-CV's angleInt is in SECOND_QUADRANT";
    } else if(angle_diff >180  && angle_diff < 270){
        polygonPoint temp_lpA_robot,temp_lpB_robot,temp_lpA_im,temp_lpB_im;
        temp_lpA_robot.x = DCRF;
        temp_lpA_robot.y = -0.5 * WROBOT;
        temp_lpB_robot.x = -DCRR;
        temp_lpB_robot.y = 0.5 * WROBOT;
        temp_lpA_im.x = -DCRI;
        temp_lpA_im.y = -0.5 * WIM + OFFIM;
        temp_lpB_im.x = -DCRI - LIM;
        temp_lpB_im.y = -0.5 * WIM + OFFIM;
        cornerAngleInfo_.LpA_robot_ = temp_lpA_robot;
        cornerAngleInfo_.LpB_robot_ = temp_lpB_robot;
        cornerAngleInfo_.LpA_im_ = temp_lpA_im;
        cornerAngleInfo_.LpB_im_ = temp_lpB_im;
        cornerAngleInfo_.WLA_ = (0.5 * WIM + OFFIM) + Nlp_ * WWORK;
        cornerAngleInfo_.WLB_ = (0.5 * WIM + OFFIM) + Nap_ * WWORK;
        cornerAngleInfo_.F1_ = -1;
        //free space FT-CPA 使用
        cornerAngleInfo_.WLA_FS_ = -0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.WLB_FS_ = 0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.F1_WORKAREA_ = 1;
        polygonPoint  tempA,tempB;
        tempA.x = - DCRI - DWA - LWORK;
        tempA.y = 0.5 * WWORK + OFFWORK;
        tempB.x = - DCRI - DWA ;
        tempB.y = 0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.LpA_WORKAREA_ = tempA;
        cornerAngleInfo_.LpB_WORKAREA_ = tempB;
        LOG(INFO) << "FT-CPA-CV's angleInt is in THIRD_QUADRANT";
    } else {
        polygonPoint temp_lpA_robot,temp_lpB_robot,temp_lpA_im,temp_lpB_im;
        temp_lpA_robot.x = DCRF;
        temp_lpA_robot.y = 0.5 * WROBOT;
        temp_lpB_robot.x = -DCRR ;
        temp_lpB_robot.y = 0.5 * WROBOT;
        temp_lpA_im.x = -DCRI;
        temp_lpA_im.y = 0.5 * WIM + OFFIM;
        temp_lpB_im.x = -DCRI - LIM;
        temp_lpB_im.y = 0.5 * WIM + OFFIM;
        cornerAngleInfo_.LpA_robot_ = temp_lpA_robot;
        cornerAngleInfo_.LpB_robot_ = temp_lpB_robot;
        cornerAngleInfo_.LpA_im_ = temp_lpA_im;
        cornerAngleInfo_.LpB_im_ = temp_lpB_im;
        cornerAngleInfo_.WLA_ = (0.5 * WIM + OFFIM) + Nlp_ * WWORK;
        cornerAngleInfo_.WLB_ = (0.5 * WIM + OFFIM) + Nap_ * WWORK;
        cornerAngleInfo_.F1_ = 1;
        //free space FT-CPA 使用
        cornerAngleInfo_.WLA_FS_ = -0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.WLB_FS_ = (0.5 * WWORK + OFFWORK)
                                   - cos(angleInt_) * cos(angleInt_) * WWORK;
        cornerAngleInfo_.F1_WORKAREA_ = -1;
        polygonPoint  tempA,tempB;
        tempA.x = - DCRI - DWA - LWORK;
        tempA.y = -0.5 * WWORK + OFFWORK;
        tempB.x = - DCRI - DWA ;
        tempB.y = -0.5 * WWORK + OFFWORK;
        cornerAngleInfo_.LpA_WORKAREA_ = tempA;
        cornerAngleInfo_.LpB_WORKAREA_ = tempB;
        LOG(INFO) << "FT-CPA-CV's angleInt is in FOURTH_QUADRANT";
    }
    angleInt_ = fishAngleInt ;
}

void turingFtcpacvLocation::calculatePointsAandBForCurve(){
    //根据机器人计算出的A、B点
    A_robot_.x = - cornerAngleInfo_.LpA_robot_.x
           + cornerAngleInfo_.LpA_robot_.y/tan(angleInt_)
           - cornerAngleInfo_.WLA_/sin(angleInt_);
    A_robot_.y = 0;
    double angleLpB = atan(
            cornerAngleInfo_.LpB_robot_.x/cornerAngleInfo_.LpB_robot_.y);
    double angleLpBx = angleInt_
                       + cornerAngleInfo_.F1_
                         * fabs(angleLpB)
                       - cornerAngleInfo_.F1_ * 1/2 * M_PI;
    double dis_LPB = sqrt(
            cornerAngleInfo_.LpB_robot_.x * cornerAngleInfo_.LpB_robot_.x
            + cornerAngleInfo_.LpB_robot_.y * cornerAngleInfo_.LpB_robot_.y);
    B_robot_.y = sin(angleLpBx) * dis_LPB + cornerAngleInfo_.WLB_;
    B_robot_.x = B_robot_.y/tan(angleInt_);

    //根据农具计算出的A、B点
    A_im_.x = - cornerAngleInfo_.LpA_im_.x
           + cornerAngleInfo_.LpA_im_.y/tan(angleInt_)
           - cornerAngleInfo_.WLA_/sin(angleInt_);
    A_im_.y = 0;
    double angleLpB_im = atan(
            cornerAngleInfo_.LpB_im_.x/cornerAngleInfo_.LpB_im_.y);
    double angleLpBx_im = angleInt_
                       + cornerAngleInfo_.F1_
                         * fabs(angleLpB_im)
                       - cornerAngleInfo_.F1_ * 1/2 * M_PI;
    double dis_LPB_im = sqrt(
            cornerAngleInfo_.LpB_im_.x * cornerAngleInfo_.LpB_im_.x
            + cornerAngleInfo_.LpB_im_.y * cornerAngleInfo_.LpB_im_.y);
    B_im_.y = sin(angleLpBx_im) * dis_LPB_im + cornerAngleInfo_.WLB_;
    B_im_.x = B_im_.y/tan(angleInt_);

    //free space 计算出的A、B 点
    A_WORKAREA_.x = - cornerAngleInfo_.LpA_WORKAREA_.x
           + cornerAngleInfo_.LpA_WORKAREA_.y/tan(angleInt_)
           - cornerAngleInfo_.WLA_FS_/sin(angleInt_);
    A_WORKAREA_.y = 0;
    double angleLpB_wa = atan(
            cornerAngleInfo_.LpB_WORKAREA_.x/cornerAngleInfo_.LpB_WORKAREA_.y);
    double angleLpBx_wa = angleInt_
                       + cornerAngleInfo_.F1_WORKAREA_
                         * fabs(angleLpB_wa)
                       - cornerAngleInfo_.F1_WORKAREA_ * 1/2 * M_PI;
    double dis_LPB_wa = sqrt(
            cornerAngleInfo_.LpB_WORKAREA_.x * cornerAngleInfo_.LpB_WORKAREA_.x
            + cornerAngleInfo_.LpB_WORKAREA_.y * cornerAngleInfo_.LpB_WORKAREA_.y);
    B_WORKAREA_.y = sin(angleLpBx_wa) * dis_LPB_wa + cornerAngleInfo_.WLB_FS_;
    B_WORKAREA_.x = B_WORKAREA_.y/tan(angleInt_);

    //Algorithm2
    A_.x = std::min(std::min(A_robot_.x,A_im_.x),A_WORKAREA_.x);
    A_.y = 0;
    if(angleInt_ > 0 && angleInt_ < M_PI){
        B_.y = std::max(std::max(B_robot_.x,B_im_.x),B_WORKAREA_.x);
    }else if(angleInt_ > M_PI  && angleInt_ < 2 * M_PI){
        B_.y = std::min(std::min(B_robot_.y,B_im_.y),B_WORKAREA_.y);
    }
    B_.x = B_.y /tan(angleInt_);
}

polygonPoint   turingFtcpacvLocation::getCurveStartPtA(){
       return A_;
}

polygonPoint   turingFtcpacvLocation::getCurveendPtB(){
       return B_;
}

polygonPoint  turingFtcpacvLocation::getCurveStartPtARobot(){
       return A_robot_;
}

polygonPoint turingFtcpacvLocation::getCurveendPtBRobot(){
       return B_robot_;
}

polygonPoint turingFtcpacvLocation::getCurveStartPtAIm(){
      return A_im_;
}

polygonPoint turingFtcpacvLocation::getCurveEndPtBIm(){
      return B_im_;
}

polygonPoint turingFtcpacvLocation::getCurveStartPtAWorkarea(){
      return A_WORKAREA_;
}

polygonPoint turingFtcpacvLocation::getCurveEndPtBWorkarea(){
     return B_WORKAREA_;
}

double turingFtcpacvLocation::getCurveAngleInt(){
       return angleInt_;
}

double    turingFtcpacvLocation::getArriveLineHeading(){
      return arriveLineHeading_;
}