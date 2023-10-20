//
// Created by zzm on 2023/9/5.
//
#include "cornerTuring_location.h"


cornerTuringLocation::~cornerTuringLocation() {

};

cornerTuringLocation::cornerTuringLocation(
        std::vector<polygonPoint> arriveLine,
        std::vector<polygonPoint> leaveLine){
   //arriveLine描述起始向量
   //leaveLine描述结束向量
   polygonPoint vector_1,vector_2,reference_vector;
   vector_1.x = arriveLine[1].x - arriveLine[0].x;
   vector_1.y = arriveLine[1].y - arriveLine[0].y;
   vector_2.x = leaveLine[1].x - leaveLine[0].x;
   vector_2.y = leaveLine[1].y - leaveLine[0].y;
   reference_vector.x = 0;
   reference_vector.y = 1;

    //计算CCPA转弯需要的angle
    double angle_temp_1 = common::commonMath::computeTwolineAngleDu(reference_vector, vector_1);
    double angle_temp_2 = common::commonMath::computeTwolineAngleDu(reference_vector, vector_2);
    double diff_angle = angle_temp_1 * M_PI / 180 - angle_temp_2 * M_PI / 180;
    if(diff_angle > 0){
        CCPAAngleInt_= diff_angle;
    }else{
        CCPAAngleInt_ = diff_angle + 2 * M_PI;
    }
//   //
//   double angle_1 = common::commonMath::computeTwolineAngleDu(vector_1,reference_vector);
//   if(angle_1 < 0){
//       angle_1 += 360;
//   }
//    arriveLineHeading2_ = angle_1;
//   double angle_2 = common::commonMath::computeTwolineAngleDu(vector_2,reference_vector);
//   if(angle_2 < 0){
//       angle_2 += 360;
//   }

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

    //计算鱼尾需要的angle
    double fish_angle_temp_1 = common::commonMath::computeTwolineAngleDu(vector_1,reference_vector);
    double fish_angle_temp_2 = common::commonMath::computeTwolineAngleDu(vector_2,reference_vector);
    double fishAngleInt;
    double fish_diff_angle = fish_angle_temp_1 * M_PI / 180 - fish_angle_temp_2 * M_PI / 180;
    if(fish_diff_angle > 0){
        fishAngleInt = fish_diff_angle;
    }else{
        fishAngleInt = fish_diff_angle + 2 * M_PI;
    }

    //temp code
//    double angle_diff1 = angle_1 - angle_2;
//   if(angle_diff1 < 0){
//       angle_diff1 += 360;
//   }

   //按照逆时针处理
//   double angle_diff = angle_1 - angle_2;
//   if(angle_diff < 0){
//       angle_diff += 360;
//   }
     double angle_diff = fishAngleInt * 180 / M_PI;
//   LOG(INFO) << "the angle 1 2 is : "
//             << angle_1
//             << " "
//             << angle_2;
//   LOG(INFO) << "angle diff is : "
//             << angle_diff;
   if(angle_diff >0 && angle_diff < 90){
       angleType_ = angleType::FIRST_QUADRANT;
       LOG(INFO) << "FIRST_QUADRANT";
       F1_ = 1;
       F2_ = 1;
       F3_ = 0;
   }else if(angle_diff >=90  && angle_diff < 180){
       angleType_ = angleType::SECOND_QUADRANT;
       LOG(INFO) << "SECOND_QUADRANT";
       F1_ = -1;
       F2_ = 1;
       F3_ = 0;
   }else if(angle_diff >180  && angle_diff < 270){
       angleType_ = angleType::THIRD_QUADRANT;
       F1_ = 1;
       F2_ = -1;
       F3_ = 1;
       LOG(INFO) << "THIRD_QUADRANT";
   }else{
       angleType_ = angleType::FOURTH_QUADRANT;
       F1_ = -1;
       F2_ = -1;
       F3_ = 1;
       LOG(INFO) << "FOURTH_QUADRANT";
   }
   angleInt_ = fishAngleInt ;
    LOG(INFO) << "the arrive line and leave line angle du is  : "
              << angleInt_ * 180 / M_PI;
    LOG(INFO) << "the CCPA angle is : "
              << CCPAAngleInt_;
//   LOG(INFO) << "the origin curve angle is : " << angle_diff1;
//   angleInt_ = fish_diff_angle ;

};

void cornerTuringLocation::decideLpAandLpB(){
     switch(angleType_){
         case angleType::FIRST_QUADRANT:{
             cornerAngleInfoMap_.lpA_ = cornerPosition::REAR_LEFT_WORKING_AREA;
             cornerAngleInfoMap_.lpB_ = cornerPosition::FRONT_LEFT_WORKING_AREA;
             cornerAngleInfoMap_.WLA_ = 0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.WLB_ = (-0.5 * WWORK + OFFWORK)
                                        + cos(angleInt_) * cos(angleInt_) * WWORK;
             cornerAngleInfoMap_.F1_ = 1;
             polygonPoint  tempA,tempB;
             tempA.x = - DCRI - DWA - LWORK;
             tempA.y = 0.5 * WWORK + OFFWORK;
             tempB.x = - DCRI - DWA ;
             tempB.y = 0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.A_ = tempA;
             cornerAngleInfoMap_.B_ = tempB;
             break;
         }
         case angleType::SECOND_QUADRANT:{
             cornerAngleInfoMap_.lpA_ = cornerPosition::REAR_RIGHT_WORKING_AREA;
             cornerAngleInfoMap_.lpB_ = cornerPosition::FRONT_RIGHT_WORKING_AREA;
             cornerAngleInfoMap_.WLA_ = 0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.WLB_ = (-0.5 * WWORK + OFFWORK);
             cornerAngleInfoMap_.F1_ = -1;
             polygonPoint  tempA,tempB;
             tempA.x = - DCRI - DWA - LWORK;
             tempA.y = -0.5 * WWORK + OFFWORK;
             tempB.x = - DCRI - DWA ;
             tempB.y = -0.5 * WWORK + OFFWORK ;
             cornerAngleInfoMap_.A_ = tempA;
             cornerAngleInfoMap_.B_ = tempB;
             break;
         }
         case angleType::THIRD_QUADRANT:{
             cornerAngleInfoMap_.lpA_ = cornerPosition::REAR_LEFT_WORKING_AREA;
             cornerAngleInfoMap_.lpB_ = cornerPosition::FRONT_LEFT_WORKING_AREA;
             cornerAngleInfoMap_.WLA_ = -0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.WLB_ = 0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.F1_ = 1;
             polygonPoint  tempA,tempB;
             tempA.x = - DCRI - DWA - LWORK;
             tempA.y = 0.5 * WWORK + OFFWORK;
             tempB.x = - DCRI - DWA ;
             tempB.y = 0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.A_ = tempA;
             cornerAngleInfoMap_.B_ = tempB;
             break;
         }
         case angleType::FOURTH_QUADRANT:{
             cornerAngleInfoMap_.lpA_ = cornerPosition::REAR_RIGHT_WORKING_AREA;
             cornerAngleInfoMap_.lpB_ = cornerPosition::FRONT_RIGHT_WORKING_AREA;
             cornerAngleInfoMap_.WLA_ = -0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.WLB_ = (0.5 * WWORK + OFFWORK)
                                        - cos(angleInt_) * cos(angleInt_) * WWORK;
             cornerAngleInfoMap_.F1_ = -1;
             polygonPoint  tempA,tempB;
             tempA.x = - DCRI - DWA - LWORK;
             tempA.y = -0.5 * WWORK + OFFWORK;
             tempB.x = - DCRI - DWA ;
             tempB.y = -0.5 * WWORK + OFFWORK;
             cornerAngleInfoMap_.A_ = tempA;
             cornerAngleInfoMap_.B_ = tempB;
             break;
         }
         default:{
             break;
         }
     }
}

void cornerTuringLocation::calculatePointsAandBForCurve(){
        A_.x = - cornerAngleInfoMap_.A_.x
                + cornerAngleInfoMap_.A_.y/tan(angleInt_)
                - cornerAngleInfoMap_.WLA_/sin(angleInt_);
        A_.y = 0;
        double angleLpB = atan(
                cornerAngleInfoMap_.B_.x/cornerAngleInfoMap_.B_.y);
        double angleLpBx = angleInt_
                          + cornerAngleInfoMap_.F1_
                          * fabs(angleLpB)
                          - cornerAngleInfoMap_.F1_ * 1/2 * M_PI;
        double dis_LPB = sqrt(
                cornerAngleInfoMap_.B_.x * cornerAngleInfoMap_.B_.x
                + cornerAngleInfoMap_.B_.y * cornerAngleInfoMap_.B_.y);
        B_.y = sin(angleLpBx) * dis_LPB + cornerAngleInfoMap_.WLB_;
        B_.x = B_.y/tan(angleInt_);
}


polygonPoint   cornerTuringLocation::getCurveStartPtA(){
    return A_;
}

polygonPoint   cornerTuringLocation::getCurveendPtB(){
    return B_;
}

double cornerTuringLocation::getCurveAngleInt(){
    return angleInt_;
}

double cornerTuringLocation::getCurveaboutF1(){
    return  F1_;
}

double cornerTuringLocation::getCurveaboutF2(){
    return  F2_;
}

double cornerTuringLocation::getCurveaboutF3(){
    return  F3_;
}

double cornerTuringLocation::getCurveArrriveLineHeading() {
    return arriveLineHeading_;
}

double cornerTuringLocation::getCurveArrriveLineHeading2() {
    return arriveLineHeading2_;
}

double cornerTuringLocation::getCurveCCPAAngleInt(){
    return CCPAAngleInt_;
}