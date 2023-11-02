//
// Created by zzm on 2023/10/19.
//
#include "cornerTuring_FT_CPA_CV_Algorithm.h"

cornerTuringFTCPACVAlgorithm::cornerTuringFTCPACVAlgorithm( const double angleInt,
                                                            const polygonPoint referencePt,
                                                            const double arriveLineHeading,
                                                            const polygonPoint pathStartPt,
                                                            const polygonPoint pathEndPt,
                                                            const std::vector<polygonPoint> arriveLine,
                                                            const std::vector<polygonPoint> leaveLine):
                                                                angleInt_(angleInt),
                                                                referencePt_(referencePt),
                                                                arriveLineHeading_(arriveLineHeading),
                                                                pathStartPt_(pathStartPt),
                                                                pathEndPt_(pathEndPt),
                                                                arriveLine_(arriveLine),
                                                                leaveLine_(leaveLine){
     if(angleInt > 0  && angleInt < M_PI){
         F2_ = 1;
     }else if(angleInt > M_PI && angleInt < 2 * M_PI){
         F2_ = -1;
     }

     if(angleInt_ > M_PI){
         angleInt2_ = 2 * M_PI - angleInt_;
     } else {
         angleInt2_ = angleInt_;
     }
}

void cornerTuringFTCPACVAlgorithm::computeLimitPtInPart2(){
      polygonPoint pr,pim;
      pr.x = fabs(-DCRR) ;
      pr.y = fabs(-0.5 * WROBOT);
      if(angleInt_ > 0 && angleInt_ < M_PI){
          pim.x = fabs(-DCRI-LIM);
          pim.y = fabs(-0.5 *WIM + OFFIM);
      }else{
          pim.x = fabs(-DCRI -LIM);
          pim.y = fabs(0.5 * WIM + OFFIM);
      }
      double tempM = pr.x * pr.x + (pr.y + CIRCLE_RIDIS_R) * (pr.y + CIRCLE_RIDIS_R);
      double tempN = pim.x * pim.x + (pim.y + CIRCLE_RIDIS_R) * (pim.y + CIRCLE_RIDIS_R);
      double dRCC = sqrt(tempM);
      double dIMCC = sqrt(tempN);
      double dRFL = DNCZ;
      double dIFL = DNCZ + DEXTRA;
      if( -DCRI-LIM > 0 ||  dIMCC < dRCC){
          dFL_ = dRFL;
          pLIM_ = pr;
          dLIMCC_ = dRCC;
      }else{
          dFL_ = dIFL;
          pLIM_ = pim;
          dLIMCC_ = dIMCC;
      }
}


void cornerTuringFTCPACVAlgorithm::computeTheAnglesForFTCPACV(){
    angleP1_ = atan((CIRCLE_RIDIS_R + pLIM_.y)/(pLIM_.x));
    dCCYP1_ = CIRCLE_RIDIS_R + pLIM_.y + dFL_;
    double temp  = dCCYP1_/dLIMCC_;
    angleP2_ = asin(temp);
    dCCX_ = dCCYP1_/tan(angleP2_);
    angleStart_ = angleP2_ - angleP1_;
    angleEnd_ = 0.5 * M_PI - angleP1_;
    std::cout << "the FT-CPA-CV angleStart is : " << angleStart_ * 180/ M_PI <<std::endl;
    std::cout << "the FT-CPA-CV angleEnd is : " << angleEnd_ * 180/ M_PI << std::endl;
    std::cout << "the FT-CPA-CV angleInt2 is : " << angleInt2_ * 180/ M_PI << std::endl;
}

void cornerTuringFTCPACVAlgorithm::SelectTheRequiredParts(){
    if(dCCYP1_ > dLIMCC_  || angleStart_ > angleInt2_){
        parttype_ = PARTTYPE::REQUIRED_PART_1;
        std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1!" << std::endl;
    }else if(angleEnd_ > angleInt2_) {
        parttype_ = PARTTYPE::REQUIRED_PART_1_2;
        std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1_2!" << std::endl;
    }else {
        parttype_ = PARTTYPE::REQUIRED_PART_1_2_3;
        std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1_2_3" <<std::endl;
    }
}

void cornerTuringFTCPACVAlgorithm::computePath(){
    //1.生成对应的弯道path
   switch(parttype_){
       case PARTTYPE::REQUIRED_PART_1:{
           std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1 !";
           computePathAboutPart1(0,angleInt2_);
           break;
       }
       case PARTTYPE::REQUIRED_PART_1_2:{
           std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1_2 !";
           computePathAboutPart1(0,angleStart_);
           computePathAboutPart2(angleStart_,angleInt2_);
           break;
       }
       case PARTTYPE::REQUIRED_PART_1_2_3:{
           std::cout << "the FT-CPA-CV belong to REQUIRED_PART_1_2_3 !";
           computePathAboutPart1(0,angleStart_);
           computePathAboutPart2(angleStart_,angleEnd_);
           computePathAboutPart3(angleEnd_,angleInt2_);
           break;
       }
       default:{
           break;
       }
   }
    reprojectionPts(
            storageCurvePathPart1_,
            storageCurvePathPart2_,
            storageCurvePathPart3_,
            storageCurvePath_);
   //更新整个弯道的起始点和结束点
   common::commonMath::curveStartAndEndPtUpdate(arriveLine_,
                                                leaveLine_,
                                                storageCurvePath_);

   curveStartPt_ = storageCurvePath_[0];
   curveEndPt_ = storageCurvePath_[storageCurvePath_.size()-1];
   //2.生成两条直道对应的path
    computePathAboutStraightLine();
    //3.给路径点赋予弯道前进后退的属性
    for(int i = 0;i < pathStraight1_.size();i++){
         if(i == 0){
             pathStraight1_[i].pathPtType_ = pathPtType::SWITCHPT;
         }else{
             pathStraight1_[i].pathPtType_ = pathPtType::BACKWARD;
         }
    }

    for(auto i = 0; i < storageCurvePath_.size();i++){
        if(i == 0){
            storageCurvePath_[i].pathPtType_ = pathPtType::SWITCHPT;
        }else{
            storageCurvePath_[i].pathPtType_ = pathPtType::FORWARD;
        }
    }

    for(int i = 0;i < pathStraight2_.size();i++){
        if(i == 0){
            pathStraight2_[i].pathPtType_ = pathPtType::SWITCHPT;
        }else {
            pathStraight2_[i].pathPtType_ = pathPtType::BACKWARD;
        }
    }

    //4.将更新属性后点存储得到最后的path
    for(auto i : pathStraight1_){
        storaveCurvePathAll_.push_back(i);
    }
    for(auto i : storageCurvePath_){
        storaveCurvePathAll_.push_back(i);
    }
    for(auto i : pathStraight2_){
        storaveCurvePathAll_.push_back(i);
    }

}

void cornerTuringFTCPACVAlgorithm::computePathAboutStraightLine(){
        //获取第一段path,根据part的不同区分是否考虑有path2
        std::vector<polygonPoint>  path1;
        std::vector<polygonPoint>  path2;
        path1.push_back(pathStartPt_);
        path1.push_back(curveStartPt_);
        switch(parttype_){
            case PARTTYPE::REQUIRED_PART_1:{
                std::cout << "the path2 is ZREO !";
                break;
            }
            case PARTTYPE::REQUIRED_PART_1_2:
            case PARTTYPE::REQUIRED_PART_1_2_3:{
                path2.push_back(curveEndPt_);
                path2.push_back(pathEndPt_);
            }
        }

        double dis1 = common::commonMath::distance2(path1[0],path1[1]);
        double count  = dis1/CPA_DIFF_DIS;
        pathStraight1_ = common::commonMath::densify2(path1,count);
        if(!path2.empty()){
            double dis2 = common::commonMath::distance2(path2[0],path2[1]);
            double count2 = dis2/CPA_DIFF_DIS;
            pathStraight2_ = common::commonMath::densify2(path2,count2);
        }
}

void cornerTuringFTCPACVAlgorithm::computePathAboutPart1(double angleStart,double angleEnd){
    double angle_start = angleStart;
    double angle_end = angleEnd;

    double allLength =  CIRCLE_RIDIS_R * (angle_end - angle_start);

    int num_samples = std::ceil(fabs(allLength)/CPA_DIFF_DIS); //向上取整
    double angle_increment =
            (angle_end - angle_start ) / (num_samples - 1);

    //存储采样点的容器
    std::vector<polygonPoint> sample_points;

    for(int i = 0;i < num_samples -1 ;i++){
        double current_angle = angle_start + i * angle_increment;
        double x = CIRCLE_RIDIS_R * sin(current_angle);
        double y = F2_ * CIRCLE_RIDIS_R * (1 - cos(current_angle));
        polygonPoint tempPt(x,y);
        sample_points.push_back(tempPt);
    }

    //对最后一段进行10等分做特殊处理
    double temp_last_start_angle = angle_start + (num_samples - 1) * angle_increment;
    double last_angle_increment =
            angle_increment/100;
    for(int i = 99;i < 100;i++){
        double current_angle = temp_last_start_angle + last_angle_increment * i;
        double x = CIRCLE_RIDIS_R * sin(current_angle);
        double y = F2_ * CIRCLE_RIDIS_R * (1 - cos(current_angle));
        polygonPoint  tempPt(x,y);
        sample_points.push_back(tempPt);
    }
    storageCurvePath_ = sample_points;
    storageCurvePathPart1_ = sample_points;

}

void cornerTuringFTCPACVAlgorithm::computePathAboutPart2(
                                                         double angleStart,
                                                         double angleEnd){
    double angle_start = angleStart;
    double angle_end = angleEnd;

    double allLength =  CIRCLE_RIDIS_R * (angle_end - angle_start);

    int num_samples = std::ceil(fabs(allLength)/CPA_DIFF_DIS); //向上取整
    double angle_increment =
            (angle_end - angle_start ) / (num_samples - 1);

    //存储采样点的容器
    std::vector<polygonPoint> sample_points;

    double dLPCR = sqrt(pLIM_.x * pLIM_.x + pLIM_.y * pLIM_.y);
    double angleLPCR = atan(pLIM_.y/pLIM_.x);
    for(int i = 0;i < num_samples -1 ;i++){
        double current_angle = angle_start + i * angle_increment;
        //计算xlp(anglehp2)
        double xLP = pLIM_.x *
                log(sin(angleStart_) * (cos(current_angle) - 1)/(sin(current_angle) * (cos(angleStart_) - 1)))
                - dCCX_;
        double x = xLP + dLPCR * cos(current_angle + angleLPCR);
        double y = F2_ * (dLPCR * sin(current_angle + angleLPCR) - pLIM_.y - dFL_);
        polygonPoint tempPt(x,y);
        sample_points.push_back(tempPt);
    }

    //对最后一段进行10等分做特殊处理
//    double temp_last_start_angle = angle_start + (num_samples - 1) * angle_increment;
//    double last_angle_increment =
//            angle_increment/100;
//    for(int i = 99;i < 100;i++){
//        double current_angle = temp_last_start_angle + last_angle_increment * i;
//        //计算xlp(anglehp2)
//        double xLP = pLIM_.x *
//                     log(fabs(sin(angleStart_) * (cos(current_angle) - 1)/(sin(current_angle) * (cos(angleStart_) - 1))))
//                     - dCCX_;
//        double x = xLP + dLPCR * cos(current_angle + angleLPCR);
//        double y = F2_ * (dLPCR * sin(current_angle + angleLPCR) - pLIM_.y - dFL_);
//        polygonPoint  tempPt(x,y);
//        sample_points.push_back(tempPt);
//    }

    for(auto it : sample_points){
        storageCurvePath_.push_back(it);
    }
    storageCurvePathPart2_ = sample_points;
}

void cornerTuringFTCPACVAlgorithm::computePathAboutPart3(
                                                            double angleStart,
                                                            double angleEnd){
    double angle_start = angleStart;
    double angle_end = angleEnd;

    double allLength =  CIRCLE_RIDIS_R * (angle_end - angle_start);
    int num_samples = std::ceil(fabs(allLength)/CPA_DIFF_DIS); //向上取整
    double angle_increment =
            (angle_end - angle_start ) / (num_samples - 1);
    //存储采样点的容器
    std::vector<polygonPoint> sample_points;
    double xLPAngleend = pLIM_.x
            * log(fabs(sin(angleStart_) * (cos(angleEnd_) - 1)/(sin(angleEnd_) * (cos(angleStart_) - 1))))
            -dCCX_;
    for(int i = 1;i < num_samples -1 ;i++){
        double current_angle = angle_start + i * angle_increment;
        double x = xLPAngleend + CIRCLE_RIDIS_R * sin(current_angle) ;
        double y = F2_ * (dLIMCC_ - dCCYP1_ + CIRCLE_RIDIS_R * (1 - cos(current_angle)));
        polygonPoint tempPt(x,y);
        sample_points.push_back(tempPt);
    }

    //对最后一段进行10等分做特殊处理
    double temp_last_start_angle = angle_start + (num_samples - 1) * angle_increment;
    double last_angle_increment =
            angle_increment/100;
    for(int i = 99;i < 100;i++){
        double current_angle = temp_last_start_angle + last_angle_increment * i;
        double x = xLPAngleend + CIRCLE_RIDIS_R * sin(current_angle) ;
        double y = F2_ * (dLIMCC_ - dCCYP1_ + CIRCLE_RIDIS_R * (1 - cos(current_angle)));
        polygonPoint  tempPt(x,y);
        sample_points.push_back(tempPt);
    }

    for(auto it : sample_points){
        storageCurvePath_.push_back(it);
    }
    storageCurvePathPart3_ = sample_points;
}

void cornerTuringFTCPACVAlgorithm::reprojectionPts(
                             std::vector<polygonPoint> & storageCurvePathPart1,
                             std::vector<polygonPoint> & storageCurvePathPart2,
                             std::vector<polygonPoint> & storageCurvePathPart3,
                             std::vector<polygonPoint> & storageCurvePath){
    //计算第一次坐标转换
    double xShift;
    //根据不同的part选择不同的局部坐标原点
    switch(parttype_){
        case PARTTYPE::REQUIRED_PART_1:{
            double x = CIRCLE_RIDIS_R * sin(angleInt2_);
            double y = F2_ * CIRCLE_RIDIS_R * (1 - cos(angleInt2_));
            polygonPoint crEndPt(x,y);
            xShift = -crEndPt.x + crEndPt.y / tan(angleInt_);
            break;
        }
        case PARTTYPE::REQUIRED_PART_1_2:{
            double dLPCR = sqrt(pLIM_.x * pLIM_.x + pLIM_.y * pLIM_.y);
            double angleLPCR = atan(pLIM_.y/pLIM_.x);
            double xLP = pLIM_.x *
                         log(fabs(sin(angleStart_) * (cos(angleInt2_) - 1)/(sin(angleInt2_) * (cos(angleStart_) - 1))))
                         - dCCX_;
            double x = xLP + dLPCR * cos(angleInt2_ + angleLPCR);
            double y = F2_ * (dLPCR * sin(angleInt2_ + angleLPCR) - pLIM_.y - dFL_);
            polygonPoint crEndPt(x,y);
            xShift = -crEndPt.x + crEndPt.y / tan(angleInt_);
            break;
        }
        case PARTTYPE::REQUIRED_PART_1_2_3:{
            double xLP = pLIM_.x *
                         log(fabs(sin(angleStart_) * (cos(angleEnd_) - 1)/(sin(angleEnd_) * (cos(angleStart_) - 1))))
                         - dCCX_;
            double x = xLP + CIRCLE_RIDIS_R * sin(angleInt2_);
            double y = F2_ * (dLIMCC_ - dCCYP1_ + CIRCLE_RIDIS_R * (1 - cos(angleInt2_)));
            polygonPoint crEndPt(x,y);
            xShift = -crEndPt.x + crEndPt.y / tan(angleInt_);
            break;
        }
        default:{
            break;
        }
    }


     for(auto& i : storageCurvePathPart1){
         i.x =  i.x + xShift;
     }


    for(auto& i : storageCurvePathPart2){
        i.x =  i.x + xShift;
    }


    for(auto& i : storageCurvePathPart3){
        i.x =  i.x + xShift;
    }


    for(auto& i : storageCurvePath){
        i.x =  i.x + xShift;
    }

    //计算第二次坐标转换
    reprojectionGlobalPts(storageCurvePathPart1);
    reprojectionGlobalPts(storageCurvePathPart2);
    reprojectionGlobalPts(storageCurvePathPart3);
    reprojectionGlobalPts(storageCurvePath);

    //对FT-CPA-CV算法生成的路径进行路径点过滤
    //  common::commonMath::curvePtsFilter(storageCurvePathPart1);
    //  common::commonMath::curvePtsFilter(storageCurvePathPart2);
    //  common::commonMath::curvePtsFilter(storageCurvePathPart3);
    //  common::commonMath::curvePtsFilter(storageCurvePath);

}

void cornerTuringFTCPACVAlgorithm::reprojectionGlobalPts(std::vector<polygonPoint> & pts){
    for(auto & i: pts){
        // 将坐标转换为相对于新坐标系的偏移量
        double offsetX = i.x;
        double offsetY = i.y;

        // 计算逆向旋转后的坐标
        double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
        double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

        // 将逆向旋转后的坐标转换为原始坐标系
        i.x = reversedX + referencePt_.x;
        i.y = reversedY + referencePt_.y;
    }
}


std::vector<polygonPoint> cornerTuringFTCPACVAlgorithm::getPathAboutPart1(){
    return storageCurvePathPart1_;
}

std::vector<polygonPoint> cornerTuringFTCPACVAlgorithm::getPathAboutPart2(){
    return storageCurvePathPart2_;
}

std::vector<polygonPoint> cornerTuringFTCPACVAlgorithm::getPathAboutPart3(){
    return storageCurvePathPart3_;
}

std::vector<polygonPoint> cornerTuringFTCPACVAlgorithm::getPathAboutCurve(){
    return storageCurvePath_;
}

std::vector<polygonPoint>  cornerTuringFTCPACVAlgorithm::getPathStraight1(){
    return pathStraight1_;
}

std::vector<polygonPoint>  cornerTuringFTCPACVAlgorithm::getPathStraight2(){
    return pathStraight2_;
}

std::vector<polygonPoint> cornerTuringFTCPACVAlgorithm::getPathAboutALL(){
    return storaveCurvePathAll_;
}