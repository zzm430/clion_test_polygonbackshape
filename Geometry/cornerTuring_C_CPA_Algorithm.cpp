//
// Created by zzm on 23-9-22.
//
#include<math.h>
#include "cornerTuring_C_CPA_Algorithm.h"
#include "common/utilpath/path_polygonPoint.h"
#include "common/math/common_math.h"

cornerTuringCCPAAlgorithm::cornerTuringCCPAAlgorithm(
                                                  double angleInt,
                                                  double Rsw,
                                                  double Nswath,
                                                  bool arriveAndLeaveAngleType,
                                                  polygonPoint fieldCornerPt,
                                                  double arriveLineHeading)
                                                  :angleInt_(angleInt),
                                                   Rsw_(Rsw),
                                                   Nswath_(Nswath),
                                                   arriveAndLeaveAngleType_(arriveAndLeaveAngleType),
                                                   fieldCornerPt_(fieldCornerPt),
                                                   arriveLineHeading_(arriveLineHeading){
    double angleC;
    if(JUDGE_CLOCKWISE){
        if(angleInt_>= M_PI) {
            angleC = angleInt_ - M_PI;
        }else{
            angleC = angleInt_ + M_PI;
        }
    }else{
        if(angleInt_ <= M_PI){
            angleC = M_PI - angleInt_;
        }else{
            angleC = 3 * M_PI - angleInt;
        }
    }
    angleOC_ = 2 * M_PI - angleC;
    angleC_= angleC;
}


void  cornerTuringCCPAAlgorithm::calculateAngleCC2() {

    double dC23fb = (Rsw_ * sin(0.5 * angleOC_) * (2 * Rsw_ + SET_HEADLAND_WIDTH_WHL))/
            (Rsw_ + SET_HEADLAND_WIDTH_WHL + Rsw_ * sin(0.5 * angleOC_));

    double dC32fb = 2 * Rsw_ + SET_HEADLAND_WIDTH_WHL - dC23fb;

    angleCC2_ = asin((Rsw_ + SET_HEADLAND_WIDTH_WHL)/dC32fb);
}

void cornerTuringCCPAAlgorithm::calculateCirclesCenter(){
    circleC2_center_.x = - cos(0.5 * angleOC_) * Rsw_;
    circleC2_center_.y = - sin(0.5 * angleOC_) * Rsw_;

    circleC1_center_.x = circleC2_center_.x - cos(angleCC2_) * (2 * Rsw_ + SET_HEADLAND_WIDTH_WHL);
    circleC1_center_.y = circleC2_center_.y + sin(angleCC2_) * (2 * Rsw_ + SET_HEADLAND_WIDTH_WHL);

    circleC3_center_.x =
            circleC2_center_.x + cos(angleCC2_ - M_PI + angleOC_) * (2 * Rsw_ + SET_HEADLAND_WIDTH_WHL);
    circleC3_center_.y =
            circleC2_center_.y + sin(angleCC2_ - M_PI + angleOC_) * (2 * Rsw_ + SET_HEADLAND_WIDTH_WHL);

    circleCV_center_.x = - (Rsw_ + SET_HEADLAND_WIDTH_WHL)/tan(0.5 * angleC_);
    circleCV_center_.y = Rsw_ + SET_HEADLAND_WIDTH_WHL;
}


void cornerTuringCCPAAlgorithm::calculateNewFieldBorder(){

    if(!arriveAndLeaveAngleType_){
        double angleC1_start = M_PI;
        double angleC1_end = 0.5 * M_PI + angleCC2_;

        double angleC2_start = -0.5 * M_PI + angleCC2_;
        double angleC2_end = 1.5 * M_PI - angleCC2_ - angleOC_;

        double angleC3_start = 2.5 * M_PI - angleCC2_ - angleOC_;
        double angleC3_end = angleC_;

        double C1_allLength = (Rsw_ + SET_HEADLAND_WIDTH_WHL) * (angleC1_end - angleC1_start);
        double C2_allLength = Rsw_ * (angleC2_end - angleC2_start);
        double C3_allLength = (Rsw_ + SET_HEADLAND_WIDTH_WHL)  * (angleC3_end - angleC3_start);

        //等间隔采样点
        auto C1_pts = common::commonMath::equalIntervalDiff(C1_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC1_start,
                                                            angleC1_end,
                                                            Rsw_ + SET_HEADLAND_WIDTH_WHL,
                                                            circleC1_center_);
        auto C2_pts = common::commonMath::equalIntervalDiff(C2_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC2_start,
                                                            angleC2_end,
                                                            Rsw_,
                                                            circleC2_center_);

        auto C3_pts = common::commonMath::equalIntervalDiff(C3_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC3_start,
                                                            angleC3_end,
                                                            Rsw_ + SET_HEADLAND_WIDTH_WHL,
                                                            circleC3_center_);

        std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA1border.txt";
        std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA2border.txt";
        std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA3border.txt";
        normalPrint C1file(C1name);
        normalPrint C2file(C2name);
        normalPrint C3file(C3name);
        C1file.writePts(C1_pts);
        C2file.writePts(C2_pts);
        C3file.writePts(C3_pts);
        //后续需要对这个路径进行裁剪

    }else{
        double angleCV_start = M_PI;
        double angleCV_end = angleC_;

        double C4_allLength = (Rsw_ + SET_HEADLAND_WIDTH_WHL)  * (angleCV_end - angleCV_start);

        auto CV_pts = common::commonMath::equalIntervalDiff(C4_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleCV_start,
                                                            angleCV_end,
                                                            Rsw_ + SET_HEADLAND_WIDTH_WHL,
                                                            circleCV_center_);
        std::string C4name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA4border.txt";
        normalPrint C1file(C4name);
        C1file.writePts(CV_pts);

    }

}

void cornerTuringCCPAAlgorithm::calculatePath(){

    double circleC1_R,circleC2_R, circleC3_R,circleCV_R;
    if(JUDGE_CLOCKWISE){  //逆时针
        circleC1_R = Rsw_ + (0.5 * WWORK + OFFWORK) + (Nswath_ - 1) * WWORK;
        circleC2_R = Rsw_ + SET_HEADLAND_WIDTH_WHL - (0.5 * WWORK + OFFWORK) - (Nswath_ - 1) * WWORK;
        circleC3_R = Rsw_ +  (0.5 * WWORK + OFFWORK) +  (Nswath_ - 1) * WWORK;
        circleCV_R = Rsw_ + (0.5 * WWORK + OFFWORK) +  (Nswath_ - 1) * WWORK;
    }else{
        circleC1_R = Rsw_ - (-0.5 * WWORK + OFFWORK) + (Nswath_ - 1) * WWORK;
        circleC2_R = Rsw_ + SET_HEADLAND_WIDTH_WHL + (-0.5 * WWORK + OFFWORK) - (Nswath_ - 1) * WWORK;
        circleC3_R = Rsw_ - (-0.5 * WWORK + OFFWORK) + (Nswath_ - 1) * WWORK;
        circleCV_R = Rsw_ - (-0.5 * WWORK + OFFWORK) + (Nswath_ - 1) * WWORK;
    }

    if(!arriveAndLeaveAngleType_){
        double angleC1_start = M_PI;
        double angleC1_end = 0.5 * M_PI + angleCC2_;

        double angleC2_start = -0.5 * M_PI + angleCC2_;
        double angleC2_end = 1.5 * M_PI - angleCC2_ - angleOC_;

        double angleC3_start = 2.5 * M_PI - angleCC2_ - angleOC_;
        double angleC3_end = angleC_;

        double C1_allLength = circleC1_R * (angleC1_end - angleC1_start);
        double C2_allLength = circleC2_R * (angleC2_end - angleC2_start);
        double C3_allLength = circleC3_R  * (angleC3_end - angleC3_start);

        //等间隔采样点
        auto C1_pts = common::commonMath::equalIntervalDiff(C1_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC1_start,
                                                            angleC1_end,
                                                            circleC1_R,
                                                            circleC1_center_);
        auto C2_pts = common::commonMath::equalIntervalDiff(C2_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC2_start,
                                                            angleC2_end,
                                                            circleC2_R,
                                                            circleC2_center_);

        auto C3_pts = common::commonMath::equalIntervalDiff(C3_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleC3_start,
                                                            angleC3_end,
                                                            circleC3_R,
                                                            circleC3_center_);
        //如果angleInt > M_PI,则将路径点y值全部乘以-1
        if(angleInt_> M_PI){
            correctForCornerOrientation(C1_pts);
            correctForCornerOrientation(C2_pts);
            correctForCornerOrientation(C3_pts);
            reprojectionCCA(C1_pts);
            reprojectionCCA(C2_pts);
            reprojectionCCA(C3_pts);
        }
        std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA1path.txt";
        std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA2path.txt";
        std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA3path.txt";
        normalPrint C1file(C1name);
        normalPrint C2file(C2name);
        normalPrint C3file(C3name);
        C1file.writePts(C1_pts);
        C2file.writePts(C2_pts);
        C3file.writePts(C3_pts);

        //后续需要裁剪处理

    }else{

        double angleCV_start = M_PI;
        double angleCV_end = angleC_;
        double C4_allLength = (Rsw_ + SET_HEADLAND_WIDTH_WHL)  * (angleCV_end - angleCV_start);
        auto CV_pts = common::commonMath::equalIntervalDiff(C4_allLength,
                                                            FISHNail_DIFF_DIS,
                                                            angleCV_start,
                                                            angleCV_end,
                                                            circleCV_R,
                                                            circleCV_center_);
        std::string C4name = "/home/zzm/Desktop/test_path_figure-main/src/CCPA4path.txt";
        normalPrint C4file(C4name);
        C4file.writePts(CV_pts);
    }
}


void cornerTuringCCPAAlgorithm::reprojectionCCA(std::vector<polygonPoint> & pts){
//    for(auto& i : pts){
//        i.x = fieldCornerPt_.x + i.x * cos(arriveLineHeading_ - 0.5 * M_PI)
//                + i.y * sin(arriveLineHeading_ - 0.5 * M_PI);
//        i.y = fieldCornerPt_.y - i.x * sin(arriveLineHeading_ - 0.5 * M_PI)
//                + i.y * cos(arriveLineHeading_ - 0.5 * M_PI);
//    }
    // 将坐标转换为相对于新坐标系的偏移量
    for(auto& i : pts){
        double offsetX = i.x;
        double offsetY = i.y;

        // 计算逆向旋转后的坐标
        double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
        double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

        // 将逆向旋转后的坐标转换为原始坐标系
        polygonPoint reversedPoint;
        reversedPoint.x = reversedX + fieldCornerPt_.x;
        reversedPoint.y = reversedY + fieldCornerPt_.y;
        i.x = reversedPoint.x;
        i.y = reversedPoint.y;
    }

}

void   cornerTuringCCPAAlgorithm::correctForCornerOrientation(std::vector<polygonPoint> & vecPts){
    for(auto& i: vecPts){
        i.y = i.y * (-1);
    }
}

polygonPoint cornerTuringCCPAAlgorithm::getCircleC1Center() {
    return circleC1_center_;
}

polygonPoint cornerTuringCCPAAlgorithm::getCircleC2Center() {
    return circleC2_center_;
}

polygonPoint cornerTuringCCPAAlgorithm::getCircleC3Center() {
    return circleC3_center_;
}

polygonPoint cornerTuringCCPAAlgorithm::getCircleCVCenter() {
    return circleCV_center_;
}