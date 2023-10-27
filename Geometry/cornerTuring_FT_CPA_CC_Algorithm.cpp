//
// Created by zzm on 23-10-10.
//

#include "cornerTuring_FT_CPA_CC_Algorithm.h"

cornerTuringFTCPACCAlgorithm::cornerTuringFTCPACCAlgorithm(
        std::vector<polygonPoint> arriveLine,
        std::vector<polygonPoint> leaveLine):arriveLine_(arriveLine),
                                             leaveLine_(leaveLine){
    //将arriveline、leaveLine 外扩 RIDGE_WIDTH_LENGTH/2
    //利用leaveline[0]计算垂足点
    auto footPt_1 = common::commonMath::computeFootPoint(
            leaveLine_[1],
            arriveLine_[0],
            arriveLine_[1]);

    auto footPt_2 = common::commonMath::computeFootPoint(
            arriveLine_[0],
            leaveLine_[0],
            leaveLine_[1]);

    std::vector<polygonPoint>   direction_line_1,direction_line_2;

    direction_line_1.push_back(footPt_1);
    direction_line_1.push_back(leaveLine_[1]);

    direction_line_2.push_back(footPt_2);
    direction_line_2.push_back(arriveLine_[0]);

    auto  extendArriveLine = common::commonMath::computeLineTranslationPoints(
            arriveLine_,
            direction_line_1,
            0);
    auto tempArriveLine = arriveLine_;

    auto extendLeaveLine = common::commonMath::computeLineTranslationPoints(
            leaveLine_,
            direction_line_2,
            0);
    //将extendArriveLine 和extendLeaveLine两端延长20m
    auto extendArriveLine_1 = common::commonMath::findPointExtendSegment2(
            extendArriveLine[0],
            extendArriveLine[1],
            20);
    auto  extendLeaveLine_1 =  common::commonMath::findPointExtendSegment2(
            extendLeaveLine[0],
            extendLeaveLine[1],
            20);

#ifdef  DEBUG_CPA_INFO
    std::ofstream   temp;
    temp.open("/home/zzm/Desktop/test_path_figure-main/src/extendArriveAndLeavelineFTCPACC.txt",
              std::ios::out);
    temp << " " << extendArriveLine_1[0].x << " " << extendArriveLine_1[1].x <<
         " " << extendLeaveLine_1[0].x << " " << extendLeaveLine_1[1].x << std::endl;
    temp << " " << extendArriveLine_1[0].y << " " << extendArriveLine_1[1].y <<
         " " << extendLeaveLine_1[0].y << " " << extendLeaveLine_1[1].y << std::endl;
    temp.close();
#endif

    //计算extendArriveLine 和extendLeaveLine的交点
    Segment seg1(pointbst(extendArriveLine_1[0].x, extendArriveLine_1[0].y),
                 pointbst(extendArriveLine_1[1].x, extendArriveLine_1[1].y));
    Segment seg2(pointbst(extendLeaveLine_1[0].x, extendLeaveLine_1[0].y),
                 pointbst(extendLeaveLine_1[1].x, extendLeaveLine_1[1].y));
    std::deque<pointbst> output1;
    boost::geometry::intersection(seg1, seg2, output1);
    if (!output1.empty()){
        std::cout << "Intersection point: ("
                  << output1.front().x()
                  << ", "
                  << output1.front().y()
                  << ")"
                  << std::endl;
    }
    referenctPt_.x = output1.front().x();
    referenctPt_.y = output1.front().y();

    cornerTuringLocation   cornerTuringLocationtest(extendArriveLine,
                                                    extendLeaveLine);
    cornerTuringLocationtest.decideLpAandLpB();
    cornerTuringLocationtest.calculatePointsAandBForCurve();
    angleInt_ = cornerTuringLocationtest.getCurveAngleInt();
    arriveLineHeading_ = cornerTuringLocationtest.getCurveArrriveLineHeading();

    arriveLineHeading_ = arriveLineHeading_ * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading_ = -arriveLineHeading_;

    double tempfg = cornerTuringLocationtest.getCurveCCPAAngleInt();
    std::cout << "the FTCPACC angleInt is : "
              << angleInt_
              << std::endl;
    double  angleC = DBL_MAX;
    if(angleInt_ < M_PI){
        angleC = angleInt_ + M_PI;
        F2_ = 1;
    }else{
        angleC = 3 * M_PI - angleInt_;
        F2_ = -1;
    }

    polygonPoint  circle_center;
    circle_center.x = CIRCLE_RIDIS_R / (-tan(0.5 * angleC));
    circle_center.y = - CIRCLE_RIDIS_R;

    double angle_start = 0;
    double angle_end = M_PI - angleC;

//    double angle_start = 0;
//    double angle_end = 2 *  M_PI ;
    double allLength =  CIRCLE_RIDIS_R * (angle_end - angle_start);
    storageAllPath_ = common::commonMath::equalIntervalDiff(
            allLength,
            CPA_DIFF_DIS,
            angle_start,
            angle_end,
            CIRCLE_RIDIS_R,
            circle_center);

    //更新局部坐标点的方向
    for(auto & i : storageAllPath_){
          i.y = i.y * F2_;
    }

    //对路径点更新到全局坐标系下
    reprojectionFTCPACC(storageAllPath_);

    //更新整个弯道的起始点和结束点
    common::commonMath::curveStartAndEndPtUpdate(arriveLine_,
                                                 leaveLine_,
                                                 storageAllPath_);

#ifdef DEBUG_CPA_INFO
    std::string C4name = "/home/zzm/Desktop/test_path_figure-main/src/FTCPACC.txt";
    normalPrint C4file(C4name);
    C4file.writePts(storageAllPath_);
#endif

}

void cornerTuringFTCPACCAlgorithm::reprojectionFTCPACC(std::vector<polygonPoint> & pts){
    // 将坐标转换为相对于新坐标系的偏移量
    for(auto& i : pts){
        double offsetX = i.x;
        double offsetY = i.y;
        // 计算逆向旋转后的坐标
        double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
        double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

        // 将逆向旋转后的坐标转换为原始坐标系
        polygonPoint reversedPoint;
        reversedPoint.x = reversedX + referenctPt_.x;
        reversedPoint.y = reversedY + referenctPt_.y;
        i.x = reversedPoint.x;
        i.y = reversedPoint.y;
    }
}

std::vector<polygonPoint>  cornerTuringFTCPACCAlgorithm::getPath(){
    return  storageAllPath_;
}
