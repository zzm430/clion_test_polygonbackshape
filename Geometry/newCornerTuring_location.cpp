//
// Created by zzm on 2023/9/6.
//
#include "newCornerTuring_location.h"
newCornerTuringLocation::~newCornerTuringLocation() {

};

newCornerTuringLocation::newCornerTuringLocation(
        std::vector<polygonPoint> arriveLine,
        std::vector<polygonPoint> leaveLine){
    //arriveLine描述起始向量
    //leaveLine描述结束向量
    //计算向量的夹角转换为0到2MPI之间
    polygonPoint vector_1,vector_2;
    vector_1.x = arriveLine[0].x - arriveLine[1].x;
    vector_1.y = arriveLine[0].y - arriveLine[1].y;
    vector_2.x = leaveLine[1].x - leaveLine[0].x;
    vector_2.y = leaveLine[1].y - leaveLine[0].y;
    double angle = common::commonMath::computeTwolineAngleDu(vector_1,vector_2);
    if(fabs(angle) >0 && fabs(angle) < 90){
        angleType_ = curveAngleType::ANGLE_ACUTE;
        angleInt_ = angle * M_PI / 180;
    }else if(fabs(angle) >=90 < 180){
        angleType_ = curveAngleType::ANGLE_OBTUSE;
        angleInt_ = angle * M_PI / 180;
    }
    LOG(INFO) << "the arrive line and leave line angle is : " << angleInt_
              << " du is : "
              << angleInt_ * 180 / M_PI ;
    computeLpAandLpB(arriveLine,leaveLine);
};


void newCornerTuringLocation::computeLpAandLpB(
        std::vector<polygonPoint> arriveLine,
        std::vector<polygonPoint> leaveLine){
     auto line1 =   common::commonMath::computeLineTranslationPoints(
             arriveLine,
             leaveLine,
             2);
     std::vector<polygonPoint>  tempArriveLine;
     tempArriveLine = arriveLine;
     std::reverse(tempArriveLine.begin(),tempArriveLine.end());
     auto line2 =  common::commonMath::computeLineTranslationPoints(
               leaveLine,
               tempArriveLine,
               2);
     std::cout << "line 1 is : " << line1[0].x << " " << line1[0].y  << " " << line1[1].x << " " << line1[1].y << std::endl;
     std::cout << "line 2 is : " << line2[0].x << " " << line2[0].y  << " " << line2[1].x << " " << line2[1].y << std::endl;
     //求两个线段之间的交点
    segmentm seg1(pointm(line1[0].x, line1[0].y), pointm(line1[1].x, line1[1].y));
    segmentm seg2(pointm(line2[0].x, line2[0].y), pointm(line2[1].x, line2[1].y));
    std::deque<pointm> output;
    boost::geometry::intersection(seg1, seg2, output);
    if (!output.empty()){
        std::cout << "Intersection point1: ("
                  << output.front().x()
                  << ", "
                  << output.front().y()
                  << ")"
                  << std::endl;
    }else{
        std::cout << "No intersection point found." << std::endl;
    }
    //计算该点到arriveline的垂足点
    polygonPoint footPt = common::commonMath::computeFootPoint(
            polygonPoint(output.front().x(),output.front().y()),
            arriveLine[0],
            arriveLine[1]);
    std::cout <<"the foot pt is : " << footPt.x << " " << footPt.y  << std::endl;
    //计算B点坐标
    auto ptAs = common::commonMath::findPointExtendSegment(
            arriveLine[0],
            footPt,
            DIS_2,
            true,
            1);
    startA_.x = ptAs[0].x;
    startA_.y = ptAs[0].y;
    std::cout << "the A is : " << startA_.x << " " << startA_.y << std::endl;
    switch(angleType_){
        case curveAngleType::ANGLE_ACUTE:{
            //将arriveline外扩2m，leaveline外扩2m
            std::vector<polygonPoint>  tempLeaveLine;
            tempLeaveLine  = leaveLine;
            std::reverse(tempLeaveLine.begin(),tempLeaveLine.end());
            auto line3 =  common::commonMath::computeLineTranslationPoints(
                    arriveLine,
                    tempLeaveLine,
                    2);
            auto line4 = common::commonMath::computeLineTranslationPoints(
                    leaveLine,
                    arriveLine,
                    2);
            auto line3_transd = common::commonMath::findPointExtendSegment2(line3[0],line3[1],10);
            auto line4_transd = common::commonMath::findPointExtendSegment2(line4[0],line4[1],10);
            std::cout << "line f1 is : " << line3_transd[0].x << " " << line3_transd[0].y  << " " << line3_transd[1].x << " " << line3_transd[1].y << std::endl;
            std::cout << "line f2 is : " << line4_transd[0].x << " " << line4_transd[0].y  << " " << line4_transd[1].x << " " << line4_transd[1].y << std::endl;
            //求两个线段之间的交点
            segmentm seg3(pointm(line3_transd[0].x, line3_transd[0].y), pointm(line3_transd[1].x, line3_transd[1].y));
            segmentm seg4(pointm(line4_transd[0].x, line4_transd[0].y), pointm(line4_transd[1].x, line4_transd[1].y));
            std::deque<pointm> output1;
            boost::geometry::intersection(seg3, seg4, output1);
            if (!output1.empty()){
                std::cout << "Intersection point: ("
                          << output1.front().x()
                          << ", "
                          << output1.front().y()
                          << ")"
                          << std::endl;
            }
            //求该交点与线段leaveline做垂点
            polygonPoint footPt = common::commonMath::computeFootPoint(
                    polygonPoint(output1.front().x(),output1.front().y()),
                    leaveLine[0],
                    leaveLine[1]);
            //计算B点坐标
            polygonPoint ptB = common::commonMath::findPointOnSegment(
                    footPt,
                    leaveLine[1],
                        DIS_1,
                    true);
            endB_ = ptB;
            std::cout <<"the foot pt  B is : " << footPt.x << " " << footPt.y  << std::endl;
            std::cout << "the B is : " << endB_.x << " " << endB_.y << std::endl;
            break;
        }
        case curveAngleType::ANGLE_OBTUSE:{
            //arrive line 外扩2m
            std::vector<polygonPoint>  tempLeaveLine;
            tempLeaveLine  = leaveLine;
            std::reverse(tempLeaveLine.begin(),tempLeaveLine.end());
            auto line5 = common::commonMath::computeLineTranslationPoints(
                    arriveLine,
                    tempLeaveLine,
                    2);
            //live line 沿着固定方向延长 10m
            auto longlinepts = common::commonMath::findPointExtendSegment2(leaveLine[0],leaveLine[1],10);
            //longlinepts 内缩2m
            std::vector<polygonPoint>  tempArriveLine;
            tempArriveLine = arriveLine;
            std::reverse(tempArriveLine.begin(),tempArriveLine.end());
            auto line6 = common::commonMath::computeLineTranslationPoints(
                    longlinepts,
                    tempArriveLine,
                    2);
            //计算两条线段交点
            std::cout << "line m1 is : " << line5[0].x << " " << line5[0].y  << " " << line5[1].x << " " << line5[1].y << std::endl;
            std::cout << "line m2 is : " << line6[0].x << " " << line6[0].y  << " " << line6[1].x << " " << line6[1].y << std::endl;
            segmentm seg3(pointm(line5[0].x, line5[0].y), pointm(line5[1].x, line5[1].y));
            segmentm seg4(pointm(line6[0].x, line6[0].y), pointm(line6[1].x, line6[1].y));
            std::deque<pointm> output1;
            boost::geometry::intersection(seg3, seg4, output1);
            if (!output1.empty()){
                std::cout << "Intersection point: ("
                          << output1.front().x()
                          << ", "
                          << output1.front().y()
                          << ")"
                          << std::endl;
            }
            //求该交点与leaveline的垂足点
            polygonPoint footPt = common::commonMath::computeFootPoint(
                    polygonPoint(output1.front().x(),output1.front().y()),
                    leaveLine[0],
                    leaveLine[1]);
            polygonPoint ptB = common::commonMath::findPointOnSegment(
                    footPt,
                    leaveLine[1],
                    DIS_1,
                    true);
            endB_ = ptB;
            std::cout << "the foot B pt is : " << footPt.x << " " << footPt.y << std::endl;
            std::cout << "the end B pt is : " << ptB.x << " "<< ptB.y << std::endl;
            break;
        }
        default:{
            break;
        }
    }
}

polygonPoint  newCornerTuringLocation::getCurveStartAPt(){
    return startA_;
}

polygonPoint  newCornerTuringLocation::getCurveEndBPt(){
    return endB_;
}
