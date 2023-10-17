//
// Created by zzm on 2023/10/13.
//
//拖拉机轮廓，x轴正方向沿着尾部矩形质心到头部矩形质心，y轴正方向为沿着逆时针正交
#include "common/plot/tractorPolygonShow.h"

tractorPolygonShow::tractorPolygonShow(
                                       polygonPoint referencePt,
                                       double arriveLineHeading,
                                       std::vector<polygonPoint> orginCurvePathPts):
                                                          referencePt_(referencePt),
                                                          arriveLineHeading_(arriveLineHeading),
                                                          orginCurvePathPts_(orginCurvePathPts){
     computeLocalTractorPolygonPts();
     transferTractorPolygonPts();
}
void tractorPolygonShow::computeLocalTractorPolygonPts(){
     polygonPoint A,B,C,D,N,E,F,G,H;//N为第二个矩形的质心
     A.x = DCRF;
     A.y = TRACTOR_WIDTH/2.0;
     B.x = DCRF;
     B.y = - TRACTOR_WIDTH/2.0;
     C.x = -(TRACTOR_HEIGH - DCRF);
     C.y = TRACTOR_WIDTH/2.0;
     D.x = -(TRACTOR_HEIGH - DCRF);
     D.y = - TRACTOR_WIDTH/2.0;
     E.x = -(DCRI + DWA );
     E.y = WWORK/2.0;
     F.x = -(DCRI + DWA );
     F.y = -WWORK/2.0;
     G.x = -(DCRI + DWA + LWORK);
     G.y = WWORK/2.0;
     H.x = -(DCRI + DWA + LWORK);
     H.y = -WWORK/2.0;
     localPolyPts_.push_back(A);
     localPolyPts_.push_back(B);
     localPolyPts_.push_back(D);
     localPolyPts_.push_back(C);
     localPolyPts_.push_back(E);
     localPolyPts_.push_back(F);
     localPolyPts_.push_back(H);
     localPolyPts_.push_back(G);
}

void tractorPolygonShow::transferTractorPolygonPts(){
    //先从车体坐标系下旋转到局部坐标系下
    std::vector<polygonPoint>  storageTransPts;
    for(int i = 1;i < orginCurvePathPts_.size() ;i++){
        //计算局部坐标的heading
        polygonPoint  refer_x_vector,vector_1;
        refer_x_vector.x = 1;
        refer_x_vector.y = 0;
        vector_1.x = orginCurvePathPts_[i].x - orginCurvePathPts_[i-1].x;
        vector_1.y = orginCurvePathPts_[i].y - orginCurvePathPts_[i-1].y;
        double angle_x = common::commonMath::computeTwolineAngleDu(vector_1,refer_x_vector);
        if(angle_x < 0){
            angle_x += 360;
        }
//        //转换成弧度
        angle_x = angle_x * M_PI / 180;
//        angle_x += M_PI;
//        angle_x = - angle_x;
        for(auto m  : localPolyPts_){
            polygonPoint tempPt;
            tempPt.x = m.x * cos(angle_x) - m.y * sin(angle_x);
            tempPt.y = m.y * cos(angle_x) + m.y * sin(angle_x);
            storageTransPts.push_back(tempPt);
        }
    }
    //旋转到大地坐标系下
//     for(int  i = 0;i <  localPolyPts_.size();i++){
//         double offsetX = localPolyPts_[i].x;
//         double offsetY = localPolyPts_[i].y;
//         // 计算逆向旋转后的坐标
//         double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
//         double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;
//
//         // 将逆向旋转后的坐标转换为原始坐标系
//         polygonPoint reversedPoint;
//         reversedPoint.x = reversedX + referencePt_.x;
//         reversedPoint.y = reversedY + referencePt_.y;
////         polygonPoint reversedPoint;
////         reversedPoint.x = localPolyPts_[i].x;
////         reversedPoint.y = localPolyPts_[i].y;
//         if(i < 4){
//             tractorPolyHeadPts_.push_back(reversedPoint);
//         }else{
//             tractorPolyTailPts_.push_back(reversedPoint);
//         }
//     }
    //旋转到大地坐标系下
    for(int  i = 0;i <  storageTransPts.size();i++){
        double offsetX = storageTransPts[i].x;
        double offsetY = storageTransPts[i].y;
        // 计算逆向旋转后的坐标
        double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
        double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

        // 将逆向旋转后的坐标转换为原始坐标系
        polygonPoint reversedPoint;
        reversedPoint.x = reversedX + referencePt_.x;
        reversedPoint.y = reversedY + referencePt_.y;
//         polygonPoint reversedPoint;
//         reversedPoint.x = localPolyPts_[i].x;
//         reversedPoint.y = localPolyPts_[i].y;
        if(i < 4){
            tractorPolyHeadPts_.push_back(reversedPoint);
        }else{
            tractorPolyTailPts_.push_back(reversedPoint);
        }
    }
}

std::vector<polygonPoint> tractorPolygonShow::getTractorPolygonHeadPts(){
    return tractorPolyHeadPts_;
}

std::vector<polygonPoint> tractorPolygonShow::getTractorPolygonTailPts(){
    return   tractorPolyTailPts_;
}