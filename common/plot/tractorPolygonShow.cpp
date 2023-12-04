//
// Created by zzm on 2023/10/13.
//
//拖拉机轮廓，x轴正方向沿着尾部矩形质心到头部矩形质心，y轴正方向为沿着逆时针正交
#include "common/plot/tractorPolygonShow.h"

tractorPolygonShow::tractorPolygonShow(
                                       int index,
                                       std::vector<polygonPoint> orginCurvePathPts):
                                                          index_(index),
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
    std::vector<polygonPoint>  storageTransPts;
    polygonPoint  refer_x_vector,vector_1;
    refer_x_vector.x = 1;
    refer_x_vector.y = 0;
    vector_1.x = orginCurvePathPts_[index_].x - orginCurvePathPts_[index_-1].x;
    vector_1.y = orginCurvePathPts_[index_].y - orginCurvePathPts_[index_-1].y;
    double angle_x = common::commonMath::computeTwolineAngleDu(vector_1,refer_x_vector);
    if(angle_x < 0){
        angle_x += 360;
    }
    //转换成弧度
    angle_x = angle_x * M_PI / 180;
    for(int  m  = 0; m <  localPolyPts_.size();m++){
        polygonPoint tempPt;
        tempPt.x = localPolyPts_[m].x * cos(angle_x) + localPolyPts_[m].y * sin(angle_x);
        tempPt.y = localPolyPts_[m].y * cos(angle_x) - localPolyPts_[m].x * sin(angle_x);
        tempPt.x = tempPt.x + orginCurvePathPts_[index_].x;
        tempPt.y = tempPt.y + orginCurvePathPts_[index_].y;
        storageTransPts.push_back(tempPt);
        if(m < 4){
            tractorPolyHeadPts_.push_back(tempPt);
        }else{
            tractorPolyTailPts_.push_back(tempPt);
        }
    }
}

std::vector<polygonPoint> tractorPolygonShow::getTractorPolygonHeadPts(){
    return tractorPolyHeadPts_;
}

std::vector<polygonPoint> tractorPolygonShow::getTractorPolygonTailPts(){
    return   tractorPolyTailPts_;
}