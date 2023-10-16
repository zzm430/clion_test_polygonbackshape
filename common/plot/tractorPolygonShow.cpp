//
// Created by zzm on 2023/10/13.
//
//拖拉机轮廓，x轴正方向沿着尾部矩形质心到头部矩形质心，y轴正方向为沿着逆时针正交
#include "common/plot/tractorPolygonShow.h"

tractorPolygonShow::tractorPolygonShow(polygonPoint referencePt, double arriveLineHeading):
                                                          referencePt_(referencePt),
                                                          arriveLineHeading_(arriveLineHeading){
     computeLocalTractorPolygonPts();
     transferTractorPolygonPts();
}
void tractorPolygonShow::computeLocalTractorPolygonPts(){
     polygonPoint A,B,C,D,N,E,F,G,H;//N为第二个矩形的质心
     A.x = DCRF;
     A.y = TRACTOR_WIDTH/2;
     B.x = DCRF;
     B.y = - TRACTOR_WIDTH/2;
     C.x = -(TRACTOR_HEIGH - DCRF);
     C.y = TRACTOR_WIDTH/2;
     D.x = -(TRACTOR_HEIGH - DCRF);
     D.y = - TRACTOR_WIDTH/2;
     E.x = -(DCRI + DWA );
     E.y = WWORK/2;
     F.x = -(DCRI + DWA );
     F.y = -WWORK/2;
     G.x = -(DCRI + DWA + LWORK);
     G.y = WWORK/2;
     H.x = -(DCRI + DWA + LWORK);
     H.y = -WWORK/2;
     localPolyPts_.push_back(A);
     localPolyPts_.push_back(B);
     localPolyPts_.push_back(C);
     localPolyPts_.push_back(D);
     localPolyPts_.push_back(E);
     localPolyPts_.push_back(F);
     localPolyPts_.push_back(G);
     localPolyPts_.push_back(H);
}

void tractorPolygonShow::transferTractorPolygonPts(){
     for(int  i = 0;i <  localPolyPts_.size();i++){
         double offsetX = localPolyPts_[i].x;
         double offsetY = localPolyPts_[i].y;
         // 计算逆向旋转后的坐标
         double reversedX = offsetX * cos(arriveLineHeading_) -  offsetY * sin(arriveLineHeading_);
         double reversedY = offsetY * cos(arriveLineHeading_) +  offsetX * sin(arriveLineHeading_) ;

         // 将逆向旋转后的坐标转换为原始坐标系
         polygonPoint reversedPoint;
         reversedPoint.x = reversedX + referencePt_.x;
         reversedPoint.y = reversedY + referencePt_.y;
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