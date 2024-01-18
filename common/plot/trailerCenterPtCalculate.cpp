//
// Created by zzm on 24-1-16.
//
#include "common/plot/trailerCenterPtCalculate.h"

trailerCenterPtCalculate::trailerCenterPtCalculate(const polygonPoint & tractor_center_pt):
                                             tractor_center_pt_(tractor_center_pt){
    transTractorPtToTrailerPt();
}

void trailerCenterPtCalculate::transTractorPtToTrailerPt(){
    polygonPoint tempPt;
    double dis_tractorCenter_trailerCenter =  TRACTOR_CENTER_TO_TRAILER_CENTER_DIS;

    double angle = tractor_center_pt_.heading();
    trailer_center_pt_.x = -dis_tractorCenter_trailerCenter * cos(angle) ;
    trailer_center_pt_.y = -dis_tractorCenter_trailerCenter * sin(angle);
    trailer_center_pt_.x = trailer_center_pt_.x + tractor_center_pt_.x;
    trailer_center_pt_.y = trailer_center_pt_.y + tractor_center_pt_.y;
}


polygonPoint  trailerCenterPtCalculate::getTrailerCenterPt(){
    return trailer_center_pt_;
}






