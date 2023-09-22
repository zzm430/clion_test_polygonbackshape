//
// Created by zzm on 23-9-22.
//
#include <cmath>
#include "cornerTuring_Implement_Radius.h"
#include "common/common_param/common_parameters.h"

void  cornerTuringImplementRadius::calculateMiniTuringRadiusConsiderImplement() {
    double WEFF =  MINEFF * WWORK;
    double DCRCW = DCRI + DWA + 0.5 * LWORK;
    double a = 1 - 1/(MINEFF * MINEFF);
    double b = WWORK - WWORK / (MINEFF * MINEFF);
    double c = DCRCW * DCRCW + (0.5 - 1/(4 * MINEFF * MINEFF) - (MINEFF * MINEFF)/4) * WWORK * WWORK;
    double dccwy = (-b - sqrt(b * b - 4 * a * c))/(2 * a);

    Rsw_ = sqrt(dccwy * dccwy + DCRCW * DCRCW);
    Rwork_ = sqrt((dccwy + WWORK) * (dccwy + WWORK) + DCRCW * DCRCW);
}

double cornerTuringImplementRadius::getRsw() {
    return Rsw_;
}

double cornerTuringImplementRadius::getRwork() {
    return Rwork_;
}