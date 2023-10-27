//
// Created by zzm on 23-9-22.
//

#ifndef POLYGONBACKSHAPE_CORNERTURING_IMPLEMENT_RADIUS_H
#define POLYGONBACKSHAPE_CORNERTURING_IMPLEMENT_RADIUS_H
#include <iostream>
class cornerTuringImplementRadius {
public:
    cornerTuringImplementRadius() = default;
    ~cornerTuringImplementRadius() = default;

    void calculateMiniTuringRadiusConsiderImplement();
    double getRsw();
    double getRwork();
private:
    double Rsw_ ;
    double Rwork_;
};



#endif //POLYGONBACKSHAPE_CORNERTURING_IMPLEMENT_RADIUS_H
