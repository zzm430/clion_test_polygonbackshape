//
// Created by zzm on 23-11-14.
//

#ifndef POLYGONBACKSHAPE_FRENET_CONVERTER_H
#define POLYGONBACKSHAPE_FRENET_CONVERTER_H

#include <iostream>
#include <common/utilpath/path_polygonPoint.h>

class FrenetConverter{
public:
    FrenetConverter() = default;
    virtual ~FrenetConverter() = default;

    //[x,y] --->[s,d]
   static  std::pair<double ,double> cartesianToFrenet1D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double x,
           double y);


   //[s,d] --->[x,y]
   static std::pair<double ,double> frenetToCartesian1D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double s_condition,
           double d_condition);

   //[x,y,v,theta] --->[s,s',d,d']

    static std::vector<std::pair<double ,double>> cartesianToFrenet2D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double rkappa,
           double x,
           double y,
           double v,
           double theta);

   //[s,s',d,d'] --->[x,y,v,theta]
   static std::vector<double> frenetToCartesian2D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double rkappa,
           std::vector<double> s_condition,
           std::vector<double> d_condition);

   //[x,y,v,theta,k,a]--->[s,s',s'',d,d',d'']
   static  std::vector<std::pair<double ,double>> cartesianToFrenet3D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double rkappa,
           double rdkappa,
           double x,
           double y,
           double v,
           double a,
           double theta,
           double kappa);

   //[s,s',s'',d,d',d'']--->[x,y,v,theta,k,a]

    static std::vector<double>  frenetToCartesian3D(
           double rs,
           double rx,
           double ry,
           double rtheta,
           double rkappa,
           double rdkappa,
           std::vector<double> s_condition,
           std::vector<double> d_condition);

   static  double normalizeAngle(double angle);



};


#endif //POLYGONBACKSHAPE_FRENET_CONVERTER_H
