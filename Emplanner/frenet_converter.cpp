//
// Created by zzm on 23-11-14.
//


#include <Emplanner/frenet_converter.h>

//[x,y] --->[s,d]
std::pair<double ,double> FrenetConverter::cartesianToFrenet1D(
        double rs,
        double rx,
        double ry,
        double rtheta,
        double x,
        double y){

    double dx  = x  - rx;
    double dy  = y  - ry;

    double cos_theta_r = cos(rtheta);
    double sin_theta_r = sin(rtheta);

    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    double d_condition = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd);
    double s_condition = rs;

    return {s_condition,d_condition};
}


//[s,d] --->[x,y]
std::pair<double ,double> FrenetConverter::frenetToCartesian1D(
        double rs,
        double rx,
        double ry,
        double rtheta,
        double s_condition,
        double d_condition){

    if(fabs(rs - s_condition) >= 0.0001){
        std::cout << "the reference point s and s_condition[0] dont't match " << std::endl;
    }

    double cos_theta_r = cos(rtheta);
    double sin_theta_r = sin(rtheta);

    double x = rx - sin_theta_r * d_condition;
    double y = ry + cos_theta_r * d_condition;

    return {x,y};

}

//[x,y,v,theta] --->[s,s',d,d']

std::vector<std::pair<double ,double>> FrenetConverter::cartesianToFrenet2D(
        double rs,
        double rx,
        double ry,
        double rtheta,
        double rkappa,
        double x,
        double y,
        double v,
        double theta){

    double dx = x - rx;
    double dy = y - ry;

    double cos_theta_r = cos(rtheta);
    double sin_theta_r = sin(rtheta);

    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    double d0 = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd);


    double delta_theta = theta - rtheta;
    double tan_delta_theta = tan(delta_theta);
    double cos_delta_theta = cos(delta_theta);

    double one_minus_kappa_r_d = 1 - rkappa * d0;
    double d1 = one_minus_kappa_r_d * tan_delta_theta;

    double s1 =  v * cos_delta_theta / one_minus_kappa_r_d;

    std::pair<double ,double >  data1=std::make_pair(d0,d1);
    std::pair<double ,double >  data2=std::make_pair(rs,s1);

    std::vector<std::pair<double ,double>> data;
    data.push_back(data1);
    data.push_back(data2);
    return data;
}

//[s,s',d,d'] --->[x,y,v,theta]
std::vector<double> FrenetConverter::frenetToCartesian2D(
        double rs,
        double rx,
        double ry,
        double rtheta,
        double rkappa,
        std::vector<double> s_condition,
        std::vector<double> d_condition){
    if(fabs(rs - s_condition[0]) > 0.0001){
        std::cout <<" the reference point s and s_condition[0] don't match ";
    }

    double cos_theta_r = cos(rtheta);
    double sin_theta_r = sin(rtheta);

    double  x = rx - sin_theta_r * d_condition[0];
    double  y = ry + cos_theta_r * d_condition[0];

    double  one_minus_kappa_r_d = 1 - rkappa * d_condition[0];
    double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    double delta_theta = atan2(d_condition[1], one_minus_kappa_r_d);
    double cos_delta_theta = cos(delta_theta);

    double theta = normalizeAngle(delta_theta + rtheta);

    double d_dot = d_condition[1] * s_condition[1];

    double v = sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d
            * s_condition[1] * s_condition[1] + d_dot * d_dot);

    std::vector<double>  data;
    data.push_back(x);
    data.push_back(y);
    data.push_back(v);
    data.push_back(theta);
    return data;
}

//[x,y,v,theta,k,a]--->[s,s',s'',d,d',d'']
std::vector<std::pair<double ,double>> FrenetConverter::cartesianToFrenet3D(
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
        double kappa){
    double  dx = x - rx;
    double  dy = y - ry;

    double cos_theta_r = cos(rtheta);
    double sin_theta_r = sin(rtheta);

    double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    double d0 = copysign(sqrt(dx * dx + dy * dy), cross_rd_nd);

    double delta_theta = theta - rtheta;
    double tan_delta_theta = tan(delta_theta);
    double cos_delta_theta = cos(delta_theta);

    double one_minus_kappa_r_d = 1 - rkappa * d0;
    double d1 = one_minus_kappa_r_d * tan_delta_theta;

    double kappa_r_d_prime = rdkappa * d0 + rkappa * d1;

    double d2 = (-kappa_r_d_prime * tan_delta_theta +
                      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
                      (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa));
     double s0 = rs;
     double s1 = v * cos_delta_theta / one_minus_kappa_r_d;
     double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
     double s2 = ((a * cos_delta_theta -
                   s1 * s1 *
                   (d1 * delta_theta_prime - kappa_r_d_prime)) /
                  one_minus_kappa_r_d);
     std::vector<std::pair<double ,double>> data;
     data.push_back({s0,d0});
     data.push_back({s1,d1});
     data.push_back({s2,d2});
     return  data;

}

//[s,s',s'',d,d',d'']--->[x,y,v,theta,k,a]

std::vector<double>  FrenetConverter::frenetToCartesian3D(
        double rs,
        double rx,
        double ry,
        double rtheta,
        double rkappa,
        double rdkappa,
        std::vector<double> s_condition,
        std::vector<double> d_condition){
    if(fabs(rs - s_condition[0])>= 0.0001){
        std::cout << "the reference point s and s_condition[0] don't match" << std::endl;
    }

   double cos_theta_r = cos(rtheta);
   double sin_theta_r = sin(rtheta);

   double  x = rx - sin_theta_r * d_condition[0];
   double y = ry + cos_theta_r * d_condition[0];

   double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];
   double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
   double delta_theta = atan2(d_condition[1], one_minus_kappa_r_d);
   double cos_delta_theta = cos(delta_theta);

   double theta = normalizeAngle(delta_theta + rtheta);
   double kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1];

   double kappa = ((((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
               cos_delta_theta * cos_delta_theta) /
              (one_minus_kappa_r_d) +
              rkappa) *
             cos_delta_theta / (one_minus_kappa_r_d));


   double d_dot = d_condition[1] * s_condition[1];

   double v = sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot);

   double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (kappa) - rkappa;
   double a = (s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
         s_condition[1] * s_condition[1] / cos_delta_theta *
         (d_condition[1] * delta_theta_prime - kappa_r_d_prime));

  // return x, y, v, a, theta, kappa
  std::vector<double>  data;
  data.push_back(x);
  data.push_back(y);
  data.push_back(v);
  data.push_back(a);
  data.push_back(theta);
  data.push_back(kappa);

  return data;

}


// alpha = np.arctan2(dnewy, dnewx)
// kappa = (ddnewx*dnewy-ddnewy*dnewx)/(dnewx*dnewx+dnewy*dnewy)**(3.0/2.0)
// norm to [-pi, pi]
double FrenetConverter::normalizeAngle(double angle){
    double a = fmod(angle + M_PI,2 * M_PI);
    if(a < 0.0){
        a += 2* M_PI;
    }
    return a - M_PI;
}

