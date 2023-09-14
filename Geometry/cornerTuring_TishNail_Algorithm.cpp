//
// Created by zzm on 23-9-11.
//
#include "Geometry/cornerTuring_TishNail_Algorithm.h"
cornerTuringTishNail::~cornerTuringTishNail(){

}

cornerTuringTishNail::cornerTuringTishNail(
                         polygonPoint A,
                         polygonPoint B,
                         double angleInt,
                         double& RC2,
                         double F1,
                         double F2){
    polygonPoint  pt_1,pt_2,pt_3 ;
    pt_1.x = A.x;
    pt_1.y = F2 * CIRCLE_RIDIS_R ;

    //计算第二个圆
    double dC3BX =  CIRCLE_RIDIS_R * sin(angleInt);
    double dC3BY =  CIRCLE_RIDIS_R * cos(angleInt);
    pt_3.x = B.x - dC3BX * F2;
    pt_3.y = B.y + dC3BY * F2;
    if(fabs(pt_1.x - pt_3.x) < 0.01){
        pt_2.y = 0.5 * (pt_1.y + pt_3.y);
        double tempVal = (RC2 + CIRCLE_RIDIS_R) * (RC2 + CIRCLE_RIDIS_R) - (pt_1.y - pt_2.y) * (pt_1.y - pt_2.y);
        pt_2.x = pt_1.x - F1 * sqrt(tempVal);
    }else if(fabs(pt_1.y - pt_3.y) < 0.01){
        pt_2.x = 0.5 * (pt_1.x + pt_3.x);
        double tempV =  (RC2 + CIRCLE_RIDIS_R) * (RC2 + CIRCLE_RIDIS_R) - (pt_1.x - pt_2.x) * (pt_1.x - pt_2.x );
        pt_2.y = pt_1.y - F2 * sqrt(tempV);
    }else{
       double a1 = - (pt_3.x - pt_1.x)/(pt_3.y - pt_1.y);
       double b1 = 0.5 * (pt_1.y + pt_3.y + (pt_3.x - pt_1.x) * (pt_3.x + pt_1.x)/(pt_3.y - pt_1.y));
       //计算c2
       double a = 1 + a1 * a1;
       double b = -2 * pt_1.x - 2 * a1 * pt_1.y + 2 * a1 * b1;
       double c = pt_1.x * pt_1.x + pt_1.y * pt_1.y + b1 * b1 - 2*b1*pt_1.y - CIRCLE_RIDIS_R * CIRCLE_RIDIS_R
                  - RC2 * RC2 - 2 * CIRCLE_RIDIS_R * RC2;
       if((pt_1.y > 0 && pt_1.y > pt_3.y) || (pt_1.y < 0 && pt_1.y < pt_3.y)){
           pt_2.x = (-b + sqrt(b * b - 4 * a * c))/(2 * a);
       }else{
           pt_2.x = (-b - sqrt(b * b - 4 * a * c))/(2 * a);
       }
       pt_2.y = pt_2.x * a1 + b1;
    }
    storage_circle_center_.push_back(pt_1);
    storage_circle_center_.push_back(pt_2);
    storage_circle_center_.push_back(pt_3);

    //计算出相切时圆C2的半径
    int insec_pt = 0;
    double count_upper = 0;

    double order_R = 6;
    double trans_R = 6;
    for(int i = 0;i < 1000;i++){
         double buffer_distance_1 = trans_R ;             // radius of circle
         double buffer_distance_2 = order_R ;
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance_1);
        boost::geometry::strategy::buffer::join_round join_strategy;
        boost::geometry::strategy::buffer::end_round end_strategy;
        boost::geometry::strategy::buffer::point_circle circle_strategy;
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy2(buffer_distance_2);

        boost::geometry::model::multi_polygon<polygonBoost> result_1;
        boost::geometry::model::multi_polygon<polygonBoost> result_2;

        pointBoost pt1(pt_1.x,pt_1.y);
        pointBoost pt2(pt_2.x,pt_2.y);
        boost::geometry::buffer(pt1, result_1,
                                distance_strategy, side_strategy,
                                join_strategy, end_strategy, circle_strategy);
        // center of circle
        boost::geometry::buffer(pt2, result_2,
                                distance_strategy2, side_strategy,
                                join_strategy, end_strategy, circle_strategy);
        std::deque<polygonBoost> intersectionGeometry;
        boost::geometry::intersection(result_1,result_2,intersectionGeometry);
        std::cout << "the insecpts is : " << intersectionGeometry.front().outer().size() << std::endl;
        std::cout << "the aaaa is : " << intersectionGeometry.size() << std::endl;

        order_R += 0.1;
        if(intersectionGeometry.size()){
            std::cout << "the bbbbb " <<  boost::geometry::wkt(intersectionGeometry.front()) << std::endl;
            break;
        }
    }
    order_R = count_upper - 1;
    for(int i  = 0;i < 1000;i++){
//         double buffer_distance_1 = trans_R ;             // radius of circle
//         double buffer_distance_2 = order_R ;
//        const int points_per_circle = 36;
//        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance_1);
//        boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
//        boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
//        boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
//        boost::geometry::strategy::buffer::side_straight side_strategy;
//        boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy2(buffer_distance_2);
//
//        boost::geometry::model::polygon<polygonBoost> result_1;
//        boost::geometry::model::polygon<polygonBoost> result_2;
//
//        pointBoost pt1(pt_1.x,pt_1.y);
//        pointBoost pt2(pt_2.x,pt_2.y);
//        boost::geometry::buffer(pt1, result_1,
//                                distance_strategy, side_strategy,
//                                join_strategy, end_strategy, circle_strategy);
//        // center of circle
////        boost::geometry::buffer(pt2, result_2,
////                                distance_strategy2, side_strategy,
////                                join_strategy, end_strategy, circle_strategy);
//        polygonBoost intersectionGeometry;
//        boost::geometry::intersection(result_1,result_2,intersectionGeometry);
//        std::cout << "the insecpts is : " << intersectionGeometry.outer().size();
//        order_R += 0.01;
//        if(intersectionGeometry.outer().size() >1){
//            RC2 = order_R;
//            break;
//        }
    }

}

void cornerTuringTishNail::cornerTuringPath( polygonPoint A,
                                             polygonPoint B,
                                             double RC2,
                                             double F3){
    auto pt1 = storage_circle_center_[0];
    auto pt2 = storage_circle_center_[1];
    auto pt3 = storage_circle_center_[2];
    double alphaXC12 = atan2((pt1.y - pt2.y),(pt2.x - pt1.x));
    double alphaXC23 = atan2((pt3.y - pt2.y),(pt2.x - pt3.x));
    double alphaC3By = atan((B.y - pt3.y)/(B.x - pt3.x));
    double alphaC21y = atan((pt2.x - pt1.x)/(pt2.y - pt1.y));

    //处理C1
    double C1alphaStart = (1 - F3 ) * M_PI ;
    double C1alphaEnd  = alphaXC12 ;
    //十等分之后进行取点
    double angleC1diff = (C1alphaEnd - C1alphaStart)/100;
    for(int i = 0;i <= 100;i++){
        double C1alpha = C1alphaStart + i * angleC1diff;
        polygonPoint  tempPt;
        tempPt.x = pt1.x + sin(C1alpha) * CIRCLE_RIDIS_R;
        tempPt.y = pt1.y + cos(C1alpha) * CIRCLE_RIDIS_R;
        C1path_.push_back(tempPt);
    }


    //处理C2
    double C2alphaStart = - M_PI + alphaXC12;
    double C2alphaEnd = alphaXC23;
    double angleC2diff = (C2alphaEnd - C2alphaStart)/1000;
    for(int i = 0; i <= 1000;i++){
        double C2alpha = C2alphaStart + i * angleC2diff;
        polygonPoint  tempPt;
        tempPt.x = pt2.x + sin(C2alpha) * RC2;
        tempPt.y = pt2.y + cos(C2alpha) * RC2;
        C2path_.push_back(tempPt);
    }


    //处理C3
    double C3alphaStart = alphaXC23 + M_PI;
    double C3alphaEnd = M_PI/2 - alphaC3By;
    double angleC3diff = (C3alphaEnd - C3alphaStart)/100;
    for(int i = 0;i <= 100;i++){
        double C3alpha = C3alphaStart + i * angleC3diff;
        polygonPoint tempPt;
        tempPt.x = pt3.x + sin(C3alpha) * CIRCLE_RIDIS_R;
        tempPt.y = pt3.y + cos(C3alpha) * CIRCLE_RIDIS_R;
        C3path_.push_back(tempPt);
    }


    std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/C1path.txt";
    std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/C2path.txt";
    std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/C3path.txt";
    normalPrint C1file(C1name);
    normalPrint C2file(C2name);
    normalPrint C3file(C3name);
    C1file.writePts(C1path_);
    C2file.writePts(C2path_);
    C3file.writePts(C3path_);
}

std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC1path(){
    return C1path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC2path(){
    return C2path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC3path(){
    return C3path_;
}