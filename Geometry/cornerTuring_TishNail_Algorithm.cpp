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
                         double F2,
                         double F3){
    chooseOptimalpath(  A, B, angleInt, RC2, F1, F2, F3);
}

void cornerTuringTishNail::computeCircleCenter(polygonPoint A,
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
}

void cornerTuringTishNail::computeCircleC2Radius(  polygonPoint A,
                                                 polygonPoint B,
                                                 double angleInt,
                                                 double& RC2,
                                                 double F1,
                                                 double F2){
    auto pt_1 = storage_circle_center_[0];
    auto pt_2 = storage_circle_center_[1];
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
            count_upper = order_R - 0.1;
            std::cout << "the bbbbb " <<  boost::geometry::wkt(intersectionGeometry.front()) << std::endl;
            break;
        }
    }
    // 精确找C2圆半径的部分代码
    order_R = count_upper - 0.1;
    for(int i  = 0;i < 1000;i++){
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
//        std::cout << "the insecpts 222 is : " << intersectionGeometry.front().outer().size() << std::endl;
//        std::cout << "the aaaa 222 is : " << intersectionGeometry.size() << std::endl;

        order_R += 0.01;
        if(intersectionGeometry.size()){
            RC2 = order_R - 0.01 ;
            std::cout << "the bbbbb " <<  boost::geometry::wkt(intersectionGeometry.front()) << std::endl;
            break;
        }
    }
}


void cornerTuringTishNail::chooseOptimalpath(  polygonPoint A,
                                               polygonPoint B,
                                               double angleInt,
                                               double& RC2,
                                               double F1,
                                               double F2,
                                               double F3){
    double maxLengthCost = DBL_MAX ;
    computeCircleCenter(A,  B, angleInt, RC2, F1, F2);
    computeCircleC2Radius( A,B,angleInt,RC2,F1,F2);
    maxLengthCost = computeLengthCostPath();
    for(int i = 1;i < 2;i++){
        computeCircleCenter(A,  B, angleInt, RC2, F1, F2);   //重新计算C2的圆心
        cornerTuringPath(A,B,RC2,F3);
        std::cout << "the i is : " << i << std::endl;
        double lengthCost =  computeLengthCostPath();
        std::cout << "the length cost is : " << lengthCost << std::endl;
        if(lengthCost < maxLengthCost){
            RC2 = RC2 * 1.1;
        }else{
            break;
        }
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

    //求3个圆之间的交点
    auto insect1_2 = computeCircleInterSectPt(pt1,pt2,CIRCLE_RIDIS_R,RC2);  //C1,C2之间的交点
    auto insect3_2 = computeCircleInterSectPt(pt3,pt2,CIRCLE_RIDIS_R,RC2);  //C2,C3之间的交点

    //处理C1
    double C1alphaStart = (1 - F3 ) * M_PI ;
    double C1alphaEnd  = alphaXC12 ;

    //等分之后进行取点
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

    //C1、C2、C3的总长度
    double C1_allLength = CIRCLE_RIDIS_R * (C1alphaEnd - C1alphaStart);
    double C2_allLength = CIRCLE_RIDIS_R * (C2alphaEnd - C2alphaStart);
    double C3_allLength = CIRCLE_RIDIS_R * (C3alphaEnd - C3alphaStart);

    //等间隔采样点
    auto C1_pts = common::commonMath::equalIntervalDiff(C1_allLength,
                                                        FISHNail_DIFF_DIS,
                                                        C1alphaStart,
                                                        C1alphaEnd,
                                                        CIRCLE_RIDIS_R,
                                                        pt1);
    auto C2_pts = common::commonMath::equalIntervalDiff(C2_allLength,
                                                        FISHNail_DIFF_DIS,
                                                        C2alphaStart,
                                                        C2alphaEnd,
                                                        RC2,
                                                        pt2);

    auto C3_pts = common::commonMath::equalIntervalDiff(C3_allLength,
                                                        FISHNail_DIFF_DIS,
                                                        C3alphaStart,
                                                        C3alphaEnd,
                                                        CIRCLE_RIDIS_R,
                                                        pt3);
    C1path_ = C1_pts;
    C2path_ = C2_pts;
    C3path_ = C3_pts;

    std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/C1path.txt";
    std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/C2path.txt";
    std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/C3path.txt";
    normalPrint C1file(C1name);
    normalPrint C2file(C2name);
    normalPrint C3file(C3name);
    C1file.writePts(C1path_);
    C2file.writePts(C2path_);
    C3file.writePts(C3path_);

    //对原始3个线段进行截取对应路径
    fishNailC1path_ = computeInterceptPath(C1path_,insect1_2);
    auto tempC2path = computeInterceptPath(C2path_,insect1_2);
    std::reverse(tempC2path.begin(),tempC2path.end());
    fishNailC2path_ = computeInterceptPath(tempC2path,insect3_2);
//    std::reverse(fishNailC2path_.begin(),fishNailC2path_.end());
    auto tempPath3 = C3path_;
    std::reverse(tempPath3.begin(),tempPath3.end());
    fishNailC3path_ = computeInterceptPath(tempPath3,insect3_2);
    std::reverse(fishNailC3path_.begin(),fishNailC3path_.end());

//    std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/C1path.txt";
//    std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/C2path.txt";
//    std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/C3path.txt";
//    normalPrint C1file(C1name);
//    normalPrint C2file(C2name);
//    normalPrint C3file(C3name);
//    C1file.writePts(fishNailC1path_);
//    C2file.writePts(fishNailC2path_);
//    C3file.writePts(fishNailC3path_);



}

double cornerTuringTishNail::computeLengthCostPath(){
    double cost1 = common::commonMath::computePathAllLength(fishNailC1path_);
    double cost2 = common::commonMath::computePathAllLength(fishNailC2path_);
    double cost3 = common::commonMath::computePathAllLength(fishNailC3path_);
    double allCost = cost1 + cost2 + cost3;
    return  allCost;
}

std::vector<polygonPoint>  cornerTuringTishNail::computeInterceptPath(std::vector<polygonPoint> & path,
                                                                      polygonPoint  order_pt){
    std::vector<polygonPoint>   allPath;
    for(auto i : path){
        double dis = common::commonMath::distance2(i,order_pt);
        allPath.push_back(i);
        if(fabs(dis) < 0.1){
            break;
        }
    }
    return allPath;
}


polygonPoint  cornerTuringTishNail::computeCircleInterSectPt(
        polygonPoint pt1,
        polygonPoint pt2,
        const double& R1,
        double& R2){
    double buffer_distance_1 = R1 ;             // radius of circle
    double buffer_distance_2 = R2 ;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy(buffer_distance_1);
    boost::geometry::strategy::buffer::join_round join_strategy;
    boost::geometry::strategy::buffer::end_round end_strategy;
    boost::geometry::strategy::buffer::point_circle circle_strategy;
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::strategy::buffer::distance_symmetric<double> distance_strategy2(buffer_distance_2);

    boost::geometry::model::multi_polygon<polygonBoost> result_1;
    boost::geometry::model::multi_polygon<polygonBoost> result_2;

    pointBoost circle_1(pt1.x,pt1.y);
    pointBoost circle_2(pt2.x,pt2.y);
    boost::geometry::buffer(circle_1, result_1,
                            distance_strategy, side_strategy,
                            join_strategy, end_strategy, circle_strategy);
    // center of circle
    boost::geometry::buffer(circle_2, result_2,
                            distance_strategy2, side_strategy,
                            join_strategy, end_strategy, circle_strategy);
    std::deque<polygonBoost> intersectionGeometry;
    boost::geometry::intersection(result_1,result_2,intersectionGeometry);

    pointBoost centroidm;
    polygonBoost polygonInsect;
    if(intersectionGeometry.size()){
        std::cout << "the bbbbb mmmmmmm " <<  boost::geometry::wkt(intersectionGeometry.front()) << std::endl;
        //计算最小多边形的质心


        boost::geometry::centroid(intersectionGeometry.front(), centroidm);

        std::cout  << "the insect  polygon centroid is :"
                   << "["
                   << centroidm.x()
                   << ","
                   << centroidm.y()
                   << "]"
                   << std::endl;
    }
    polygonPoint  insecPt;
    insecPt.x = centroidm.x();
    insecPt.y = centroidm.y();
    return  insecPt;

}

std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC1path(){
    return fishNailC1path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC2path(){
    return fishNailC2path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getFishNailC3path(){
    return fishNailC3path_;
}

std::vector<polygonPoint>   cornerTuringTishNail::getC1path(){
    return C1path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getC2path(){
    return C2path_;
}
std::vector<polygonPoint>   cornerTuringTishNail::getC3path(){
    return C3path_;
}

