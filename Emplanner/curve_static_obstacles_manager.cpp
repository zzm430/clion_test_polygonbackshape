//
// Created by zzm on 23-11-27.
//

#include "Emplanner/curve_static_obstacles_manager.h"

curveStaticObstaclesManager::curveStaticObstaclesManager(std::vector<std::vector<polygonPoint>>  & originPath,
                                                         std::vector<std::vector<polygonPoint>> &polygonPts){
     //原始关键点路径 、 障碍物的轮廓
     auto numbers_pts = originPath.size();

     for(int i = 0;i < numbers_pts;i++ ){
         //线段扩展的矩形与障碍物做碰撞检测

     }

    typedef double coordinate_type;
    typedef boost::geometry::model::d2::point_xy<coordinate_type> point;
    typedef boost::geometry::model::polygon<point> polygon;

    // Declare strategies
    const double buffer_distance = 2.0;
    const int points_per_circle = 10;
    boost::geometry::strategy::buffer::distance_symmetric<coordinate_type> distance_strategy(buffer_distance);
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;

    // Declare output
    boost::geometry::model::multi_polygon<polygon> result;

    // Declare/fill a linestring
    boost::geometry::model::linestring<point> ls;
    ls.push_back(point(11.25,53.3));
    ls.push_back(point(97.658,487));

//    boost::geometry::read_wkt("LINESTRING(0 0,10 6)", ls);

    // Create the buffer of a linestring
//    boost::geometry::buffer(ls, result,
//                            distance_strategy, side_strategy,
//                            join_strategy, end_strategy, circle_strategy);
    boost::geometry::buffer(ls, result,
                            distance_strategy,side_strategy,
                            join_strategy, end_strategy, circle_strategy);

    //求两个多边形是否相交
    polygon    obstacle_polygon;
    for(auto i : polygonPts[0]){
        obstacle_polygon.outer().push_back(point(i.x,i.y));
    }
    std::deque<polygon> intersectionGeometry;
    boost::geometry::intersection(result,obstacle_polygon,intersectionGeometry);
    std::cout << "the insecpts is : " << intersectionGeometry.front().outer().size() << std::endl;
    std::cout << "the aaaa is : " << intersectionGeometry.size() << std::endl;


    if(intersectionGeometry.size()){
        std::cout << "the bbbbb1 " <<  boost::geometry::wkt(intersectionGeometry.front()) << std::endl;
    }

    //计算多边形的质心
    point centroid;
    boost::geometry::centroid(obstacle_polygon, centroid);

    std::cout << "the  polygon centroid is :"
              << "["
              << centroid.x()
              << ","
              << centroid.y()
              << "]"
              << std::endl;

    //计算出三段对应的参考线
    std::vector<polygonPoint>  reference_pts1,reference_pts2,reference_pts3,reference_pts4,reference_pts5;
    std::vector<polygonPoint>  work_line;
    work_line.push_back(polygonPoint(11.25,53.3));
    work_line.push_back(polygonPoint(97.658,487));
    polygonPoint centroid_pt(centroid.x(),centroid.y());

    //计算质心投影到中心线上的点
    auto projection_pt = common::commonMath::computeFootPoint(
            centroid_pt,
            work_line[0],
            work_line[1]);
    auto A1 = common::commonMath::findPointOnSegment(
            work_line[0],
            projection_pt,
            6,
            false);
    auto A3 = common::commonMath::findPointOnSegment(
            work_line[0],
            projection_pt,
            10,
            false);
    auto A2 = common::commonMath::findPointOnSegment(
            projection_pt,
            work_line[1],
            6,
            true);
    auto A4 = common::commonMath::findPointOnSegment(
            projection_pt,
            work_line[1],
            10,
            true);

    std::vector<polygonPoint>  line1;
    line1.push_back(A3);
    line1.push_back(A1);

    std::vector<polygonPoint>  line2;
    line2.push_back(polygonPoint(A2));
    line2.push_back(polygonPoint(A4));

    std::vector<polygonPoint>  line3;
    line3.push_back(polygonPoint(A1));
    line3.push_back(polygonPoint(A2));

    reference_pts1 = common::commonMath::densify2(
            line1,
            10);
    reference_pts3 = common::commonMath::densify2(
            line2,
            10);

    //计算C、D点
    polygonPoint vector_origin_line(work_line[1].x - work_line[0].x,
                                    work_line[1].y - work_line[0].x);
    polygonPoint vector_vertical_line(-vector_origin_line.y,vector_origin_line.x);

    //计算虚拟垂直线段
    std::vector<polygonPoint>  vertical_line;
    polygonPoint  vertical_pt1,vertical_pt2;
    double distance  = 4;
    double segmentLength = sqrt(pow(vector_vertical_line.x,2) + pow(vector_vertical_line.y,2));
    double dx = vector_vertical_line.x / segmentLength;
    double dy = vector_vertical_line.y / segmentLength;
    vertical_pt1.x = centroid_pt.x + dx * distance;
    vertical_pt1.y = centroid_pt.y + dy * distance;
    vertical_pt2.x = centroid_pt.x - dx * distance;
    vertical_pt2.y = centroid_pt.y - dy * distance;

    //计算垂直线段与障碍物多边形的交点
    linestring_type segmentVirtual;
    boost::geometry::append(segmentVirtual,pointbst(
            vertical_pt1.x,
            vertical_pt1.y));
    boost::geometry::append(segmentVirtual,pointbst(
            vertical_pt2.x,
            vertical_pt2.y));

    std::deque<pointbst> intersectionGeometry1;
    boost::geometry::intersection(segmentVirtual,obstacle_polygon,intersectionGeometry1);

    std::vector<polygonPoint>  tempPts;
    for(auto i : intersectionGeometry1){
        tempPts.push_back(polygonPoint(i.x(),i.y()));
        std::cout  << "dfdf1111111111111  " << i.x() << " " << i.y() << std::endl;
    }
    std::cout << "the aaaa111 is : " << intersectionGeometry1.size() << std::endl;


    //计算C、D点到中心线的距离
    double distanceC,distanceD;
    distanceC = common::commonMath::distanceToLineSegment(tempPts[0],work_line[0],work_line[1]);
    distanceD = common::commonMath::distanceToLineSegment(tempPts[1],work_line[0],work_line[1]);
    std::cout << "the distance C and D is : " << distanceC << " " << distanceD << std::endl;

    //计算第3条参考线的离散点
   auto  reference_pts2_temp = common::commonMath::densify2(
            line3,
            10);

    //将第三条参考线上的点按照指定垂直中心线的方向平移
    //假如往C方向平移
    std::cout <<"the distance C is : " << distanceC << std::endl;
    double distance_virtual_line  = RIDGE_WIDTH_LENGTH/2 + distanceC + 0.5;
    reference_pts2 = common::commonMath::computePtsToOrderedDirectionMove(
            reference_pts2_temp,
            vector_vertical_line,
            distance_virtual_line);

#ifdef DEBUG_STATIC_OBSTACLE
     std::ofstream  streamLine;
     streamLine.open("/home/zzm/Desktop/test_path_figure-main/src/referenceLine2.txt",std::ios::out);
     for(auto i : reference_pts2){
         streamLine << " " << i.x;
     }
     streamLine << std::endl;
     for(auto j : reference_pts2){
         streamLine << " " << j.y;
     }
     streamLine << std::endl;
#endif

     //应该是5段参考线组成
     //计算第4段参考线，A1和第三段的起点
     auto line4size = reference_pts2.size();
     std::vector<polygonPoint>  line4;
     std::vector<polygonPoint>  line5;
     line4.push_back(A1);
     line4.push_back(reference_pts2[line4size/2]);
     line5.push_back(reference_pts2[line4size/2]);
     line5.push_back(A2);

     reference_pts4 = common::commonMath::densify2(line4,10);
     reference_pts5 = common::commonMath::densify2(line5,10);

     //整合所有参考线
     std::vector<polygonPoint>  reference_allpts;
     for(auto i : reference_pts1){
         reference_allpts.push_back(i);
     }
     for(auto i : reference_pts4){
         reference_allpts.push_back(i);
     }
//     for(auto i : reference_pts2){
//         reference_allpts.push_back(i);
//     }
     for(auto i : reference_pts5){
         reference_allpts.push_back(i);
     }
     for(auto i : reference_pts3){
         reference_allpts.push_back(i);
     }



     //参考线平滑算法
     std::vector<double> optx;
     std::vector<double> opty;
     std::vector<std::pair<double,double>> transd_referencePts;
     for(auto i : reference_allpts){
         std::pair<double,double> temp(i.x,i.y);
         transd_referencePts.push_back(temp);
     }
    smoothalgorithm::femSmoothManager  smoothFem(transd_referencePts);

    smoothFem.initiate();

    auto consider_static_obstacles_pts = smoothFem.get_smoothed_pts();

    //pjpo处理
    //针对参考线A3到A4初始参考线进行差值
    std::vector<polygonPoint>   line_A34;
    line_A34.push_back(A3);
    line_A34.push_back(A4);
    std::vector<polygonPoint> inter_pts =  common::commonMath::densify2(line_A34,80);
    //计算点位初始值
    std::vector<std::pair<double,double>> trans_pts ;
    for(auto i : inter_pts){
        trans_pts.push_back({i.x,i.y});
    }

    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;

    computePathProfileInfo::ComputePathProfile(
                                              trans_pts,
                                              &headings,
                                              &accumulated_s,
                                              &kappas,
                                              &dkappas);
    std::vector<slPoint> reference_sl_pts;
    for(int i = 0;i < trans_pts.size();i++){
        slPoint tempPt;
        tempPt.x = trans_pts[i].first;
        tempPt.y = trans_pts[i].second;
        tempPt.heading = headings[i];
        tempPt.s_ = accumulated_s[i];
        reference_sl_pts.push_back(tempPt);
    }


    //将平滑过的参考线转到sl坐标系下
//    std::vector<std::pair<double,double>>  transd_reference_sl_pts;


//    for(auto i : consider_static_obstacles_pts){
//        //从原始参考线中找到距离最近的那个点
//
//    }
//    for(auto i : reference_sl_pts){
//        transd_reference_sl_pts  = FrenetConverter::cartesianToFrenet1D(
//               i.s_,
//               i.x,
//               i.y,
//               i.heading,
//
//        );
//    }






    std::cout << "the bbbbb 2" <<  boost::geometry::wkt(result) << std::endl;



}