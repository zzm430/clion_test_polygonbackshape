//
// Created by zzm on 2023/6/1.
//
#include <common/math/common_math.h>
#include "path_polygonplan.h"

namespace aiforce{
namespace Route_Planning{

//void pathPolygonPlan::initiate() {
//
//}


void pathPolygonPlan::computeNarrowPolygons(std::vector<Point> &points) {
    double buffer_distance = -DBL_MAX;
    //按照垄宽得到缩小后的各个多边形
    //默认给出的点位时按照逆时针转的
    boost::geometry::model::multi_polygon<polygon> minPolygon;
    double count_first_ridge;   //第一笼多边形内缩的顶点数
    double count_second_ridge;  //第二垄多边形内缩的顶点数
    for(auto i = 1; i <=MAX_TRAVERSALS_NUMBERS;i++ ){
        const int points_per_circle = 0;
        if(i == 1){             //第1垄内缩垄宽/2,剩余垄内缩垄宽
            buffer_distance =  - RIDGE_WIDTH_LENGTH/2;
        }else{
            buffer_distance = i * -RIDGE_WIDTH_LENGTH + RIDGE_WIDTH_LENGTH/2;
        }
        boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
                                         distance_strategy(buffer_distance);
#ifdef  CHOOSE_NARROW_TYPE
        boost::geometry::strategy::buffer::join_round join_strategy(0.01);
#else
        boost::geometry::strategy::buffer::join_round join_strategy;
#endif
        boost::geometry::strategy::buffer::end_round end_strategy;
        boost::geometry::strategy::buffer::point_circle circle_strategy;
        boost::geometry::strategy::buffer::side_straight side_strategy;

        // Declare output
        boost::geometry::model::multi_polygon<polygon> result;
        boost::geometry::model::multi_polygon<polygon> mpol;
        polygon instance_polygon;
        for (auto it = points.rbegin(); it != points.rend(); it++) {
            point instance_point;
            instance_point.x((*it).x);
            instance_point.y((*it).y);
            instance_polygon.outer().push_back(instance_point);
        }
        mpol.push_back(instance_polygon);
        boost::geometry::buffer(mpol, result,
                                distance_strategy, side_strategy,
                                join_strategy, end_strategy, circle_strategy);
        std::vector<polygonPoint>  tempPolygon;
        //遇到矩形的内缩的截止条件，做筛选处理
        if(result.empty()) {
            dealWithLastSeveralPolygons();
            computeLastRidgeSituation();
            break;
        }
        minPolygon = result;
        storageNarrowPolygonPoints2_.push_back(result);
        count_narrow_polygon_numbers_ += 1;
        for(auto it = result.begin();it != result.end(); it++){
            for(auto j = it->outer().begin();j != it->outer().end();j++){
                polygonPoint tempPoint;
                tempPoint.x = (*j).x();
                tempPoint.y = (*j).y();
                tempPolygon.push_back(tempPoint);
            }
        }
        storageNarrowPolygonPoints_.push_back(tempPolygon);

        //由于后续判断回字形入口model
        if(i == 1){
            count_first_ridge = result.begin()->outer().size();
        }else if(i == 2){
            count_second_ridge = result.begin()->outer().size();
            if(count_first_ridge ==
               count_second_ridge){
                //计算出各个斜率直线并保存
                LOG(INFO) << "the count_first_ridge == count_second_ridge !";
                computeEveryKbLine();
            }else{
                LOG(ERROR) << "the count_firsg_ridge != count_second_ridge !";
            }
        }else {
            judgePointPosition(i,result);  //为找到最长的内缩边提供count
        }
    }
    LOG(INFO) << "the narrow polygon_number size is :"<< count_narrow_polygon_numbers_;

//    //计算最小多边形的质心
//    point centroid;
//    boost::geometry::centroid(minPolygon, centroid);
//    min_polygon_centroid_.x = centroid.x();
//    min_polygon_centroid_.y = centroid.y();
//    LOG(INFO) << "the min  polygon centroid is :"
//              << "["
//              << centroid.x()
//              << ","
//              << centroid.y()
//              << "]";
//    LOG(INFO) << "the count narrow polygon nubmers is : "
//              << count_narrow_polygon_numbers_;
}

void pathPolygonPlan::dealWithLastSeveralPolygons(){
      auto num = storageNarrowPolygonPoints_.size();
      if(storageNarrowPolygonPoints_[num-1].size() == 6){
         last_polys_type_ = lastPolyIdentify::POLY_FIVE;
         LOG(INFO) << "the last poly type is : poly_five !";
         return;
      }
      if(storageNarrowPolygonPoints_[num-1].size() == 5){
          last_polys_type_ = lastPolyIdentify::POLY_FOUR;
          LOG(INFO) << "the last poly type is : poly_four !";
          return;
      }
      for(int i = 1;i <= num; i++)
      {
          auto poly_pts_size = storageNarrowPolygonPoints_[i-1].size();
          if(poly_pts_size == 5 ){
              if(poly_pts_size == 5 &&
                 storageNarrowPolygonPoints_[i].size() == 4){
                  //内缩到出现三角形,目前只考虑内缩四边形内的第一个三角形
                  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
                          distance_strategy(-RIDGE_WIDTH_LENGTH/2);
                  boost::geometry::strategy::buffer::join_round join_strategy(0.01);

                  boost::geometry::strategy::buffer::end_round end_strategy;
                  boost::geometry::strategy::buffer::point_circle circle_strategy;
                  boost::geometry::strategy::buffer::side_straight side_strategy;

                  // Declare output
                  boost::geometry::model::multi_polygon<polygon> result;
                  boost::geometry::model::multi_polygon<polygon> mpol;
                  polygon instance_polygon;

                  for (auto it: storageNarrowPolygonPoints_[i-1]) {
                      point instance_point;
                      instance_point.x(it.x);
                      instance_point.y(it.y);
                      instance_polygon.outer().push_back(instance_point);
                  }
                  mpol.push_back(instance_polygon);
                  boost::geometry::buffer(mpol, result,
                                          distance_strategy, side_strategy,
                                          join_strategy, end_strategy, circle_strategy);
                  std::vector<polygonPoint> poly_res;
                  for(auto j = result.begin()->outer().begin();
                      j != result.begin()->outer().end();
                      j ++){
                      polygonPoint  temp_point;
                      temp_point.x = j->x();
                      temp_point.y = j->y();
                      poly_res.push_back(temp_point);
                  }
                  if(poly_res.size() == 4){
                      LOG(INFO) << "the last poly type is : poly_four_and_three !";
                      last_polys_type_ = lastPolyIdentify::POLY_FOUR_AND_THREE;
                      last_several_polygons_.push_back(storageNarrowPolygonPoints_[i-1]);
                      record_poly_index_ = i;
                  } else{
                      LOG(INFO) << "--------------------------------------------------------------";
                      LOG(INFO) << "the shape of quadrangle after being retracted is quadrangle !";
                      LOG(INFO) << "the quadrangle size is : " << poly_res.size();
                      last_polys_type_ = lastPolyIdentify::POLY_FOUR_AND_FOUR;
                      last_several_polygons_.push_back(storageNarrowPolygonPoints_[i-1]);
                      record_poly_index_ = i;
                  }
                  LOG(INFO) << "the record poly index is : " << record_poly_index_;
                  LOG(INFO) <<  "--------------------------------------------------------------";
                  last_several_polygons_.push_back(poly_res);
              }
          }
      }
      std::ofstream  tempfile;
      tempfile.open("/home/zzm/Desktop/test_path_figure-main/src/templastridge.txt",std::ios::out);
      for(auto it :last_several_polygons_){
          for(auto j : it){
              tempfile << " " << j.x;
          }
      }
      tempfile << std::endl;
      for(auto it : last_several_polygons_){
          for(auto j: it){
              tempfile << " " << j.y;
          }
      }
      tempfile << std::endl;
      tempfile.close();
}

void pathPolygonPlan::computeLastRidgeSituation(){
    switch (last_polys_type_){
        case lastPolyIdentify::POLY_FOUR_AND_THREE:{
            computeLastRidgePoints4and3();
            break;
        }
        case lastPolyIdentify::POLY_FOUR_AND_FOUR:{
            computeLastRidgePoints4and4();
            break;
        }
        case lastPolyIdentify::POLY_FOUR:{
            computeLastRidgePoints4();
            break;
        }
        case lastPolyIdentify::POLY_FIVE:{

            break;
        }
        default:{
            LOG(INFO) << "the last bridge has not been considered yet !";
            break;
        }

    }
}

void pathPolygonPlan::computeLastRidgePoints4(){
    //算出四边形的点
    int num = storageNarrowPolygonPoints_.size();
    auto lastPolyPts = storageNarrowPolygonPoints_[num-1];
    double distance1 = common::commonMath::distance2(lastPolyPts[0],lastPolyPts[1]);
    double distance2 = common::commonMath::distance2(lastPolyPts[1],lastPolyPts[2]);
    double distance3 = common::commonMath::distance2(lastPolyPts[2],lastPolyPts[3]);
    double distance4 = common::commonMath::distance2(lastPolyPts[3],lastPolyPts[4]);
    std::map<double ,int>  dis_map;
    dis_map[distance1] = 1;
    dis_map[distance2] = 2;
    dis_map[distance3] = 3;
    dis_map[distance4] = 4;
    std::vector<double> stor_dis;
    for(const auto & pair : dis_map){
        stor_dis.push_back(pair.first);
    }
    int num_1 = dis_map[stor_dis[0]];
    int num_2 = dis_map[stor_dis[1]];
    LOG(INFO) << "the last polygon smallest distance is　："  <<  stor_dis[0];
    LOG(INFO) << "the last polygon small distance is : "     <<   stor_dis[1];
    if(stor_dis[0]  > RIDGE_WIDTH_LENGTH || stor_dis[1] > RIDGE_WIDTH_LENGTH ){
        //说明需要在中间外加一条线段
        polygonPoint point_middle_1,point_middle_2;
        point_middle_1.x =  (lastPolyPts[num_1-1].x + lastPolyPts[num_1].x)/2;
        point_middle_1.y =  (lastPolyPts[num_1-1].y + lastPolyPts[num_1].y)/2;
        point_middle_2.x =  (lastPolyPts[num_2-1].x + lastPolyPts[num_2].x)/2;
        point_middle_2.y =  (lastPolyPts[num_2-1].y + lastPolyPts[num_2].y)/2;
        line_middle_.push_back(point_middle_1);
        line_middle_.push_back(point_middle_2);
    }
}
void pathPolygonPlan::computeLastRidgePoints4and4(){
    std::vector<polygonPoint> stor_pts = deleteRepeatPolyPts(last_several_polygons_[1]);
    Polygon_2 poly_4,poly_rect;
    for(auto it = stor_pts.rbegin();it != stor_pts.rend();it ++){
        poly_4.push_back(Point_2(it->x,it->y));
    }

    CGAL::min_rectangle_2(
            poly_4.vertices_begin(), poly_4.vertices_end(), std::back_inserter(poly_rect));

    std::ofstream test_rect;
    test_rect.open("/home/zzm/Desktop/test_path_figure-main/src/test_rect.txt",std::ios::out);
    for(auto it = poly_rect.begin(); it != poly_rect.end();it++){
        test_rect << " " << (*it).x();
    }
    test_rect << std::endl;
    for(auto it = poly_rect.begin(); it != poly_rect.end();it++){
        test_rect << " " << (*it).y();
    }
    test_rect<< std::endl;
    test_rect.close();

    std::ofstream test_4;
    test_4.open("/home/zzm/Desktop/test_path_figure-main/src/test_4.txt",std::ios::out);
    for(auto it = last_several_polygons_[1].begin(); it != last_several_polygons_[1].end();it++){
        test_4 << " " << (*it).x;
    }
    test_4 << std::endl;
    for(auto it = last_several_polygons_[1].begin(); it != last_several_polygons_[1].end();it++){
        test_4 << " " << (*it).y;
    }
    test_4 << std::endl;
    test_4.close();

    //计算出对应的矩形的两个短边
   std::vector<polygonPoint>  polygon_rect;
   for(auto it = poly_rect.begin();it != poly_rect.end();it++){
       polygon_rect.push_back(polygonPoint((*it).x(),(*it).y()));
   }

   double distance1 = common::commonMath::distance2(polygon_rect[0],polygon_rect[1]);
   double distance2 = common::commonMath::distance2(polygon_rect[1],polygon_rect[2]);

   //构建两个矩形的短边AB、 DC
   std::vector<polygonPoint>  line_AB,line_DC;
   //构建移动点需要的两个line BC \ CD
   std::vector<polygonPoint>  line_BC,line_CD;
   if(distance1 < distance2){
       line_AB.push_back(polygon_rect[0]);
       line_AB.push_back(polygon_rect[1]);
       line_DC.push_back(polygon_rect[3]);
       line_DC.push_back(polygon_rect[2]);
       line_BC.push_back(polygon_rect[1]);
       line_BC.push_back(polygon_rect[2]);
       line_CD.push_back(polygon_rect[2]);
       line_CD.push_back(polygon_rect[3]);
   }else{
       line_AB.push_back(polygon_rect[1]);
       line_AB.push_back(polygon_rect[2]);
       line_DC.push_back(polygon_rect[0]);
       line_DC.push_back(polygon_rect[3]);
       line_BC.push_back(polygon_rect[2]);
       line_BC.push_back(polygon_rect[3]);
       line_CD.push_back(polygon_rect[3]);
       line_CD.push_back(polygon_rect[0]);
   }
   line_rec_short_AB_.push_back(line_AB[0]);
   line_rec_short_AB_.push_back(line_AB[1]);
   line_rec_short_DC_.push_back(line_DC[0]);
   line_rec_short_DC_.push_back(line_DC[1]);

   double distanceAB = common::commonMath::distance2(line_AB[0],line_AB[1]);
   double distanceDC = common::commonMath::distance2(line_DC[0],line_DC[1]);

   LOG(INFO) << "the last ridge rectangle short bridge AB length is : " << distanceAB;
   LOG(INFO) << "the last ridge rectangle short bridge DC length is : " << distanceDC;

   //计算AB和DC上的点位信息
   double integer_number = ceil(distanceAB/RIDGE_WIDTH_LENGTH);
   double mod = common::commonMath::doubleMod(distanceAB,RIDGE_WIDTH_LENGTH);
   double last_path_cover = mod * RIDGE_WIDTH_LENGTH;
   LOG(INFO) << "last path cover distance is :" << last_path_cover;

   for(int i = 1;i <= integer_number;i++){
       std::vector<polygonPoint> points;
       if(i == 1){
           points = common::commonMath::computeLineTranslationPoints(
                    line_BC,
                    line_CD,
                    RIDGE_WIDTH_LENGTH/2);
       }else{
           points = common::commonMath::computeLineTranslationPoints(
                   line_BC,
                   line_CD,
                   RIDGE_WIDTH_LENGTH/2 + RIDGE_WIDTH_LENGTH *(i-1));
       }
       line_rec_short_BA_.push_back(points[0]);
       line_rec_short_CD_.push_back(points[1]);
   }
}

void pathPolygonPlan::computeLastRidgePoints4and3(){
     //去除三角形的重复点位信息，输出为顺时针

    std::vector<polygonPoint> stor_pts = last_several_polygons_[1];
//    //计算最小外接矩形
//    Polygon_2 poly_rect,poly_tangle;
//    for(auto it = stor_pts.rbegin() ;it !=  stor_pts.rend(); it ++){
//        poly_tangle.push_back(Point_2(it->x,it->y));
//    }

    //找到三角形中最长的边
    double distance_max = -DBL_MAX;
    std::vector<polygonPoint>  line_longAB;
    polygonPoint  max_point;
    for(int i = 0 ; i <  last_several_polygons_[1].size()-1;i++){
        double distance = common::commonMath::distance2(last_several_polygons_[1][i],
                                                        last_several_polygons_[1][i+1]);
        if(distance >distance_max){
            distance_max = distance ;
            line_longAB.clear();
            line_longAB.push_back(last_several_polygons_[1][i]);
            line_longAB.push_back(last_several_polygons_[1][i+1]);
        }
    }
    for(int i = 0;i < last_several_polygons_[1].size()-1;i++){
        if(last_several_polygons_[1][i] != line_longAB[0] &&
           last_several_polygons_[1][i] != line_longAB[1]){
            max_point = last_several_polygons_[1][i];
        }
    }
    line_long_AB_.push_back(line_longAB[0]);
    line_long_AB_.push_back(line_longAB[1]);
    //计算max_point到lineAB的投影点
    polygonPoint footprint;
    footprint = common::commonMath::computeFootPoint(
            max_point,
            line_longAB[0],
            line_longAB[1]
            );
    //计算平移次数
    double distance_footpt = common::commonMath::distance2(footprint,
                                                           max_point);
    double integer_number = ceil(distance_footpt / RIDGE_WIDTH_LENGTH);
    double mod = common::commonMath::doubleMod(distance_footpt,RIDGE_WIDTH_LENGTH);
    double last_path_cover = mod * RIDGE_WIDTH_LENGTH;
    double final_increase_points = integer_number;

    LOG(INFO) << "the mod is ： " << mod ;
    LOG(INFO) << "final increase points  number is : " << final_increase_points;
    LOG(INFO) << "last path cover distance is : " << last_path_cover;

    //计算平移的点位信息
    std::vector<polygonPoint>  move_pts;
    std::vector<polygonPoint>  line_DC;
    line_DC.push_back(footprint);
    line_DC.push_back(max_point);
    std::vector<polygonPoint> line_AC;
    line_AC.push_back(line_longAB[0]);
    line_AC.push_back(max_point);
    std::vector<polygonPoint> line_BC;
    line_BC.push_back(line_longAB[1]);
    line_BC.push_back(max_point);

    for(int i = 1 ;i <= integer_number;i++){
        std::vector<polygonPoint> points;
        if(i==1){
             points = common::commonMath::computeLineTranslationPoints(
                    line_longAB,
                    line_DC,
                    RIDGE_WIDTH_LENGTH/2);
        }else{
            points = common::commonMath::computeLineTranslationPoints(
                    line_longAB,
                    line_DC,
                    RIDGE_WIDTH_LENGTH/2 + RIDGE_WIDTH_LENGTH * (i-1));
        }

        //计算与线段AC、线段BC的交点
        Segment_2 segment1(Point_2(points[0].x,points[0].y),
                           Point_2(points[1].x,points[1].y));
        Segment_2 segment2(Point_2(line_AC[0].x,line_AC[0].y),
                           Point_2(line_AC[1].x,line_AC[1].y));
        Segment_2 segment3(Point_2(line_BC[0].x,line_BC[0].y),
                           Point_2(line_BC[1].x,line_BC[1].y));

         auto result1 =  CGAL::intersection(segment1,segment2);
         auto result2 =  CGAL::intersection(segment1,segment3);

         if(result1 && result2){
             const Point_2* p1 = boost::get<Point_2>(&*result1);
             const Point_2* p2 = boost::get<Point_2>(&*result2);
             move_pts.push_back(polygonPoint(p1->x(),p1->y()));
             move_pts.push_back(polygonPoint(p2->x(),p2->y()));
             last_ridge_AC_pts_.push_back(polygonPoint(p1->x(),p1->y()));
             last_ridge_BC_pts_.push_back(polygonPoint(p2->x(),p2->y()));
         }
    }

    //将AC\BC整体向上平移一定距离
    std::vector<polygonPoint>  move_lines;
    for(int i = 0;i < last_ridge_AC_pts_.size();i++){
        move_lines.clear();
        move_lines.push_back(last_ridge_AC_pts_[i]);
        move_lines.push_back(last_ridge_BC_pts_[i]);
        auto move_points = common::commonMath::computeLineTranslationPoints(
                      move_lines,
                      line_DC,
                      RIDGE_WIDTH_LENGTH/2);
        move_last_ridge_AC_pts_.push_back(move_points[0]);
        move_last_ridge_BC_pts_.push_back(move_points[1]);
    }

    std::ofstream test_pts;
    test_pts.open("/home/zzm/Desktop/test_path_figure-main/src/test_pts.txt",std::ios::out);
    for(auto it = move_pts.begin(); it != move_pts.end();it++){
        test_pts << " " << it->x;
    }
    test_pts << std::endl;
    for(auto it = move_pts.begin(); it != move_pts.end();it++){
        test_pts << " " << it->y;
    }
    test_pts<< std::endl;
    test_pts.close();

//
    std::ofstream test_tangle;
    test_tangle.open("/home/zzm/Desktop/test_path_figure-main/src/test_tangle.txt",std::ios::out);
    for(auto it = stor_pts.begin(); it != stor_pts.end();it++){
        test_tangle << " " << (*it).x;
    }
    test_tangle << std::endl;
    for(auto it = stor_pts.begin(); it != stor_pts.end();it++){
        test_tangle << " " << (*it).y;
    }
    test_tangle<< std::endl;
    test_tangle.close();

    //获取矩形的中间点位
    //找到矩形的最左侧的点
//    polygonPoint   left_point;
//    left_point.x = DBL_MAX;
//    left_point.y = DBL_MAX;
//    for(auto it = poly_rect.begin();it != poly_rect.end();it++){
//        if(left_point.x > (*it).x()){
//            left_point.x = (*it).x();
//            left_point.y = (*it).y();
//        }
//    }
//    //确定这个点的长边短边
//    //先找到前后点
//    std::vector<polygonPoint>  rec_points;
//    for(auto it = poly_rect.begin();it != poly_rect.end();it++){
//        rec_points.push_back(polygonPoint((*it).x(),(*it).y()));
//    }
//    std::vector<polygonPoint> prev_and_next_pts = common::commonMath::computeForwardAndBackPoints(
//                                                     rec_points,
//                                                     left_point);
//    double distance1 = common::commonMath::distance2(prev_and_next_pts[0],
//                                                     left_point);
//    double distance2 = common::commonMath::distance2( prev_and_next_pts[1],
//                                                      left_point);
//    std::vector<polygonPoint>  lineAB, lineAC;  //AB短边,AC长边
//    double distance;
//    if(distance1 < distance2){
//        lineAB.push_back(left_point);
//        lineAB.push_back(prev_and_next_pts[0]);
//        lineAC.push_back(left_point);
//        lineAC.push_back(prev_and_next_pts[1]);
//        distance = distance1;
//    }else{
//        lineAB.push_back(left_point);
//        lineAB.push_back(prev_and_next_pts[1]);
//        lineAC.push_back(left_point);
//        lineAC.push_back(prev_and_next_pts[0]);
//        distance = distance2;
//    }
//
//
//    LOG(INFO) << "the last triangle  base side length :" << distance;
//    double integer_number = ceil(distance / RIDGE_WIDTH_LENGTH);
//    double mod = common::commonMath::doubleMod(distance,RIDGE_WIDTH_LENGTH);
//    double last_path_cover = mod * RIDGE_WIDTH_LENGTH;
//    double final_increase_points = integer_number;
//    LOG(INFO) << "the mod is ： " << mod ;
//    LOG(INFO) << "final increase points  number is : " << final_increase_points;
//    LOG(INFO) << "last path cover distance is : " << last_path_cover;
//    //根据AB方向确定AC线段需要扩展点的多少
//    std::vector<polygonPoint>  pointsAs;
//    std::vector<polygonPoint>  pointsCs;
//    std::vector<polygonPoint>  points;
//    for(int i = 1;i <= integer_number;i++){
//        if(i == 1){
//               points = common::commonMath::computeLineTranslationPoints(
//                    lineAC,
//                    lineAB,
//                    RIDGE_WIDTH_LENGTH/2);
//        }else{
//            points = common::commonMath::computeLineTranslationPoints(
//                    lineAC,
//                    lineAB,
//                    RIDGE_WIDTH_LENGTH*(i-1)+RIDGE_WIDTH_LENGTH/2);
//        }
//        pointsAs.push_back(points[0]);
//        pointsAs.push_back(points[1]);
//        last_ridge_rectangle_points_.push_back(points[0]);
//        last_ridge_rectangle_points_.push_back(points[1]);
//    }
//    std::ofstream test_middle;
//    test_middle.open("/home/zzm/Desktop/test_path_figure-main/src/test_middle.txt",std::ios::out);
//    for(auto it = pointsAs.begin(); it != pointsAs.end();it++){
//        test_middle << " " << (*it).x;
//    }
//    test_middle << std::endl;
//    for(auto it = pointsAs.begin(); it != pointsAs.end();it++){
//        test_middle << " " << (*it).y;
//    }
//    test_middle<< std::endl;
//    test_middle.close();

}

//计算多边形的kbline和对应的顶点相对应起来,未使用
// 0,1,2,....
void pathPolygonPlan::computeOriginPolygonKbline(std::vector<Point> &points){
    //boost库使用的是顺时针，正常为逆时针
    for(int  i = 0 ;i <  points.size()-1;i++){
        lineOrignPolygon linepoint;
        linepoint.k =  (points[i+1].y - points[i].y)/(points[i+1].x - points[i].x);
        linepoint.b =  points[i].y - linepoint.k * points[i].x;
        linepoint.i =  i;
        linepoint.j =  i+1;
        polygonPoint temp;
        temp.x = points[i].x;
        temp.y = points[i].y;
        lineOriginPolygonInfo_[i] = linepoint;
    }
    //计算最后一个点到第一个点的line
    lineOrignPolygon linepoint_1;
    linepoint_1.k = (points[points.size()-1].y - points[0].y)
            /(points[points.size() -1].x - points[0].x);
    linepoint_1.b = points[0].y - linepoint_1.k * points[0].x;
    linepoint_1.i = points[points.size()-1].x;
    linepoint_1.j = 0;
    polygonPoint temp_1;
    temp_1.x = points[points.size()-1].x;
    temp_1.y = points[points.size()-1].y;
    lineOriginPolygonInfo_[points.size()-1] = linepoint_1;
}

//约定每个多边形的起点都是从回字形的入口点开始
void pathPolygonPlan::updatePolygonPointsSequence1(){
    int num = insertedPtToNarrowPolygon_.size();
    int number;
    std::vector<polygonPoint>  storage_points;
    for(int i = 0;i < num -1;i++){
        for(auto it : insertedPtToNarrowPolygon_[i]){
            if(it.entrance_){
                int num_1 = insertedPtToNarrowPolygon_[i].size();
                for(int j = 0;j < num_1;j++){
                    if(fabs(insertedPtToNarrowPolygon_[i][j].x - it.x) < 0.1 &&
                       fabs(insertedPtToNarrowPolygon_[i][j].y - it.y) < 0.1){
                               number = j;
                    }
                }
                polygonPoint next;
                storage_points.push_back(it);
                for(int f = 1;f <= num_1 - 1;f++){
                    next = insertedPtToNarrowPolygon_[i][(number+1)%num_1];
                    storage_points.push_back(next);
                    number = number + 1;
                }

                middle_points_polygon_.push_back(storage_points);
                number = 0;
                storage_points.clear();
                break;
            }
        }
    }
    middle_points_polygon_.push_back(insertedPtToNarrowPolygon_[num-1]);
}



const std::vector<std::vector<polygonPoint>> pathPolygonPlan::getMiddlePointsPolygon() const{
    return middle_points_polygon_;
}


std::vector<polygonPoint>  pathPolygonPlan::deleteRepeatPolyPts(std::vector<polygonPoint> points){
    std::vector<polygonPoint>   temp_storage_pts;
    int num = points.size();
    for(const auto & p : points){
        if(std::find(temp_storage_pts.begin(),temp_storage_pts.end(),p)
                == temp_storage_pts.end()){
            temp_storage_pts.push_back(p);
        }
    }
    return temp_storage_pts;
}

void   pathPolygonPlan::filteredBackShapeKeyPoints(){
    std::vector<polygonPoint>  temp_storage_pts;
    int num = backShape_keypoints_.size();
    for(int it = 0 ;it < backShape_keypoints_.size() -2;it++){
         temp_storage_pts = deleteRepeatPolyPts(backShape_keypoints_[it]);
         filtered_backshape_keypoints_.push_back(temp_storage_pts);
    }
    filtered_backshape_keypoints_.push_back(backShape_keypoints_[num -2]);
    filtered_backshape_keypoints_.push_back(backShape_keypoints_[num -1]);
}


void pathPolygonPlan::computeKeypointsRelativeInfo(){
    std::ofstream  test;
    test.open("/home/zzm/Desktop/test_path_figure-main/src/test0620.txt",std::ios::out);
    std::ofstream  test1;
    test1.open("/home/zzm/Desktop/test_path_figure-main/src/test06201.txt",std::ios::out);
    for(int i = 0; i < filtered_backshape_keypoints_.size();i++){
        for(int j = 1; j < filtered_backshape_keypoints_[i].size();j++){
            ridgeKeypoint tempPtInfo;
            tempPtInfo.start_dis = SET_STARTTURN_DISTANCE;
            tempPtInfo.end_dis  = SET_ENDTURN_DISTANCE;
            tempPtInfo.ridge_index = i + 1;
            tempPtInfo.keypoint_index = j;
            tempPtInfo.numbers = filtered_backshape_keypoints_[i].size();
            auto forward_last_points =               //注意点的顺序
                    common::commonMath::computeForwardAndBackPoints(filtered_backshape_keypoints_[i],
                                                                    filtered_backshape_keypoints_[i][j]);
            tempPtInfo.start_curve_point =
                    common::commonMath::findPointOnSegment(forward_last_points[0],
                                                          filtered_backshape_keypoints_[i][j],
                                                           SET_STARTTURN_DISTANCE,
                                                           false);
            //计算下一垄的第二个点
            polygonPoint last_point;
            if(i+1 < filtered_backshape_keypoints_.size()-1){
                 last_point =  filtered_backshape_keypoints_[i+1][1];
            }
            if(j == filtered_backshape_keypoints_[i].size()-1){
                tempPtInfo.end_curve_point =
                        common::commonMath::findPointOnSegment(filtered_backshape_keypoints_[i][j],
                                                               last_point,
                                                               SET_ENDTURN_DISTANCE,
                                                               true);
            }else{
                tempPtInfo.end_curve_point =
                        common::commonMath::findPointOnSegment(filtered_backshape_keypoints_[i][j],
                                                               forward_last_points[1],
                                                               SET_ENDTURN_DISTANCE,
                                                               true);
            }
            backshape_keypts_info_[filtered_backshape_keypoints_[i][j]] = tempPtInfo;
        }
    }
}

std::vector<pathInterface::pathPoint> pathPolygonPlan::computeRidgeRoutingpts(int ridge_index){
        //暂时约定每一垄第一个点是不需要进行处理弯道的
        pathInterface::pathPoint  point_1;
        ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
        std::vector<std::vector<double>> finalpath;
        std::vector<pathInterface::pathPoint>  storageAllPath;
        double startPoint[3],endPoint[3];
        auto  ordered_points = filtered_backshape_keypoints_[ridge_index];
        //第一个点
        if(SET_REVERSING_FLAG){
            point_1.x = ordered_points[0].x;
            point_1.y = ordered_points[0].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
        }else{        //这样处理有问题，这个弯道的结束点并不一定等于直线段的起点，暂时这样处理
            if(ridge_index == 0){
                point_1.x = ordered_points[0].x;
                point_1.y = ordered_points[0].y;
                point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                point_1.ridge_number = ridge_index;
                storageAllPath.push_back(point_1);
            }else{
                //do nothing
            }
        }

        //后续点位均包含弯道处理
        //统计需要弯道处理的点
        int num = filtered_backshape_keypoints_[ridge_index].size() -1;
        for(int i = 1;i <=num;i++){
            //点位弯道处理
            auto temp_point =  ordered_points[i];
            Point point_temp1 = backshape_keypts_info_[temp_point].start_curve_point;
            Point point_temp2 = backshape_keypts_info_[temp_point].end_curve_point;
            startPoint[0] = point_temp1.x;
            startPoint[1] = point_temp1.y;
            startPoint[2] = point_temp1.heading;
            endPoint[0] = point_temp2.x;
            endPoint[1] = point_temp2.y;
            endPoint[2] = point_temp2.heading;
            r->reedsShepp(startPoint,endPoint);
            finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0;j < finalpath.size();j++){
                pathInterface::pathPoint   pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }

            //判断是否需要添加
            if(SET_REVERSING_FLAG){
                //弯道结束需要增加3个点位信息
                aiforce::Route_Planning::polygonPoint p2;
                p2.x = point_temp2.x;
                p2.y = point_temp2.y;
                auto increase_points3 =
                        common::commonMath::findPointExtendSegment(temp_point,
                                                                   p2,
                                                                   SET_CONVERTDIRECTION_DIST,
                                                                   true,
                                                                   SET_CONVERTDIRECTION_COUNT);
                for(auto it : increase_points3){
                    it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                    it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                    it.ridge_number = ridge_index;
                    storageAllPath.push_back(it);
                }
                //增加倒车点位信息
                std::vector<polygonPoint>  lineInfo;
                lineInfo.push_back(p2);
                lineInfo.push_back(temp_point);
                auto back_points =
                        common::commonMath::densify(lineInfo,
                                SET_BACK_DIS);
                for(auto it : back_points){
                       pathInterface::pathPoint temp_point;
                       temp_point.x = it.x;
                       temp_point.y = it.y;
                       temp_point.path_point_mode1 =  pathInterface::pathPointMode1::WORK_AREA;
                       temp_point.path_point_mode2 =  pathInterface::pathPointMode2::BACK;
                       temp_point.ridge_number = ridge_index;
                       storageAllPath.push_back(temp_point);
                }
                //结束倒车增加3个点位信息
                auto ending_back_points =
                        common::commonMath::findPointExtendSegment(temp_point,
                                                                   p2,
                                                                   SET_CONVERTDIRECTION_DIST,
                                                                   false,
                                                                   SET_CONVERTDIRECTION_COUNT);
                for(auto it : ending_back_points){
                    it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                    it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                    it.ridge_number = ridge_index;
                    storageAllPath.push_back(it);
                }
            }else{
                //不倒车处理
            }
        }
        return storageAllPath;
}

const std::vector<std::vector<polygonPoint>> pathPolygonPlan::getFilteredBackShapeKeyPoints()  const{
    return filtered_backshape_keypoints_;
}

//计算内缩第一次和第二次构成的往中心走的线段的kbline
void pathPolygonPlan::computeEveryKbLine(){
     auto first_polygon    =    deleteRepeatPolyPts(storageNarrowPolygonPoints_[0]);
     LOG(INFO) << "narrow  first polygon(no repeat points) size is :" << first_polygon.size();
     auto delete_polygon = deleteRepeatPolyPts(storageNarrowPolygonPoints_[1]);
     LOG(INFO) << "narrow  second polygon(no repeat points) size is :" << delete_polygon.size();
     auto second_polygon = anewStoragePolygonPoints(delete_polygon,
             first_polygon[0]);

    for(int i = 0; i < first_polygon.size() ;i++){
        double segmentLength =
                sqrt(pow(second_polygon[i].x - first_polygon[i].x,2) +
                     pow(second_polygon[i].y - first_polygon[i].y,2));
        double dx = second_polygon[i].x - first_polygon[i].x;
        double dy = second_polygon[i].y - first_polygon[i].y;

        lineKb temp;
        temp.count = 0;
        temp.fixedPt.x = first_polygon[i].x;
        temp.fixedPt.y = first_polygon[i].y;
        temp.dx = dx;
        temp.dy = dy;
        polygonPoint temp1,temp2;
        temp1.x = first_polygon[i].x;
        temp1.y = first_polygon[i].y;
        temp2.x = second_polygon[i].x;
        temp2.y = second_polygon[i].y;
        temp.points.push_back(temp1);
        temp.points.push_back(temp2);
        temp.distance = sqrt((temp2.x - temp1.x) * (temp2.x - temp1.x) +
                                     (temp2.y - temp1.y) * (temp2.y - temp1.y));
        k_b_data_.push_back(temp);
    }
    LOG(INFO) << "all kb line nubmer is :" << k_b_data_.size();
}

//沿着某个点最近的点开始对多边形的点重新开始排序
std::vector<polygonPoint>  pathPolygonPlan::anewStoragePolygonPoints(std::vector<polygonPoint> points,
                                                                      polygonPoint given_point){
    polygonPoint min_point;
    double min_distance = DBL_MAX;
    std::vector<polygonPoint> storage_points;
    //找到距离最小的点
    for(auto it : points){
         double result = common::commonMath::distanceTwoPolygonPoints(it,given_point);
         if(min_distance > result){
             min_point.x = it.x;
             min_point.y = it.y;
             min_distance = result;
         }
    }
    //从这个点开始进行顺时针排序
    int num = points.size();
    int number ;
    for(int i = 0;i < num;i++){
        if(fabs(points[i].x - min_point.x) < 0.1 &&
           fabs(points[i].y - min_point.y) < 0.1){
            number = i;
        }
    }
    polygonPoint next;
    storage_points.push_back(min_point);
    for(int i = 1;i <=num-1;i++){
        next = points[(number+1)%num];
        storage_points.push_back(next);
        number = number + 1;
    }
    return storage_points;
}

//统计内缩后的多边形们沿着一个方向内缩的最大深度
void pathPolygonPlan::judgePointPosition(int i ,
                             boost::geometry::model::multi_polygon<polygon> result ){
    //bug fmod()函数除以自身余数为自身
    double times,y_value,distancePts_2,distancePts_near_2;
    double judge_value_x,judge_value_y;
    Point vector_p1p2 ;
    if(i != 1 && i != 2){
        for(auto it = result.begin();
            it != result.end();
            it++){
            for(auto j = it->outer().begin();
                j != it->outer().end()-1;
                j++){
                for(auto& im : k_b_data_) {
                     vector_p1p2.x = (*j).x() - im.fixedPt.x;
                     vector_p1p2.y = (*j).y() - im.fixedPt.y;
                     double dot = vector_p1p2.x * im.dx + vector_p1p2.y * im.dy;
                     double len1 = sqrt(im.dx * im.dx + im.dy * im.dy);
                     double len2 = sqrt(vector_p1p2.x * vector_p1p2.x + vector_p1p2.y *vector_p1p2.y);
                     double cosvalue = dot / (len1 * len2);
                     double angle = acos(cosvalue) * 180 / M_PI;

                     int num = im.points.size();
                     distancePts_near_2 = sqrt( ((*j).x() - im.points[num-1].x) *
                                                        ((*j).x() - im.points[num-1].x) +
                                                        ((*j).y() - im.points[num-1].y) *
                                                                ((*j).y() - im.points[num-1].y));
                    if ((fabs(angle)< 5) &&
                            (fabs(distancePts_near_2 - im.distance) < 0.5)){
                        im.count += 1;
                        polygonPoint temp;
                        temp.x = (*j).x();
                        temp.y = (*j).y();
                        im.points.push_back(temp);
                    }
                }
            }
        }
    }
}

//找到内缩对应的最长边对应的原始多边形顶点
void pathPolygonPlan::findSuitableEntrance(std::vector<Point> points ){
    for(auto i  : k_b_data_){
        LOG(INFO) << "count is :" << i.count << " " << i.fixedPt.x << " "<< i.fixedPt.y;
        if( i.count == count_narrow_polygon_numbers_ - 2  ||
            i.count == count_narrow_polygon_numbers_ - 3  ||
            i.count == count_narrow_polygon_numbers_ -1 ){ //找到对应的最长边

            LOG(INFO) << "find suitable entrance ! the line is longest !";
            if(i.count == count_narrow_polygon_numbers_ - 3){
                LOG(INFO) << "find the kb line point size  is :" << i.count + 2;
                LOG(INFO) << "the last ridge is triangle !";
                LOG(INFO) << "narrow polygons size == kb line points size +1 !";
            }
            if(i.count == count_narrow_polygon_numbers_ - 1){
                LOG(INFO) << "find the kb line point size is :" << i.count + 2;
                LOG(INFO) << "narrow polygons size == kb line points size !";
            }
            if(i.count == count_narrow_polygon_numbers_ - 2){
                LOG(INFO) << "find the kb line point size is :" << i.count + 2;
                LOG(INFO) << "narrow polygons size == kb line points size +2!";
            }

            //求一个足够长的线段与原始多边形相交的点
//            double k_value = i.k;
//            double b = i.b;
            double temp_x = 0,temp_y = 0;
//            if(i.points[i.points.size()-1].x > i.points[i.points.size()-2].x){ //x在增大
                temp_x = i.points[i.points.size()-1].x  - i.dx * SET_VIRTUAL_LINE_LENGTH;
                temp_y = i.points[i.points.size()-1].y  - i.dy * SET_VIRTUAL_LINE_LENGTH;
//            }else{ //x在减小
//                temp_x = i.points[i.points.size()-1].x - i.dx * SET_VIRTUAL_LINE_LENGTH;
//                temp_y = i.points[i.points.size()-1].y - i.dy * SET_VIRTUAL_LINE_LENGTH;
//            }
           LOG(INFO) << "the last point is :" << i.points[i.points.size()-1].x << " " << i.points[i.points.size()-1].y;
            //计算该线段与原始多边形的交点
            polygon  poly;
            for(auto it : points){
                poly.outer().push_back(point(it.x,it.y));
            }
            linestring_type  line;
            line.push_back(point(i.points[i.points.size()-1].x,
                                 i.points[i.points.size()-1].y));
            line.push_back(point(temp_x,
                                 temp_y));
            std::vector<point> output;
            boost::geometry::intersection(line,poly,output);

            //线段最内部的点
            line_origin_entrance_.push_back(i.points[i.points.size()-1]);
            Point tempPt;
            tempPt.x = output[0].x();
            tempPt.y = output[0].y();
            LOG(INFO) << "temp           " << tempPt.x << " " << tempPt.y;
            //线段最外侧的点
            line_origin_entrance_.push_back(tempPt);
            return;   //暂时只算一次
        }
    }
}

const std::vector<Point> pathPolygonPlan::getLineOriginEntrance() const {
    return line_origin_entrance_;
}


//给出一个点，得到这个点和最小多边形的质心一共的交点
void pathPolygonPlan::computePolygonsAndLineNode(Point node) {
    //创建线段
     linestring_type  line;
     line.push_back(point(min_polygon_centroid_.x,
                               min_polygon_centroid_.y));
     line.push_back(point(node.x,node.y));
     //计算该线段和多边形的交点们
     for(int i = 0; i < count_narrow_polygon_numbers_;i++ ){
         for(auto it = storageNarrowPolygonPoints2_[i].begin();
                  it != storageNarrowPolygonPoints2_[i].end();
                  it++){
             std::vector<point> tempPoints;
             boost::geometry::intersection(line, *it,tempPoints );
             for(auto i_idx : tempPoints){
                     polygonPoint tempPoint;
                     tempPoint.x = i_idx.x();
                     tempPoint.y = i_idx.y();
                     polygonIntersectionPoints_.push_back(tempPoint);
             }
         }
     }
}


//给出一个线段，求该线段与内缩多边形们的交点
void  pathPolygonPlan::computePolygonsAndLineNode(std::vector<Point> linePoints) {
    linestring_type  line;
    line.push_back(point(linePoints[0].x,linePoints[0].y));
    line.push_back(point(linePoints[1].x,linePoints[1].y));

    for(int i = 0; i < count_narrow_polygon_numbers_;i++){
        for(auto it = storageNarrowPolygonPoints2_[i].begin();
                 it != storageNarrowPolygonPoints2_[i].end();
                 it++){
            std::vector<point> tempPoints;
            boost::geometry::intersection(line,*it,tempPoints);
            for(auto i_idx: tempPoints){
                polygonPoint tempPoint;
                tempPoint.x = i_idx.x();
                tempPoint.y = i_idx.y();
                polygonIntersectionPoints_.push_back(tempPoint);
            }
        }
    }
}

void pathPolygonPlan::updatePolygonPointsIncrease(){
    int num = storageNarrowPolygonPoints_.size();
    for(auto it = 0 ;it < storageNarrowPolygonPoints_.size()-1;it++){
        auto poly_points = insertPointToPolygon(
                polygonIntersectionPoints_[it],
                storageNarrowPolygonPoints_[it]);
        insertedPtToNarrowPolygon_.push_back(poly_points);
    }
    //最后一垄特殊处理重新排序
    auto tempPolygon = storageNarrowPolygonPoints_[num-1];
    //用于更新最后一垄是四边形的特殊情况
    if(last_polys_type_ == lastPolyIdentify::POLY_FOUR){
           int size_polygon = tempPolygon.size();
            LOG(INFO) << "the last polygon size is :" << size_polygon ;
            int number ;
            std::vector<polygonPoint>  end_polygon;
            double distance1 = common::commonMath::distance2(line_origin_entrance_[0],
                                                              tempPolygon[0]);
            double distance2 = common::commonMath::distance2(line_origin_entrance_[0],
                                                              tempPolygon[1]);
            double distance3 = common::commonMath::distance2(line_origin_entrance_[0],
                                                               tempPolygon[2]);
            double distance4 = common::commonMath::distance2(line_origin_entrance_[0],
                                                              tempPolygon[3]);
            std::vector<double>  temp_dis_stor;
            temp_dis_stor.push_back(distance1);
            temp_dis_stor.push_back(distance2);
            temp_dis_stor.push_back(distance3);
            temp_dis_stor.push_back(distance4);
            double min_dis = DBL_MAX;
            int ordered_count;
            for(int i = 0; i < 4;i++){
                if(min_dis > temp_dis_stor[i]){
                    min_dis = temp_dis_stor[i];
                    ordered_count = i;
                }
            }
            end_polygon.push_back(tempPolygon[ordered_count]);
            number = ordered_count;
            polygonPoint next;
            for(int j = 1;j <= size_polygon -1;j++){
                next = tempPolygon[(number+1)%size_polygon];
                end_polygon.push_back(next);
                number = number + 1;
            }
            end_polygon.push_back(tempPolygon[ordered_count]);
            insertedPtToNarrowPolygon_.push_back(end_polygon);
            return;
    }
    insertedPtToNarrowPolygon_.push_back(tempPolygon);
}

const std::vector<std::vector<polygonPoint>>  pathPolygonPlan::getInsertedPolygonPointsIncrease() const{
    return  insertedPtToNarrowPolygon_;
}
//
//void pathPolygonPlan::updatePolygonPointsSequencem(){
//      std::vector<std::vector<Point>> temp_points;
//      for(auto i : storageNarrowPolygonPoints_){
//          std::vector<Point> temp;
//          for(auto j : i){
//              Point point_m;
//              point_m.x = j.x;
//              point_m.y = j.y;
//              temp.push_back(point_m);
//          }
//          temp_points.push_back(temp);
//      }
//      for(auto it : temp_points){
//        auto polygonPts = common::commonMath::updatePolygonPointsSequence(it);
//        updatedPolygonPointsSequence_.push_back(polygonPts);
//      }
//}

// const  std::vector<std::vector<Point>> pathPolygonPlan::getUpdatedPolygonPointsSequence() const {
//    return updatedPolygonPointsSequence_;
//}

void pathPolygonPlan::computebackShapeKeypoints(){
    std::vector<polygonPoint>  temp_points;
    LOG(INFO) << "compute polygon intersection points size is :"
              << polygonIntersectionPoints_.size();
    int diff_narrow_and_intsPts = count_narrow_polygon_numbers_
            - polygonIntersectionPoints_.size();
    LOG(INFO) << "narrow polygon number - polygonsectionpoints  = " << diff_narrow_and_intsPts;
#ifdef  JUDGE_CLOCKWISE
    if(fabs(diff_narrow_and_intsPts) == 1 ||
       fabs(diff_narrow_and_intsPts) == 0 ||
       fabs(diff_narrow_and_intsPts) == 2){
        LOG(INFO) << "the situation + the last ridge can finish plan !";
//        //未包含最后一垄
//        for(int i = 0;i < count_narrow_polygon_numbers_ -1;i++){
//            if(i == 0){
//                for(int  it = 1 ; it < middle_points_polygon_[i].size() ;it ++){
//                    temp_points.push_back(middle_points_polygon_[i][it]);
//                }
//                temp_points.push_back(middle_points_polygon_[i][0]);
//            }else{
//                for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
//                    temp_points.push_back(middle_points_polygon_[i][it]);
//                }
//                temp_points.push_back(middle_points_polygon_[i][0]);
//            }
////        for(int  it = 0 ; it < storageNarrowPolygonPoints_[i].size() -1;it ++){
////               temp_points.push_back(storageNarrowPolygonPoints_[i][it]);
////        }
//            backShape_keypoints_.push_back(temp_points);
//            temp_points.clear();
//        }
//        //添加最后一笼
//        backShape_keypoints_.push_back(middle_points_polygon_[count_narrow_polygon_numbers_-1]);

        //分类处理
        switch(last_polys_type_){
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_THREE:{
                //未包含最后三角形的垄
                for(int i = 0;i <  record_poly_index_;i++){
                    if(i == 0){
                        for(int  it = 1 ; it < middle_points_polygon_[i].size() ;it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }else if(i == record_poly_index_ -1){
                        for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        //这里是为了处理最后一笼时找到入口
                        temp_points.push_back(polygonPoint(middle_points_polygon_[i][1].x + 0.1,
                                                           middle_points_polygon_[i][1].y));
                    }else{
                        for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }
                    backShape_keypoints_.push_back(temp_points);
                    temp_points.clear();
                }
                //倒数第二垄
                //倒数第二垄第二个点是比较的关键点
               computeLastRidgeKeyPoints4and3();
                break;
            }
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_FOUR:{
                //首先判断一下有没有可能倒数第二笼并没有入口的情况；
                if(fabs(diff_narrow_and_intsPts) == 2){
                    record_poly_index_  = count_narrow_polygon_numbers_ - 2;
                }
                for(int i = 0;i <  record_poly_index_;i++){
                    if(i == 0){
                        for(int  it = 1 ; it < middle_points_polygon_[i].size() ;it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }else{
                        for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }
                    backShape_keypoints_.push_back(temp_points);
                    temp_points.clear();
                }
                if(fabs(diff_narrow_and_intsPts) == 2){
                    backShape_keypoints_.push_back(storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-2]);
                }
                computeLastRidgeKeyPoints4and4(fabs(diff_narrow_and_intsPts));
                break;
            }
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR:{
                int num = storageNarrowPolygonPoints_.size();
                for(int i = 0;i <  num -1;i++){
                    if(i == 0){
                        for(int  it = 1 ; it < middle_points_polygon_[i].size() ;it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }else{
                        for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
                            temp_points.push_back(middle_points_polygon_[i][it]);
                        }
                        temp_points.push_back(middle_points_polygon_[i][0]);
                    }
                    backShape_keypoints_.push_back(temp_points);
                    temp_points.clear();
                }
                //添加最后一垄的处理
                if(!line_middle_.empty()){
                    computeLastRidgeKeyPoints4();
                }else{
                    backShape_keypoints_.push_back(middle_points_polygon_[num-1]);
                }
                break;
            }
            default:{
                LOG(INFO) << "This situation has not been taken into account !";
            }
        }
    }else if(fabs(diff_narrow_and_intsPts) == 3){
        LOG(INFO) << "the situation + the last two ridge can finish plan !";
        for(int i = 0;i < count_narrow_polygon_numbers_ -2;i++){
            if(i == 0){
                for(int  it = 1 ; it < middle_points_polygon_[i].size() ;it ++){
                    temp_points.push_back(middle_points_polygon_[i][it]);
                }
                temp_points.push_back(middle_points_polygon_[i][0]);
            }else{
                for(int  it = 1 ; it < middle_points_polygon_[i].size();it ++){
                    temp_points.push_back(middle_points_polygon_[i][it]);
                }
                temp_points.push_back(middle_points_polygon_[i][0]);
            }
//        for(int  it = 0 ; it < storageNarrowPolygonPoints_[i].size() -1;it ++){
//               temp_points.push_back(storageNarrowPolygonPoints_[i][it]);
//        }
            backShape_keypoints_.push_back(temp_points);
            temp_points.clear();
        }
        backShape_keypoints_.push_back(storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-2]);
        backShape_keypoints_.push_back(storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-1]);
    }else {
        LOG(INFO) << "the situation + no consider !";
    }
#else
    if(fabs(diff_narrow_and_intsPts) == 1 ||
       fabs(diff_narrow_and_intsPts) == 0){
        LOG(INFO) << "the situation + the last ridge can finish plan !";
        //未包含最后一垄
        for(int i = 0;i < count_narrow_polygon_numbers_ -1;i++){
                for(auto it = middle_points_polygon_[i].rbegin();
                         it != middle_points_polygon_[i].rend();
                         it++){
                    temp_points.push_back(*it);
                }
            backShape_keypoints_.push_back(temp_points);
            temp_points.clear();
        }
        //添加最后两笼
        std::vector<polygonPoint>  last_ridge;
        for(auto it = middle_points_polygon_[count_narrow_polygon_numbers_-1].rbegin() ;
                 it != middle_points_polygon_[count_narrow_polygon_numbers_-1].rend();
                 it++){
            last_ridge.push_back(*it);
        }
        backShape_keypoints_.push_back(last_ridge);
    }else if(fabs(diff_narrow_and_intsPts) == 2){
        LOG(INFO) << "the situation + the last two ridge can finish plan !";
        for(int i = 0;i < count_narrow_polygon_numbers_ -2;i++){
            for(auto it = middle_points_polygon_[i].rbegin();
                it != middle_points_polygon_[i].rend();
                it++){
                temp_points.push_back(*it);
            }
            backShape_keypoints_.push_back(temp_points);
            temp_points.clear();
        }
        std::vector<polygonPoint> last_first_ridge;
        std::vector<polygonPoint> last_second_ridge;
        for(auto it = storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-2].rbegin();
                 it != storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-2].rend();
                 it ++){
            last_second_ridge.push_back(*it);
        }
        for(auto it = storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-1].rbegin();
                 it != storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-1].rend();
                 it ++){
            last_first_ridge.push_back(*it);
        }
        backShape_keypoints_.push_back(last_second_ridge);
        backShape_keypoints_.push_back(last_first_ridge);
    } else {
        LOG(INFO) << "the situation + no consider !";
    }
#endif
}


void  pathPolygonPlan::computeLastRidgeKeyPoints4(){
       int num = storageNarrowPolygonPoints_.size();
//       backShape_keypoints_.push_back(middle_points_polygon_[num-1]);
       std::vector<polygonPoint> storage_last_keypts;
       for(auto it : middle_points_polygon_[num - 1]){
           storage_last_keypts.push_back(it);
       }
       //增加四边形中线
       int last_ridge_num = middle_points_polygon_[num - 1].size();
       auto point_ordered = middle_points_polygon_[num - 1][last_ridge_num -1];
       double distance1 = common::commonMath::distance2(line_middle_[0],
                                                        point_ordered);
       double distance2 = common::commonMath::distance2(line_middle_[1],
                                                         point_ordered);
       if(distance1 < distance2){
           storage_last_keypts.push_back(line_middle_[0]);
           storage_last_keypts.push_back(line_middle_[1]);
       }else{
           storage_last_keypts.push_back(line_middle_[1]);
           storage_last_keypts.push_back(line_middle_[0]);
       }
       backShape_keypoints_.push_back(storage_last_keypts);
}

void  pathPolygonPlan::computeLastRidgeKeyPoints4and3(){
      int number = record_poly_index_;
      polygonPoint specify_point;
      std::vector<polygonPoint> temp_points;
      std::vector<polygonPoint> filtered_points = deleteRepeatPolyPts(middle_points_polygon_[number-1]);
      specify_point = filtered_points[1];
      LOG(INFO) << "the last ridege entrance point is : (" << specify_point.x << "," << specify_point.y << ")";
      specify_point.heading = atan2(filtered_points[1].y - filtered_points[0].y,
                                    filtered_points[1].x - filtered_points[0].x);
      //将倒数第二垄加入到关键点位中
      for(int i = 1; i < filtered_points.size();i++){
            temp_points.push_back(filtered_points[i]);
      }
      //这里这样处理是为了后续过滤时不将此点删除
      temp_points.push_back(polygonPoint(filtered_points[1].x +0.1,filtered_points[1].y));
      // backShape_keypoints_.push_back(temp_points);
      //将倒数第一笼存储到关键点位中
      //根据指定点计算最后一笼的关键点信息
      temp_points.clear();
      //比较AC、BC从哪个line开始进入
      //一共四个入口AC\BC\C左\C右
      double distance1 = common::commonMath::distance2(line_long_AB_[0],specify_point);
      double distance2 = common::commonMath::distance2(line_long_AB_[1],specify_point);
      double distance3 = common::commonMath::distance2(last_ridge_AC_pts_[last_ridge_AC_pts_.size()-1],specify_point);
      double distance4 = common::commonMath::distance2(last_ridge_BC_pts_[last_ridge_BC_pts_.size()-1],specify_point);
      LOG(INFO) << "ac distance is ： " << distance1;
      LOG(INFO) << "bc distance is : " << distance2;
      LOG(INFO) << "c-ac distance is ： "<< distance3;
      LOG(INFO) << "c-bc distance is : " << distance4;
      std::unordered_map<double,double>  transfer_flag;
      transfer_flag[distance1] = 1;
      transfer_flag[distance2] = 2;
      transfer_flag[distance3] = 3;
      transfer_flag[distance4] = 4;
      std::vector<double>  stor_distance;
      stor_distance.push_back(distance1);
      stor_distance.push_back(distance2);
      stor_distance.push_back(distance3);
      stor_distance.push_back(distance4);
      double min_dis = DBL_MAX;
      std::vector<polygonPoint>  storage_last_keypts;
      int spec_number ;
      for(int i = 0;i < 4;i++){
          if(min_dis > stor_distance[i]){
              min_dis = stor_distance[i];
              spec_number = i ;
          }
      }
      spec_number +=1;
      LOG(INFO) << "the specical number is : " << spec_number;
      switch (spec_number){
          case 1:{
              LOG(INFO) << "the last ridge entrance, enter the situation AC line !";
              storage_last_keypts.push_back(line_long_AB_[0]);
              storage_last_keypts.push_back(line_long_AB_[1]);
              for(int i = 0;i < move_last_ridge_AC_pts_.size();i++){
                  if(i % 2 == 0){
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                  } else{
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                  }
              }
              break;
          }
          case 2:{
              LOG(INFO) << "the last ridge entrance, enter the situation BC line !";
              storage_last_keypts.push_back(line_long_AB_[1]);
              storage_last_keypts.push_back(line_long_AB_[0]);
              for(int i = 0;i < move_last_ridge_AC_pts_.size();i++){
                  if(i % 2 == 0){
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                  }else{
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                  }
              }
              break;
          }
          case 3:{
              LOG(INFO) << "the last ridge entrance, enter the situation C-AC line !";
              std::reverse(move_last_ridge_AC_pts_.begin(),move_last_ridge_AC_pts_.end());
              std::reverse(move_last_ridge_BC_pts_.begin(),move_last_ridge_BC_pts_.end());
              for(int i = 0;i < move_last_ridge_BC_pts_.size();i++){
                  if(i % 2 == 0){
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                  }else{
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                  }
              }
              //将ABline距离较近的点排序处理
              double distance1 = common::commonMath::distance2(storage_last_keypts[storage_last_keypts.size()-1],
                                                               line_long_AB_[0]);
              double distance2 = common::commonMath::distance2(storage_last_keypts[storage_last_keypts.size()-1],
                                                               line_long_AB_[1]);
              if(distance1 < distance2){
                  storage_last_keypts.push_back(line_long_AB_[0]);
                  storage_last_keypts.push_back(line_long_AB_[1]);
              }else{
                  storage_last_keypts.push_back(line_long_AB_[1]);
                  storage_last_keypts.push_back(line_long_AB_[0]);
              }
              break;
          }
          case 4:{
              LOG(INFO) << "the last ridge entrance, enter the situation C-BC line !";
              std::reverse(move_last_ridge_AC_pts_.begin(),move_last_ridge_AC_pts_.end());
              std::reverse(move_last_ridge_BC_pts_.begin(),move_last_ridge_BC_pts_.end());
              for(int i = 0;i < move_last_ridge_AC_pts_.size();i++){
                  if(i % 2 == 0){
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                  }else{
                      storage_last_keypts.push_back(move_last_ridge_AC_pts_[i]);
                      storage_last_keypts.push_back(move_last_ridge_BC_pts_[i]);
                  }
              }
              //将ABline距离较近的点排序处理
              double distance1 = common::commonMath::distance2(storage_last_keypts[storage_last_keypts.size()-1],
                                                               line_long_AB_[0]);
              double distance2 = common::commonMath::distance2(storage_last_keypts[storage_last_keypts.size()-1],
                                                               line_long_AB_[1]);
              if(distance1 < distance2){
                  storage_last_keypts.push_back(line_long_AB_[0]);
                  storage_last_keypts.push_back(line_long_AB_[1]);
              }else{
                  storage_last_keypts.push_back(line_long_AB_[1]);
                  storage_last_keypts.push_back(line_long_AB_[0]);
              }
              break;
          }
          default:
              break;
      }
      backShape_keypoints_.push_back(storage_last_keypts);
}

void pathPolygonPlan::computeLastRidgeKeyPoints4and4(int count_flag){
     int number = record_poly_index_;
    polygonPoint specify_point;
    std::vector<polygonPoint> temp_points;
    std::vector<polygonPoint> filtered_points = deleteRepeatPolyPts(middle_points_polygon_[number-1]);

    if(count_flag == 2){
        int number_m = backShape_keypoints_[backShape_keypoints_.size()-1].size();
        specify_point = backShape_keypoints_[backShape_keypoints_.size()-1][number_m-1];
    }else{
            specify_point = filtered_points[1];
    }
    LOG(INFO) << "the last ridege entrance point is : (" << specify_point.x << "," << specify_point.y << ")";
    specify_point.heading = atan2(filtered_points[1].y - filtered_points[0].y,
                                  filtered_points[1].x - filtered_points[0].x);
    //根据指定点计算距离其最近的点位信息A\B\D\C
    double distance1 = common::commonMath::distance2(line_rec_short_AB_[0],specify_point);
    double distance2 = common::commonMath::distance2(line_rec_short_AB_[1],specify_point);
    double distance3 = common::commonMath::distance2(line_rec_short_DC_[0],specify_point);
    double distance4 = common::commonMath::distance2(line_rec_short_DC_[1],specify_point);
    LOG(INFO) << "A distance is ： "  <<  distance1;
    LOG(INFO) << "B distance is :  "  <<  distance2;
    LOG(INFO) << "D distance is ： "  <<  distance3;
    LOG(INFO) << "C distance is :  "  <<  distance4;
    std::vector<double> stor_distance;
    stor_distance.push_back(distance1);
    stor_distance.push_back(distance2);
    stor_distance.push_back(distance3);
    stor_distance.push_back(distance4);
    double min_dis = DBL_MAX;
    int spec_number;
    for(int i = 0;i < 4;i++){
        if(min_dis > stor_distance[i]){
            min_dis = stor_distance[i];
            spec_number = i;
        }
    }
    spec_number +=1;
    LOG(INFO) << "the specical number is : " << spec_number;
    std::vector<polygonPoint> storage_last_keypts;
    switch(spec_number){
        case 1:{
            LOG(INFO) << "the last ridge A entrance is Minimum distance !";
            std::reverse(line_rec_short_BA_.begin(),line_rec_short_BA_.end());
            std::reverse(line_rec_short_CD_.begin(),line_rec_short_CD_.end());
            for(int i = 0;i < line_rec_short_BA_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                }else{
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                }
            }
            break;
        }
        case 2:{
            LOG(INFO) << "the last ridge B entrance is Minimum distance !";
            for(int i = 0;i < line_rec_short_BA_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                }else{
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                }
            }
            break;
        }
        case 3:{
            LOG(INFO) << "the last ridge D entrance is Mininum distance !";
            std::reverse(line_rec_short_BA_.begin(),line_rec_short_BA_.end());
            std::reverse(line_rec_short_CD_.begin(),line_rec_short_CD_.end());
            for(int i = 0;i < line_rec_short_BA_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                }else{
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                }
            }
            break;
        }
        case 4:{
            LOG(INFO) << "the last ridge C entrance is Mininum distance !";
            for(int i = 0;i < line_rec_short_BA_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                }else{
                    storage_last_keypts.push_back(line_rec_short_BA_[i]);
                    storage_last_keypts.push_back(line_rec_short_CD_[i]);
                }
            }
            break;
        }
        default:
            break;
    }
    backShape_keypoints_.push_back(storage_last_keypts);
}

const std::vector<std::vector<polygonPoint>> pathPolygonPlan::getBackShapeKeyPoints() const{
    return backShape_keypoints_;
}

const std::vector<std::vector<polygonPoint>> pathPolygonPlan::getNarrowPolygonPoints()const{
    return storageNarrowPolygonPoints_;
}

const polygonPoint pathPolygonPlan::getMinPolygonCentroid() const {
    return min_polygon_centroid_;
}

const int pathPolygonPlan::countNarrowPolygonNumbers() const {
    return count_narrow_polygon_numbers_;
}

const std::vector<polygonPoint> pathPolygonPlan::getPolygonAndLineNodes() const{
    return polygonIntersectionPoints_;
}

bool pathPolygonPlan::pointOnSegment(polygonPoint p, polygonPoint a, polygonPoint b) {
    double crossProduct = (p.y - a.y) * (b.x - a.x) - (p.x - a.x) * (b.y - a.y);
    if (fabs(crossProduct) > 1e-8) { // 如果叉积不为0，则点不在直线上
        return false;
    }

    double dotProduct = (p.x - a.x) * (b.x - a.x) + (p.y - a.y)*(b.y - a.y);
    if (dotProduct < -1e-8 || dotProduct > ((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y))) {
        // 如果点与a、b两个点的向量夹角大于90度或者大于ab向量的长度，则点不在线段上
        return false;
    }
}

std::vector<polygonPoint> pathPolygonPlan::insertPointToPolygon(polygonPoint insertPoint,
                                             std::vector<polygonPoint> polygonPoints){
    std::vector<polygonPoint>  storage_points;
    polygonPoints.pop_back();
    int num = polygonPoints.size();
    for(int i = 0; i < polygonPoints.size()-1;i++){
          if(pointOnSegment(insertPoint,
                            polygonPoints[i],
                            polygonPoints[i+1])){
             storage_points.push_back(polygonPoints[i]);
             polygonPoint  temp_point;
             temp_point.x = insertPoint.x;
             temp_point.y = insertPoint.y;
             temp_point.entrance_ = true;
             storage_points.push_back(temp_point);
             storage_points.push_back(polygonPoints[i+1]);
          }else{
              storage_points.push_back(polygonPoints[i]);
              storage_points.push_back(polygonPoints[i+1]);
          }
    }
    if(pointOnSegment(insertPoint,
                      polygonPoints[num-1],
                      polygonPoints[0])){
        storage_points.push_back(polygonPoints[num-1]);
        polygonPoint  temp_point;
        temp_point.x = insertPoint.x;
        temp_point.y = insertPoint.y;
        temp_point.entrance_ = true;
        storage_points.push_back(temp_point);
    }else{
        storage_points.push_back(polygonPoints[num-1]);
    }
    return storage_points;
}

}
}
