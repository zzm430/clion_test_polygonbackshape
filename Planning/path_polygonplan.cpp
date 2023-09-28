//
// Created by zzm on 2023/6/1.
//
#include <common/math/common_math.h>
#include <Geometry/newCornerTuring_location.h>
#include "path_polygonplan.h"


namespace aiforce{
namespace Route_Planning{

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
            pointbst instance_point;
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
//        if(result.empty()) {
////            dealWithLastSeveralPolygons();
            computeLastRidgeSituation();
//            break;
//        }
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

pathPolygonPlan::pathPolygonPlan(){

}

pathPolygonPlan::~pathPolygonPlan(){

}

void pathPolygonPlan::initialize() {
    curveModeChoose_ =  curveModeChoose::MODE_FISHNAIL;
    curveLocationChoose_ = curveLocationChoose::MODE_HEADLANDS_CURVE__START_END;
}

void pathPolygonPlan::cgalNarrowPolygons(std::vector<Point> &points){
    int num_size = points.size();
    //去掉最后一个闭环点
    cgal_Polygon_2 poly;
    for(int i = 0;i < num_size-1; i++){
        poly.push_back(cgal_Point(points[i].x,points[i].y));
    }
    cgal_SsPtr  iss =
            CGAL::create_interior_straight_skeleton_2(
                                poly.vertices_begin(),
                                poly.vertices_end());
    std::vector<polygonPoint>  storage_polypts;
    std::vector<polygonPoint>   inner_polypts;
    LOG(INFO) << "---------------------------------------------------------------";
    LOG(INFO) << "the straight skeleton vertices is : "
              << iss.get()->size_of_vertices();

    for(auto i = iss.get()->halfedges_begin();
             i != iss.get()->halfedges_end();
             i++){
        print_point(i->opposite()->vertex()->point()) ;
        polygonPoint temp;
        temp.x = i->opposite()->vertex()->point()[0];
        temp.y = i->opposite()->vertex()->point()[1];
        if(i->is_bisector()){
            temp.judge_bisector_ = sideBisector::BISECTOR;
        }else {
            temp.judge_bisector_ = sideBisector::CONTOUR;
        }
        if(i->is_inner_bisector()){
            temp.judge_bisector_ = sideBisector::INNER_BISECTOR;
        }
        std::cout << "->" ;
        print_point(i->vertex()->point());
        std::cout << " " << ( i->is_bisector() ? "bisector" : "contour" ) << std::endl;
        polygonPoint temdd;
        temdd.x = i->vertex()->point()[0];
        temdd.y = i->vertex()->point()[1];
        storage_polypts.push_back(temp);
        storage_polypts.push_back(temdd);
        if(i->is_inner_bisector()){
            print_point(i->opposite()->vertex()->point());
            polygonPoint temp_m;
            temp_m.x = i->opposite()->vertex()->point()[0];
            temp_m.y = i->opposite()->vertex()->point()[1];
            std::cout << "->";
            print_point(i->vertex()->point());
            polygonPoint temp_n;
            temp_n.x = i->vertex()->point()[0];
            temp_n.y = i->vertex()->point()[1];
            inner_polypts.push_back(temp_m);
            inner_polypts.push_back(temp_n);
            last_inner_skeleton_keypts_[temp_m].push_back(temp_n);
        }
        cgalPtMaping_[temdd].push_back(temp) ;
    }

    //根据指定的顶点找到对应的骨架路径
    polygonPoint entrance_point;
    entrance_point.x = points[3].x;
    entrance_point.y = points[3].y;
    std::vector<polygonPoint>  storagePoints;
    int num_inner = inner_polypts.size();
    //根据指定的点找到对应的直骨架入口路径信息
    //存储第一个点为对应的边的顶点
    lastPoly_innerpts_.push_back(entrance_point);
    //存储第二个点为对应的骨架点
    polygonPoint transPt;
    for(auto it :cgalPtMaping_){
        if(it.first == entrance_point){
            for(auto f : it.second){
                if(f.judge_bisector_ == sideBisector::BISECTOR){
                    lastPoly_innerpts_.push_back(f);
                    transPt = f;
                }
            }
        }
    }

    //存储剩余点内骨架点位
    auto all_sken_pts = storage_polypts.size();
    while(transPt != storage_polypts[all_sken_pts-1] &&
          transPt != storage_polypts[all_sken_pts-2]){
          auto temp_pts = cgalPtMaping_[transPt];
          std::vector<polygonPoint> storageTemppt;
          for(auto it : temp_pts) {
              if (it.judge_bisector_ == sideBisector::INNER_BISECTOR) {
                  storageTemppt.push_back(it);
              }
          }
          if(storageTemppt.size() == 1){
              lastPoly_innerpts_.push_back(storageTemppt[0]);
              transPt = storageTemppt[0];
          }else{
              double min_distance  = DBL_MAX;
              polygonPoint min_pt;
              for(auto it : storageTemppt){
                  auto dis = common::commonMath::distance2(it,storage_polypts[all_sken_pts-1]);
                  if(dis < min_distance){
                      min_distance = dis;
                      min_pt = it;
                  }
              }
              lastPoly_innerpts_.push_back(min_pt);
              transPt = min_pt;
          }
    }

    //计算内缩多边形
    double  buffer_distance = DBL_MAX;
    double f;
    std::vector<cgal_PolygonPtrVector> offset_polys;  //专门用于cgal
    cgal_PolygonPtrVector last_poly_ptr;
    bool  flag_enter = false;

    for(auto i = 1;i <= MAX_TRAVERSALS_NUMBERS;i++){
        if(i == 1){
            buffer_distance =  RIDGE_WIDTH_LENGTH/2;
        }else{
            buffer_distance = i * RIDGE_WIDTH_LENGTH - RIDGE_WIDTH_LENGTH/2;
        }
        std::vector<polygonPoint>   poly_pts;
        cgal_PolygonPtrVector offset_polygons =  CGAL::create_offset_polygons_2<cgal_Polygon_2>(
                                                             buffer_distance,*iss);
        if(offset_polygons.size() == 1 && !flag_enter){
            for(auto it : offset_polygons){
                auto m = *it;
                for(auto j : *it){
                    polygonPoint temppt;
                    temppt.x = j.x();
                    temppt.y = j.y();
                    poly_pts.push_back(temppt);
                }
            }
            last_poly_ptr = offset_polygons;
            poly_pts.push_back(poly_pts[0]);     //这个点可以看做是多边形的最后一个点也可以后续计算下个路口的入口点
            cgalPolypts_.push_back(poly_pts);
        }
//        print_polygons(offset_polygons);
        if(offset_polygons.size() > 1 || flag_enter){
            //第一次进入
            if(offset_polygons.size() > 1 && !flag_enter){
                flag_enter = true;
                LOG(INFO) << "spilt polygons appear !";
                LOG(INFO) << "spilt polygons size is : " << offset_polygons.size();
                int num_polys = offset_polygons.size();
                //对生成的分裂多边形们进行遍历，存入对应的数据结构A和B们中
                for(int i = 0;i < num_polys; i++){
                    for(auto it : *offset_polygons[i]){
                        polygonPoint temppt;
                        temppt.x = it.x();
                        temppt.y = it.y();
                        poly_pts.push_back(temppt);
                    }
                    poly_pts.push_back(poly_pts[0]);
                    std::vector<std::vector<polygonPoint>>  temp;
                    temp.push_back(poly_pts);
                    poly_pts.clear();
                    bufferspiltPolysAandBs_.push_back(temp);
                }
            }else if(flag_enter){
                int num = offset_polygons.size();
                for(int i = 0;i < num ;i++){
                    polygon pt_poly;
                    for(int j = 0;j < bufferspiltPolysAandBs_.size();j++){
                        for(auto it :bufferspiltPolysAandBs_[j][0]){
                            pt_poly.outer().push_back(pointbst(it.x,it.y));
                        }
                        bool flag =  boost::geometry::within(
                                pointbst(offset_polygons[i]->begin()->x(),
                                 offset_polygons[i]->begin()->y()), pt_poly);
                        if(flag){
                            for(auto it : *offset_polygons[i]){
                                polygonPoint temppt;
                                temppt.x = it.x();
                                temppt.y = it.y();
                                poly_pts.push_back(temppt);
                            }
                            poly_pts.push_back(poly_pts[0]);
                            bufferspiltPolysAandBs_[j].push_back(poly_pts);
                            poly_pts.clear();
                            break;
                        }
                    }
                }
            }
        }

        if(offset_polygons.size() == 0){
            LOG(INFO) << "already narrow to last ,the narrow poly size is " << i - 1 ;
            cgal_narrow_size_  = i - 1;
            break;
        }
    }
//    std::ofstream test_skeleton_4;
//    test_skeleton_4.open("/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_4.txt",
//                         std::ios::out);
//    for(int i  = 0;i < bufferspiltPolysAandBs_[1].size() ;i++){
//        for(auto j : bufferspiltPolysAandBs_[1][i]){
//            test_skeleton_4 << " " << j.x;
//        }
//    }
//    test_skeleton_4 << std::endl;
//    for(int i = 0; i < bufferspiltPolysAandBs_[1].size() ;i++){
//        for(auto j : bufferspiltPolysAandBs_[1][i]){
//            test_skeleton_4 << " " << j.y;
//        }
//    }
//    test_skeleton_4 << std::endl;
//    test_skeleton_4.close();

    //计算回字形入口需要的线段们
    computeEntranceLines(points);

    std::ofstream  cgal_pts_entrance;
    cgal_pts_entrance.open("/home/zzm/Desktop/test_path_figure-main/src/cgal_pts_entrance.txt",
                           std::ios::out);
    for(auto it : entrance_lines_){
        cgal_pts_entrance << " " << it.x ;
    }
    cgal_pts_entrance << std::endl;
    for(auto it : entrance_lines_){
        cgal_pts_entrance << " " << it.y;
    }
    cgal_pts_entrance << std::endl;
    cgal_pts_entrance.close();

    //对多个分裂多边形进行分开存储
    processSpiltPolys();
    //判断是否要增加内部直骨架路线，应该是包含分裂多边形最后一个多边形:last_poly_ptr
    judgeIncreaseskeleton(inner_polypts);

    //求回字形的入口点位信息
    if(JUDGE_CLOCKWISE){
        //求回字形的内缩多边形和各个线段的交点信息
        int polys_size = cgalPolypts_.size();
        std::map<int,std::vector<polygonPoint>>   find_entrance_pts;
        for(int i = 0;i < polys_size;i++){
            std::vector<pointbst> output;
            std::vector<polygonPoint> trans_output;
            for(int j = 0;j < entrance_lines_.size();j++){
                  if( j % 2 == 0){
                      polygon  poly;
                      for(auto it :cgalPolypts_[i]){
                          poly.outer().push_back(pointbst(it.x,it.y));
                      }
                      linestring_type  line;
                      line.push_back(pointbst(entrance_lines_[j].x,entrance_lines_[j].y));
                      line.push_back(pointbst(entrance_lines_[j + 1].x,entrance_lines_[j + 1].y));
                      boost::geometry::intersection(line,poly,output);
                      if(output.size() == 0){
                          continue;
                      }else{
                          for(auto it : output){
                              polygonPoint temp_pt;
                              temp_pt.x = it.x();
                              temp_pt.y = it.y();
                              find_entrance_pts[i].push_back(temp_pt);
                          }
                      }
                  }
            }
        }
        for(int i = 0;i < find_entrance_pts.size();i++){
            if(!find_entrance_pts[i].empty()){
                if(find_entrance_pts[i].size()){
                    entrance_pts_.push_back(find_entrance_pts[i][find_entrance_pts[i].size()-1]);
                }else{
                    entrance_pts_.push_back(find_entrance_pts[i][0]);
                }
            }
        }
        //将未与入口线段们相交的多边形统一处理
        int judge_number  = find_entrance_pts.size();

        LOG(INFO) << "the find entrance pts size is : " << judge_number;

        auto temp =  cgalPolypts_.size() - judge_number;
       if(temp > SET_POLY_TRANSFER_THR){
           cgalLastPolyType_ = cgalLastPolyIdentify::POLY_LEAVE;
           LOG(INFO) << "the cgalLastPolyType_ is : POLY_LEAVE !";
           find_entrance_pts_size_ = judge_number;
           computeLeaveSituation(judge_number-1);
       }else{
           //当最后一笼是四边形并且temp == 0时
           int polySize = cgalPolypts_.size();
           if(temp == 0  &&
               cgalPolypts_[polySize-1].size() == 5 ){
               cgalLastPolyType_ = cgalLastPolyIdentify::POLY_NONE;
               LOG(INFO) << "the cgalLastPolyType_ is : POLY_NONE !";
               int countSize = cgalPolypts_.size();
               computeLeaveSituation(countSize-1-SET_POLY_TRANSFER_THR);
           }else{
               LOG(INFO) << "the cgalLastPolyType_ is : POLY_LESS_THR !";
               cgalLastPolyType_ = cgalLastPolyIdentify::POLY_LESS_THR;
               int countSize = cgalPolypts_.size();
               computeLeaveSituation(countSize-1-SET_POLY_TRANSFER_THR);
           }
       }
       //在这里对如果出现分裂多边形的情况平行线点位处理
       if(!bufferspiltPolysAandBs_.empty()){
           computeLeaveSituation();
        }
    }

    std::ofstream test_skeleton_6;
    test_skeleton_6.open("/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_6.txt",
                         std::ios::out);
    for(int i  = 0;i < cgalPolypts_.size() ;i++){
        for(auto j : cgalPolypts_[i]){
            test_skeleton_6 << " " << j.x;
        }
    }
    test_skeleton_6 << std::endl;
    for(int i = 0; i <cgalPolypts_.size() ;i++){
        for(auto j : cgalPolypts_[i]){
            test_skeleton_6 << " " << j.y;
        }
    }
    test_skeleton_6 << std::endl;
    test_skeleton_6.close();

    std::ofstream   test_skeleton_2;
    test_skeleton_2.open("/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_2.txt",
                   std::ios::out);
    for(auto it : storage_polypts){
        test_skeleton_2 << " " << it.x ;
    }
    test_skeleton_2 << std::endl;
    for(auto it : storage_polypts){
        test_skeleton_2 << " " << it.y;
    }
    test_skeleton_2 << std::endl;
    test_skeleton_2.close();

    std::vector<polygonPoint> storage_poinm;
    int judge_j = iss.get()->size_of_vertices();
    int j = 0;
    for(auto i = iss.get()->vertices_begin();
        i != iss.get()->vertices_end();
        i++){
           j++;
           if(i->is_skeleton()){
                   polygonPoint temp;
                   temp.x = i->point()[0];
                   temp.y = i->point()[1];
                   storage_poinm.push_back(temp);
           }
    }

    std::ofstream test_skeleton_5;
    test_skeleton_5.open("/home/zzm/Desktop/test_path_figure-main/src/entrance_lines.txt",
                         std::ios::out);
    for(auto it : entrance_lines_){
        test_skeleton_5 << " " << it.x ;
    }
    test_skeleton_5 << std::endl;
    for(auto it : entrance_lines_){
        test_skeleton_5 << " " << it.y;
    }
    test_skeleton_5 << std::endl;
    test_skeleton_5.close();


    std::ofstream test_skeleton_3;
    test_skeleton_3.open("/home/zzm/Desktop/test_path_figure-main/src/test_skeleton_3.txt",
                         std::ios::out);
    test_skeleton_3 << std::endl;
    test_skeleton_3.close();


    LOG(INFO) <<"the enter here ---------------------------------------!!!!!!!!!!!!!!!!!!!!!!";

    std::ofstream   test_111_poly;
    test_111_poly.open("/home/zzm/Desktop/test_path_figure-main/src/test_111_poly.txt",std::ios::out);
    test_111_poly <<std::endl;
    test_111_poly.close();
}

void pathPolygonPlan::processSpiltPolys(){
    //选择分裂多边形哪个作为入口
    std::map<double,int>  polys_map;
    double distance;
    if(!bufferspiltPolysAandBs_.empty()){
        for(int it = 0; it < bufferspiltPolysAandBs_.size();it++){
            polygon poly_m;
            for(auto m : bufferspiltPolysAandBs_[it][0]){
                poly_m.outer().push_back(pointbst(m.x,m.y));
            }
            auto n_size = entrance_lines_.size();
            distance = boost::geometry::distance(
                    pointbst(entrance_lines_[n_size - 1].x,entrance_lines_[n_size -1].y),
                    poly_m);
            LOG(INFO) << "the distance is : " << distance;
            polys_map[distance] = it;
        }
        //对排序后的多边形索引进行存储
        for(auto it = polys_map.begin();it != polys_map.end();it++){
            storage_order_polys_distance_.push_back(it->second);
        }
        //将索引为0的分裂多边形的存储到cgalPolypts_中去
        int order_index = storage_order_polys_distance_[0];
        for(auto it : bufferspiltPolysAandBs_[order_index]){
            cgalPolypts_.push_back(it);
        }
    }
}

void pathPolygonPlan::computeLeaveSituation(int last_ordered_poly_index){
    //判断此多边形的凹凸性
    partition_Polygon_2  transed_poly;
    partition_Polygon_list list_polys;
    std::vector<polygonPoint>   virtual_origin_poly;
    for(int i = 0;i < cgalPolypts_[last_ordered_poly_index].size() - 1;i++){
        transed_poly.push_back(partition_Point_2(
                cgalPolypts_[last_ordered_poly_index][i].x,
                cgalPolypts_[last_ordered_poly_index][i].y));
    }
   bool flag_convex_2 = CGAL::is_convex_2(transed_poly.begin(),transed_poly.end());
    LOG(INFO) << "the flag_convex_2 is : " << flag_convex_2;
    if(!flag_convex_2){
        LOG(INFO) << "This polygon is a concave polygon ！";
        std::vector<cgal_Point> points ;
        for(int i = 0;i < cgalPolypts_[last_ordered_poly_index].size() - 1;i++){
            points.push_back(cgal_Point(
                    cgalPolypts_[last_ordered_poly_index][i].x,
                    cgalPolypts_[last_ordered_poly_index][i].y));
        }
        std::vector<std::size_t> indices(points.size()), out;
        std::iota(indices.begin(), indices.end(),0);
        CGAL::convex_hull_2(indices.begin(), indices.end(), std::back_inserter(out),
                            Convex_hull_traits_2(CGAL::make_property_map(points)));
        std::vector<polygonPoint>   tempPoly;
        for(std::size_t i : out){
            polygonPoint  tempPt;
            tempPt.x = points[i].x();
            tempPt.y = points[i].y();
            tempPoly.push_back(tempPt);
        }
        tempPoly.push_back(tempPoly[0]);
        virtual_origin_poly = tempPoly;

        //针对特殊情况处理
        if(!bufferspiltPolysAandBs_.empty()){
            set_flag_about_buffer_spilt_polys_ = false;  //此处有bug
        }
        std::ofstream   convexHull;
        convexHull.open("/home/zzm/Desktop/test_path_figure-main/src/convexHull.txt",std::ios::out);
        for( std::size_t i : out){
            convexHull << " " << points[i].x();
        }
        convexHull << std::endl;
        for( std::size_t i : out){
            convexHull << " " << points[i].y();
        }
        convexHull << std::endl;
        convexHull.close();
        LOG(INFO) << "the list_polys size is : " << list_polys.size();
    }else{
         virtual_origin_poly =  cgalPolypts_[last_ordered_poly_index];
    }
    double  buffer_distance = RIDGE_WIDTH_LENGTH/2;
   //从此多边形开始内缩垄宽/2;
    cgal_Polygon_2 poly_temp;
    for(int i = 0; i < virtual_origin_poly.size();i++){
        poly_temp.push_back(cgal_Point(
                virtual_origin_poly[i].x,
                virtual_origin_poly[i].y));
    }
    cgal_SsPtr iss =
            CGAL::create_interior_straight_skeleton_2(
                    poly_temp.vertices_begin(),
                    poly_temp.vertices_end());
    std::vector<cgal_PolygonPtrVector> offset_polys;
    cgal_PolygonPtrVector offset_polygons =  CGAL::create_offset_polygons_2<cgal_Polygon_2>(
            buffer_distance,*iss);
    //基于此内缩多边形后续生成剩余关键点
    LOG(INFO) << "virtual origin poly is : "
              << offset_polygons.size();
    std::vector<polygonPoint> stor_poly;
    for(auto it : offset_polygons){
        for(auto j : *it){
            polygonPoint temppt;
            temppt.x = j.x();
            temppt.y = j.y();
            stor_poly.push_back(temppt);
        }
    }
    //将多边形构成闭环
    stor_poly.push_back(stor_poly[0]);
    double max_dis_ridge = -DBL_MAX;
    int count_ordered;
    for(int i = 0;i < stor_poly.size()-1; i++){
        double dis = common::commonMath::distance2(stor_poly[i],stor_poly[i+1]);
        if(dis > max_dis_ridge){
            max_dis_ridge = dis;
            count_ordered = i;
        }
    }
    std::vector<polygonPoint> longest_line;
    longest_line.push_back(stor_poly[count_ordered]);
    longest_line.push_back(stor_poly[count_ordered+1]);
    std::vector<polygonPoint> other_points;
    for(int i = 0;i < stor_poly.size();i++){
        if(stor_poly[i] != stor_poly[count_ordered] &&
           stor_poly[i] != stor_poly[count_ordered+1]){
            other_points.push_back(stor_poly[i]);
        }
    }
    double max_dis = -DBL_MAX;
    polygonPoint max_dis_pt;
    polygonPoint foot_print;
    for(auto it : other_points){
        auto  temp_foot_print = common::commonMath::computeFootPoint(it,
                                                                     longest_line[0],
                                                                     longest_line[1]);
        double dis_pts = common::commonMath::distance2(it,temp_foot_print);
        if(dis_pts > max_dis){
            max_dis = dis_pts;
            foot_print = temp_foot_print;
            max_dis_pt = it;
        }
    }

    //缩小的原来的图形绘制
    std::ofstream test_virtual_origin_poly;
    test_virtual_origin_poly.open(
            "/home/zzm/Desktop/test_path_figure-main/src/test_virtual_origin_poly.txt",
            std::ios::out);
    for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
        test_virtual_origin_poly << " " << (*it).x;
    }
    test_virtual_origin_poly << std::endl;
    for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
        test_virtual_origin_poly << " " << (*it).y;
    }
    test_virtual_origin_poly << std::endl;
    test_virtual_origin_poly.close();

    //计算需要平移次数
    double integer_number =  ceil(max_dis/RIDGE_WIDTH_LENGTH);
    double mod = common::commonMath::doubleMod(max_dis,RIDGE_WIDTH_LENGTH);
    double last_path_cover = mod * RIDGE_WIDTH_LENGTH;

    LOG(INFO) << "max distance is : " << max_dis;
    LOG(INFO) << "need move size is :" << integer_number;
    LOG(INFO) << "the last line cover is :" << last_path_cover;

    //计算平移的点位信息
    std::vector<polygonPoint>  foot_line;
    foot_line.push_back(foot_print);
    foot_line.push_back(max_dis_pt);

    //选择最长边的中线和顶点作为分界线
    //首先计算最长边的中点
    polygonPoint   middle_point;
    middle_point.x = (longest_line[0].x + longest_line[1].x)/2;
    middle_point.y = (longest_line[0].y + longest_line[1].y)/2;
    std::vector<polygonPoint> middle_line;
    middle_line.push_back(middle_point);
    middle_line.push_back(max_dis_pt);

    //这里需要对这个longest_line进行延伸一下防止与四边形没有交点
    auto temp_pt1 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            500,
            true,
            1);
    auto temp_pt2 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            500,
            false,
            1);
    longest_line.clear();
    longest_line.push_back(polygonPoint(temp_pt1[0].x,temp_pt1[0].y));
    longest_line.push_back(polygonPoint(temp_pt2[0].x,temp_pt2[0].y));

    //求内缩多边形与最长的线段的一个线段交点
    polygon  poly_f;
    std::reverse(stor_poly.begin(),stor_poly.end());
    for(auto it : stor_poly){
        poly_f.outer().push_back(pointbst(it.x,it.y));
    }
    //将最长边往最远点移动一定距离
    auto longest_pts = common::commonMath::computeLineTranslationPoints(
            longest_line,
            foot_line,
            RIDGE_WIDTH_LENGTH/2);

    linestring_type  line;
    line.push_back(pointbst(longest_pts[0].x,
                         longest_pts[0].y));
    line.push_back(pointbst(longest_pts[1].x,longest_pts[1].y));
    std::vector<pointbst> output;
    boost::geometry::intersection(line,poly_f,output);
    polygonPoint  vector_1,vector_2;
    polygonPoint  judgeDirection;
    judgeDirection.x = output[0].x();
    judgeDirection.y = output[0].y();
    vector_1 = common::commonMath::construceVector(max_dis_pt,middle_point);
    vector_2 = common::commonMath::construceVector(judgeDirection,middle_point);
    double judge_direction = common::commonMath::pointLocation(vector_1,vector_2);
    if(judge_direction == 1){
        move_pts_line_1_.push_back(polygonPoint(output[0].x(),output[0].y()));
        move_pts_line_2_.push_back(polygonPoint(output[1].x(),output[1].y()));
    }else{
        move_pts_line_1_.push_back(polygonPoint(output[1].x(),output[1].y()));
        move_pts_line_2_.push_back(polygonPoint(output[0].x(),output[0].y()));
    }


    std::vector<polygonPoint>   tempStorage;
    for(int i = 1;i <= integer_number;i++){
        std::vector<polygonPoint> points;
        if(i == 1){
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH);
        }else{
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH + RIDGE_WIDTH_LENGTH *(i-1));
        }

        //计算平移后的线段与四边形的交点
        polygon  poly;
        std::reverse(stor_poly.begin(),stor_poly.end());
        for(auto it : stor_poly){
            poly.outer().push_back(pointbst(it.x,it.y));
        }
        linestring_type  line;
        line.push_back(pointbst(points[0].x,
                             points[0].y));
        line.push_back(pointbst(points[1].x,points[1].y));
        std::vector<pointbst> output;
        boost::geometry::intersection(line,poly,output);
        LOG(INFO) << "output size is :" << output.size();
        if(output.size() != 2){
            LOG(ERROR) << "the line points is wrong !";
            break;
        }
        polygonPoint  trans_pt;
        trans_pt.x = output[0].x();
        trans_pt.y = output[0].y();
        std::vector<polygonPoint> origin_line;
        vector_2  = common::commonMath::construceVector(trans_pt,middle_point);
        double cross_2 =  common::commonMath::pointLocation(vector_1,vector_2);
        if(cross_2 == 1){
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto result_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_1_.push_back(polygonPoint(result_pts[0].x,result_pts[0].y));
            move_pts_line_2_.push_back(polygonPoint(result_pts[1].x,result_pts[1].y));
            tempStorage.push_back(origin_line[0]);
            tempStorage.push_back(origin_line[1]);
        }else{
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto res_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_1_.push_back(polygonPoint(res_pts[1].x,res_pts[1].y));
            move_pts_line_2_.push_back(polygonPoint(res_pts[0].x,res_pts[0].y));
            tempStorage.push_back(origin_line[0]);
            tempStorage.push_back(origin_line[1]);
        }
    }

    std::vector<polygonPoint>  temp;
    for(auto it : move_pts_line_1_){
        temp.push_back(it);
    }
    for(auto it : move_pts_line_2_){
        temp.push_back(it);
    }

    std::ofstream test_rect;
    test_rect.open("/home/zzm/Desktop/test_path_figure-main/src/test_move_pts.txt",std::ios::out);
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).x;
    }
    test_rect << std::endl;
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).y;
    }
    test_rect<< std::endl;
    test_rect.close();
};
void pathPolygonPlan::computeLeaveSituation(std::vector<polygonPoint> & origin_poly){
    cgal_FT  buffer_distance = RIDGE_WIDTH_LENGTH/2;
    auto virtual_origin_poly =  origin_poly;
    //从此多边形开始外扩垄宽/2;
    cgal_Polygon_2 poly_temp;
//    std::reverse(origin_poly.begin(), origin_poly.end());
    for(int i = 0; i < virtual_origin_poly.size() - 1 ;i++){
        poly_temp.push_back(cgal_Point(
                virtual_origin_poly[i].x,
                virtual_origin_poly[i].y));
    }
    std::vector<cgal_Polygon_2> offset_polys;
    cgal_PolygonPtrVector outer_offset_polygons =
            CGAL::create_exterior_skeleton_and_offset_polygons_2(
                    buffer_distance,
                    poly_temp);
    std::vector<polygonPoint> stor_poly;
    print_polygons(outer_offset_polygons);
    for(auto m : *outer_offset_polygons[1]){
        polygonPoint temppt;
        temppt.x = m.x();
        temppt.y = m.y();
        stor_poly.push_back(temppt);
    }

    //将多边形构成闭环
    stor_poly.push_back(stor_poly[0]);
    double max_dis_ridge = -DBL_MAX;
    int count_ordered;
    for(int i = 0;i < stor_poly.size()-1; i++){
        double dis = common::commonMath::distance2(stor_poly[i],stor_poly[i+1]);
        if(dis > max_dis_ridge){
            max_dis_ridge = dis;
            count_ordered = i;
        }
    }
    std::vector<polygonPoint> longest_line;
    longest_line.push_back(stor_poly[count_ordered]);
    longest_line.push_back(stor_poly[count_ordered+1]);
    std::vector<polygonPoint> other_points;
    for(int i = 0;i < stor_poly.size();i++){
        if(stor_poly[i] != stor_poly[count_ordered] &&
           stor_poly[i] != stor_poly[count_ordered+1]){
            other_points.push_back(stor_poly[i]);
        }
    }
    double max_dis = -DBL_MAX;
    polygonPoint max_dis_pt;
    polygonPoint foot_print;
    for(auto it : other_points){
        auto  temp_foot_print = common::commonMath::computeFootPoint(it,
                                                                     longest_line[0],
                                                                     longest_line[1]);
        double dis_pts = common::commonMath::distance2(it,temp_foot_print);
        if(dis_pts > max_dis){
            max_dis = dis_pts;
            foot_print = temp_foot_print;
            max_dis_pt = it;
        }
    }

    //扩大的原来的图形绘制
    std::ofstream test_virtual_origin_poly;
    test_virtual_origin_poly.open(
            "/home/zzm/Desktop/test_path_figure-main/src/test_virtual_origin_poly.txt",
            std::ios::out);
    for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
        test_virtual_origin_poly << " " << (*it).x;
    }
    test_virtual_origin_poly << std::endl;
    for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
        test_virtual_origin_poly << " " << (*it).y;
    }
    test_virtual_origin_poly << std::endl;
    test_virtual_origin_poly.close();

    //计算需要平移次数
    double integer_number =  ceil(max_dis/RIDGE_WIDTH_LENGTH);
    double mod = common::commonMath::doubleMod(max_dis,RIDGE_WIDTH_LENGTH);
    double last_path_cover = mod * RIDGE_WIDTH_LENGTH;

    LOG(INFO) << "max distance is : " << max_dis;
    LOG(INFO) << "need move size is :" << integer_number;
    LOG(INFO) << "the last line cover is :" << last_path_cover;

    //计算平移的点位信息
    std::vector<polygonPoint>  foot_line;
    foot_line.push_back(foot_print);
    foot_line.push_back(max_dis_pt);

    //选择最长边的中线和顶点作为分界线
    //首先计算最长边的中点
    polygonPoint   middle_point;
    middle_point.x = (longest_line[0].x + longest_line[1].x)/2;
    middle_point.y = (longest_line[0].y + longest_line[1].y)/2;
    std::vector<polygonPoint> middle_line;
    middle_line.push_back(middle_point);
    middle_line.push_back(max_dis_pt);

    //这里需要对这个longest_line进行延伸一下防止与四边形没有交点
    auto temp_pt1 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            500,
            true,
            1);
    auto temp_pt2 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            500,
            false,
            1);
    longest_line.clear();
    longest_line.push_back(polygonPoint(temp_pt1[0].x,temp_pt1[0].y));
    longest_line.push_back(polygonPoint(temp_pt2[0].x,temp_pt2[0].y));

    //求外扩多边形与最长的线段的一个线段交点
    polygon  poly_f;
    std::reverse(stor_poly.begin(),stor_poly.end());
    for(auto it : stor_poly){
        poly_f.outer().push_back(pointbst(it.x,it.y));
    }
    //将最长边往最远点移动一定距离
    auto longest_pts = common::commonMath::computeLineTranslationPoints(
            longest_line,
            foot_line,
            RIDGE_WIDTH_LENGTH/2);

    linestring_type  line;
    line.push_back(pointbst(longest_pts[0].x,
                         longest_pts[0].y));
    line.push_back(pointbst(longest_pts[1].x,longest_pts[1].y));
    std::vector<pointbst> output;
    boost::geometry::intersection(line,poly_f,output);
    polygonPoint  vector_1,vector_2;
    polygonPoint  judgeDirection;
    judgeDirection.x = output[0].x();
    judgeDirection.y = output[0].y();
    vector_1 = common::commonMath::construceVector(max_dis_pt,middle_point);
    vector_2 = common::commonMath::construceVector(judgeDirection,middle_point);
    double judge_direction = common::commonMath::pointLocation(vector_1,vector_2);
    if(judge_direction == 1){
        move_pts_line_B_1_.push_back(polygonPoint(output[0].x(),output[0].y()));
        move_pts_line_B_2_.push_back(polygonPoint(output[1].x(),output[1].y()));
    }else{
        move_pts_line_B_1_.push_back(polygonPoint(output[1].x(),output[1].y()));
        move_pts_line_B_2_.push_back(polygonPoint(output[0].x(),output[0].y()));
    }

    std::vector<polygonPoint>   tempStorage;
    for(int i = 1; i <= integer_number; i++){
        std::vector<polygonPoint> points;
        if(i == 1){
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH);
        }else{
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH + RIDGE_WIDTH_LENGTH *(i-1));
        }

        //计算平移后的线段与四边形的交点
        polygon  poly;
        std::reverse(stor_poly.begin(),stor_poly.end());
        for(auto it : stor_poly){
            poly.outer().push_back(pointbst(it.x,it.y));
        }
        linestring_type  line;
        line.push_back(pointbst(points[0].x,
                             points[0].y));
        line.push_back(pointbst(points[1].x,points[1].y));
        std::vector<pointbst> output;
        boost::geometry::intersection(line,poly,output);
        LOG(INFO) << "output size is :" << output.size();
        if(output.size() != 2){
            LOG(ERROR) << "the line points is wrong !";
            break;
        }
        polygonPoint  trans_pt;
        trans_pt.x = output[0].x();
        trans_pt.y = output[0].y();
        std::vector<polygonPoint> origin_line;
        vector_2  = common::commonMath::construceVector(trans_pt,middle_point);
        double cross_2 =  common::commonMath::pointLocation(vector_1,vector_2);
        if(cross_2 == 1){
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto result_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_B_1_.push_back(polygonPoint(result_pts[0].x,result_pts[0].y));
            move_pts_line_B_2_.push_back(polygonPoint(result_pts[1].x,result_pts[1].y));
            tempStorage.push_back(origin_line[0]);
            tempStorage.push_back(origin_line[1]);
        }else{
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto res_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_B_1_.push_back(polygonPoint(res_pts[1].x,res_pts[1].y));
            move_pts_line_B_2_.push_back(polygonPoint(res_pts[0].x,res_pts[0].y));
            tempStorage.push_back(origin_line[0]);
            tempStorage.push_back(origin_line[1]);
        }
    }

    std::vector<polygonPoint>  temp;
    for(auto it : move_pts_line_B_1_){
        temp.push_back(it);
    }
    for(auto it : move_pts_line_B_2_){
        temp.push_back(it);
    }

    std::ofstream test_rect;
    test_rect.open("/home/zzm/Desktop/test_path_figure-main/src/test_move_pts_B.txt",std::ios::out);
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).x;
    }
    test_rect << std::endl;
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).y;
    }
    test_rect<< std::endl;
    test_rect.close();
};

void pathPolygonPlan::computeLeaveSituation(){
    cgal_FT  buffer_distance = RIDGE_WIDTH_LENGTH/2;
    std::vector<polygonPoint>  temp;
    for(int i = 1;i < bufferspiltPolysAandBs_.size();i++){
        int f = storage_order_polys_distance_[i];
        auto virtual_origin_poly =  bufferspiltPolysAandBs_[f][0];
        //此多边形开始外扩垄宽/2;
        cgal_Polygon_2 poly_temp;
        //std::reverse(origin_poly.begin(), origin_poly.end());
        for(int i = 0; i < virtual_origin_poly.size() - 1 ;i++){
            poly_temp.push_back(cgal_Point(
                    virtual_origin_poly[i].x,
                    virtual_origin_poly[i].y));
        }
        std::vector<cgal_Polygon_2> offset_polys;
        cgal_PolygonPtrVector outer_offset_polygons =
                CGAL::create_exterior_skeleton_and_offset_polygons_2(
                        buffer_distance,
                        poly_temp);
        std::vector<polygonPoint> stor_poly;
        print_polygons(outer_offset_polygons);
        for(auto m : *outer_offset_polygons[1]){
            polygonPoint temppt;
            temppt.x = m.x();
            temppt.y = m.y();
            stor_poly.push_back(temppt);
        }

        //将多边形构成闭环
        stor_poly.push_back(stor_poly[0]);
        double max_dis_ridge = -DBL_MAX;
        int count_ordered;
        for(int i = 0;i < stor_poly.size()-1; i++){
            double dis = common::commonMath::distance2(stor_poly[i],stor_poly[i+1]);
            if(dis > max_dis_ridge){
                max_dis_ridge = dis;
                count_ordered = i;
            }
        }
        std::vector<polygonPoint> longest_line;
        longest_line.push_back(stor_poly[count_ordered]);
        longest_line.push_back(stor_poly[count_ordered+1]);
        std::vector<polygonPoint> other_points;
        for(int i = 0;i < stor_poly.size();i++){
            if(stor_poly[i] != stor_poly[count_ordered] &&
               stor_poly[i] != stor_poly[count_ordered+1]){
                other_points.push_back(stor_poly[i]);
            }
        }
        double max_dis = -DBL_MAX;
        polygonPoint max_dis_pt;
        polygonPoint foot_print;
        for(auto it : other_points){
            auto  temp_foot_print = common::commonMath::computeFootPoint(it,
                                                                         longest_line[0],
                                                                         longest_line[1]);
            double dis_pts = common::commonMath::distance2(it,temp_foot_print);
            if(dis_pts > max_dis){
                max_dis = dis_pts;
                foot_print = temp_foot_print;
                max_dis_pt = it;
            }
        }

        //扩大的原来的图形绘制
        std::ofstream test_virtual_origin_poly;
        test_virtual_origin_poly.open(
                "/home/zzm/Desktop/test_path_figure-main/src/test_virtual_origin_poly_common.txt",
                std::ios::out);
        for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
            test_virtual_origin_poly << " " << (*it).x;
        }
        test_virtual_origin_poly << std::endl;
        for(auto it = stor_poly.begin(); it != stor_poly.end();it++){
            test_virtual_origin_poly << " " << (*it).y;
        }
        test_virtual_origin_poly << std::endl;
        test_virtual_origin_poly.close();

        //计算需要平移次数
        double integer_number =  ceil(max_dis/RIDGE_WIDTH_LENGTH);
        double mod = common::commonMath::doubleMod(max_dis,RIDGE_WIDTH_LENGTH);
        double last_path_cover = mod * RIDGE_WIDTH_LENGTH;

        LOG(INFO) << "max distance is : " << max_dis;
        LOG(INFO) << "need move size is :" << integer_number;
        LOG(INFO) << "the last line cover is :" << last_path_cover;

        //计算平移的点位信息
        std::vector<polygonPoint>  foot_line;
        foot_line.push_back(foot_print);
        foot_line.push_back(max_dis_pt);

        //选择最长边的中线和顶点作为分界线
        //首先计算最长边的中点
        polygonPoint   middle_point;
        middle_point.x = (longest_line[0].x + longest_line[1].x)/2;
        middle_point.y = (longest_line[0].y + longest_line[1].y)/2;
        std::vector<polygonPoint> middle_line;
        middle_line.push_back(middle_point);
        middle_line.push_back(max_dis_pt);

        //这里需要对这个longest_line进行延伸一下防止与四边形没有交点
        auto temp_pt1 = common::commonMath::findPointExtendSegment(
                longest_line[0],
                longest_line[1],
                500,
                true,
                1);
        auto temp_pt2 = common::commonMath::findPointExtendSegment(
                longest_line[0],
                longest_line[1],
                500,
                false,
                1);
        longest_line.clear();
        longest_line.push_back(polygonPoint(temp_pt1[0].x,temp_pt1[0].y));
        longest_line.push_back(polygonPoint(temp_pt2[0].x,temp_pt2[0].y));

        //求外扩多边形与最长的线段的一个线段交点
        polygon  poly_f;
        std::reverse(stor_poly.begin(),stor_poly.end());
        for(auto it : stor_poly){
            poly_f.outer().push_back(pointbst(it.x,it.y));
        }
        //将最长边往最远点移动一定距离
        auto longest_pts = common::commonMath::computeLineTranslationPoints(
                longest_line,
                foot_line,
                RIDGE_WIDTH_LENGTH/2);

        linestring_type  line;
        line.push_back(pointbst(longest_pts[0].x,
                             longest_pts[0].y));
        line.push_back(pointbst(longest_pts[1].x,longest_pts[1].y));
        std::vector<pointbst> output;
        boost::geometry::intersection(line,poly_f,output);
        polygonPoint  vector_1,vector_2;
        polygonPoint  judgeDirection;
        judgeDirection.x = output[0].x();
        judgeDirection.y = output[0].y();
        vector_1 = common::commonMath::construceVector(max_dis_pt,middle_point);
        vector_2 = common::commonMath::construceVector(judgeDirection,middle_point);
        double judge_direction = common::commonMath::pointLocation(vector_1,vector_2);
        if(judge_direction == 1){
            move_pts_line_B_1_.push_back(polygonPoint(output[0].x(),output[0].y()));
            move_pts_line_B_2_.push_back(polygonPoint(output[1].x(),output[1].y()));
        }else{
            move_pts_line_B_1_.push_back(polygonPoint(output[1].x(),output[1].y()));
            move_pts_line_B_2_.push_back(polygonPoint(output[0].x(),output[0].y()));
        }

        std::vector<polygonPoint>   tempStorage;
        for(int i = 1; i <= integer_number; i++){
            std::vector<polygonPoint> points;
            if(i == 1){
                points = common::commonMath::computeLineTranslationPoints(
                        longest_line,
                        foot_line,
                        RIDGE_WIDTH_LENGTH);
            }else{
                points = common::commonMath::computeLineTranslationPoints(
                        longest_line,
                        foot_line,
                        RIDGE_WIDTH_LENGTH + RIDGE_WIDTH_LENGTH *(i-1));
            }

            //计算平移后的线段与四边形的交点
            polygon  poly;
            std::reverse(stor_poly.begin(),stor_poly.end());
            for(auto it : stor_poly){
                poly.outer().push_back(pointbst(it.x,it.y));
            }
            linestring_type  line;
            line.push_back(pointbst(points[0].x,
                                 points[0].y));
            line.push_back(pointbst(points[1].x,points[1].y));
            std::vector<pointbst> output;
            boost::geometry::intersection(line,poly,output);
            LOG(INFO) << "output size is :" << output.size();
            if(output.size() != 2){
                LOG(ERROR) << "the line points is wrong !";
                break;
            }
            polygonPoint  trans_pt;
            trans_pt.x = output[0].x();
            trans_pt.y = output[0].y();
            std::vector<polygonPoint> origin_line;
            vector_2  = common::commonMath::construceVector(trans_pt,middle_point);
            double cross_2 =  common::commonMath::pointLocation(vector_1,vector_2);
            if(cross_2 == 1){
                origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
                origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
                auto result_pts = common::commonMath::computeLineTranslationPoints(
                        origin_line,
                        foot_line,
                        RIDGE_WIDTH_LENGTH/2);
                move_pts_line_B_1_.push_back(polygonPoint(result_pts[0].x,result_pts[0].y));
                move_pts_line_B_2_.push_back(polygonPoint(result_pts[1].x,result_pts[1].y));
                tempStorage.push_back(origin_line[0]);
                tempStorage.push_back(origin_line[1]);
            }else{
                origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
                origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
                auto res_pts = common::commonMath::computeLineTranslationPoints(
                        origin_line,
                        foot_line,
                        RIDGE_WIDTH_LENGTH/2);
                move_pts_line_B_1_.push_back(polygonPoint(res_pts[1].x,res_pts[1].y));
                move_pts_line_B_2_.push_back(polygonPoint(res_pts[0].x,res_pts[0].y));
                tempStorage.push_back(origin_line[0]);
                tempStorage.push_back(origin_line[1]);
            }
        }
        move_pts_line_B_all_1_.push_back(move_pts_line_B_1_);
        move_pts_line_B_all_2_.push_back(move_pts_line_B_2_);
        for(auto it : move_pts_line_B_1_){
            temp.push_back(it);
        }
        for(auto it : move_pts_line_B_2_){
            temp.push_back(it);
        }
        move_pts_line_B_1_.clear();
        move_pts_line_B_2_.clear();
    }
    std::ofstream test_rect;
    test_rect.open("/home/zzm/Desktop/test_path_figure-main/src/test_move_pts_B.txt",std::ios::out);
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).x;
    }
    test_rect << std::endl;
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).y;
    }
    test_rect<< std::endl;
    test_rect.close();
};

void pathPolygonPlan::judgePolysSample(int* record_spilt_index,bool&  have_spilt_poly){
    int pt_number = cgalPolypts_.size();
    for(int i = 0;i < pt_number ;i++){
        polygon temp;
        for(auto it = cgalPolypts_[i].rbegin()  ;it != cgalPolypts_[i].rend();it++){
            temp.outer().push_back(pointbst(it->x,it->y));
        }
        bool issample =  boost::geometry::is_valid(temp);
        temp.outer().clear();
        if(!issample){
            double buffer_distance = -DBL_MAX;
            *record_spilt_index = i ;
            LOG(INFO) << "record spilt index is : " << *record_spilt_index;
            have_spilt_poly = true;
            break;
        }
    }
}

void pathPolygonPlan::spiltPolyTo2(
        bool have_spilt_poly,
        std::vector<polygonPoint>  &spilt_origin_poly,
        polygon& spilt_ori,
        double&  buffer_distance,
        std::vector<std::vector<polygonPoint>>&  storage_spilt_first_polys,
        std::vector<std::vector<polygonPoint>>& storage_spilt_second_polys,
        point&                            spilt_first_poly_center){
    if(have_spilt_poly){
        for(auto f = spilt_origin_poly.rbegin();f != spilt_origin_poly.rend();f++){
            spilt_ori.outer().push_back(pointbst(f->x,f->y));
        }
        for(int i = 1; i <= MAX_TRAVERSALS_NUMBERS;i++){
            buffer_distance = i * -RIDGE_WIDTH_LENGTH ;
            boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
                    distance_strategy(buffer_distance);
            boost::geometry::strategy::buffer::join_round join_strategy(0.01);
            boost::geometry::strategy::buffer::end_round end_strategy;
            boost::geometry::strategy::buffer::point_circle circle_strategy;
            boost::geometry::strategy::buffer::side_straight side_strategy;
            boost::geometry::model::multi_polygon<polygon> result;
            boost::geometry::model::multi_polygon<polygon> mpol;
            mpol.push_back(spilt_ori);
            boost::geometry::buffer(mpol, result,
                                    distance_strategy, side_strategy,
                                    join_strategy, end_strategy, circle_strategy);
            std::vector<polygonPoint>  tempPolygon;
            for(auto it = result.begin();it != result.end(); it++){
                for(auto j = it->outer().rbegin();j != it->outer().rend();j++){
                    polygonPoint tempPoint;
                    tempPoint.x = (*j).x();
                    tempPoint.y = (*j).y();
                    tempPolygon.push_back(tempPoint);
                }
            }
            if(tempPolygon.size() == 0){
                LOG(INFO) << "already narrow to last !";
                break;
            }
            //对tempPolygon构成的内缩多边形进行拆分
            auto first_poly_pt = tempPolygon[0];
            int  first_poly_end;
            for(int i = 1;i < tempPolygon.size();i++){
                if(first_poly_pt == tempPolygon[i]){
                    first_poly_end = i;
                    break;
                }
            }
            std::vector<polygonPoint>   spilt_first_poly_pts;
            std::vector<polygonPoint>   spilt_second_poly_pts;
            for(int i = 0;i <= first_poly_end; i++){
                spilt_first_poly_pts.push_back(tempPolygon[i]);
            }
            for(int i = first_poly_end + 1;i < tempPolygon.size();i++){
                spilt_second_poly_pts.push_back(tempPolygon[i]);
            }
            //增加判断是否属于同类多边形
            if(i == 1){
                storage_spilt_first_polys.push_back(spilt_first_poly_pts);
                if(!spilt_second_poly_pts.empty()){
                    storage_spilt_second_polys.push_back(spilt_second_poly_pts);
                }
            }else{
                if(!storage_spilt_first_polys.empty() ||
                   !storage_spilt_second_polys.empty()){
                    if(!spilt_first_poly_pts.empty() &&
                       !spilt_second_poly_pts.empty()){
                        polygon pt_poly;
                        for(auto it : storage_spilt_first_polys[0]){
                            pt_poly.outer().push_back(pointbst(it.x,it.y));
                        }

                        bool flag =  boost::geometry::within(pointbst(spilt_first_poly_pts[0].x,
                                                                   spilt_first_poly_pts[0].y), pt_poly);
                        if(flag){
                            storage_spilt_first_polys.push_back(spilt_first_poly_pts);
                            storage_spilt_second_polys.push_back(spilt_second_poly_pts);
                        }else{
                            storage_spilt_first_polys.push_back(spilt_second_poly_pts);
                            storage_spilt_second_polys.push_back(spilt_first_poly_pts);
                        }
                    }else if(!spilt_first_poly_pts.empty() &&
                             spilt_second_poly_pts.empty()){
                        polygon pt_poly;
                        for(auto it : storage_spilt_first_polys[0]){
                            pt_poly.outer().push_back(pointbst(it.x,it.y));
                        }

                        bool flag =  boost::geometry::within(pointbst(spilt_first_poly_pts[0].x,
                                                                   spilt_first_poly_pts[0].y), pt_poly);
                        if(flag){
                            storage_spilt_first_polys.push_back(spilt_first_poly_pts);
                        }else{
                            storage_spilt_second_polys.push_back(spilt_first_poly_pts);
                        }
                    }else if(!spilt_second_poly_pts.empty() &&
                             spilt_first_poly_pts.empty()){
                        polygon pt_poly;
                        for(auto it : storage_spilt_first_polys[0]){
                            pt_poly.outer().push_back(pointbst(it.x,it.y));
                        }

                        bool flag =  boost::geometry::within(pointbst(spilt_first_poly_pts[0].x,
                                                                   spilt_first_poly_pts[0].y), pt_poly);
                        if(flag){
                            storage_spilt_first_polys.push_back(spilt_second_poly_pts);
                        }else{
                            storage_spilt_second_polys.push_back(spilt_second_poly_pts);
                        }
                    }
                }
            }
        }
    }
}

void pathPolygonPlan::computeEntranceLines(std::vector<Point> &points){
    //将最起点的点位延长一段距离,用来找该点对应的多边形的前后点
    auto vector_1 = common::commonMath::findPointExtendSegment(
            lastPoly_innerpts_[1],
            lastPoly_innerpts_[0],
            5,
            true,
            1);
    //找到延伸的线段与地块的交点
    polygon  poly;
    for(auto it : points){
        poly.outer().push_back(pointbst(it.x,it.y));
    }
    linestring_type  line;
    line.push_back(pointbst(
            vector_1[0].x,
            vector_1[0].y));
    line.push_back(pointbst(
            lastPoly_innerpts_[1].x,
            lastPoly_innerpts_[1].y));
    std::vector<pointbst> output;
    boost::geometry::intersection(line,poly,output);
    polygonPoint orderd_pt(output[0].x(),output[0].y());
    //找到对应该点的前后点位
    std::vector<Point> forwardAndBack =
            common::commonMath::computeForwardAndBackPoints(
                    points,
                    orderd_pt);
    std::vector<polygonPoint>  direction_line;
    direction_line.push_back(polygonPoint(output[0].x(),output[0].y()));
    direction_line.push_back(polygonPoint(forwardAndBack[0].x,forwardAndBack[0].y));
    //回字形的顶点线段们往direction_line平移 RIDGE_WIDTH/2;
    //将入口线段第一个点更新为虚构的延长点并存储为线段集
    std::vector<polygonPoint> vecPoints;
    vecPoints.push_back(polygonPoint(vector_1[0].x,vector_1[0].y));
    for(int i = 1;i < lastPoly_innerpts_.size();i++){
        vecPoints.push_back(lastPoly_innerpts_[i]);
    }
    int num = vecPoints.size();
    std::vector<polygonPoint> vecMovePoints;
    for(int i = 0;i < num - 1;i++){
        //将生成的线段按照指定的方向平移
        std::vector<polygonPoint> lineOrigin;
        lineOrigin.push_back(vecPoints[i]);
        lineOrigin.push_back(vecPoints[i + 1]);
        auto movedpts = common::commonMath::computeLineTranslationPoints(
                lineOrigin,
                direction_line,
                RIDGE_WIDTH_LENGTH);
        for(auto it : movedpts){
            entrance_lines_.push_back(it);
        }
    }
    LOG(INFO) << "entrance_lines size is :" << entrance_lines_.size();
}

void pathPolygonPlan::computeLastRidgeInnerPoints(std::vector<polygonPoint> & points){
      auto polygon_last = cgalPolypts_[cgal_narrow_size_-1];
      cgal_Polygon_2 poly;
     for(int i = 0;i < polygon_last.size(); i++){
        poly.push_back(cgal_Point(polygon_last[i].x,polygon_last[i].y));
      }
     for(auto it : points){
         auto result = CGAL::bounded_side_2(poly.vertices_begin(),
                               poly.vertices_end(),
                               cgal_Point(it.x,it.y));
          if (result == -1){
              polygonPoint temp;
              temp.x = it.x;
              temp.y = it.y;
              lastPoly_innerpts_.push_back(temp);
          }
     }
}



//void pathPolygonPlan::dealWithLastSeveralPolygons(){
//      auto num = storageNarrowPolygonPoints_.size();
//    LOG(INFO) << "--------------------------------------------------------------";
//      if(storageNarrowPolygonPoints_[num-1].size() == 6){
//         last_polys_type_ = lastPolyIdentify::POLY_FIVE;
//         LOG(INFO) << "the last poly type is : poly_five !";
//         return;
//      }
//      if(storageNarrowPolygonPoints_[num-1].size() == 5){
//          last_polys_type_ = lastPolyIdentify::POLY_FOUR;
//          LOG(INFO) << "the last poly type is : poly_four !";
//          return;
//      }
//      for(int i = 1;i <= num; i++)
//      {
//          auto poly_pts_size = storageNarrowPolygonPoints_[i-1].size();
//          if(poly_pts_size == 5 ){
//              if(poly_pts_size == 5 &&
//                 storageNarrowPolygonPoints_[i].size() == 4){
//                  //内缩到出现三角形,目前只考虑内缩四边形内的第一个三角形
//                  boost::geometry::strategy::buffer::distance_symmetric<coordinate_type>
//                          distance_strategy(-RIDGE_WIDTH_LENGTH/2);
//                  boost::geometry::strategy::buffer::join_round join_strategy(0.01);
//
//                  boost::geometry::strategy::buffer::end_round end_strategy;
//                  boost::geometry::strategy::buffer::point_circle circle_strategy;
//                  boost::geometry::strategy::buffer::side_straight side_strategy;
//
//                  // Declare output
//                  boost::geometry::model::multi_polygon<polygon> result;
//                  boost::geometry::model::multi_polygon<polygon> mpol;
//                  polygon instance_polygon;
//
//                  for (auto it: storageNarrowPolygonPoints_[i-1]) {
//                      point instance_point;
//                      instance_point.x(it.x);
//                      instance_point.y(it.y);
//                      instance_polygon.outer().push_back(instance_point);
//                  }
//                  mpol.push_back(instance_polygon);
//                  boost::geometry::buffer(mpol, result,
//                                          distance_strategy, side_strategy,
//                                          join_strategy, end_strategy, circle_strategy);
//                  std::vector<polygonPoint> poly_res;
//                  for(auto j = result.begin()->outer().begin();
//                      j != result.begin()->outer().end();
//                      j ++){
//                      polygonPoint  temp_point;
//                      temp_point.x = j->x();
//                      temp_point.y = j->y();
//                      poly_res.push_back(temp_point);
//                  }
//                  if(poly_res.size() == 4){
//                      LOG(INFO) << "the last poly type is : poly_four_and_three !";
//                      last_polys_type_ = lastPolyIdentify::POLY_FOUR_AND_THREE;
//                      last_several_polygons_.push_back(storageNarrowPolygonPoints_[i-1]);
//                      record_poly_index_ = i;
//                  } else {
//                      LOG(INFO) << "the shape of quadrangle after being retracted is quadrangle !";
//                      LOG(INFO) << "the quadrangle size is : " << poly_res.size();
//                      last_polys_type_ = lastPolyIdentify::POLY_FOUR_AND_FOUR;
//                      last_several_polygons_.push_back(storageNarrowPolygonPoints_[i-1]);
//                      record_poly_index_ = i;
//                  }
//                  LOG(INFO) << "the record poly index is : " << record_poly_index_;
//                  last_several_polygons_.push_back(poly_res);
//              }
//          }
//      }
//      LOG(INFO) <<  "--------------------------------------------------------------";
//      std::ofstream  tempfile;
//      tempfile.open("/home/zzm/Desktop/test_path_figure-main/src/templastridge.txt",std::ios::out);
//      for(auto it :last_several_polygons_){
//          for(auto j : it){
//              tempfile << " " << j.x;
//          }
//      }
//      tempfile << std::endl;
//      for(auto it : last_several_polygons_){
//          for(auto j: it){
//              tempfile << " " << j.y;
//          }
//      }
//      tempfile << std::endl;
//      tempfile.close();
//}

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
//    std::vector<polygonPoint> stor_pts = deleteRepeatPolyPts(last_several_polygons_[1]);
//    Polygon_2 poly_4,poly_rect;
//    for(auto it = stor_pts.rbegin();it != stor_pts.rend();it ++){
//        poly_4.push_back(Point_2(it->x,it->y));
//    }

//    CGAL::min_rectangle_2(
//            poly_4.vertices_begin(), poly_4.vertices_end(), std::back_inserter(poly_rect));
    std::vector<polygonPoint> stor_pts = last_several_polygons_[1];
    //找到四边形的最长边
    double max_dis_ridge = -DBL_MAX;
    int  count_ordered;
    for(int i = 0;i < 4;i++){
        double dis = common::commonMath::distance2(stor_pts[i],stor_pts[i+1]);
        if(dis > max_dis_ridge){
            max_dis_ridge = dis;
            count_ordered = i;
        }
    }
    std::vector<polygonPoint> longest_line;
    longest_line.push_back(stor_pts[count_ordered]);
    longest_line.push_back(stor_pts[count_ordered+1]);
    std::vector<polygonPoint> other_points;
    for(int i = 0;i < 4;i++){
        if(stor_pts[i] != stor_pts[count_ordered] &&
           stor_pts[i] != stor_pts[count_ordered+1]){
            other_points.push_back(stor_pts[i]);
        }
    }
    double max_dis = -DBL_MAX;
    polygonPoint max_dis_pt;
    polygonPoint foot_print;
    for(auto it : other_points){
         auto  temp_foot_print = common::commonMath::computeFootPoint(it,
                                                               longest_line[0],
                                                               longest_line[1]);
         double dis_pts = common::commonMath::distance2(it,temp_foot_print);
         if(dis_pts > max_dis){
             max_dis = dis_pts;
             foot_print = temp_foot_print;
             max_dis_pt = it;
         }
    }

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

    //计算需要平移次数
    double integer_number =  ceil(max_dis/RIDGE_WIDTH_LENGTH);
    double mod = common::commonMath::doubleMod(max_dis,RIDGE_WIDTH_LENGTH);
    double last_path_cover = mod * RIDGE_WIDTH_LENGTH;

    LOG(INFO) << "max distance is : " << max_dis;
    LOG(INFO) << "need move size is :" << integer_number;
    LOG(INFO) << "the last line cover is :" << last_path_cover;

    //计算平移的点位信息
    std::vector<polygonPoint>  foot_line;
    foot_line.push_back(foot_print);
    foot_line.push_back(max_dis_pt);

    polygonPoint  vector_1,vector_2;
    vector_1 = common::commonMath::construceVector(max_dis_pt,foot_print);
    vector_2 = common::commonMath::construceVector(longest_line[0],foot_print);
    double judge_direction = common::commonMath::pointLocation(vector_1,vector_2);
    if(judge_direction == 1){
        move_pts_line_1_.push_back(longest_line[0]);
        move_pts_line_2_.push_back(longest_line[1]);
    }else{
        move_pts_line_1_.push_back(longest_line[1]);
        move_pts_line_2_.push_back(longest_line[0]);
    }

    //这里需要对这个longest_line进行延伸一下防止与四边形没有交点
    auto temp_pt1 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            10,
            true,
            1);
    auto temp_pt2 = common::commonMath::findPointExtendSegment(
            longest_line[0],
            longest_line[1],
            10,
            false,
            1);
    longest_line.clear();
    longest_line.push_back(polygonPoint(temp_pt1[0].x,temp_pt1[0].y));
    longest_line.push_back(polygonPoint(temp_pt2[0].x,temp_pt2[0].y));
    for(int i = 1;i <= integer_number;i++){
        std::vector<polygonPoint> points;
        if(i == 1){
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                     foot_line,
                     RIDGE_WIDTH_LENGTH/2);
        }else{
            points = common::commonMath::computeLineTranslationPoints(
                    longest_line,
                     foot_line,
                     RIDGE_WIDTH_LENGTH/2 + RIDGE_WIDTH_LENGTH *(i-1));
        }

        //计算平移后的线段与四边形的交点
        polygon  poly;
        std::reverse(stor_pts.begin(),stor_pts.end());
        for(auto it : stor_pts){
            poly.outer().push_back(pointbst(it.x,it.y));
        }
        linestring_type  line;
        line.push_back(pointbst(points[0].x,
                             points[0].y));
        line.push_back(pointbst(points[1].x,points[1].y));
        std::vector<pointbst> output;
        boost::geometry::intersection(line,poly,output);
        LOG(INFO) << "output size is :" << output.size();
        if(output.size() != 2){
            LOG(ERROR) << "the line points is wrong !";
            break;
        }
        polygonPoint  trans_pt;
        trans_pt.x = output[0].x();
        trans_pt.y = output[0].y();
        std::vector<polygonPoint> origin_line;
        vector_2  = common::commonMath::construceVector(trans_pt,foot_print);
        double cross_2 =  common::commonMath::pointLocation(vector_1,vector_2);
        if(cross_2 == 1){
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto result_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_1_.push_back(polygonPoint(result_pts[0].x,result_pts[0].y));
            move_pts_line_2_.push_back(polygonPoint(result_pts[1].x,result_pts[1].y));
        }else{
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto res_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_line_1_.push_back(polygonPoint(res_pts[1].x,res_pts[1].y));
            move_pts_line_2_.push_back(polygonPoint(res_pts[0].x,res_pts[0].y));
        }
    }
    std::vector<polygonPoint>  temp;
    for(auto it : move_pts_line_1_){
        temp.push_back(it);
    }
    for(auto it : move_pts_line_2_){
        temp.push_back(it);
    }
    std::ofstream test_rect;
    test_rect.open("/home/zzm/Desktop/test_path_figure-main/src/test_move_pts.txt",std::ios::out);
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).x;
    }
    test_rect << std::endl;
    for(auto it = temp.begin(); it != temp.end();it++){
        test_rect << " " << (*it).y;
    }
    test_rect<< std::endl;
    test_rect.close();
}


void pathPolygonPlan::judgeIncreaseskeleton(
               std::vector<polygonPoint> &  inner_polypts ){
    int num_size = cgalPolypts_.size();
    auto poly_last_ridge =  cgalPolypts_[num_size - 1];
    double min_angle = DBL_MAX;
    polygonPoint min_point;
    for(auto it = poly_last_ridge.begin();
             it != poly_last_ridge.end();
             it++){
        auto current = *it;
        auto previous = *(it == poly_last_ridge.begin() ? poly_last_ridge.end()-1: it-1);
        auto next = *(it == poly_last_ridge.end()-1 ? poly_last_ridge.begin()  : it+1);
        // 计算当前顶点的角度
        double angle = std::atan2(next.y - current.y, next.x - current.x) -
                       std::atan2(previous.y - current.y, previous.x - current.x);
        angle = angle * (180 / M_PI);
        if(angle > 180){
            angle = 360 - angle ;
        }
        if(fabs(angle) < min_angle){
            min_angle = fabs(angle);
            min_point.x = current.x;
            min_point.y = current.y;
        }
        if(fabs(angle) < 30) {
            flag_increase_last_skeleton_ = true;
        }
        std::cout << "Angle at vertex ("
                  << current.x << ", " << current.y
                  << "): " << angle << std::endl;
    }
    //判断其顶点个数，如果是三角形则同样增加
    if(poly_last_ridge.size() == 3){
        LOG(INFO) << "increase last skeleton because of trangle !"
        << " the poly size is : " << poly_last_ridge.size();
        flag_increase_last_skeleton_ = true;
    }
    if(flag_increase_last_skeleton_){
        LOG(INFO) << "increase last skeleton !";
    }
    //检测内部直骨架点位的方向
    std::unordered_set<polygonPoint,polyPointHash>   storage_inner_pts;
    cgal_Polygon_2 tmp;
    for(auto m : poly_last_ridge){
        tmp.push_back(cgal_Point(m.x,m.y));
    }
    for(auto it : inner_polypts){
        cgal_Point  tempPt(it.x,it.y);
        cgal_Bounded_side bounded_side = CGAL::bounded_side_2(
                                                 tmp.begin(),
                                                 tmp.end(),
                                                 tempPt);
        if (bounded_side == CGAL::ON_BOUNDED_SIDE){
            if(storage_inner_pts.find(it) == storage_inner_pts.end()){
                storage_inner_pts.insert(it);
            }
        }else {
            //DONOTHING;
        }
    }
    //判断内点的个数
    if(storage_inner_pts.size() == 1){
        //执行生成最后path的操作
        //找到最后的路径的方向
        for(int i = 0;i < inner_polypts.size();i++){
                auto on_it = common::commonMath::isPointOnSegment(
                        inner_polypts[i],
                        inner_polypts[i+1],
                        *storage_inner_pts.begin());
                auto on_im = common::commonMath::isPointOnSegment(
                        inner_polypts[i],
                        inner_polypts[i+1],
                        min_point);
                if(on_it && on_im){
                    //生成path关键点位
                    //找到对应的同方向点位
                    polygonPoint vector_1;
                    vector_1.x = min_point.x - storage_inner_pts.begin()->x;
                    vector_1.y = min_point.y - storage_inner_pts.begin()->y;
                    polygonPoint vector_2;
                    vector_2.x = inner_polypts[i+1].x - inner_polypts[i].x;
                    vector_2.y = inner_polypts[i+1].y - inner_polypts[i].y;
                    double angle_1 = common::commonMath::computeTwoLineAngle(vector_1,vector_2);
                    LOG(INFO) << "the two vector angle is :" << fabs(angle_1) * (180 / M_PI);
                    if(fabs(angle_1) *(180 / M_PI) < 5 ){
                        //生成path记录
                        storage_keypts_inner_skeleton_.push_back(*storage_inner_pts.begin());
                        storage_keypts_inner_skeleton_.push_back(inner_polypts[i+1]);
                        polygonPoint  judge_pt = inner_polypts[i+1];
                        polygonPoint  order_pt = *storage_inner_pts.begin();
                        while(last_inner_skeleton_keypts_[judge_pt].size() != 3 ||
                              last_inner_skeleton_keypts_[judge_pt].size() != 1){
                            auto temp_pts = last_inner_skeleton_keypts_[judge_pt];
                            auto temp_1 = judge_pt;
                            for(auto it : temp_pts){
                                if( it != order_pt){
                                    storage_keypts_inner_skeleton_.push_back(it);
                                    judge_pt = it;
                                }else{
                                    order_pt = temp_1;
                                }
                            }
                        }
                    }else{
                        storage_keypts_inner_skeleton_.push_back(*storage_inner_pts.begin());
                        storage_keypts_inner_skeleton_.push_back(inner_polypts[i]);
                        polygonPoint  judge_pt = inner_polypts[i];
                        polygonPoint  order_pt = *storage_inner_pts.begin();
                        while(last_inner_skeleton_keypts_[judge_pt].size() == 2){
                            LOG(INFO) << "the point is :" << judge_pt.x <<" "
                                      << judge_pt.y << " "
                                      << last_inner_skeleton_keypts_[judge_pt].size();
                            auto temp_pts = last_inner_skeleton_keypts_[judge_pt];
                            auto temp_1 = judge_pt;
                            for(auto it : temp_pts){
                                if( it != order_pt){
                                    storage_keypts_inner_skeleton_.push_back(it);
                                    judge_pt = it;
                                }else{
                                    order_pt = temp_1;
                                }
                            }

                        }
                    }
                    break;
                }
        }
    }
    if(!SET_FLAG_INNER_SKELETON_PATH){
        storage_keypts_inner_skeleton_.clear();
        return;
    }
    if(!storage_keypts_inner_skeleton_.empty()){
        LOG(INFO) << "this map  increases innner skeleton path !";
    }
    std::ofstream  inner_skeleton_path;
    inner_skeleton_path.open(
            "/home/zzm/Desktop/test_path_figure-main/src/inner_skeleton_path.txt",
            std::ios::out);
    for(auto it : storage_keypts_inner_skeleton_){
        inner_skeleton_path << " " << it.x;
    }
    inner_skeleton_path << std::endl;
    for(auto it : storage_keypts_inner_skeleton_){
        inner_skeleton_path <<" " << it.y;
    }
    inner_skeleton_path << std::endl;
    inner_skeleton_path.close();
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

    //计算max_point到lineAB的投影点
    polygonPoint footprint;
    footprint = common::commonMath::computeFootPoint(
            max_point,
            line_longAB[0],
            line_longAB[1]);
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
    std::vector<polygonPoint> foot_line;
    foot_line.push_back(footprint);
    foot_line.push_back(max_point);
    polygonPoint vector_1,vector_2;
    vector_1 = common::commonMath::construceVector(max_point,footprint);
    vector_2 = common::commonMath::construceVector(line_longAB[0],footprint);
    double judge_direction = common::commonMath::pointLocation(vector_1,vector_2);
    if(judge_direction == 1){
        move_pts_tangle_line_1_.push_back(line_longAB[0]);
        move_pts_tangle_line_2_.push_back(line_longAB[1]);
    }else{
        move_pts_tangle_line_1_.push_back(line_longAB[1]);
        move_pts_tangle_line_2_.push_back(line_longAB[0]);
    }

    for(int i = 1 ;i <= integer_number;i++){
        std::vector<polygonPoint> points;
        if(i==1){
             points = common::commonMath::computeLineTranslationPoints(
                    line_longAB,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
        }else{
            points = common::commonMath::computeLineTranslationPoints(
                    line_longAB,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2 + RIDGE_WIDTH_LENGTH * (i-1));
        }

        //计算平移后的线段与四边形的交点
        polygon poly;
        std::reverse(stor_pts.begin(),stor_pts.end());
        for(auto it : stor_pts){
            poly.outer().push_back(pointbst(it.x,it.y));
        }
        linestring_type line;
        line.push_back(pointbst(points[0].x,
                             points[0].y));
        line.push_back(pointbst(points[1].x,points[1].y));
        std::vector<pointbst> output;
        boost::geometry::intersection(line,poly,output);
        LOG(INFO) << "output size is :" << output.size();
        if(output.size() != 2){
            LOG(ERROR) << "the line points is wrong !";
            break;
        }
        polygonPoint  trans_pt;
        trans_pt.x = output[0].x();
        trans_pt.y = output[0].y();
        std::vector<polygonPoint> origin_line;
        vector_2  = common::commonMath::construceVector(trans_pt,footprint);
        double cross_2 =  common::commonMath::pointLocation(vector_1,vector_2);
        if(cross_2 == 1){
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto result_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_tangle_line_1_.push_back(polygonPoint(result_pts[0].x,result_pts[0].y));
            move_pts_tangle_line_2_.push_back(polygonPoint(result_pts[1].x,result_pts[1].y));
        }else{
            origin_line.push_back(polygonPoint(output[0].x(),output[0].y()));
            origin_line.push_back(polygonPoint(output[1].x(),output[1].y()));
            auto res_pts = common::commonMath::computeLineTranslationPoints(
                    origin_line,
                    foot_line,
                    RIDGE_WIDTH_LENGTH/2);
            move_pts_tangle_line_1_.push_back(polygonPoint(res_pts[1].x,res_pts[1].y));
            move_pts_tangle_line_2_.push_back(polygonPoint(res_pts[0].x,res_pts[0].y));
        }
    }

    std::vector<polygonPoint>  temp;
    for(auto it : move_pts_tangle_line_1_){
        temp.push_back(it);
    }
    for(auto it : move_pts_tangle_line_2_){
        temp.push_back(it);
    }
    std::ofstream test_pts;
    test_pts.open("/home/zzm/Desktop/test_path_figure-main/src/test_pts.txt",std::ios::out);
    for(auto it = temp.begin(); it != temp.end();it++){
        test_pts << " " << it->x;
    }
    test_pts << std::endl;
    for(auto it = temp.begin(); it != temp.end();it++){
        test_pts << " " << it->y;
    }
    test_pts<< std::endl;
    test_pts.close();


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
    for(int i = 0;i < num - 1;i++){
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
    for(int it = 0 ;it < backShape_keypoints_.size() - 2;it++){
         temp_storage_pts = deleteRepeatPolyPts(backShape_keypoints_[it]);
         filtered_backshape_keypoints_.push_back(temp_storage_pts);
    }
//    filtered_backshape_keypoints_.push_back(backShape_keypoints_[num -2]);
   //对最后一垄单独过滤
   temp_storage_pts.clear();
    for(int i =  0;i < backShape_keypoints_[num -2].size()-1;i++){
        if(backShape_keypoints_[num - 2][i] != backShape_keypoints_[num - 2][i+1]) {
            temp_storage_pts.push_back(backShape_keypoints_[num-2][i]);
        }
    }
    temp_storage_pts.push_back(backShape_keypoints_[num - 2][backShape_keypoints_[num -2].size()-1]);
    filtered_backshape_keypoints_.push_back(temp_storage_pts);
    temp_storage_pts.clear();
   for(int i = 0;i < backShape_keypoints_[num-1].size()-1;i++){
       if(backShape_keypoints_[num - 1][i] != backShape_keypoints_[num - 1][i+1]){
           temp_storage_pts.push_back(backShape_keypoints_[num -1][i]);
       }
   }
    temp_storage_pts.push_back(backShape_keypoints_[num - 1][backShape_keypoints_[num -1].size()-1]);
    filtered_backshape_keypoints_.push_back(temp_storage_pts);
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
            if((last_polys_type_ == aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_FOUR ||
               last_polys_type_ == aiforce::Route_Planning::lastPolyIdentify ::POLY_FOUR_AND_THREE)&&
               i == filtered_backshape_keypoints_.size() -2 &&
               j == filtered_backshape_keypoints_[i].size() - 1){
               auto temp_point = filtered_backshape_keypoints_[i][filtered_backshape_keypoints_[i].size()-1];
               auto temp_point1 = filtered_backshape_keypoints_[i][filtered_backshape_keypoints_[i].size()-2];
               tempPtInfo.start_curve_point =  common::commonMath::findPointOnSegment(temp_point,
                                                                                           temp_point1,
                                                                                           SET_STARTTURN_DISTANCE,
                                                                                           true);
            } else {
                tempPtInfo.start_curve_point =
                        common::commonMath::findPointOnSegment(forward_last_points[0],
                                                               filtered_backshape_keypoints_[i][j],
                                                               SET_STARTTURN_DISTANCE,
                                                               false);
            }
            //计算下一垄的第二个点
            polygonPoint last_point;
            if(i+1 < filtered_backshape_keypoints_.size()-1){
                last_point =  filtered_backshape_keypoints_[i+1][1];
            }
            //处理最后一笼是POLY_FOUR的情况下last_point
            if(i == filtered_backshape_keypoints_.size() -1 -1 &&
               last_polys_type_ == aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR){
                last_point = common::commonMath::findPointOnSegment(
                                        filtered_backshape_keypoints_[i][tempPtInfo.numbers-1],
                                        filtered_backshape_keypoints_[i+1][1],
                                        5,
                                        true);
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

std::vector<pathInterface::pathPoint>  pathPolygonPlan::cgalComputeRidgeRoutingpts(int ridge_index){
    //暂时约定每一垄第一个点是不需要进行处理弯道的
    pathInterface::pathPoint  point_1;
    ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    std::vector<pathInterface::pathPoint>  storageAllPath;
    double startPoint[3],endPoint[3];
    int ridges_counts  = cgalbackShape_keypoints_.size();
    //处理最后一个path(skeleton)
    if(ridge_index == ridges_counts ){
        //处理连接第一个点的弯道
        int  n_size =   cgalbackShape_keypoints_[ridge_index-1].size();
        startPoint[0] = cgalbackShape_keypoints_[ridge_index-1][n_size-1].x;
        startPoint[1] = cgalbackShape_keypoints_[ridge_index-1][n_size-1].y;
        startPoint[2] = cgalbackShape_keypoints_[ridge_index-1][n_size-1].heading;
        endPoint[0] = storage_keypts_inner_skeleton_[0].x;
        endPoint[1] = storage_keypts_inner_skeleton_[0].y;
        endPoint[2] = storage_keypts_inner_skeleton_[0].heading;
        r->reedsShepp(startPoint,endPoint);
        finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0;j < finalpath.size();j++){
            pathInterface::pathPoint   pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            //针对给定点位计算点位的heading,用来判断前进倒退标志
            if(finalpath.size() > 4){
                if(j < (finalpath.size() - 4)){
                    polygonPoint p1, p2;
                    p1.x = finalpath[j+1][0] - finalpath[j][0];
                    p1.y = finalpath[j+1][1] - finalpath[j][1];
                    p2.x = finalpath[j+2][0] - finalpath[j+1][0];
                    p2.y = finalpath[j+2][1] - finalpath[j+1][1];
                    auto angle = common::commonMath::computeTwolineAngleDu(p1,p2);
                    if(angle > 0){
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                    }else{
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::BACK;
                    }
                }else{
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                }
            }else{
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            }
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
        }
        for(int i = 1;i < storage_keypts_inner_skeleton_.size() - 1;i++){
            auto temp_point =  storage_keypts_inner_skeleton_[i];
            Point point_temp1 = backshape_keypts_info_[temp_point].start_curve_point;
            Point point_temp2 = backshape_keypts_info_[temp_point].end_curve_point;
            startPoint[0] = point_temp1.x;
            startPoint[1] = point_temp1.y;
            startPoint[2] = point_temp1.heading;
            endPoint[0] = point_temp2.x;
            endPoint[1] = point_temp2.y;
            endPoint[2] = point_temp2.heading;
            LOG(INFO) << "heading 1 is : " << startPoint[2]
                      << "heading 2 is : " << endPoint[2];

            if(fabs(fabs(startPoint[2]) - fabs(endPoint[2])) < 0.3){
                point_1.x = storage_keypts_inner_skeleton_[i].x;
                point_1.y = storage_keypts_inner_skeleton_[i].y;
                point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                point_1.ridge_number = ridge_index;
                storageAllPath.push_back(point_1);
                continue;
            }
            r->reedsShepp(startPoint,endPoint);
            finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0;j < finalpath.size(); j++){
                pathInterface::pathPoint   pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                //针对给定点位计算点位的heading,用来判断前进倒退标志
                if(finalpath.size() > 4){
                    if(j < (finalpath.size() - 4)){
                        polygonPoint p1, p2;
                        p1.x = finalpath[j+1][0] - finalpath[j][0];
                        p1.y = finalpath[j+1][1] - finalpath[j][1];
                        p2.x = finalpath[j+2][0] - finalpath[j+1][0];
                        p2.y = finalpath[j+2][1] - finalpath[j+1][1];
                        auto angle = common::commonMath::computeTwolineAngleDu(p1,p2);
                        if(angle > 0){
                            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                        }else{
                            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::BACK;
                        }
                    }else{
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                    }
                }else{
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                }
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
        }
        //添加终点
        point_1.x = storage_keypts_inner_skeleton_[storage_keypts_inner_skeleton_.size() - 1].x;
        point_1.y = storage_keypts_inner_skeleton_[storage_keypts_inner_skeleton_.size() - 1].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        return storageAllPath;
    }
    auto  ordered_points = cgalbackShape_keypoints_[ridge_index];
    int num = ordered_points.size();

    //分情况处理最后一垄
    if(ridge_index == cgalbackShape_keypoints_.size() - 1) {
        switch (cgalLastPolyType_) {
            case cgalLastPolyIdentify::POLY_LEAVE:
            case cgalLastPolyIdentify::POLY_NONE:
            case cgalLastPolyIdentify::POLY_LESS_THR: {
                cgalComputeLastRidgeRoutingParallelLines(storageAllPath);
                break;
            }
            default: {
                break;
            }
        }
        return storageAllPath;
    }

    point_1.x = ordered_points[0].x;
    point_1.y = ordered_points[0].y;
    point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
    point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
    point_1.ridge_number = ridge_index;
    storageAllPath.push_back(point_1);

    switch(curveLocationChoose_){
        case curveLocationChoose::MODE_HEADLANDS_CURVE__START_END:{
            cgalComputeFishNailRidgePath(
                    num,
                    ordered_points,
                    ridge_index,
                   storageAllPath);
            break;
        }
        case curveLocationChoose::MODE_CUSTOM_CURVE_START_END:{
            cgalComputeRSpath(
                     num,
                     ordered_points,
                     ridge_index,
                     storageAllPath,
                     r);
            break;
        }
        default:{
            break;
        }
    }

   return storageAllPath;
}

void pathPolygonPlan::cgalComputeFishNailRidgePath(
        int & num,
        std::vector<polygonPoint> & ordered_points,
        int & ridge_index,
        std::vector<pathInterface::pathPoint> & storageAllPath){
    for(int  i = 1;i <= num;i++){
        //点位弯道处理
        auto temp_point = ordered_points[i];
        auto finalpath = backshape_fishnail_curve_path_[temp_point];
        for(int j = 0;j < finalpath.size();j++){
            pathInterface::pathPoint   pathPointCurve;
            pathPointCurve.x = finalpath[j].x;
            pathPointCurve.y = finalpath[j].y;
           if(finalpath[j].pathPtType_ == pathPtType::FORWARD){
               pathPointCurve.path_point_mode2 =  pathInterface::pathPointMode2::FORWARD;
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.ridge_number = ridge_index;
               storageAllPath.push_back(pathPointCurve);
           }else if(finalpath[j].pathPtType_ == pathPtType::BACKWARD){
               pathPointCurve.path_point_mode2 =  pathInterface::pathPointMode2::BACK;
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.ridge_number = ridge_index;
               storageAllPath.push_back(pathPointCurve);
           }else if(finalpath[j].pathPtType_ == pathPtType::SWITCHPT){
               pathPointCurve.path_point_mode2 =  pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.ridge_number = ridge_index;
               storageAllPath.push_back(pathPointCurve);
               storageAllPath.push_back(pathPointCurve);
               storageAllPath.push_back(pathPointCurve);
           }
        }
    }
}

void  pathPolygonPlan::cgalComputeRSpath(
        int & num,
        std::vector<polygonPoint> & ordered_points,
        int & ridge_index,
        std::vector<pathInterface::pathPoint> & storageAllPath,
        ReedsSheppStateSpace   *r){
    double startPoint[3],endPoint[3];
    for(int i = 1;i < num;i++){
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
        auto finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0;j < finalpath.size();j++){
            pathInterface::pathPoint   pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            //针对给定点位计算点位的heading,用来判断前进倒退标志
            if(finalpath.size() > 4){
                if(j < (finalpath.size() - 4)){
                    polygonPoint p1, p2;
                    p1.x = finalpath[j+1][0] - finalpath[j][0];
                    p1.y = finalpath[j+1][1] - finalpath[j][1];
                    p2.x = finalpath[j+2][0] - finalpath[j+1][0];
                    p2.y = finalpath[j+2][1] - finalpath[j+1][1];
                    auto angle = common::commonMath::computeTwolineAngleDu(p1,p2);
                    if(angle > 0){
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                    }else{
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::BACK;
                    }
                }else{
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                }
            }else{
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            }
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
        }

//        //弯道结束需要增加3个点位信息
//        polygonPoint p2;
//        p2.x = point_temp2.x;
//        p2.y = point_temp2.y;
//        auto increase_points3 =
//                common::commonMath::findPointExtendSegment(temp_point,
//                                                           p2,
//                                                           SET_CONVERTDIRECTION_DIST,
//                                                           true,
//                                                           SET_CONVERTDIRECTION_COUNT);
//        for(auto it : increase_points3){
//            it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
//            it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
//            it.ridge_number = ridge_index;
//            storageAllPath.push_back(it);
//        }
//        //增加倒车点位信息
//        std::vector<polygonPoint>  lineInfo;
//        lineInfo.push_back(p2);
//        lineInfo.push_back(temp_point);
//        auto back_points =
//                common::commonMath::densify(lineInfo,
//                                            SET_BACK_DIS);
//        for(auto it : back_points){
//            pathInterface::pathPoint temp_point;
//            temp_point.x = it.x;
//            temp_point.y = it.y;
//            temp_point.path_point_mode1 =  pathInterface::pathPointMode1::WORK_AREA;
//            temp_point.path_point_mode2 =  pathInterface::pathPointMode2::BACK;
//            temp_point.ridge_number = ridge_index;
//            storageAllPath.push_back(temp_point);
//        }
//        //结束倒车增加3个点位信息
//        auto ending_back_points =
//                common::commonMath::findPointExtendSegment(temp_point,
//                                                           p2,
//                                                           SET_CONVERTDIRECTION_DIST,
//                                                           false,
//                                                           SET_CONVERTDIRECTION_COUNT);
//        for(auto it : ending_back_points){
//            it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
//            it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
//            it.ridge_number = ridge_index;
//            storageAllPath.push_back(it);
//        }
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
        int ridges_counts  = filtered_backshape_keypoints_.size();
        //最后一垄特殊处理
    if(ridge_index == ridges_counts -1 ){
        //计算倒数第二笼最后一个点的heading
        std::vector<polygonPoint> vector_heading ;
        int pts_size = filtered_backshape_keypoints_[ridge_index-1].size();
        for(int i = pts_size-2;i <= pts_size-1;i++){
            vector_heading.push_back(filtered_backshape_keypoints_[ridge_index-1][i]);
        }
        double heading_spec_pt = common::commonMath::computeTwoLineAngle(vector_heading[0],
                                                                         vector_heading[1]);
        double heading_first_pt = common::commonMath::computeTwoLineAngle(ordered_points[0],
                                                                          ordered_points[1]);
        //根据最后一垄的种类区分
        switch(last_polys_type_){
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR:{
                computeLastRidgeRoutingFour(storageAllPath,
                                            ridge_index);
                return storageAllPath;
            }
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_FOUR:{
                computeLastRidgeRoutingFourAndFour(storageAllPath,ridge_index);
                return storageAllPath;
            }
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_THREE:{
                computeLastRidgeRoutingFourAndFour(storageAllPath,ridge_index);
                return storageAllPath;
            }
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FIVE:{

                return  storageAllPath;
            }
            default:{

                return storageAllPath;
            }
        }
    }
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
        if((last_polys_type_ == lastPolyIdentify::POLY_FOUR_AND_FOUR ||
           last_polys_type_ == lastPolyIdentify::POLY_FOUR_AND_THREE)&&
           ridge_index == ridges_counts -2){
            num = num -1 ;
        }
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
                polygonPoint p2;
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
        if((last_polys_type_ == lastPolyIdentify::POLY_FOUR_AND_FOUR ||
           last_polys_type_ == lastPolyIdentify::POLY_FOUR_AND_THREE)&&
           ridge_index == ridges_counts -2) {
                point_1.x = ordered_points[num+1].x;
                point_1.y = ordered_points[num+1].y;
                point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                point_1.ridge_number = ridge_index;
                storageAllPath.push_back(point_1);
            //计算此点的heading
            polygonPoint vector_p;
            vector_p.x = ordered_points[num +1].x - ordered_points[num].x;
            vector_p.y = ordered_points[num +1].y - ordered_points[num].y;
            double heading = common::commonMath::calculateHeading(vector_p);
            if(heading < 0){
                heading += 2 * M_PI;
            }
            filtered_backshape_keypoints_[ridge_index][num+1].heading = heading;
        }
        return storageAllPath;
}

void pathPolygonPlan::computeLastRidgeRoutingFour(std::vector<pathInterface::pathPoint> &storageAllPath,
                                                  int ridge_index){
    ReedsSheppStateSpace   *p=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    double startPoint[3],endPoint[3];
    int num = filtered_backshape_keypoints_[ridge_index].size() ;
    //四边形分为三类处理，一类是四边形，一类是长边进入的四边形，一类是短边进的四边形
    //从短边进与从长边进入
    double distance_1 =
            common::commonMath::distance2(filtered_backshape_keypoints_[ridge_index][1],
                                          filtered_backshape_keypoints_[ridge_index][0]);
    double distance_2 =
            common::commonMath::distance2(filtered_backshape_keypoints_[ridge_index][2],
                                          filtered_backshape_keypoints_[ridge_index][1]);
    if(num == 5){
        LOG(INFO) << "the last bridge belong to four shape !";
        if(fabs(distance_1) < fabs(distance_2)){
            LOG(INFO) << "the last bridge belong to four shape ,from short bridge enter !";
            polygonPoint vector_m;
            vector_m.x = filtered_backshape_keypoints_[ridge_index][2].x - filtered_backshape_keypoints_[ridge_index][1].x;
            vector_m.y = filtered_backshape_keypoints_[ridge_index][2].y - filtered_backshape_keypoints_[ridge_index][1].y;
            double heading_1 = common::commonMath::calculateHeading(vector_m);
            polygonPoint vector_n;
            vector_n.x = filtered_backshape_keypoints_[ridge_index][4].x - filtered_backshape_keypoints_[ridge_index][3].x;
            vector_n.y = filtered_backshape_keypoints_[ridge_index][4].y - filtered_backshape_keypoints_[ridge_index][3].y;
            double heading_2 = common::commonMath::calculateHeading(vector_n);

            if(heading_1 < 0){
                heading_1 += 2 *M_PI;
            }
            if(heading_2 < 0){
                heading_2 += 2* M_PI;
            }
            auto curve_A_end_pt = common::commonMath::findPointOnSegment(
                    filtered_backshape_keypoints_[ridge_index][1],
                    filtered_backshape_keypoints_[ridge_index][2],
                    6,
                    true);

            startPoint[0] = filtered_backshape_keypoints_[ridge_index][0].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][0].y;
            startPoint[2] = heading_1;
            endPoint[0] = curve_A_end_pt.x;
            endPoint[1] = curve_A_end_pt.y;
            endPoint[2] = heading_2;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
            //暂时没有设置倒车处理，后续可在此处添加
            startPoint[0] = filtered_backshape_keypoints_[ridge_index][2].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][2].y;
            startPoint[2] = heading_1;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][3].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][3].y;
            endPoint[2] = heading_2;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }

            pathInterface::pathPoint point_n;
            point_n.x = filtered_backshape_keypoints_[ridge_index][4].x;
            point_n.y = filtered_backshape_keypoints_[ridge_index][4].y;
            point_n.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_n.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_n.ridge_number = ridge_index;
            storageAllPath.push_back(point_n);
            return ;
        }else{
            LOG(INFO) << "the last bridge belong to four shape ,from long bridge enter !";
            polygonPoint vector_m;
            vector_m.x = filtered_backshape_keypoints_[ridge_index][1].x - filtered_backshape_keypoints_[ridge_index][0].x;
            vector_m.y = filtered_backshape_keypoints_[ridge_index][1].y - filtered_backshape_keypoints_[ridge_index][0].y;
            double heading_1 = common::commonMath::calculateHeading(vector_m);
            polygonPoint vector_n;
            vector_n.x = filtered_backshape_keypoints_[ridge_index][3].x - filtered_backshape_keypoints_[ridge_index][2].x;
            vector_n.y = filtered_backshape_keypoints_[ridge_index][3].y - filtered_backshape_keypoints_[ridge_index][2].y;
            double heading_2 = common::commonMath::calculateHeading(vector_n);

            if(heading_1 < 0){
                heading_1 += 2 *M_PI;
            }
            if(heading_2 < 0){
                heading_2 += 2* M_PI;
            }
            startPoint[0] = filtered_backshape_keypoints_[ridge_index][1].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][1].y;
            startPoint[2] = heading_1;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][2].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][2].y;
            endPoint[2] = heading_2;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }

            pathInterface::pathPoint point_n;
            point_n.x = filtered_backshape_keypoints_[ridge_index][3].x;
            point_n.y = filtered_backshape_keypoints_[ridge_index][3].y;
            point_n.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_n.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_n.ridge_number = ridge_index;
            storageAllPath.push_back(point_n);
            polygonPoint vector_f;
            vector_f.x = filtered_backshape_keypoints_[ridge_index][4].x - filtered_backshape_keypoints_[ridge_index][3].x;
            vector_f.y = filtered_backshape_keypoints_[ridge_index][4].y - filtered_backshape_keypoints_[ridge_index][3].y;
            double heading_3 = common::commonMath::calculateHeading(vector_f);

            startPoint[0] = filtered_backshape_keypoints_[ridge_index][3].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][3].y;
            startPoint[2] = heading_2;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][4].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][4].y;
            endPoint[2] = heading_3;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
            return ;
        }
    }

    if(fabs(distance_1) < fabs(distance_2)){ //从短边进入
        LOG(INFO) << "the last ridge add middle line,from  short bridge enter !";
        int num_upper = filtered_backshape_keypoints_[ridge_index-1].size();
        auto first_point  = filtered_backshape_keypoints_[ridge_index-1][num_upper-1];
        auto second_point = filtered_backshape_keypoints_[ridge_index][1];
        auto third_point  = filtered_backshape_keypoints_[ridge_index][2];
        auto fourth_point = filtered_backshape_keypoints_[ridge_index][3];
        auto ordered_point = common::commonMath::findPointOnSegment(
                second_point,
                third_point,
                6,
                true);
        polygonPoint vector_p;
        vector_p.x = third_point.x - second_point.x;
        vector_p.y = third_point.y - second_point.y;
        double heading_oredered_second = common::commonMath::calculateHeading(vector_p);
        polygonPoint vector_t;
        vector_t.x = second_point.x - first_point.x;
        vector_t.y = second_point.y - first_point.y;
        double heading_ordered_first = common::commonMath::calculateHeading(vector_t);
        if(heading_ordered_first < 0){
            heading_ordered_first += 2 * M_PI;
        }
        if(heading_oredered_second < 0){
            heading_oredered_second += 2 * M_PI;
        }
        startPoint[0] = first_point.x;
        startPoint[1] = first_point.y;
        startPoint[2] = heading_ordered_first;
        endPoint[0] = ordered_point.x;
        endPoint[1] = ordered_point.y;
        endPoint[2] = heading_oredered_second;
        p->reedsShepp(startPoint,endPoint);
        finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0; j < finalpath.size();j++){
            pathInterface::pathPoint pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
        }
        //判断是否增加弯道处理
        if(SET_REVERSING_FLAG){
            //弯道结束需要增加3个点位信息
            auto increase_points3 =
                    common::commonMath::findPointExtendSegment(second_point,
                                                               ordered_point,
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
            std::vector<polygonPoint> lineInfo;
            lineInfo.push_back(ordered_point);
            lineInfo.push_back(second_point);
            auto back_points =
                    common::commonMath::densify(
                            lineInfo,
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
                    common::commonMath::findPointExtendSegment(ordered_point,
                                                               second_point,
                                                               SET_CONVERTDIRECTION_DIST,
                                                               true,
                                                               SET_CONVERTDIRECTION_COUNT);
            for(auto it : ending_back_points){
                it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                it.ridge_number = ridge_index;
                storageAllPath.push_back(it);
            }
            //处理下一个弯道
            polygonPoint p4;
            p4.x = filtered_backshape_keypoints_[ridge_index][4].x - fourth_point.x;
            p4.y = filtered_backshape_keypoints_[ridge_index][4].y - fourth_point.y;
            double heading_four =
                    common::commonMath::calculateHeading(p4);
            double heading_four1 = heading_four;
            if( heading_four < 0 ){
                heading_four += 2*M_PI;
            }
            if(heading_oredered_second < 0){
                heading_oredered_second += 2 *M_PI;
            }
            startPoint[0] = third_point.x;
            startPoint[1] = third_point.y;
            startPoint[2] = heading_oredered_second;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][3].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][3].y;
            endPoint[2] = heading_four;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
            //处理下一个弯道
            polygonPoint p6;
            p6.x =  filtered_backshape_keypoints_[ridge_index][6].x -
                    filtered_backshape_keypoints_[ridge_index][5].x;
            p6.y = filtered_backshape_keypoints_[ridge_index][6].y -
                   filtered_backshape_keypoints_[ridge_index][5].y;
            double heading_six =
                    common::commonMath::calculateHeading(p6);
            if(heading_six < 0){
                heading_six += M_PI;
            }
            startPoint[0] =  filtered_backshape_keypoints_[ridge_index][4].x;
            startPoint[1] =  filtered_backshape_keypoints_[ridge_index][4].y;
            startPoint[2] = heading_four1;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][5].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][5].y;
            endPoint[2] = heading_six;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for( int j = 0; j < finalpath.size();j++ ){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
            //添加最后终点
            pathInterface::pathPoint point_m;
            point_m.x = filtered_backshape_keypoints_[ridge_index][6].x;
            point_m.y = filtered_backshape_keypoints_[ridge_index][6].y;
            point_m.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_m.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_m.ridge_number = ridge_index;
            storageAllPath.push_back(point_m);
        }
    }else{
        LOG(INFO) << "the last ridge add middle line, from long bridge enter !";
        auto curve_A_StartPt = common::commonMath::findPointOnSegment(
                filtered_backshape_keypoints_[ridge_index][0],
                filtered_backshape_keypoints_[ridge_index][1],
                6,
                false);
        auto curve_A_endPt = common::commonMath::findPointOnSegment(
                filtered_backshape_keypoints_[ridge_index][1],
                filtered_backshape_keypoints_[ridge_index][2],
                6,
                true);
        polygonPoint vector_A_start ;
        vector_A_start.x = filtered_backshape_keypoints_[ridge_index][1].x - filtered_backshape_keypoints_[ridge_index][0].x;
        vector_A_start.y = filtered_backshape_keypoints_[ridge_index][1].y - filtered_backshape_keypoints_[ridge_index][0].y;
        polygonPoint vector_A_end;
        vector_A_end.x = filtered_backshape_keypoints_[ridge_index][2].x - filtered_backshape_keypoints_[ridge_index][1].x;
        vector_A_end.y = filtered_backshape_keypoints_[ridge_index][2].y - filtered_backshape_keypoints_[ridge_index][1].y;
        double heading_A_start = common::commonMath::calculateHeading(vector_A_start);
        double heading_A_end = common::commonMath::calculateHeading(vector_A_end);

        startPoint[0] = curve_A_StartPt.x;
        startPoint[1] = curve_A_StartPt.y;
        startPoint[2] = heading_A_start;
        endPoint[0] = curve_A_endPt.x;
        endPoint[1] = curve_A_endPt.y;
        endPoint[2] = heading_A_end;
        p->reedsShepp(startPoint,endPoint);
        finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0; j < finalpath.size();j++){
            pathInterface::pathPoint pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
        }
        if(SET_REVERSING_FLAG){
               // 弯道结束需要增加3个点位信息
            auto increase_points3 =
                    common::commonMath::findPointExtendSegment( filtered_backshape_keypoints_[ridge_index][1],
                                                                curve_A_endPt,
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
            std::vector<polygonPoint> lineInfo;
            lineInfo.push_back(curve_A_endPt);
            lineInfo.push_back(filtered_backshape_keypoints_[ridge_index][1]);
            auto back_points =
                    common::commonMath::densify(
                            lineInfo,
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
                    common::commonMath::findPointExtendSegment(curve_A_endPt,
                                                               filtered_backshape_keypoints_[ridge_index][1],
                                                               SET_CONVERTDIRECTION_DIST,
                                                               true,
                                                               SET_CONVERTDIRECTION_COUNT);
            for(auto it : ending_back_points){
                it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                it.ridge_number = ridge_index;
                storageAllPath.push_back(it);
            }
            //B点处理
            auto curve_B_end_pt = common::commonMath::findPointOnSegment(
                           filtered_backshape_keypoints_[ridge_index][2],
                           filtered_backshape_keypoints_[ridge_index][3],
                           6,
                           true);
            polygonPoint vector_B_end;
            vector_B_end.x = filtered_backshape_keypoints_[ridge_index][3].x - filtered_backshape_keypoints_[ridge_index][2].x;
            vector_B_end.y = filtered_backshape_keypoints_[ridge_index][3].y - filtered_backshape_keypoints_[ridge_index][2].y;
            double heading_B_end = common::commonMath::calculateHeading(vector_B_end);
            startPoint[0] = filtered_backshape_keypoints_[ridge_index][1].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][1].y;
            startPoint[2] = heading_A_end;
            endPoint[0] = curve_B_end_pt.x;
            endPoint[1] = curve_B_end_pt.y;
            endPoint[2] = heading_B_end;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }
            // 弯道结束需要增加3个点位信息
            auto increase_points3_B =
                    common::commonMath::findPointExtendSegment( filtered_backshape_keypoints_[ridge_index][2],
                                                                curve_B_end_pt,
                                                                SET_CONVERTDIRECTION_DIST,
                                                                true,
                                                                SET_CONVERTDIRECTION_COUNT);
            for(auto it : increase_points3_B){
                it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                it.ridge_number = ridge_index;
                storageAllPath.push_back(it);
            }
            //增加倒车点位信息
            std::vector<polygonPoint> lineInfo_B;
            lineInfo_B.push_back(curve_B_end_pt);
            lineInfo_B.push_back(filtered_backshape_keypoints_[ridge_index][2]);
            auto back_points_B =
                    common::commonMath::densify(
                            lineInfo_B,
                            SET_BACK_DIS);
            for(auto it : back_points_B){
                pathInterface::pathPoint temp_point;
                temp_point.x = it.x;
                temp_point.y = it.y;
                temp_point.path_point_mode1 =  pathInterface::pathPointMode1::WORK_AREA;
                temp_point.path_point_mode2 =  pathInterface::pathPointMode2::BACK;
                temp_point.ridge_number = ridge_index;
                storageAllPath.push_back(temp_point);
            }
            //结束倒车增加3个点位信息
            auto ending_back_points_B =
                    common::commonMath::findPointExtendSegment(curve_B_end_pt,
                                                               filtered_backshape_keypoints_[ridge_index][2],
                                                               SET_CONVERTDIRECTION_DIST,
                                                               true,
                                                               SET_CONVERTDIRECTION_COUNT);
            for(auto it : ending_back_points_B){
                it.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
                it.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
                it.ridge_number = ridge_index;
                storageAllPath.push_back(it);
            }

            //添加最后一个弯道
            polygonPoint  vector_E_pt;
            vector_E_pt.x = filtered_backshape_keypoints_[ridge_index][6].x - filtered_backshape_keypoints_[ridge_index][5].x;
            vector_E_pt.y = filtered_backshape_keypoints_[ridge_index][6].y - filtered_backshape_keypoints_[ridge_index][5].y;
            double heading_E = common::commonMath::calculateHeading(vector_E_pt);
            startPoint[0] = filtered_backshape_keypoints_[ridge_index][3].x;
            startPoint[1] = filtered_backshape_keypoints_[ridge_index][3].y;
            startPoint[2] = heading_B_end;
            endPoint[0] = filtered_backshape_keypoints_[ridge_index][5].x;
            endPoint[1] = filtered_backshape_keypoints_[ridge_index][5].y;
            endPoint[2] = heading_E;
            p->reedsShepp(startPoint,endPoint);
            finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int j = 0; j < finalpath.size();j++){
                pathInterface::pathPoint pathPointCurve;
                pathPointCurve.x = finalpath[j][0];
                pathPointCurve.y = finalpath[j][1];
                pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                pathPointCurve.ridge_number = ridge_index;
                storageAllPath.push_back(pathPointCurve);
            }

            //增加最后一个点
            pathInterface::pathPoint point_f;
            point_f.x = filtered_backshape_keypoints_[ridge_index][6].x;
            point_f.y = filtered_backshape_keypoints_[ridge_index][6].y;
            point_f.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_f.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_f.ridge_number = ridge_index;
            storageAllPath.push_back(point_f);
        }
    }
}

void pathPolygonPlan::cgalComputeLastRidgeRoutingParallelLines(
         std::vector<pathInterface::pathPoint> &storageAllPath){
    ReedsSheppStateSpace   *p=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    double startPoint[3],endPoint[3];
    int num = cgalbackShape_keypoints_.size();
    auto ordered_pts = cgalbackShape_keypoints_[num-1];
    pathInterface::pathPoint  point_1;
    int num_upper = cgalbackShape_keypoints_[num-2].size();
    //获取连接最后一垄的最后一个关键点
    auto spec_point = cgalbackShape_keypoints_[num-2][num_upper-1];
    //计算第一个弯道
    startPoint[0] = spec_point.x;
    startPoint[1] = spec_point.y;
    startPoint[2] = spec_point.heading;
    endPoint[0] = ordered_pts[0].x;
    endPoint[1] = ordered_pts[0].y;
    endPoint[2] = ordered_pts[0].heading;
    p->reedsShepp(startPoint,endPoint);
    finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
    for(int j = 0; j < finalpath.size();j++){
        pathInterface::pathPoint pathPointCurve;
        pathPointCurve.x = finalpath[j][0];
        pathPointCurve.y = finalpath[j][1];
        if(finalpath.size() > 4){
            if(j < (finalpath.size() - 4)){
                polygonPoint p1, p2;
                p1.x = finalpath[j+1][0] - finalpath[j][0];
                p1.y = finalpath[j+1][1] - finalpath[j][1];
                p2.x = finalpath[j+2][0] - finalpath[j+1][0];
                p2.y = finalpath[j+2][1] - finalpath[j+1][1];
                auto angle = common::commonMath::computeTwolineAngleDu(p1,p2);
                if(angle > 0){
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                }else{
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::BACK;
                }
            }else{
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            }
        }else{
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        }
        pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
//        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        pathPointCurve.ridge_number = num - 1;
        storageAllPath.push_back(pathPointCurve);
    }
    //计算剩余弯道
    int num_last =  cgalbackShape_keypoints_[num-1].size();
    for(int i = 1;i < num_last ;i++){
        if(i >= num_last - 1){
            point_1.x = ordered_pts[num_last -1].x;
            point_1.y = ordered_pts[num_last -1].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_1.ridge_number = num - 1;
            storageAllPath.push_back(point_1);
            return;
        }
        startPoint[0] = ordered_pts[i].x;
        startPoint[1] = ordered_pts[i].y;
        startPoint[2] = ordered_pts[i].heading;
        endPoint[0] = ordered_pts[i+1].x;
        endPoint[1] = ordered_pts[i+1].y;
        endPoint[2] = ordered_pts[i+1].heading;
        p->reedsShepp(startPoint,endPoint);
        finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0; j < finalpath.size();j++){
            pathInterface::pathPoint pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            if(finalpath.size() > 4){
                if(j < (finalpath.size() - 4)){
                    polygonPoint p1, p2;
                    p1.x = finalpath[j+1][0] - finalpath[j][0];
                    p1.y = finalpath[j+1][1] - finalpath[j][1];
                    p2.x = finalpath[j+2][0] - finalpath[j+1][0];
                    p2.y = finalpath[j+2][1] - finalpath[j+1][1];
                    auto angle = common::commonMath::computeTwolineAngleDu(p1,p2);
                    if(angle > 0){
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                    }else{
                        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::BACK;
                    }
                }else{
                    pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
                }
            }else{
                pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            }
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
//            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number = num - 1;
            storageAllPath.push_back(pathPointCurve);
        }
        i++;
    }
}

void   pathPolygonPlan::computeLastRidgeRoutingFourAndFour(std::vector<pathInterface::pathPoint> &storageAllPath,
                                                           int ridge_index){
    ReedsSheppStateSpace   *p=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    double startPoint[3],endPoint[3];
    int num = filtered_backshape_keypoints_[ridge_index].size() ;
    auto ordered_pts = filtered_backshape_keypoints_[ridge_index];
    int num_upper = filtered_backshape_keypoints_[ridge_index-1].size();
    pathInterface::pathPoint  point_1;
    //计算点位的heading
    computeKeypointsHeading();
    //得到对应的path
    auto spec_point = filtered_backshape_keypoints_[ridge_index-1][num_upper-1];
    //计算第一个弯道
    startPoint[0] = spec_point.x;
    startPoint[1] = spec_point.y;
    startPoint[2] = spec_point.heading;
    endPoint[0] = filtered_backshape_keypoints_[ridge_index][0].x;
    endPoint[1] = filtered_backshape_keypoints_[ridge_index][0].y;
    endPoint[2] = filtered_backshape_keypoints_[ridge_index][0].heading;
    p->reedsShepp(startPoint,endPoint);
    finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
    for(int j = 0; j < finalpath.size();j++){
        pathInterface::pathPoint pathPointCurve;
        pathPointCurve.x = finalpath[j][0];
        pathPointCurve.y = finalpath[j][1];
        pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        pathPointCurve.ridge_number = ridge_index;
        storageAllPath.push_back(pathPointCurve);
    }
    //计算剩余弯道
    for(int i = 1;i < num ;i++){
        if(i >= num-1){
            point_1.x = filtered_backshape_keypoints_[ridge_index][num-1].x;
            point_1.y = filtered_backshape_keypoints_[ridge_index][num -1].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            return;
        }
        startPoint[0] = filtered_backshape_keypoints_[ridge_index][i].x;
        startPoint[1] = filtered_backshape_keypoints_[ridge_index][i].y;
        startPoint[2] = filtered_backshape_keypoints_[ridge_index][i].heading;
        endPoint[0] = filtered_backshape_keypoints_[ridge_index][i+1].x;
        endPoint[1] = filtered_backshape_keypoints_[ridge_index][i+1].y;
        endPoint[2] = filtered_backshape_keypoints_[ridge_index][i+1].heading;
        p->reedsShepp(startPoint,endPoint);
        finalpath = p->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int j = 0; j < finalpath.size();j++){
            pathInterface::pathPoint pathPointCurve;
            pathPointCurve.x = finalpath[j][0];
            pathPointCurve.y = finalpath[j][1];
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
        }
        i++;
    }
}

void pathPolygonPlan::computeLastRidgeRoutingFourAndThree(
                   std::vector<pathInterface::pathPoint> &storageAllPath,
                   int ridge_index){
    ReedsSheppStateSpace   *p=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    double startPoint[3],endPoint[3];
    int num = filtered_backshape_keypoints_[ridge_index].size();
    int num_upper = filtered_backshape_keypoints_[ridge_index-1].size();
    pathInterface::pathPoint  point_1;
    //计算点位heading
    computeKeypointsHeading();

}

void pathPolygonPlan::computeKeypointsHeading(){
    int num = filtered_backshape_keypoints_.size();
    auto ridge_last_pts = filtered_backshape_keypoints_[num-1];
    double heading;
    for(int i = 0;i < ridge_last_pts.size();i++){
            polygonPoint vector_p;
            vector_p.x = ridge_last_pts[i+1].x - ridge_last_pts[i].x;
            vector_p.y = ridge_last_pts[i+1].y - ridge_last_pts[i].y;
            heading = common::commonMath::calculateHeading(vector_p);
            LOG(INFO) <<"the last ridge  compute heading is : " << heading;
            filtered_backshape_keypoints_[num - 1][i].heading = heading;
            filtered_backshape_keypoints_[num - 1][i + 1].heading = heading;
            i++;
    }
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
                            (fabs(distancePts_near_2 - im.distance) < 0.1)){
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
            double temp_x = 0,temp_y = 0;
            temp_x = i.points[i.points.size()-1].x  - i.dx * SET_VIRTUAL_LINE_LENGTH;
            temp_y = i.points[i.points.size()-1].y  - i.dy * SET_VIRTUAL_LINE_LENGTH;

           LOG(INFO) << "the last point is :" << i.points[i.points.size()-1].x << " " << i.points[i.points.size()-1].y;
            //计算该线段与原始多边形的交点
            polygon  poly;
            for(auto it : points){
                poly.outer().push_back(pointbst(it.x,it.y));
            }
            linestring_type  line;
            line.push_back(pointbst(i.points[i.points.size()-1].x,
                                 i.points[i.points.size()-1].y));
            line.push_back(pointbst(temp_x,
                                 temp_y));
            std::vector<pointbst> output;
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
     line.push_back(pointbst(min_polygon_centroid_.x,
                               min_polygon_centroid_.y));
     line.push_back(pointbst(node.x,node.y));
     //计算该线段和多边形的交点们
     for(int i = 0; i < count_narrow_polygon_numbers_;i++ ){
         for(auto it = storageNarrowPolygonPoints2_[i].begin();
                  it != storageNarrowPolygonPoints2_[i].end();
                  it++){
             std::vector<pointbst> tempPoints;
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
    line.push_back(pointbst(linePoints[0].x,linePoints[0].y));
    line.push_back(pointbst(linePoints[1].x,linePoints[1].y));

    for(int i = 0; i < count_narrow_polygon_numbers_;i++){
        for(auto it = storageNarrowPolygonPoints2_[i].begin();
                 it != storageNarrowPolygonPoints2_[i].end();
                 it++){
            std::vector<pointbst> tempPoints;
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

void pathPolygonPlan::cgalUpdatePolygonPointsINcrease(){
    int num  = entrance_pts_.size();
    switch (cgalLastPolyType_) {
        case cgalLastPolyIdentify::POLY_LEAVE:{
            LOG(INFO) << "the difference  between the polygon and the entrance >=2";
            for(int i = 0;i < num ;i++ ){
                auto poly_pts = insertPointToPolygon(
                        entrance_pts_[i],
                        cgalPolypts_[i]);
                cgalIncreaseptPolypts_.push_back(poly_pts);
            }
            break;
        }
        case cgalLastPolyIdentify::POLY_NONE:
        case cgalLastPolyIdentify::POLY_LESS_THR:{
            LOG(INFO) << "the difference  between the polygon and the entrance <= 2 or last poly is quadrilateral！";
            int countSize = cgalPolypts_.size();
            for(int i = 0;i < countSize - SET_POLY_TRANSFER_THR ;i++ ){
                auto poly_pts = insertPointToPolygon(
                        entrance_pts_[i],
                        cgalPolypts_[i]);
                cgalIncreaseptPolypts_.push_back(poly_pts);
            }
            break;
        }
        default:{
            LOG(INFO) << "no considered situation happen !";
            break;
        }
    }
}

void pathPolygonPlan::cgalUpatePolygonPointsSequence(){
     int num = cgalIncreaseptPolypts_.size();
     int number;
     std::vector<polygonPoint> storage_points;
     for(int i = 0;i <= num -1;i++){
         for(auto it : cgalIncreaseptPolypts_[i]){
             if(it.entrance_){
                 int num_1 = cgalIncreaseptPolypts_[i].size();
                 for(int j = 0; j < num_1;j++){
                     if(fabs(cgalIncreaseptPolypts_[i][j].x - it.x) < 0.1 &&
                        fabs(cgalIncreaseptPolypts_[i][j].y - it.y) < 0.1){
                         number = j;
                     }
                 }
                 polygonPoint next;
                 storage_points.push_back(it);
                 for(int f = 1;f <= num_1-1 ;f++){
                     next = cgalIncreaseptPolypts_[i][(number+1)%num_1];
                     storage_points.push_back(next);
                     number = number + 1;
                 }
                 cgalSequencedPolypts_.push_back(storage_points);
                 number = 0;
                 storage_points.clear();
                 break;
             }
         }
     }
}

void pathPolygonPlan::cgalComputebackShapeKeypoints(){
    int num = cgalSequencedPolypts_.size();
    std::vector<polygonPoint>  temp_points;
    //当处于POLY_LEAVE时需要重新更新num
    //    if(find_entrance_pts_size_){
    //        num = find_entrance_pts_size_ + 1;
    //}
    for(int i = 0;i < num ;i++) {
        for(int  it = 1 ; it < cgalSequencedPolypts_[i].size();it ++){
            temp_points.push_back(cgalSequencedPolypts_[i][it]);
        }
        temp_points.push_back(cgalSequencedPolypts_[i][0]);
        cgalbackShape_keypoints_.push_back(temp_points);
        temp_points.clear();
    }

    //此处需要添加关键点的弯道映射部分
    cgalComputeAKeyptsMapping();
    switch(cgalLastPolyType_){
        case aiforce::Route_Planning::cgalLastPolyIdentify::POLY_LEAVE:{
            LOG(INFO) << "keypoint type is poly_leave !";
            cgalComputeRidgeKeyPointsLeave();
            break;
        }
        case aiforce::Route_Planning::cgalLastPolyIdentify::POLY_NONE:
        case aiforce::Route_Planning::cgalLastPolyIdentify::POLY_LESS_THR:{
            LOG(INFO) << "keypoint type is poly_less_Thr or  poly_NONE!";
            //添加最后一笼
            cgalComputeRidgeKeyPointsLeave();
            break;
        }
        default:{
            LOG(INFO) << "This situation has not been taken into account !";
        }
    }
    //判断是否添加分裂多边形中的平行线
    LOG(INFO) << "--------------------------------------------------";
    if(!bufferspiltPolysAandBs_.empty() && !set_flag_about_buffer_spilt_polys_){
        LOG(INFO) << "increase spilt poly size is : " << bufferspiltPolysAandBs_.size() - 1;
        cgalComputeBRidgeKeyPointsLeave();
    }
    LOG(INFO) << "--------------------------------------------------";
    //判断是否增加内部直骨架点位映射
    if(!storage_keypts_inner_skeleton_.empty()){
        cgalIncludeLastskeletonMap();
    }
}

void pathPolygonPlan::cgalIncludeLastskeletonMap(){
    //计算最后一个关键点的heading
    int last_poly = cgalbackShape_keypoints_.size();
    int last_size = cgalbackShape_keypoints_[last_poly-1].size();
    auto last_key_pt_1 = cgalbackShape_keypoints_[last_poly - 1][last_size - 1];
    auto last_key_pt_2 = cgalbackShape_keypoints_[last_poly - 1][last_size - 2];
    cgalbackShape_keypoints_[last_poly - 1][last_size - 1].heading
            = atan2(last_key_pt_1.y - last_key_pt_2.y,
                    last_key_pt_1.x - last_key_pt_2.x);
    LOG(INFO) <<" the last key pt heading is : " <<
                 cgalbackShape_keypoints_[last_poly - 1][last_size - 1].heading * 180 / M_PI;

    //计算对应的弯道关键点位
    //计算第一个点位的heading
    int m_size = storage_keypts_inner_skeleton_.size();
    storage_keypts_inner_skeleton_[0].heading =
            atan2(storage_keypts_inner_skeleton_[1].y - storage_keypts_inner_skeleton_[0].y,
                  storage_keypts_inner_skeleton_[1].x - storage_keypts_inner_skeleton_[0].x);

    LOG(INFO) <<"the first skeleton  pt heading is : " <<
                                    storage_keypts_inner_skeleton_[0].heading * 180 / M_PI;
    //计算中间点位的映射关键点
    for(int i = 1;i <  m_size - 1; i++){
        //计算指定点的前后弯道关键点
        ridgeKeypoint tempPtInfo;
        tempPtInfo.start_dis = SET_STARTTURN_DISTANCE;
        tempPtInfo.end_dis  = SET_ENDTURN_DISTANCE;
        tempPtInfo.start_curve_point =
                common::commonMath::findPointOnSegment(
                        storage_keypts_inner_skeleton_[i-1],
                        storage_keypts_inner_skeleton_[i],
                        SET_STARTTURN_DISTANCE,
                        false);
        tempPtInfo.end_curve_point =
                common::commonMath::findPointOnSegment(
                        storage_keypts_inner_skeleton_[i],
                        storage_keypts_inner_skeleton_[i+1],
                        SET_ENDTURN_DISTANCE,
                        true);
        backshape_keypts_info_[storage_keypts_inner_skeleton_[i]] = tempPtInfo;
    }
}

void pathPolygonPlan::cgalComputeParallelLinesHeading(
             std::vector<polygonPoint> & lastRidgePts){
    double heading;
    for(int i = 0;i < lastRidgePts.size();i++){
        polygonPoint vector_p;
        vector_p.x = lastRidgePts[i+1].x - lastRidgePts[i].x;
        vector_p.y = lastRidgePts[i+1].y - lastRidgePts[i].y;
        heading = common::commonMath::calculateHeading(vector_p);
        lastRidgePts[i].heading = heading;
        lastRidgePts[i + 1].heading = heading;
        i++;
    }
}

void pathPolygonPlan::cgalComputeAKeyptsMapping(){

    int num =  cgalbackShape_keypoints_.size();
    //第一垄到 num -1 垄统一处理，最后一笼单独处理
    for(int i = 0;i <= 1 ;i++){
//        for(int j = 1 ; j < cgalbackShape_keypoints_[i].size() - 1;j++){
        for(int j = 1 ; j < cgalbackShape_keypoints_[i].size() - 1  ;j++){
//            for(int j = 3 ; j < 4;j++){
            auto ordered_pt = cgalbackShape_keypoints_[i][j];
            //计算指定点的前后弯道关键点
            ridgeKeypoint tempPtInfo;
            auto forward_last_points =                     //注意点的顺序
                    common::commonMath::computeForwardAndBackPoints(
                            cgalbackShape_keypoints_[i],
                            cgalbackShape_keypoints_[i][j]);
            std::vector<polygonPoint>  arriveLine,leaveLine;
            arriveLine.push_back(forward_last_points[0]);
            arriveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(forward_last_points[1]);
            curveDecisionManager curveDecisionManagerInstance(
                                                             i,
                                                             j,
                                                             arriveLine,
                                                             leaveLine,
                                                             cgalbackShape_keypoints_,
                                                             backshape_fishnail_curve_path_);
            curveDecisionManagerInstance.processCurveType();
            curveDecisionManagerInstance.processCurvePath();
        }

        //最后一个点单独处理,j = cgalbackShape_keypoints_[i].size() - 1
        //如果走到回型最后一个圆圈需要特殊处理
//        auto second_pt = cgalbackShape_keypoints_[i+1][1];
        int spiral_size = cgalbackShape_keypoints_[i].size();
        if(i != num-1 ){                                         //当不是回型最后一笼的处理
            std::vector<polygonPoint> last_arriveline;
            std::vector<polygonPoint> last_leaveline;
            last_arriveline.push_back(cgalbackShape_keypoints_[i][spiral_size-2]);
            last_arriveline.push_back(cgalbackShape_keypoints_[i][spiral_size-1]);
            last_leaveline.push_back(cgalbackShape_keypoints_[i+1][0]);
            last_leaveline.push_back(cgalbackShape_keypoints_[i+1][1]);
            curveDecisionManager curveDecisionManagerInstance(
                                                             i,
                                                             spiral_size-1,
                                                             last_arriveline,
                                                             last_leaveline,
                                                             cgalbackShape_keypoints_,
                                                             backshape_fishnail_curve_path_);
            curveDecisionManagerInstance.processCurveType();
            curveDecisionManagerInstance.processBorderlessFishNail(cgalbackShape_keypoints_[i][spiral_size-1]);
        }else{
            //暂时回型最后一个点不做处理
            //DoNothing
        }
    }
//     switch (curveLocationChoose_){
//         case curveLocationChoose::MODE_HEADLANDS_CURVE__START_END:{
//                  cgalComputeHeadleadsAandB();
//             break;
//         }
//         case curveLocationChoose::MODE_CUSTOM_CURVE_START_END:{
//                 cgalComputeCustomAandB();
//             break;
//         }
//         default:{
//             break;
//         }
//     }

    //最后一笼的单独处理
    cgalComputeParallelCurveMap();
}

void pathPolygonPlan::cgalComputeCustomAandB(){
    int num =  cgalbackShape_keypoints_.size();
    //后续应该改为num-1
    for(int i = 0;i <  1;i++){
        for(int j = 1 ; j < cgalbackShape_keypoints_[i].size() - 1;j++){
            auto ordered_pt = cgalbackShape_keypoints_[i][j];
            //计算指定点的前后弯道关键点
            ridgeKeypoint tempPtInfo;
            tempPtInfo.start_dis = SET_STARTTURN_DISTANCE;
            tempPtInfo.end_dis  = SET_ENDTURN_DISTANCE;
            auto forward_last_points =               //注意点的顺序
                    common::commonMath::computeForwardAndBackPoints(
                            cgalbackShape_keypoints_[i],
                            cgalbackShape_keypoints_[i][j]);
            std::vector<polygonPoint>  arriveLine,leaveLine;
            arriveLine.push_back(forward_last_points[0]);
            arriveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(forward_last_points[1]);
            newCornerTuringLocation newCornerTuringLocationInstance(
                    arriveLine,
                    leaveLine);
            polygonPoint pt1 =  newCornerTuringLocationInstance.getCurveStartAPt();
            polygonPoint pt2 =  newCornerTuringLocationInstance.getCurveEndBPt();
            tempPtInfo.start_curve_point =
                    common::commonMath::findPointOnSegment(
                            forward_last_points[0],
                            cgalbackShape_keypoints_[i][j],
                            SET_STARTTURN_DISTANCE,
                            false);
            tempPtInfo.end_curve_point =
                    common::commonMath::findPointOnSegment(
                            cgalbackShape_keypoints_[i][j],
                            forward_last_points[1],
                            SET_ENDTURN_DISTANCE,
                            true);
            tempPtInfo.start_curve_point.x = pt1.x;
            tempPtInfo.start_curve_point.y = pt1.y;
            tempPtInfo.end_curve_point.x = pt2.x;
            tempPtInfo.end_curve_point.y = pt2.y;
            backshape_keypts_info_[cgalbackShape_keypoints_[i][j]] = tempPtInfo;
        }
        //最后一个点单独处理,j = cgalbackShape_keypoints_[i].size() - 1
        auto second_pt = cgalbackShape_keypoints_[i+1][1];
        ridgeKeypoint  tempLastPt;
        tempLastPt.start_dis = SET_STARTTURN_DISTANCE;
        tempLastPt.end_dis = SET_ENDTURN_DISTANCE;
        int numSize = cgalbackShape_keypoints_[i].size();
        tempLastPt.start_curve_point =
                common::commonMath::findPointOnSegment(
                        cgalbackShape_keypoints_[i][numSize-2],
                        cgalbackShape_keypoints_[i][numSize-1],
                        SET_STARTTURN_DISTANCE,
                        false);
        tempLastPt.end_curve_point =
                common::commonMath::findPointOnSegment(
                        cgalbackShape_keypoints_[i][numSize-1],
                        second_pt,
                        SET_ENDTURN_DISTANCE,
                        true);
        backshape_keypts_info_[cgalbackShape_keypoints_[i][numSize-1]] = tempLastPt;
    }
    //最后一笼的单独处理
    int number_last = cgalbackShape_keypoints_[num-1].size();
    for(int i = 1;i < number_last; i++){
        //计算指定点的前后弯道关键点
        ridgeKeypoint tempPt;
        tempPt.start_dis = SET_STARTTURN_DISTANCE;
        tempPt.end_dis  = SET_ENDTURN_DISTANCE;
        auto forward_last_points =               //注意点的顺序
                common::commonMath::computeForwardAndBackPoints(
                        cgalbackShape_keypoints_[num-1],
                        cgalbackShape_keypoints_[num-1][i]);
        tempPt.start_curve_point =
                common::commonMath::findPointOnSegment(
                        forward_last_points[0],
                        cgalbackShape_keypoints_[num-1][i],
                        SET_STARTTURN_DISTANCE,
                        false);
        tempPt.end_curve_point =
                common::commonMath::findPointOnSegment(
                        cgalbackShape_keypoints_[num-1][i],
                        forward_last_points[1],
                        SET_ENDTURN_DISTANCE,
                        true);
        backshape_keypts_info_[cgalbackShape_keypoints_[num-1][i]] = tempPt;
    }
}

void pathPolygonPlan::cgalComputeHeadleadsAandB(){
    int num =  cgalbackShape_keypoints_.size();

    //第一垄到 num -1 垄统一处理，最后一笼单独处理
    for(int i = 0;i <= 1 ;i++){
//        for(int j = 1 ; j < cgalbackShape_keypoints_[i].size() - 1;j++){
        for(int j = 1 ; j < cgalbackShape_keypoints_[i].size() - 1  ;j++){
//            for(int j = 3 ; j < 4;j++){
            auto ordered_pt = cgalbackShape_keypoints_[i][j];
            //计算指定点的前后弯道关键点
            ridgeKeypoint tempPtInfo;
            tempPtInfo.start_dis = SET_STARTTURN_DISTANCE;
            tempPtInfo.end_dis  = SET_ENDTURN_DISTANCE;
            auto forward_last_points =                     //注意点的顺序
                    common::commonMath::computeForwardAndBackPoints(
                            cgalbackShape_keypoints_[i],
                            cgalbackShape_keypoints_[i][j]);
            std::vector<polygonPoint>  arriveLine,leaveLine;
            arriveLine.push_back(forward_last_points[0]);
            arriveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(cgalbackShape_keypoints_[i][j]);
            leaveLine.push_back(forward_last_points[1]);
            cornerTuringLocation   cornerTuringLocationtest(arriveLine,
                                                            leaveLine);
            cornerTuringLocationtest.decideLpAandLpB();
            cornerTuringLocationtest.calculatePointsAandBForCurve();
            double angleInt = cornerTuringLocationtest.getCurveAngleInt();
            double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
            double arriveLineHeading2 = cornerTuringLocationtest.getCurveArrriveLineHeading2();
            arriveLineHeading = arriveLineHeading * M_PI / 180;
            //这里按照逆时针考虑的，所以取负
            arriveLineHeading = -arriveLineHeading;
            double F1 = cornerTuringLocationtest.getCurveaboutF1();
            double F2 = cornerTuringLocationtest.getCurveaboutF2();
            double F3 = cornerTuringLocationtest.getCurveaboutF3();

//            //添加验证C-CPA代码
//            if(j == 2 && i == 0){
//                cornerTuringImplementRadius cornerTuringImplementRadius1;
//                cornerTuringImplementRadius1.calculateMiniTuringRadiusConsiderImplement();
//                auto Rsw = cornerTuringImplementRadius1.getRsw();
//                cornerTuringCCPAAlgorithm  cornerTuringCCPAAlgorithm1(
//                                                                        angleInt,
//                                                                        Rsw,
//                                                                        15,
//                                                                        false,
//                                                                        cgalbackShape_keypoints_[i][j],
//                                                                        arriveLineHeading);
//                cornerTuringCCPAAlgorithm1.calculateAngleCC2();
//                cornerTuringCCPAAlgorithm1.calculateCirclesCenter();
//                cornerTuringCCPAAlgorithm1.calculatePath();
//                std::cout << "the angleInt is : " << angleInt * 180 / M_PI;
//
//            }

            polygonPoint pt1 =  cornerTuringLocationtest.getCurveStartPtA();
            polygonPoint pt2 =  cornerTuringLocationtest.getCurveendPtB();

            double RC2 = 6 ;
            cornerTuringTishNail cornerTuringTishNailtest(pt1,pt2,angleInt,RC2,F1,F2,F3);

            std::vector<polygonPoint>  ptAB;
            ptAB.push_back(pt1);
            ptAB.push_back(pt2);
            normalMatrixTranslate   testtemp;
            for(auto it : ptAB){
               auto pt = testtemp.reverseRotatePoint(it,cgalbackShape_keypoints_[i][j],
                                            arriveLineHeading);
//                LOG(INFO)  << "the pt transd   is : " << pt.x << " " << pt.y ;
            }

            //计算关键点对应的鱼尾路径点并一一对应存储
            auto local_C1path = cornerTuringTishNailtest.getFishNailC1path();
            auto local_C2path = cornerTuringTishNailtest.getFishNailC2path();
            auto local_C3path = cornerTuringTishNailtest.getFishNailC3path();

//            auto local_C1path = cornerTuringTishNailtest.getC1path();
//            auto local_C2path = cornerTuringTishNailtest.getC2path();
//            auto local_C3path = cornerTuringTishNailtest.getC3path();

            std::vector<polygonPoint> temp_test_C1Path ;
            std::vector<polygonPoint> temp_test_C2Path;
            std::vector<polygonPoint> temp_test_C3Path;

            //转换到世界坐标系下
            normalMatrixTranslate  normalMatrixTranslateInstance;

            std::vector<polygonPoint>  storage_origin_path;
            int num_size_C1 = local_C1path.size();
            int num_size_C2 = local_C2path.size();
            int num_size_C3 = local_C3path.size();

            for(int m = 0 ;m <  num_size_C1 - 1;m++){
                auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                        local_C1path[m],
                        cgalbackShape_keypoints_[i][j],
                        arriveLineHeading);
                tempPt.pathPtType_ = pathPtType::FORWARD;
                temp_test_C1Path.push_back(tempPt);
                storage_origin_path.push_back(tempPt);
            }

            auto C1_LAST_PT = local_C1path[num_size_C1-1];
            auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                    C1_LAST_PT,
                    cgalbackShape_keypoints_[i][j],
                    arriveLineHeading);
            tempPt.pathPtType_ = pathPtType::SWITCHPT;
            temp_test_C1Path.push_back(tempPt);
            storage_origin_path.push_back(tempPt);

            for(int m =0 ;m <  num_size_C2 -1;m++){
                auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                        local_C2path[m],
                        cgalbackShape_keypoints_[i][j],
                        arriveLineHeading);
                tempPt.pathPtType_ = pathPtType::BACKWARD;
                temp_test_C2Path.push_back(tempPt);
                storage_origin_path.push_back(tempPt);
            }

            auto C2_LAST_PT = local_C2path[num_size_C2-1];
            auto tempPt1 = normalMatrixTranslateInstance.reverseRotatePoint(
                    C2_LAST_PT,
                    cgalbackShape_keypoints_[i][j],
                    arriveLineHeading);
            tempPt1.pathPtType_ = pathPtType::SWITCHPT;
            temp_test_C2Path.push_back(tempPt);
            storage_origin_path.push_back(tempPt1);

            for(int m =0 ;m <  num_size_C3;m++){
                auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                        local_C3path[m],
                        cgalbackShape_keypoints_[i][j],
                        arriveLineHeading);
                tempPt.pathPtType_ = pathPtType::FORWARD;
                temp_test_C3Path.push_back(tempPt);
                storage_origin_path.push_back(tempPt);
            }
            backshape_fishnail_curve_path_[cgalbackShape_keypoints_[i][j]] = storage_origin_path;


            //将A、B点转入到世界坐标系下
            std::string ptname = "/home/zzm/Desktop/test_path_figure-main/src/ptsshow.txt";
            auto & ptsshow =
                    common::Singleton::GetInstance<filePrint2>(ptname);

            for(auto it : ptAB){
                auto tempfg = normalMatrixTranslateInstance.reverseRotatePoint(
                        it,
                        cgalbackShape_keypoints_[i][j],
                        arriveLineHeading);
                ptsshow.writePt(tempfg);
            }

            if(j == 2){
                std::string C1name = "/home/zzm/Desktop/test_path_figure-main/src/C1path.txt";
                std::string C2name = "/home/zzm/Desktop/test_path_figure-main/src/C2path.txt";
                std::string C3name = "/home/zzm/Desktop/test_path_figure-main/src/C3path.txt";
                normalPrint C1file(C1name);
                normalPrint C2file(C2name);
                normalPrint C3file(C3name);
                C1file.writePts(temp_test_C1Path);
                C2file.writePts(temp_test_C2Path);
                C3file.writePts(temp_test_C3Path);
            }
        }

        //最后一个点单独处理,j = cgalbackShape_keypoints_[i].size() - 1
        //如果走到回型最后一个圆圈需要特殊处理
//        auto second_pt = cgalbackShape_keypoints_[i+1][1];
        int spiral_size = cgalbackShape_keypoints_[i].size();
        if(i != num-1 ){                                         //当不是回型最后一笼的处理
            std::vector<polygonPoint> last_arriveline;
            std::vector<polygonPoint> last_leaveline;
            last_arriveline.push_back(cgalbackShape_keypoints_[i][spiral_size-2]);
            last_arriveline.push_back(cgalbackShape_keypoints_[i][spiral_size-1]);
            last_leaveline.push_back(cgalbackShape_keypoints_[i+1][0]);
            last_leaveline.push_back(cgalbackShape_keypoints_[i+1][1]);
            cgalComputeEntraceCurvePath(
                    last_arriveline,
                    last_leaveline,
                    cgalbackShape_keypoints_[i][spiral_size - 1]);
        }else{
            //暂时回型最后一个点不做处理
            //DoNothing
        }
    }
}

void pathPolygonPlan::cgalComputeEntraceCurvePath(std::vector<polygonPoint> & arriveLine,
                                                  std::vector<polygonPoint> & leaveLine,
                                                  polygonPoint    curvePt){
    auto ordered_pt = curvePt;  //这里的curvePt指的是弯道需要的映射点并不是交点
    //计算指定点的前后弯道关键点
    ridgeKeypoint tempPtInfo;


    //将arriveline 和leaveline延长一定距离，并求交点
    auto extend_arriveline = common::commonMath::findPointExtendSegment2(arriveLine[0],arriveLine[1],10);
    auto extend_leaveline = common::commonMath::findPointExtendSegment2(leaveLine[0],leaveLine[1],10);

    Segment seg1(pointbst(extend_arriveline[0].x, extend_arriveline[0].y), pointbst(extend_arriveline[1].x, extend_arriveline[1].y));
    Segment seg2(pointbst(extend_leaveline[0].x, extend_leaveline[0].y), pointbst(extend_leaveline[1].x, extend_leaveline[1].y));
    std::deque<pointbst> output1;
    boost::geometry::intersection(seg1, seg2, output1);

    if (!output1.empty()){
        std::cout << " Entrance Curve Intersection point: ("
                  << output1.front().x()
                  << ", "
                  << output1.front().y()
                  << ")"
                  << std::endl;
    }
    polygonPoint   IntersecPt(output1.front().x(),output1.front().y());

    cornerTuringLocation   cornerTuringLocationtest(extend_arriveline,
                                                    extend_leaveline);
    cornerTuringLocationtest.decideLpAandLpB();
    cornerTuringLocationtest.calculatePointsAandBForCurve();
    double angleInt = cornerTuringLocationtest.getCurveAngleInt();
    double arriveLineHeading = cornerTuringLocationtest.getCurveArrriveLineHeading();
    arriveLineHeading = arriveLineHeading * M_PI / 180;
    //这里按照逆时针考虑的，所以取负
    arriveLineHeading = -arriveLineHeading;
    double F1 = cornerTuringLocationtest.getCurveaboutF1();
    double F2 = cornerTuringLocationtest.getCurveaboutF2();
    double F3 = cornerTuringLocationtest.getCurveaboutF3();

    polygonPoint pt1 =  cornerTuringLocationtest.getCurveStartPtA();
    polygonPoint pt2 =  cornerTuringLocationtest.getCurveendPtB();

    double RC2 = CIRCLE_RIDIS_R + 0.5;
    cornerTuringTishNail cornerTuringTishNailtest(pt1,pt2,angleInt,RC2,F1,F2,F3);
    LOG(INFO) << "RC2 is : " << RC2;

    std::vector<polygonPoint>  ptAB;
    ptAB.push_back(pt1);
    ptAB.push_back(pt2);
    normalPrint   printAB("/home/zzm/Desktop/test_path_figure-main/src/testAB.txt");
    printAB.writePts(ptAB);

    normalMatrixTranslate   testtemp;
    for(auto it : ptAB){
        auto pt = testtemp.reverseRotatePoint(it,IntersecPt,
                                              arriveLineHeading);
        LOG(INFO)  << "the pt transd   is : " << pt.x << " " << pt.y ;
    }

    //计算关键点对应的鱼尾路径点并一一对应存储
    auto local_C1path = cornerTuringTishNailtest.getFishNailC1path();
    auto local_C2path = cornerTuringTishNailtest.getFishNailC2path();
    auto local_C3path = cornerTuringTishNailtest.getFishNailC3path();

//            auto local_C1path = cornerTuringTishNailtest.getC1path();
//            auto local_C2path = cornerTuringTishNailtest.getC2path();
//            auto local_C3path = cornerTuringTishNailtest.getC3path();

    //转换到世界坐标系下
    normalMatrixTranslate  normalMatrixTranslateInstance;

    std::vector<polygonPoint>  storage_origin_path;
    int num_size_C1 = local_C1path.size();
    int num_size_C2 = local_C2path.size();
    int num_size_C3 = local_C3path.size();

    for(int m = 0 ;m <  num_size_C1 - 1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C1path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        storage_origin_path.push_back(tempPt);
    }

    auto C1_LAST_PT = local_C1path[num_size_C1-1];
    auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
            C1_LAST_PT,
            IntersecPt,
            arriveLineHeading);
    tempPt.pathPtType_ = pathPtType::SWITCHPT;
    storage_origin_path.push_back(tempPt);

    for(int m =0 ;m <  num_size_C2 -1;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C2path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::BACKWARD;
        storage_origin_path.push_back(tempPt);
    }

    auto C2_LAST_PT = local_C2path[num_size_C2-1];
    auto tempPt1 = normalMatrixTranslateInstance.reverseRotatePoint(
            C2_LAST_PT,
            IntersecPt,
            arriveLineHeading);
    tempPt1.pathPtType_ = pathPtType::SWITCHPT;
    storage_origin_path.push_back(tempPt1);

    for(int m =0 ;m <  num_size_C3;m++){
        auto tempPt = normalMatrixTranslateInstance.reverseRotatePoint(
                local_C3path[m],
                IntersecPt,
                arriveLineHeading);
        tempPt.pathPtType_ = pathPtType::FORWARD;
        storage_origin_path.push_back(tempPt);
    }
    backshape_fishnail_curve_path_[curvePt] = storage_origin_path;
}





void pathPolygonPlan::cgalComputeParallelCurveMap(){
    int num =  cgalbackShape_keypoints_.size();
    int number_last = cgalbackShape_keypoints_[num-1].size();
    for(int i = 1;i < number_last; i++){
        //计算指定点的前后弯道关键点
        ridgeKeypoint tempPt;
        tempPt.start_dis = SET_STARTTURN_DISTANCE;
        tempPt.end_dis  = SET_ENDTURN_DISTANCE;
        auto forward_last_points =               //注意点的顺序
                common::commonMath::computeForwardAndBackPoints(
                        cgalbackShape_keypoints_[num-1],
                        cgalbackShape_keypoints_[num-1][i]);
        tempPt.start_curve_point =
                common::commonMath::findPointOnSegment(
                        forward_last_points[0],
                        cgalbackShape_keypoints_[num-1][i],
                        SET_STARTTURN_DISTANCE,
                        false);
        tempPt.end_curve_point =
                common::commonMath::findPointOnSegment(
                        cgalbackShape_keypoints_[num-1][i],
                        forward_last_points[1],
                        SET_ENDTURN_DISTANCE,
                        true);
        backshape_keypts_info_[cgalbackShape_keypoints_[num-1][i]] = tempPt;
    }
}

void pathPolygonPlan::cgalComputeBRidgeKeyPointsLeave(){
    polygonPoint specify_point;
    int number_m  =  cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1].size();
    specify_point =  cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-1];
    LOG(INFO) << "the last ridege entrance point is : ("
              << specify_point.x
              << ","
              << specify_point.y
              << ")";
    //计算指定点的heading
    auto spec_pt_forwardpt = cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-2];
    specify_point.heading = atan2(specify_point.y - spec_pt_forwardpt.y,
                                  specify_point.x - spec_pt_forwardpt.x);
    cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-1].heading =
            specify_point.heading;

    //利用排好序的索引
    for(int m = 0;m < move_pts_line_B_all_1_.size();m++ ){
        //根据line1和line2 确定哪个距离最近
        double distance1 =
                common::commonMath::distance2(specify_point,move_pts_line_B_all_1_[m][0]);
        double distance2 =
                common::commonMath::distance2(specify_point,move_pts_line_B_all_2_[m][0]);
        double distance3 =
                common::commonMath::distance2(specify_point,
                                              move_pts_line_B_all_1_[m][move_pts_line_B_all_1_[m].size()-1]);
        double distance4 =
                common::commonMath::distance2(specify_point,
                                              move_pts_line_B_all_2_[m][move_pts_line_B_all_2_[m].size()-1]);

        std::vector<double> stor_distance;
        stor_distance.push_back(distance1);
        stor_distance.push_back(distance2);
        stor_distance.push_back(distance3);
        stor_distance.push_back(distance4);

        LOG(INFO) << "move pts line1 distance is ： "  <<  distance1;
        LOG(INFO) << "move pts line2 distance is :  "  <<  distance2;
        LOG(INFO) << "move pts line1 end distance is ： "  <<  distance3;
        LOG(INFO) << "move pts line2 end distance is :  "  <<  distance4;

        double min_dis = DBL_MAX;
        int spec_number;
        for(int i = 0;i < 4;i++){
            if(min_dis > stor_distance[i]){
                min_dis = stor_distance[i];
                spec_number = i;
            }
        }
        spec_number +=1;
        LOG(INFO) <<"the spec number is : " << spec_number;
        std::vector<polygonPoint> storage_last_keypts;

        switch(spec_number){
            case 1: {
                LOG(INFO) << "the situation belong to minimum distance to line1 front !";
                for (int i = 0; i < move_pts_line_B_all_1_[m].size(); i++) {
                    if (i % 2 == 0) {
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                    } else {
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                    }
                }
                break;
            }
            case 2:{
                LOG(INFO) << "the situation belong to minimum distance to line2 front !";
                for(int i = 0;i < move_pts_line_B_all_1_[m].size();i++){
                    if(i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                    }
                }
                break;
            }
            case 3:{
                std::reverse(move_pts_line_B_all_1_[m].begin(),move_pts_line_B_all_1_[m].end());
                std::reverse(move_pts_line_B_all_2_[m].begin(),move_pts_line_B_all_2_[m].end());
                LOG(INFO) << "the situation belong to minimum distance to line1 back !";
                for(int i = 0;i < move_pts_line_B_all_1_[m].size();i++){
                    if( i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                    }
                }
                break;
            }
            case 4:{
                std::reverse(move_pts_line_B_all_1_[m].begin(),move_pts_line_B_all_1_[m].end());
                std::reverse(move_pts_line_B_all_2_[m].begin(),move_pts_line_B_all_2_[m].end());
                LOG(INFO) << "the situation belong to minimum distance to line2 back!";
                for(int i = 0;i < move_pts_line_B_all_1_[m].size();i++){
                    if(i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_line_B_all_1_[m][i]);
                        storage_last_keypts.push_back(move_pts_line_B_all_2_[m][i]);
                    }
                }
                break;
            }
            default:{
                LOG(INFO) << "the situation no  considered !";
                break;
            }
        }
        //此处需要添加平行线的heading计算部分
        cgalComputeParallelLinesHeading(storage_last_keypts);
        cgalbackShape_keypoints_.push_back(storage_last_keypts);
    }
}

void pathPolygonPlan::cgalComputeRidgeKeyPointsLeave(){
    polygonPoint specify_point;
    int number_m  =  cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1].size();
    specify_point =  cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-1];
    LOG(INFO) << "the last ridege entrance point is : ("
              << specify_point.x
              << ","
              << specify_point.y
              << ")";
    //计算指定点的heading
    auto spec_pt_forwardpt = cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-2];
    specify_point.heading = atan2(specify_point.y - spec_pt_forwardpt.y,
                                  specify_point.x - spec_pt_forwardpt.x);
    cgalbackShape_keypoints_[cgalbackShape_keypoints_.size()-1][number_m-1].heading =
                                                                 specify_point.heading;
    //根据line1和line2 确定哪个距离最近
    double distance1 =
            common::commonMath::distance2(specify_point,move_pts_line_1_[0]);
    double distance2 =
            common::commonMath::distance2(specify_point,move_pts_line_2_[0]);
    double distance3 =
            common::commonMath::distance2(specify_point,
                    move_pts_line_1_[move_pts_line_1_.size()-1]);
    double distance4 =
            common::commonMath::distance2(specify_point,
                    move_pts_line_2_[move_pts_line_2_.size()-1]);

    std::vector<double> stor_distance;
    stor_distance.push_back(distance1);
    stor_distance.push_back(distance2);
    stor_distance.push_back(distance3);
    stor_distance.push_back(distance4);

    LOG(INFO) << "move pts line1 distance is ： "  <<  distance1;
    LOG(INFO) << "move pts line2 distance is :  "  <<  distance2;
    LOG(INFO) << "move pts line1 end distance is ： "  <<  distance3;
    LOG(INFO) << "move pts line2 end distance is :  "  <<  distance4;

    double min_dis = DBL_MAX;
    int spec_number;
    for(int i = 0;i < 4;i++){
        if(min_dis > stor_distance[i]){
            min_dis = stor_distance[i];
            spec_number = i;
        }
    }
    spec_number +=1;
    LOG(INFO) <<"the spec number is : " << spec_number;
    std::vector<polygonPoint> storage_last_keypts;

    switch(spec_number){
        case 1: {
            LOG(INFO) << "the situation belong to minimum distance to line1 front !";
            for (int i = 0; i < move_pts_line_1_.size(); i++) {
                if (i % 2 == 0) {
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                } else {
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }
            }
            break;
        }
        case 2:{
            LOG(INFO) << "the situation belong to minimum distance to line2 front !";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }
            }
            break;
        }
        case 3:{
            std::reverse(move_pts_line_1_.begin(),move_pts_line_1_.end());
            std::reverse(move_pts_line_2_.begin(),move_pts_line_2_.end());
            LOG(INFO) << "the situation belong to minimum distance to line1 back !";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if( i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }
            }
            break;
        }
        case 4:{
            std::reverse(move_pts_line_1_.begin(),move_pts_line_1_.end());
            std::reverse(move_pts_line_2_.begin(),move_pts_line_2_.end());
            LOG(INFO) << "the situation belong to minimum distance to line2 back!";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }
            }
            break;
        }
        default:{
            LOG(INFO) << "the situation no  considered !";
            break;
        }
    }
    //此处需要添加平行线的heading计算部分
    cgalComputeParallelLinesHeading(storage_last_keypts);
    cgalbackShape_keypoints_.push_back(storage_last_keypts);
}

std::vector<std::vector<polygonPoint>>  pathPolygonPlan::cgalGetBackShapeKeyPoints(){
    return cgalbackShape_keypoints_;
}

std::vector<polygonPoint> pathPolygonPlan::cgalGetBackShapeSkeletonPts(){
    return storage_keypts_inner_skeleton_;
}


const std::vector<std::vector<polygonPoint>>  pathPolygonPlan::getInsertedPolygonPointsIncrease() const{
    return  insertedPtToNarrowPolygon_;
}


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
        //分类处理
        switch(last_polys_type_){
            case aiforce::Route_Planning::lastPolyIdentify::POLY_FOUR_AND_THREE:{
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
                    //这里需要将倒数第二笼顺序更新一下
                    int num = backShape_keypoints_.size();
                    int num_ridge = backShape_keypoints_[num-1].size();
                    auto  second_pts =
                            common::commonMath::updatePolySequenceOrdered(
                                    backShape_keypoints_[num-1][num_ridge-1],
                                    storageNarrowPolygonPoints_[count_narrow_polygon_numbers_-2]);
                    backShape_keypoints_.push_back(second_pts);
                }
               computeLastRidgeKeyPoints4and3(fabs(diff_narrow_and_intsPts));
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

void  pathPolygonPlan::computeLastRidgeKeyPoints4and3(int count_flag){
      int number = record_poly_index_;
      polygonPoint specify_point;
      std::vector<polygonPoint> temp_points;
      std::vector<polygonPoint> filtered_points = deleteRepeatPolyPts(middle_points_polygon_[number-1]);
        if(count_flag == 2){
            int number_m  =  backShape_keypoints_[backShape_keypoints_.size()-1].size();
            specify_point =  backShape_keypoints_[backShape_keypoints_.size()-1][number_m-1];
        }else{
            specify_point = filtered_points[1];
        }

      LOG(INFO) << "the last ridege entrance point is : (" << specify_point.x << "," << specify_point.y << ")";
      specify_point.heading = atan2(filtered_points[1].y - filtered_points[0].y,
                                    filtered_points[1].x - filtered_points[0].x);
        //根据line1和line2 确定哪个距离最近
        double distance1 = common::commonMath::distance2(specify_point,move_pts_tangle_line_1_[0]);
        double distance2 = common::commonMath::distance2(specify_point,move_pts_tangle_line_2_[0]);
        double distance3 = common::commonMath::distance2(specify_point,move_pts_tangle_line_1_[move_pts_tangle_line_1_.size()-1]);
        double distance4 = common::commonMath::distance2(specify_point,move_pts_tangle_line_2_[move_pts_tangle_line_2_.size()-1]);

        std::vector<double> stor_distance;
        stor_distance.push_back(distance1);
        stor_distance.push_back(distance2);
        stor_distance.push_back(distance3);
        stor_distance.push_back(distance4);

        LOG(INFO) << "move pts line1 distance is ： "  <<  distance1;
        LOG(INFO) << "move pts line2 distance is :  "  <<  distance2;
        LOG(INFO) << "move pts line1 end distance is ： "  <<  distance3;
        LOG(INFO) << "move pts line2 end distance is :  "  <<  distance4;

        double min_dis = DBL_MAX;
        int spec_number;
        for(int i = 0;i < 4;i++){
            if(min_dis > stor_distance[i]){
                min_dis = stor_distance[i];
                spec_number = i;
            }
        }
        spec_number +=1;
        LOG(INFO) <<"the spec number is : " << spec_number;
        std::vector<polygonPoint> storage_last_keypts;
        switch(spec_number){
            case 1: {
                LOG(INFO) << "the situation belong to minimum distance to line1 front !";
                for (int i = 0; i < move_pts_tangle_line_1_.size(); i++) {
                    if (i % 2 == 0) {
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                    } else {
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                    }
                }
                break;
            }
            case 2:{
                LOG(INFO) << "the situation belong to minimum distance to line2 front !";
                for(int i = 0;i < move_pts_tangle_line_1_.size();i++){
                    if(i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                    }
                }
                break;
            }
            case 3:{
                std::reverse(move_pts_tangle_line_1_.begin(),move_pts_tangle_line_1_.end());
                std::reverse(move_pts_tangle_line_2_.begin(),move_pts_tangle_line_2_.end());
                LOG(INFO) << "the situation belong to minimum distance to line1 back !";
                for(int i = 0;i < move_pts_tangle_line_1_.size();i++){
                    if( i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                    }
                }
                break;
            }
            case 4:{
                std::reverse(move_pts_tangle_line_1_.begin(),move_pts_tangle_line_1_.end());
                std::reverse(move_pts_tangle_line_2_.begin(),move_pts_tangle_line_2_.end());
                LOG(INFO) << "the situation belong to minimum distance to line2 back!";
                for(int i = 0;i < move_pts_tangle_line_1_.size();i++){
                    if(i % 2 == 0){
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                    }else{
                        storage_last_keypts.push_back(move_pts_tangle_line_1_[i]);
                        storage_last_keypts.push_back(move_pts_tangle_line_2_[i]);
                    }
                }
                break;
            }
            default:{
                LOG(INFO) << "the situation no  considered !";
                break;
            }
          }
      backShape_keypoints_.push_back(storage_last_keypts);
}

void pathPolygonPlan::computeLastRidgeKeyPoints4and4(int count_flag){
     int number = record_poly_index_;
    polygonPoint specify_point;
    std::vector<polygonPoint> temp_points;
    std::vector<polygonPoint> filtered_points = deleteRepeatPolyPts(middle_points_polygon_[number-1]);

    if(count_flag == 2){
        int number_m  =  backShape_keypoints_[backShape_keypoints_.size()-1].size();
        specify_point =  backShape_keypoints_[backShape_keypoints_.size()-1][number_m-1];
    }else{
            specify_point = filtered_points[1];
    }
    LOG(INFO) << "the last ridege entrance point is : (" << specify_point.x << "," << specify_point.y << ")";
    specify_point.heading = atan2(filtered_points[1].y - filtered_points[0].y,
                                  filtered_points[1].x - filtered_points[0].x);

    //根据line1和line2 确定哪个距离最近
    double distance1 = common::commonMath::distance2(specify_point,move_pts_line_1_[0]);
    double distance2 = common::commonMath::distance2(specify_point,move_pts_line_2_[0]);
    double distance3 = common::commonMath::distance2(specify_point,move_pts_line_1_[move_pts_line_1_.size()-1]);
    double distance4 = common::commonMath::distance2(specify_point,move_pts_line_2_[move_pts_line_2_.size()-1]);

    std::vector<double> stor_distance;
    stor_distance.push_back(distance1);
    stor_distance.push_back(distance2);
    stor_distance.push_back(distance3);
    stor_distance.push_back(distance4);

    LOG(INFO) << "move pts line1 distance is ： "  <<  distance1;
    LOG(INFO) << "move pts line2 distance is :  "  <<  distance2;
    LOG(INFO) << "move pts line1 end distance is ： "  <<  distance3;
    LOG(INFO) << "move pts line2 end distance is :  "  <<  distance4;

    double min_dis = DBL_MAX;
    int spec_number;
    for(int i = 0;i < 4;i++){
        if(min_dis > stor_distance[i]){
            min_dis = stor_distance[i];
            spec_number = i;
        }
    }
    spec_number +=1;
    LOG(INFO) <<"the spec number is : " << spec_number;
    std::vector<polygonPoint> storage_last_keypts;
    switch(spec_number){
        case 1: {
            LOG(INFO) << "the situation belong to minimum distance to line1 front !";
            for (int i = 0; i < move_pts_line_1_.size(); i++) {
                if (i % 2 == 0) {
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                } else {
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }
            }
            break;
        }
        case 2:{
            LOG(INFO) << "the situation belong to minimum distance to line2 front !";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }
            }
            break;
        }
        case 3:{
            std::reverse(move_pts_line_1_.begin(),move_pts_line_1_.end());
            std::reverse(move_pts_line_2_.begin(),move_pts_line_2_.end());
            LOG(INFO) << "the situation belong to minimum distance to line1 back !";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if( i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }
            }
            break;
        }
        case 4:{
            std::reverse(move_pts_line_1_.begin(),move_pts_line_1_.end());
            std::reverse(move_pts_line_2_.begin(),move_pts_line_2_.end());
            LOG(INFO) << "the situation belong to minimum distance to line2 back!";
            for(int i = 0;i < move_pts_line_1_.size();i++){
                if(i % 2 == 0){
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                }else{
                    storage_last_keypts.push_back(move_pts_line_1_[i]);
                    storage_last_keypts.push_back(move_pts_line_2_[i]);
                }
            }
            break;
        }
        default:{
            LOG(INFO) << "the situation no  considered !";
            break;
        }
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
    //删除重复点位
    std::vector<polygonPoint>  storate_allpts;
    for(const auto & p : storage_points){
        if(std::find(storate_allpts.begin(),storate_allpts.end(),p)
           == storate_allpts.end()){
            storate_allpts.push_back(p);
        }
    }
    return storate_allpts;
}
}
}
