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
            A1_DISTANCE,
            false);
    auto A3 = common::commonMath::findPointOnSegment(
            work_line[0],
            projection_pt,
            A3_DISTANCE,
            false);
    auto A2 = common::commonMath::findPointOnSegment(
            projection_pt,
            work_line[1],
            A2_DISTANCE,
            true);
    auto A4 = common::commonMath::findPointOnSegment(
            projection_pt,
            work_line[1],
            A4_DISTANCE,
            true);

//    std::vector<polygonPoint>  line1;
//    line1.push_back(A3);
//    line1.push_back(A1);
//
//    std::vector<polygonPoint>  line2;
//    line2.push_back(polygonPoint(A2));
//    line2.push_back(polygonPoint(A4));
//
//    std::vector<polygonPoint>  line3;
//    line3.push_back(polygonPoint(A1));
//    line3.push_back(polygonPoint(A2));



    //保证等间隔取点
    std::vector<polygonPoint> line_diff_use;
    line_diff_use.push_back(A3);
    line_diff_use.push_back(A4);
    auto reference_diff_allpts = common::commonMath::densify2(
                   line_diff_use,
                   40);
    double dis_s = 0;
    for(int i = 0; i < reference_diff_allpts.size();i++){
         double dis_2pt = common::commonMath::distance2(A3,reference_diff_allpts[i]);
         if(dis_2pt < A1_DISTANCE){
             reference_pts1.push_back(reference_diff_allpts[i]);
         }else if( dis_2pt >= A1_DISTANCE && dis_2pt < A3_DISTANCE + A2_DISTANCE){
             reference_pts2.push_back(reference_diff_allpts[i]);
         }else{
             reference_pts3.push_back(reference_diff_allpts[i]);
         }
    }


    //更新reference_pts2
    //计算C、D点
    polygonPoint vector_origin_line(work_line[1].x - work_line[0].x,
                                    work_line[1].y - work_line[0].x);
    polygonPoint vector_vertical_line(-vector_origin_line.y,vector_origin_line.x);

    if(!SIDE_PASS_CHOOSE){
        vector_vertical_line.x = vector_vertical_line.x * (-1);
        vector_vertical_line.y = vector_vertical_line.y * (-1);
    }

    //计算虚拟垂直线段
    std::vector<polygonPoint>  vertical_line;
    polygonPoint  vertical_pt1,vertical_pt2;
    double distance  = VIRTUAL_CENTROID_LINE;
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


    //将第三条参考线上的点按照指定垂直中心线的方向平移
    //假如往C方向平移
    std::cout <<"the distance C is : " << distanceC << std::endl;
    double distance_virtual_line  = RIDGE_WIDTH_LENGTH/2 + distanceC + SAFE_OBSTACLE_THR;
    if(!SIDE_PASS_CHOOSE){
        distance_virtual_line = RIDGE_WIDTH_LENGTH/2 + distanceD + SAFE_OBSTACLE_THR;
    }
    auto reference_pts2_center = reference_pts2;
    reference_pts2 = common::commonMath::computePtsToOrderedDirectionMove(
            reference_pts2,
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

     //整合所有参考线
     std::vector<polygonPoint>  reference_allpts;
     for(auto i : reference_pts1){
         reference_allpts.push_back(i);
     }

     for(int i = 0;i < reference_pts2.size() ;i++){
         reference_allpts.push_back(reference_pts2[i]);
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

     if(PJPO_USE_SWITCH) {
         //pjpo处理
         //针对参考线A3到A4初始参考线进行差值
         std::vector<polygonPoint> inter_pts;
         for (auto i : reference_pts1) {
             inter_pts.push_back(i);
         }

         for (auto i: reference_pts2_center) {
             inter_pts.push_back(i);
         }

         for (auto i: reference_pts3) {
             inter_pts.push_back(i);
         }

         //计算点位初始值
         std::vector<std::pair<double, double>> trans_pts;
         for (auto i : inter_pts) {
             trans_pts.push_back({i.x, i.y});
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


         //将平滑过的参考线转到sl坐标系下
         std::vector<std::pair<double, double>> transd_reference_sl_pts;


         //计算中线的路径信息
         DiscretizedPath center_refer_path;
         std::vector<polygonPoint> temp_storage_ori_path;
         for (int i = 0; i < inter_pts.size(); i++) {
             PathPoint temp(
                     inter_pts[i].x,
                     inter_pts[i].y,
                     0,
                     accumulated_s[i],
                     headings[i],
                     headings[i],
                     kappas[i],
                     dkappas[i],
                     0);
             center_refer_path.push_back(temp);
         }


         //计算带有障碍物的上下界
         std::vector<double> compute_l_path;
         for (int i = 0; i < consider_static_obstacles_pts.size(); i++) {
             math::Vec2d temp(consider_static_obstacles_pts[i].x,
                              consider_static_obstacles_pts[i].y);
             double accumulate_s;
             double lateral;
             double min_distance;
             int min_index;
             center_refer_path.GetProjection(temp,
                                             &accumulate_s,
                                             &lateral,
                                             &min_distance,
                                             &min_index);
             compute_l_path.push_back(lateral);
         }

         //得到偏移的参考线之后，需要和原始参考线的点位一一对应

         //调用pjpo算法实现sl坐标系下路径优化

         std::vector<double> opt_l;
         std::vector<double> opt_dl;
         std::vector<double> opt_ddl;

         std::array<double, 3> start_state = {
                 0,
                 0,
                 0};

         std::array<double, 5> w = {
                 1000.0,
                 1.0,
                 1.0,
                 1,
                 100000
         };

         std::array<double, 3> end_state = {
                 0,
                 0,
                 0
         };

         std::cout << "the refer 1 size is : " << compute_l_path.size() << std::endl;
         std::cout << "the center size is : " << center_refer_path.size() << std::endl;
         std::cout << "the center size is : " << accumulated_s.size() << std::endl;

         std::vector<std::pair<double, double>> boundary(center_refer_path.size());

         double lower_bound ;
         double upper_bound ;

         for (int i = 0; i < boundary.size(); i++) {
             lower_bound = -7;
             upper_bound = 7;
             boundary[i] = std::make_pair(lower_bound, upper_bound);  // 修改第i个元素的值
         }


         //计算ddl的边界条件
         std::vector<double> headings_M;
         std::vector<double> accumulated_s_M;
         std::vector<double> kappas_M;
         std::vector<double> dkappas_M;

         computePathProfileInfo::ComputePathProfile(
                 trans_pts,
                 &headings_M,
                 &accumulated_s_M,
                 &kappas_M,
                 &dkappas_M);

         std::vector<std::pair<double, double>> ddl_bounds;
         const double lat_acc_bound =
                 std::tan(MAX_STEER_ANGLE * M_PI / 180.0) /
                 WHEEL_BASE;
         for (int i = 0; i < consider_static_obstacles_pts.size(); i++) {
             double kappa = kappas_M[i];
             ddl_bounds.emplace_back(
                     -lat_acc_bound - kappa,
                     lat_acc_bound + kappa);
         }


         customPJPO customPJPOM;

         auto path_reference_l = compute_l_path;
         if (customPJPOM.optimizePath(start_state,
                                      end_state,
                                      path_reference_l,
                                      true,
                                      boundary,
                                      ddl_bounds,
                                      w,
                                      4000,
                                      &opt_l,
                                      &opt_dl,
                                      &opt_ddl)) {
             auto optm_l = opt_l;
             for (auto i : optm_l) {
                 std::cout << i;
             }
             std::cout << std::endl;
             std::cout << optm_l.size() << std::endl;

         }

         //将对应的s.l转到大地坐标系下
         std::vector<polygonPoint> storage_path;
         for (int i = 0; i < center_refer_path.size(); i++) {
             polygonPoint temp;
             center_refer_path.SLToXY(
                     accumulated_s[i],
                     opt_l[i],
                     temp);
             storage_path.push_back(temp);
         }

         std::ofstream test_PJPO_path;
         test_PJPO_path.open("/home/zzm/Desktop/test_path_figure-main/src/test_PJPO_path.txt", std::ios::out);
         for (auto i : storage_path) {
             test_PJPO_path << " " << i.x;
         }

         test_PJPO_path << std::endl;
         for (auto j : storage_path) {
             test_PJPO_path << " " << j.y;
         }

         test_PJPO_path << std::endl;

         std::cout << "the bbbbb 2" << boost::geometry::wkt(result) << std::endl;


         std::vector<double> headings1;
         std::vector<double> accumulated_s1;
         std::vector<double> kappas1;
         std::vector<double> dkappas1;

         std::vector<std::pair<double, double>> trans_storage_path;

         for (auto i : storage_path) {
             trans_storage_path.push_back({i.x, i.y});
         }
         computePathProfileInfo::ComputePathProfile(
                 trans_storage_path,
                 &headings1,
                 &accumulated_s1,
                 &kappas1,
                 &dkappas1);


         std::ofstream test_pjpo_curve;
         test_pjpo_curve.open("/home/zzm/Desktop/test_path_figure-main/src/test_pjpo_curve.txt", std::ios::out);
         for (auto i : kappas1) {
             test_pjpo_curve << " " << i;
         }
         test_pjpo_curve << std::endl;


         for (int im = 1; im < storage_path.size(); im++) {
             tractorPolygonShow tractorPolygonShowInstance(im,
                                                           storage_path);
             auto tractorHeadPts = tractorPolygonShowInstance.getTractorPolygonHeadPts();
             auto tractorTailPts = tractorPolygonShowInstance.getTractorPolygonTailPts();
             std::string test1 = "/home/zzm/Desktop/test_path_figure-main/src/tractorObstaclesPJPO.txt";
             auto &tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint>(test1);
             tractorHeadPtsStream.writePts(tractorHeadPts, tractorTailPts);
         }
     }

}