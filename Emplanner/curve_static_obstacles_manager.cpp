//
// Created by zzm on 23-11-27.
//

#include "Emplanner/curve_static_obstacles_manager.h"

curveStaticObstaclesManager::curveStaticObstaclesManager(std::vector<std::vector<polygonPoint>>  & originPath,
                                                         std::vector<std::vector<polygonPoint>> &polygonPts){
     //原始关键点路径 、 障碍物的轮廓
     auto numbers_pts = originPath.size();
    //针对关键点路径做碰撞检测
    for(int i = 0;i < originPath.size(); i++) {
        for (int j = 0;j < originPath[i].size() - 2;j++){
            std::vector<polygonPoint> path_line;
            std::vector<polygon> obstacle_polygons;      //选中的障碍物
            std::vector<polygonPoint> centroid_pts;      //障碍物质心
            bool crash_flag = false;
            path_line.push_back(originPath[i][j]);
            path_line.push_back(originPath[i][j+1]);
            //针对障碍物们做碰撞检测
            computeObstacleCrashCheck(
                    polygonPts,
                    path_line,
                    obstacle_polygons,
                    centroid_pts);
            if(obstacle_polygons.size()){
                crash_flag = true;
            }
            std::vector<polygonPoint> consider_static_obstacles_pts;
            if (crash_flag) {
                for(int  i = 0 ;i < obstacle_polygons.size();i++){
                    consider_static_obstacles_pts = computeReferenceLine(obstacle_polygons[i],
                                                                         centroid_pts[i],
                                                                         path_line);

                    if (PJPO_USE_SWITCH) {
                        dealPJPO(consider_static_obstacles_pts,
                                 obstacle_polygons[i],
                                 centroid_pts[i],
                                 path_line);
                    }
                }
            }
        }
    }
}


std::vector<polygonPoint> curveStaticObstaclesManager::computeReferenceLine(
                                                const polygon& obstacle_polygon,
                                                const polygonPoint & centroid_pt,
                                                const std::vector<polygonPoint> & path_line){

    //计算出三段对应的参考线
    std::vector<polygonPoint>  reference_pts1,reference_pts2,reference_pts3;
    //整合所有参考线
    std::vector<polygonPoint>  reference_allpts;

    if( !USE_REFERENCE_CENTER_LINE ) {
        computeOriginReferenceLinePts(
                obstacle_polygon,
                centroid_pt,
                path_line,
                reference_pts1,
                reference_pts2,
                reference_pts3,
                reference_allpts);
        std::cout << " dont use reference center line ! " << std::endl;

    } else {
        computeOriginReferenceCenterLinePts(
                obstacle_polygon,
                centroid_pt,
                path_line,
                reference_pts1,
                reference_pts2,
                reference_pts3,
                reference_allpts);
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
    auto  consider_static_obstacles_pts = smoothFem.get_smoothed_pts();
    return consider_static_obstacles_pts;
}

void curveStaticObstaclesManager::computeOriginReferenceLinePts(
                                                                const polygon& obstacle_polygon,
                                                                const polygonPoint&  centroid_pt,
                                                                const std::vector<polygonPoint> & path_line,
                                                                std::vector<polygonPoint> & reference_pts1,
                                                                std::vector<polygonPoint> & reference_pts2,
                                                                std::vector<polygonPoint> & reference_pts3,
                                                                std::vector<polygonPoint> & reference_allpts){
    std::vector<polygonPoint>  work_line;
    work_line.push_back(path_line[0]);
    work_line.push_back(path_line[1]);
   //polygonPoint centroid_pt(centroid.x(),centroid.y());

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

    //保证等间隔取点
    std::vector<polygonPoint> line_diff_use;
    line_diff_use.push_back(A3);
    line_diff_use.push_back(A4);
    auto reference_diff_allpts = common::commonMath::densify2(
            line_diff_use,
            REFERENCE_DIFF_NUMBER);
    double dis_s = 0;
    for(int i = 0; i < reference_diff_allpts.size();i++ ){
        double dis_2pt = common::commonMath::distance2(A3,reference_diff_allpts[i]);
        if( dis_2pt < A1_DISTANCE ){
            reference_pts1.push_back(reference_diff_allpts[i]);
        } else if ( dis_2pt >= A1_DISTANCE && dis_2pt < A3_DISTANCE + A2_DISTANCE){
            reference_pts2.push_back(reference_diff_allpts[i]);
        } else {
            reference_pts3.push_back(reference_diff_allpts[i]);
        }
    }

    //更新reference_pts2
    //计算C、D点
    polygonPoint vector_origin_line(work_line[1].x - work_line[0].x,
                                    work_line[1].y - work_line[0].y);
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
    }
    std::cout << "the intersectionGeometry1 size is : "
              << intersectionGeometry1.size()
              << std::endl;

    //计算障碍物到中心线的最大距离
    //1.将三段中心参考线转换为离散path
    DiscretizedPath anchor_path;
    transOriginPathToDiscretizedPath(reference_diff_allpts,anchor_path);
    //2.计算障碍物的l距离
    double obs_min_l(std::numeric_limits<double>::max());
    double obs_max_l(std::numeric_limits<double>::lowest());
    for(auto it = obstacle_polygon.outer().begin();
             it != obstacle_polygon.outer().end();
             it ++){
        slPoint slPointtemp;
        anchor_path.XYToSL(math::Vec2d(it->x(),it->y()),&slPointtemp);
        obs_min_l = std::fmin(obs_min_l,slPointtemp.l());
        obs_max_l = std::fmax(obs_max_l,slPointtemp.l());
    }
    //3.根据绕障规则确定需要移动的距离
    double move_thr;
    double distance_virtual_line;
    if(!SIDE_PASS_CHOOSE){
        if(obs_max_l >= 0 && obs_min_l >=0){
            distance_virtual_line =  SAFE_OBSTACLE_THR_ZERO + RIDGE_WIDTH_LENGTH/2 - fabs(obs_min_l);
        }else if( obs_max_l > 0 && obs_min_l < 0){
            distance_virtual_line = SAFE_OBSTACLE_THR_ZERO + RIDGE_WIDTH_LENGTH/2 + fabs(obs_min_l);
        }else if(obs_max_l < 0  && obs_min_l < 0){
            distance_virtual_line = SAFE_OBSTACLE_THR_ZERO + fabs(obs_min_l) + RIDGE_WIDTH_LENGTH/2;
        }
    } else {
       if(obs_max_l >= 0 && obs_min_l >=0){
           distance_virtual_line = SAFE_OBSTACLE_THR_ZERO + RIDGE_WIDTH_LENGTH/2 + fabs(obs_max_l);
       }else if(obs_max_l > 0 && obs_min_l < 0){
           distance_virtual_line = SAFE_OBSTACLE_THR_ZERO + RIDGE_WIDTH_LENGTH/2 + fabs(obs_max_l);
       }else if(obs_max_l < 0 && obs_min_l < 0){
           distance_virtual_line = SAFE_OBSTACLE_THR_ZERO + RIDGE_WIDTH_LENGTH/2 - fabs(obs_max_l);
       }
    }

    std::cout << "the distance virtual line is : "
              << distance_virtual_line
              << std::endl;
    auto reference_pts2_center = reference_pts2;
    reference_pts2 = common::commonMath::computePtsToOrderedDirectionMove(
            reference_pts2,
            vector_vertical_line,
            distance_virtual_line);

#ifdef DEBUG_STATIC_OBSTACLE
    std::ofstream  streamLine;
    streamLine.open("/home/zzm/Desktop/test_path_figure-main/src/reference_pts2.txt",std::ios::out);
    for(auto i : reference_pts2){
        streamLine << " " << i.x;
    }
    streamLine << std::endl;
    for(auto j : reference_pts2){
        streamLine << " " << j.y;
    }
    streamLine << std::endl;
#endif

    for(auto i : reference_pts1){
        reference_allpts.push_back(i);
    }

    for(int i = 0;i < reference_pts2.size() ;i++){
        reference_allpts.push_back(reference_pts2[i]);
    }

    for(auto i : reference_pts3){
        reference_allpts.push_back(i);
    }

}

void curveStaticObstaclesManager::transOriginPathToDiscretizedPath(
                                    const std::vector<polygonPoint> & origin_path,
                                    DiscretizedPath & anchor_path){

    std::vector<std::pair<double, double>> trans_pts;
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;

    for(auto i : origin_path){
        trans_pts.push_back({i.x,i.y});
    }

    computePathProfileInfo::ComputePathProfile(
            trans_pts,
            &headings,
            &accumulated_s,
            &kappas,
            &dkappas);

    for(int i = 0 ; i <  origin_path.size();i++){
        PathPoint  temp(origin_path[i].x,
                        origin_path[i].y,
                        0,
                        accumulated_s[i],
                        headings[i],
                        headings[i],
                        kappas[i],
                        dkappas[i],
                        0);
        anchor_path.push_back(temp);
    }

}

void curveStaticObstaclesManager::computeOriginReferenceCenterLinePts(
        const polygon& obstacle_polygon,
        const polygonPoint&  centroid_pt,
        const std::vector<polygonPoint> & path_line,
        std::vector<polygonPoint> & reference_pts1,
        std::vector<polygonPoint> & reference_pts2,
        std::vector<polygonPoint> & reference_pts3,
        std::vector<polygonPoint> & reference_allpts){

    std::vector<polygonPoint>  work_line;
    work_line.push_back(path_line[0]);
    work_line.push_back(path_line[1]);

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

    //保证等间隔取点
    std::vector<polygonPoint> line_diff_use;
    line_diff_use.push_back(A3);
    line_diff_use.push_back(A4);

    auto reference_diff_allpts = common::commonMath::densify2(
            line_diff_use,
            40);
    double dis_s = 0;
    for(int i = 0; i < reference_diff_allpts.size();i++ ){
        double dis_2pt = common::commonMath::distance2(A3,reference_diff_allpts[i]);
        if(dis_2pt < A1_DISTANCE){
            reference_pts1.push_back(reference_diff_allpts[i]);
        }else if( dis_2pt >= A1_DISTANCE && dis_2pt < A3_DISTANCE + A2_DISTANCE){
            reference_pts2.push_back(reference_diff_allpts[i]);
        }else{
            reference_pts3.push_back(reference_diff_allpts[i]);
        }
    }

    for(auto i : reference_pts1){
        reference_allpts.push_back(i);
    }

    for(int i = 0;i < reference_pts2.size() ;i++){
        reference_allpts.push_back(reference_pts2[i]);
    }

    for(auto i : reference_pts3){
        reference_allpts.push_back(i);
    }

}


bool curveStaticObstaclesManager::curveGeneratePathFromDiscretePts(
        const DiscretizedPath& anchor_pts,
        DiscretizedPath& pathProfile){
    if(anchor_pts.empty()){
        std::cout << "anchor points empty." << std::endl;
        return false;
    }

    std::vector<std::pair<double,double>>  temp_storage;
    for(double s = 0.0; s < anchor_pts.Length(); s+= DIFF_PTS){
        std::pair<double,double>  tempPoint{
            anchor_pts.GetPathPtFromS(s).x(),
            anchor_pts.GetPathPtFromS(s).y()};
        temp_storage.push_back(tempPoint);
    }
    //添加最后一个点
    temp_storage.push_back({anchor_pts[anchor_pts.size()-1].x(),
                            anchor_pts[anchor_pts.size()-1].y()});
    //计算对应点位的其他信息
    std::vector<double>  headings;
    std::vector<double>  accumulated_s;
    std::vector<double>  kappas;
    std::vector<double>  dkappas;
    computePathProfileInfo::ComputePathProfile(
            temp_storage,
            &headings,
            &accumulated_s,
            &kappas,
            &dkappas);

    for(int i = 0;i < temp_storage.size();i++){
        PathPoint  tempPt(
                temp_storage[i].first,
                temp_storage[i].second,
                0,
                accumulated_s[i],
                headings[i],
                headings[i],
                kappas[i],
                dkappas[i],
                0);
        pathProfile.push_back(tempPt);
    }
    return true;
}




void curveStaticObstaclesManager::computeObstacleCrashCheck(
                         const  std::vector<std::vector<polygonPoint>> &polygonPts,
                         const  std::vector<polygonPoint> & path_line,
                         std::vector<polygon>& obstacle_polygons,
                         std::vector<polygonPoint> & centroid_pts){
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
    // ls.push_back(point(11.25,53.3));
    // ls.push_back(point(97.658,487));
    ls.push_back(point(path_line[0].x,path_line[0].y));
    ls.push_back(point(path_line[1].x,path_line[1].y));

    boost::geometry::buffer(ls, result,
                            distance_strategy,side_strategy,
                            join_strategy, end_strategy, circle_strategy);

    //std::cout << "expend line wkt style is : "
    //<< boost::geometry::wkt(result.front())
    //<< std::endl;

    //判断扩展的多边形与障碍物多边形们是否相交
    polygon  obstacle_polygonTemp;
    polygonPoint centroid_pt_temp;
    for(int  i = 0;i < polygonPts.size(); i++){
        for(auto j : polygonPts[i]){
            obstacle_polygonTemp.outer().push_back(point(j.x,j.y));
        }
//        std::deque<polygon> intersectionGeometry;
////        boost::geometry::intersection(result,obstacle_polygon,intersectionGeometry);
         bool flag =  boost::geometry::intersects(result,obstacle_polygonTemp);
//        std::cout << "the obstacle is : "
//                  << boost::geometry::wkt(obstacle_polygon.outer())
//                  << std::endl;
//        std::cout << "the obstalce size is ： "
//                  << flag
//                  << std::endl;
        if( flag ){

            std::cout << "expend line wkt style is : "
                      << boost::geometry::wkt(result.front())
                      << std::endl;

            std::cout << "the obstacle is : "
                      << boost::geometry::wkt(obstacle_polygonTemp.outer())
                      << std::endl;

            //计算多边形的质心
            point centroid;
            boost::geometry::centroid(obstacle_polygonTemp, centroid);

            std::cout << "the  polygon centroid is :"
                      << "["
                      << centroid.x()
                      << ","
                      << centroid.y()
                      << "]"
                      << std::endl;

            centroid_pt_temp.x = centroid.x();
            centroid_pt_temp.y = centroid.y();
            obstacle_polygons.push_back(obstacle_polygonTemp);
            obstacle_polygonTemp.clear();
            centroid_pts.push_back(centroid_pt_temp);
            continue;
        }
        obstacle_polygonTemp.clear();
    }
}




void curveStaticObstaclesManager::pjpoInitialize(
        const std::vector<polygonPoint> & consider_static_obstacles_pts,
        DiscretizedPath & pathProfile){
    //pjpo处理
    //对平滑过后的点重新进行差值
    //先获取其属性
    DiscretizedPath anchor_path;
    transOriginPathToDiscretizedPath(consider_static_obstacles_pts,anchor_path);
    curveGeneratePathFromDiscretePts(anchor_path,pathProfile);

#ifdef DEBUG_STATIC_OBSTACLE
    std::ofstream pathProfile1;
    pathProfile1.open("/home/zzm/Desktop/test_path_figure-main/src/pathProfile1.txt", std::ios::out);
    for (auto i : pathProfile) {
        pathProfile1 << " " << i.x();
    }

    pathProfile1 << std::endl;
    for (auto j : pathProfile) {
        pathProfile1 << " " << j.y();
    }

    pathProfile1 << std::endl;
#endif

}


void curveStaticObstaclesManager::pjpodealSLBoundary(
        const  DiscretizedPath & pathProfile,
        const  std::vector<polygonPoint> orderedPolygon,
        std::vector<std::pair<double, double>>* boundary){

    //利用障碍物对其进行sl边界计算
    SLBoundary obstacleSL_pts;
    pathProfile.GetSlBoundary(orderedPolygon,&obstacleSL_pts);

    auto start_idx = static_cast<size_t>((obstacleSL_pts.start_s() )/DIFF_PTS);
    auto end_idx = static_cast<size_t>((obstacleSL_pts.end_s() ) /DIFF_PTS);

    auto min_l = obstacleSL_pts.start_l();
    auto max_l = obstacleSL_pts.end_l();

    double lower_bound ;
    double upper_bound ;

    (*boundary).resize(pathProfile.size());

    for (int i = 0; i < pathProfile.size(); i++) {
        if(i >= start_idx && i <= end_idx){
            if(USE_REFERENCE_CENTER_LINE){
                if(!SIDE_PASS_CHOOSE){    //绕右
                    lower_bound =  LOWER_BOUND  - fabs(min_l) - RIDGE_WIDTH_LENGTH/2 -  PJPO_SAFE_OBSTACLE_THR;
                    upper_bound =  UPPER_BOUND - fabs(min_l) -  RIDGE_WIDTH_LENGTH/2 -  PJPO_SAFE_OBSTACLE_THR;
                } else {   //绕左
                    lower_bound =  LOWER_BOUND  + fabs(max_l) * 2  +  RIDGE_WIDTH_LENGTH/2  +  PJPO_SAFE_OBSTACLE_THR;
                    upper_bound =  UPPER_BOUND  + fabs(max_l) * 2  +  RIDGE_WIDTH_LENGTH/2 +  PJPO_SAFE_OBSTACLE_THR;
                }
            } else {
//                  lower_bound = LOWER_BOUND ;
//                  upper_bound = UPPER_BOUND ;
                if(!SIDE_PASS_CHOOSE) {    //绕右
                    if(fabs(min_l) < PJPO_SAFE_OBSTACLE_THR_FIRST){
                        double dis_thr = PJPO_SAFE_OBSTACLE_THR_FIRST - fabs(min_l);
                        if(dis_thr < 0.5){
                            lower_bound = LOWER_BOUND - 2;
                            upper_bound = UPPER_BOUND - 2;
                        }else{
                            lower_bound =  LOWER_BOUND  - fabs(dis_thr*2) * 2 - PJPO_SAFE_OBSTACLE_THR;
                            upper_bound =  UPPER_BOUND  - fabs(dis_thr*2) * 2 - PJPO_SAFE_OBSTACLE_THR;
                        }
                    } else {
                        lower_bound = LOWER_BOUND;
                        upper_bound = UPPER_BOUND;
                    }
                } else {                   //绕左
                    if(fabs(max_l) < PJPO_SAFE_OBSTACLE_THR_FIRST){
                        double dis_thr = PJPO_SAFE_OBSTACLE_THR_FIRST - fabs(max_l);
                        lower_bound = LOWER_BOUND + fabs(dis_thr) * 2 + PJPO_SAFE_OBSTACLE_THR;
                        upper_bound = UPPER_BOUND + fabs(dis_thr) * 2 + PJPO_SAFE_OBSTACLE_THR;
                    } else {
                        lower_bound =  LOWER_BOUND ;
                        upper_bound =  UPPER_BOUND ;
                    }
                }
            }
        } else {
            lower_bound = LOWER_BOUND;
            upper_bound = UPPER_BOUND;
        }
        (*boundary)[i].first  =  lower_bound ;  // 修改第i个元素的值
        (*boundary)[i].second =  upper_bound ;
    }

}


void curveStaticObstaclesManager::dealPJPO(
        const std::vector<polygonPoint> & consider_static_obstacles_pts,
        const polygon & obstacle_polygon,
        const polygonPoint & centroid_pt,
        const std::vector<polygonPoint> & path_line ) {
        DiscretizedPath pathProfile;
        std::vector<std::pair<double, double>> boundary;

        pjpoInitialize(consider_static_obstacles_pts,pathProfile);

        std::vector<polygonPoint> transd_polygon;
        for(auto i  =  obstacle_polygon.outer().begin();
            i != obstacle_polygon.outer().end();
            i++){
            transd_polygon.push_back(polygonPoint(i->x(),i->y()));
        }

        pjpodealSLBoundary(pathProfile,transd_polygon,&boundary);

        //调用pjpo算法实现sl坐标系下路径优化
        std::vector<double> opt_l;
        std::vector<double> opt_dl;
        std::vector<double> opt_ddl;

        std::array<double, 3> start_state = {
                0,
                0,
                0
        };

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

        std::vector<std::pair<double, double>> ddl_bounds;
        const double lat_acc_bound =
                std::tan(MAX_STEER_ANGLE * M_PI / 180.0) /
                WHEEL_BASE;
        for (int i = 0; i < pathProfile.size(); i++) {
            double kappa = pathProfile[i].kappa();
            double l_thr = -lat_acc_bound - kappa;
            double r_thr = lat_acc_bound + kappa;
            if(l_thr > r_thr){
                ddl_bounds.emplace_back(0,0);
            } else {
                ddl_bounds.emplace_back(
                        l_thr,
                        r_thr);
            }
        }

        customPJPO customPJPOM;
        std::vector<double>  path_reference_l;
        for(int i = 0;i < boundary.size(); i++){
            double value = (boundary[i].first + boundary[i].second)/2;
            path_reference_l.push_back(value);
        }

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

        std::vector<polygonPoint> storage_path;
        //将对应的s.l转到大地坐标系下
        for (int i = 0; i < pathProfile.size() ; i++) {
            polygonPoint temp;
            pathProfile.SLToXY(
                    pathProfile[i].s(),
                    opt_l[i],
                    temp);
            storage_path.push_back(temp);
        }

       storage_last_path_[path_line[0]] = storage_path;

#ifdef DEBUG_STATIC_OBSTACLE
        std::string test_PJPO_path1 = "/home/zzm/Desktop/test_path_figure-main/src/test_PJPO_path1.txt";
        auto &test_PJPO_pathM = common::Singleton::GetInstance<tractorPolyPathPrint>(test_PJPO_path1);
        test_PJPO_pathM.writePts(storage_path);
#endif
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

#ifdef DEBUG_STATIC_OBSTACLE
        std::string testCurve = "/home/zzm/Desktop/test_path_figure-main/src/test_pjpo_curve.txt";
        auto &test_pjpo_curve = common::Singleton::GetInstance<tractorPolyCurve>(testCurve);
        test_pjpo_curve.writePts(kappas1);

        for (int im = 1; im < storage_path.size(); im++) {
            tractorPolygonShow tractorPolygonShowInstance(im,
                                                          storage_path);
            auto tractorHeadPts = tractorPolygonShowInstance.getTractorPolygonHeadPts();
            auto tractorTailPts = tractorPolygonShowInstance.getTractorPolygonTailPts();
            std::string test1 = "/home/zzm/Desktop/test_path_figure-main/src/tractorObstaclesPJPO.txt";
            auto &tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint>(test1);
            tractorHeadPtsStream.writePts(tractorHeadPts, tractorTailPts);
        }
#endif

}


std::vector<polygonPoint> curveStaticObstaclesManager::getCurvePath(const polygonPoint orderedPt){
    auto result = storage_last_path_[orderedPt];
    return result;
}



