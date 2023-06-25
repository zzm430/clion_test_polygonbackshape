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
        if(i == 1){           //第1垄内缩垄宽/2,剩余垄内缩垄宽
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
        if(result.empty()) {
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
    for(auto it : backShape_keypoints_){
         temp_storage_pts = deleteRepeatPolyPts(it);
         filtered_backshape_keypoints_.push_back(temp_storage_pts);
    }
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
    int size_polygon = tempPolygon.size();
    LOG(INFO) << "the last polygon size is :" << size_polygon ;
    int number ;
    std::vector<polygonPoint>  end_polygon;
    for(int i = 0 ;i < size_polygon;i++){
        if(fabs(line_origin_entrance_[0].x - tempPolygon[i].x) < 0.1 &&
            fabs(line_origin_entrance_[0].y - tempPolygon[i].y) < 0.1 ){
            number = i;
            end_polygon.push_back(tempPolygon[i]);
            polygonPoint next;
            for(int j = 1;j <= size_polygon-1;j++){
                next = tempPolygon[(number+1)%size_polygon];
                end_polygon.push_back(next);
                number = number + 1;
            }
            end_polygon.push_back(tempPolygon[i]);
        }

    }
    insertedPtToNarrowPolygon_.push_back(end_polygon);
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
       fabs(diff_narrow_and_intsPts) == 0){
        LOG(INFO) << "the situation + the last ridge can finish plan !";
        //未包含最后一垄
        for(int i = 0;i < count_narrow_polygon_numbers_ -1;i++){
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
        //添加最后一笼
        backShape_keypoints_.push_back(middle_points_polygon_[count_narrow_polygon_numbers_-1]);
    }else if(fabs(diff_narrow_and_intsPts) == 2){
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
