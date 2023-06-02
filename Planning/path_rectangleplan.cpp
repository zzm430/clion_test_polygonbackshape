#include <Planning/path_rectangleplan.h>
#include <Geometry/reeds_shepp.h>

namespace aiforce{
namespace Route_Planning
{

//routing 初始化
void pathRectanglePlan::initiate(int use_get_curve_point,double distance){
   use_get_curve_point_ = use_get_curve_point;
   back_off_interval_dis_ = distance;
   judge_increase_points_direction_ = 0;
   bow_spacing_setting_ = BACKSHAPE_SPACE_SETTING;
   useGCSPoints_ =  USE_GCS;
   backShapeFlag_ = false;
}




std::unordered_map<int,std::vector<pathBasePlan::curvePoint>>  pathRectanglePlan::getCurvePoints(){
    return key_curve_points_;
}

//======================================================================
// 函数功能：根据得到的点位判断是否需要对生成的矩形进行裁剪
// 输入参数：逆时针的矩形坐标点
// 返回参数：逆时针经过裁剪的矩形坐标点
// 编辑日期：    2023/04/28
//======================================================================
std::vector<Point> pathRectanglePlan::keyPointsCoordinateoffset(std::vector<Point>& points){
     std::vector<Point>  temp_storage;
     switch(backShapeMode_){
        case baskShapeType::L_SHAPE_AND_BACKSHAPE:
             //第1垄的信息按照原来的样子进行设计，关键点位需要横向裁剪bow_spacing_setting_ * 2 * remainder_使得坐标变成整型处理
             //将矩形框进行偏移
             temp_storage.push_back(Point(points[0].x,
                                    points[0].y + bow_spacing_setting_/2));
             temp_storage.push_back(Point((traversals_numbers_ - 1) * bow_spacing_setting_ * 2,
                                          points[1].y + bow_spacing_setting_/2));
             temp_storage.push_back(Point((traversals_numbers_ - 1) * bow_spacing_setting_ * 2,
                                          points[2].y - bow_spacing_setting_/2));
             temp_storage.push_back(Point(points[3].x,
                                    points[3].y - bow_spacing_setting_/2));
             return temp_storage;
        case baskShapeType::FONT_SHAPE_AND_BACKSHAPE:
             {
            //第1垄信息按照原来的样子设计，关键点位需要
            double jizi_number =  transferd_points_[1].x -
                    (traversals_numbers_ -1 -1)*
                    bow_spacing_setting_*2 ;
            temp_storage.push_back(Point(points[0].x,
                                   points[0].y + bow_spacing_setting_/2));
            temp_storage.push_back(Point((traversals_numbers_ -1-1)* bow_spacing_setting_*2 ,
                                         points[1].y + bow_spacing_setting_/2));
            temp_storage.push_back(Point((traversals_numbers_ -1-1)* bow_spacing_setting_*2,
                                         points[2].y - bow_spacing_setting_/2));
            temp_storage.push_back(Point(points[3].x,
                                   points[3].y-bow_spacing_setting_/2));
            return temp_storage;
           }
        case baskShapeType::NORMAL_BACKSHAPE:
            {
            temp_storage.push_back(Point(points[0].x + bow_spacing_setting_/2,
                                   points[0].y + bow_spacing_setting_/2 ));
            temp_storage.push_back(Point(points[1].x - bow_spacing_setting_/2,
                                   points[1].y + bow_spacing_setting_/2 ));
            temp_storage.push_back(Point(points[2].x - bow_spacing_setting_/2,
                                   points[2].y - bow_spacing_setting_/2 ));
            temp_storage.push_back(Point(points[3].x + bow_spacing_setting_/2,
                                   points[3].y - bow_spacing_setting_/2 ));
            return temp_storage;
          }
        default:
            LOG(INFO) << "unknown back shape !";
            return temp_storage;
     }
}

//======================================================================
// 函数功能：根据矩形的脚点生成对应的关键点信息
// 输入参数：逆时针的矩形坐标点
// 返回参数：
// 编辑日期：    2023/04/28
//======================================================================
void  pathRectanglePlan::process(std::vector<Point>& points) {
    //存储起始的角点信息
    for(auto i  : points){
        key_points_.push_back(i);
    }

    for(int i = 1; i <= traversals_numbers_;i++ ){
        Point p_temp_1,p_temp_2,p_temp_3,p_temp_4;
        p_temp_1.y = points[0].y + bow_spacing_setting_*i;
        p_temp_1.x = points[0].x;
        points[0].x = points[0].x +bow_spacing_setting_;
        p_temp_2.y = points[1].y + bow_spacing_setting_*i;
        p_temp_2.x = points[1].x - bow_spacing_setting_*i;
        p_temp_3.y = points[2].y - bow_spacing_setting_*i;
        p_temp_3.x = points[2].x - bow_spacing_setting_*i;
        p_temp_4.y = points[3].y - bow_spacing_setting_*i;
        p_temp_4.x = points[3].x + bow_spacing_setting_*i;
        if(p_temp_1.x > p_temp_2.x) {
            LOG(INFO) << "the x-coordinates of two points meet beyond the thr!"
                      <<std::endl;
            Point temp_m;
            temp_m.x = key_points_[key_points_.size()-1].x;
            temp_m.y = key_points_[key_points_.size()-3].y;
            key_points_.push_back(temp_m);
            LOG(INFO) << "the number of key points for the glyph is :"
                      << key_points_.size();
            return;
        }else if(fabs(p_temp_1.x - p_temp_2.x) < 1e-10){
            key_points_.push_back(p_temp_1);
            LOG(INFO) << "trigger point_1 == point_2 this situation !";
            return;
        }else if(fabs(p_temp_3.x - p_temp_4.x) < 0.1  &&
                 fabs(p_temp_3.y - p_temp_4.y) < 0.1 ){  //在矩形宽度是垄宽*2的整数倍时触发
            key_points_.push_back(p_temp_1);
            key_points_.push_back(p_temp_2);
            key_points_.push_back(p_temp_3);
            LOG(INFO) << "trigger point_3 == point_4 this situation !";
            return;
        }
        key_points_.push_back(p_temp_1);
        key_points_.push_back(p_temp_2);
        key_points_.push_back(p_temp_3);
        key_points_.push_back(p_temp_4);
    }
}


//======================================================================
// 函数功能：根据生成的关键点信息，获取关键点生成弯道关键点，并做个映射
// 输入参数：
// 返回参数：
// 编辑日期：    2023/04/28
//======================================================================
void  pathRectanglePlan::processCurve(){
     //先对起始的四个点进行处理,排除起始点
     curvePoint curvePointTempStart,curvePointTempEnd;

    curvePointTempStart.x = key_points_[1].x -use_get_curve_point_;
    curvePointTempStart.y = key_points_[1].y;
    curvePointTempStart.heading = 0;
    curvePointTempEnd.x = key_points_[1].x;
    curvePointTempEnd.y = key_points_[1].y + use_get_curve_point_;
    curvePointTempEnd.heading = PI/2;
    key_curve_points_[0].push_back(curvePointTempStart);
    key_curve_points_[0].push_back(curvePointTempEnd);

    curvePointTempStart.x = key_points_[2].x ;
    curvePointTempStart.y = key_points_[2].y - use_get_curve_point_;
    curvePointTempStart.heading = PI/2;
    curvePointTempEnd.x = key_points_[2].x - use_get_curve_point_;
    curvePointTempEnd.y = key_points_[2].y ;
    curvePointTempEnd.heading = PI;
    key_curve_points_[1].push_back(curvePointTempStart);
    key_curve_points_[1].push_back(curvePointTempEnd);

    curvePointTempStart.x = key_points_[3].x + use_get_curve_point_;
    curvePointTempStart.y = key_points_[3].y ;
    curvePointTempStart.heading = PI;
    curvePointTempEnd.x = key_points_[3].x ;
    curvePointTempEnd.y = key_points_[3].y - use_get_curve_point_;
    curvePointTempEnd.heading = 3 * PI/2;
    key_curve_points_[2].push_back(curvePointTempStart);
    key_curve_points_[2].push_back(curvePointTempEnd);

    //处理剩下的点
    Point p1 ,p2, p3, p4;
    p1.x = key_points_[0].x;
    p1.y = key_points_[0].y;
    p2.x = key_points_[1].x;
    p2.y = key_points_[1].y;
    p3.x = key_points_[2].x;
    p3.y = key_points_[2].y;
    p4.x = key_points_[3].x;
    p4.y = key_points_[3].y;
    int k = 3;
    for(int i = 1; i <= traversals_numbers_;i++){
        Point p_temp_1,p_temp_2,p_temp_3,p_temp_4;

        if(k == key_points_.size()){
            return;
        }
        p_temp_1.y = p1.y + bow_spacing_setting_*i;
        p_temp_1.x = p1.x;
        curvePointTempStart.x = p_temp_1.x;
        curvePointTempStart.y  = p_temp_1.y + use_get_curve_point_;
        curvePointTempStart.heading = -PI/2;
        curvePointTempEnd.x = p_temp_1.x + use_get_curve_point_;
        curvePointTempEnd.y = p_temp_1.y;
        curvePointTempEnd.heading  = 0;
        key_curve_points_[k].push_back(curvePointTempStart);
        key_curve_points_[k].push_back(curvePointTempEnd);
        k = k + 1;

        if(k == key_points_.size()){
            return;
        }
        p1.x = p1.x +bow_spacing_setting_;
        p_temp_2.y = p2.y + bow_spacing_setting_*i;
        p_temp_2.x = p2.x - bow_spacing_setting_*i;
        curvePointTempStart.x = p_temp_2.x - use_get_curve_point_;
        curvePointTempStart.y  = p_temp_2.y ;
        curvePointTempStart.heading =  0;
        curvePointTempEnd.x =  p_temp_2.x ;
        curvePointTempEnd.y = p_temp_2.y + use_get_curve_point_;
        curvePointTempEnd.heading  = PI/2;
        key_curve_points_[k].push_back(curvePointTempStart);
        key_curve_points_[k].push_back(curvePointTempEnd);
        k = k + 1;

        if(k == key_points_.size()){
            return;
        }
        p_temp_3.y = p3.y - bow_spacing_setting_*i;
        p_temp_3.x = p3.x - bow_spacing_setting_*i;
        curvePointTempStart.x = p_temp_3.x ;
        curvePointTempStart.y  =p_temp_3.y - use_get_curve_point_;
        curvePointTempStart.heading =  PI/2;
        curvePointTempEnd.x =  p_temp_3.x -use_get_curve_point_ ;
        curvePointTempEnd.y = p_temp_3.y ;
        curvePointTempEnd.heading  = PI;
        key_curve_points_[k].push_back(curvePointTempStart);
        key_curve_points_[k].push_back(curvePointTempEnd);
        k = k + 1;

        if(k == key_points_.size()){
            return;
        }
        p_temp_4.y = p4.y - bow_spacing_setting_*i;
        p_temp_4.x = p4.x + bow_spacing_setting_*i;
        curvePointTempStart.x = p_temp_4.x +use_get_curve_point_ ;
        curvePointTempStart.y  = p_temp_4.y ;
        curvePointTempStart.heading =  PI;
        curvePointTempEnd.x = p_temp_4.x ;
        curvePointTempEnd.y = p_temp_4.y - use_get_curve_point_;
        curvePointTempEnd.heading  = 3*PI/2;
        key_curve_points_[k].push_back(curvePointTempStart);
        key_curve_points_[k].push_back(curvePointTempEnd);
        k = k + 1;

        if(k == key_points_.size()){
            return;
        }
    }
}



//计算垄数
void pathRectanglePlan::processComputeRidges() {
    if(backShapeMode_ == baskShapeType::L_SHAPE_AND_BACKSHAPE ||
            backShapeMode_ == baskShapeType::FONT_SHAPE_AND_BACKSHAPE){
          ridge_numbers_ = (key_points_.size()-2-1)/2 + 1;
          ridge_numbers_ = ridge_numbers_ + 1;
    }else{
          ridge_numbers_ = (key_points_.size()-2-1)/2 + 1;
    }
}


//计算遍历次数
void  pathRectanglePlan::processComputeTraversalNumbers() {
    double temp_number = bow_spacing_setting_ * 2;
    if(fmod(transferd_points_[1].x,temp_number) < 0.001){
        traversals_numbers_ = floor(transferd_points_[1].x/temp_number) + 1;
    }else {
        traversals_numbers_ = ceil(transferd_points_[1].x/temp_number);
    }

    LOG(INFO) << "the back shape actually traverse number is :"
              << traversals_numbers_;
}


double   pathRectanglePlan::double_mod(double x, double y){
    double q  = x / y;
    long int q_number = static_cast<long int>(q);
     return (x - static_cast<double>(q_number) * y) / y;
}



//计算矩形短边属于哪种情况的回字形并确定类型
void  pathRectanglePlan::processDecideBackShapeType() {
    double temp_number = bow_spacing_setting_ * 2;
    double mod = double_mod(transferd_points_[1].x,temp_number);
    if(mod > 0 && mod < 0.5  && !backShapeFlag_){
              backShapeMode_  = baskShapeType::FONT_SHAPE_AND_BACKSHAPE;
              LOG(INFO) << "bask shape choose  ji zi type !" ;
    }else  if(mod> 0.5 &&  mod < 1&& !backShapeFlag_ || mod < 0.01){
              backShapeMode_ =  baskShapeType::L_SHAPE_AND_BACKSHAPE;
              LOG(INFO) << "bask shape chooese L type !"
                        << " " << "the mod is : " << mod ;
    }else if((mod - 0.5) < 0.01){
              backShapeMode_ =  baskShapeType::NORMAL_BACKSHAPE;
              LOG(INFO) << "bask shape choose normal type !";
    }else  if(mod < 0.5  && backShapeFlag_){
                  backShapeMode_ = baskShapeType::TURN_LEFT_BACKSHAPE;
              LOG(INFO) << "bask shape choose turn left baskshape !";
    }else if(mod > 0.5 && mod < 1  && backShapeFlag_){
              backShapeMode_ = baskShapeType::TURN_RIGHT_BACKSHAPE;
              LOG(INFO)  << "bask shape choose turn right baskshape !";
    }
    LOG(INFO)  << "the rectangle short edge length is : "
               << transferd_points_[1].x;
    LOG(INFO)  << "the back shape original traverse number is : "
               << "[" << transferd_points_[1].x/temp_number
               << "]" << " "<<  " back shape width * 2 is : "
               <<" ["<< temp_number << "]";
    remainder_ = mod;
}

//根据指定的差值间距设置对两个点进行差值处理之后，对其进行增加详细信息
void pathRectanglePlan::processTwoPointsDifference(Point &point1,
                                                   Point& point2,
                                                   std::vector<pathInterface::pathPoint> &path,
                                                   int ridge_number){
     std::vector<Point>  initial_points;
     pathInterface::pathPoint processed_point;
     std::vector<Point>  points;
     points.push_back(point1);
     points.push_back(point2);

     initial_points = densify(points,10);
     for(auto i  = 1;i < initial_points.size() -1;i++){
         processed_point.x = initial_points[i].x;
         processed_point.y = initial_points[i].y;
         processed_point.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         processed_point.path_point_mode2 = pathInterface::pathPointMode2::BACK;
         processed_point.ridge_number = ridge_number;
         path.push_back(processed_point);
     }
}


//根据指定的差值间距设置对两个点进行差值处理
std::vector<Point> pathRectanglePlan::getPointsWithFixedDistance(Point & p1,
                                                                 Point & p2,
                                                                 double distance){
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dist = sqrt(dx * dx + dy * dy);
    int numPoints = (int)(dist/distance);
    std::vector<Point> get_points;
    for(int i = 0; i <= numPoints;++i){
        Point pointTemp;
        double t = (double)i/(double)numPoints;
        double x = p1.x + t*dx;
        double y = p1.y + t*dy;
        get_points.push_back(pointTemp);
    }
    return get_points;
}


std::vector<Point>  pathRectanglePlan::getShowKeyPoints(){
    if(backShapeMode_ == baskShapeType::L_SHAPE_AND_BACKSHAPE){
            return show_key_points_only_;
    }else{
            return key_points_;
    }
}




//根据垄号生成对应的path信息
std::vector<pathInterface::pathPoint> pathRectanglePlan::processComputeRidgePath(int ridge_index){
        ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
        std::vector<std::vector<double>> finalpath;
        std::vector<pathInterface::pathPoint>  storageAllPath;
        pathInterface::pathPoint point_1,point_2;
        double startPoint[3],endPoint[3];

       if(ridge_index == 1){
           //起点
           point_1.x = key_points_[0].x;
           point_1.y = key_points_[0].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);

           //第一个弯道的起点
           point_2.x =  key_curve_points_[0][0].x;
           point_2.y =  key_curve_points_[0][0].y;
           point_2.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_2.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_2.ridge_number = ridge_index;
           storageAllPath.push_back(point_2);

           //处理第一个弯道的点
           std::vector<curvePoint> tempPoints = key_curve_points_[0];
           startPoint[0] = tempPoints[0].x;
           startPoint[1] = tempPoints[0].y;
           startPoint[2] = tempPoints[0].heading;
           endPoint[0] = tempPoints[1].x;
           endPoint[1] = tempPoints[1].y;
           endPoint[2] = tempPoints[1].heading;
           r->reedsShepp(startPoint,endPoint);
           finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
           for(int i = 0;i  < finalpath.size();i++){
               pathInterface::pathPoint   pathPointCurve;
               pathPointCurve.x = finalpath[i][0];
               pathPointCurve.y = finalpath[i][1];
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
               pathPointCurve.ridge_number = 1;
               storageAllPath.push_back(pathPointCurve);
           }
           //第一个弯道终点的处理
           //先增加第一个弯道终点
           point_1.x = key_curve_points_[0][1].x;
           point_1.y = key_curve_points_[0][1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //之后增加第一个弯道终点对应的3个点
           Point p1,p2;
           p1.x = key_points_[1].x;
           p1.y = key_points_[1].y;
           p2.x = endPoint[0];
           p2.y = endPoint[1];
           processDiff3Points(p1,
                              p2,
                              storageAllPath,
                              ridge_index);   //这几个点需要翻转数据结构


           //处理倒车的点
           Point temp_1,temp_2;
           temp_1.x = key_curve_points_[0][1].x;
           temp_1.y = key_curve_points_[0][1].y;
           temp_2.x = key_points_[1].x;
           temp_2.y = key_points_[1].y;
           //线段部分的后退点
           processTwoPointsDifference(temp_1,
                                      temp_2,
                                      storageAllPath,
                                      ridge_index);
           //一个脚点信息
           point_1.x = key_points_[1].x;
           point_1.y = key_points_[1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //最后一个线段点增加的点
           processDiff3Points(temp_1,
                              temp_2,
                              storageAllPath,
                              ridge_index);
           //前向插入一个点
           diff1PointToJudgeDirection(temp_1,
                                      temp_2,
                                      storageAllPath,
                                      ridge_index);

           //处理第二个弯道
           //处理第二个弯道的起点
           point_1.x = key_curve_points_[1][0].x;
           point_1.y = key_curve_points_[1][0].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //第二个弯道
           tempPoints = key_curve_points_[1];
           startPoint[0] = tempPoints[0].x;
           startPoint[1] = tempPoints[0].y;
           startPoint[2] = tempPoints[0].heading;
           endPoint[0] = tempPoints[1].x;
           endPoint[1] = tempPoints[1].y;
           endPoint[2] = tempPoints[1].heading;
           r->reedsShepp(startPoint,endPoint);
           finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
           for(int i = 1;i  < (int)finalpath.size();i++){
               pathInterface::pathPoint   pathPointCurve;
               pathPointCurve.x = finalpath[i][0];
               pathPointCurve.y = finalpath[i][1];
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
               pathPointCurve.ridge_number = 1;
               storageAllPath.push_back(pathPointCurve);
           }
           //第二个弯道的终点
           point_1.x = key_curve_points_[1][1].x;
           point_1.y = key_curve_points_[1][1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //第二个弯道终点增加的点，往前增加
           p1.x = point_1.x;
           p1.y = point_1.y;
           p2.x = key_curve_points_[2][0].x;
           p2.y = key_curve_points_[2][0].y;
           processDiff3Points(p1,
                              p2,
                              storageAllPath,
                              ridge_index);

           //第二个弯道倒退的点
           temp_1.x = key_curve_points_[1][1].x;
           temp_1.y = key_curve_points_[1][1].y;
           temp_2.x = key_points_[2].x;
           temp_2.y = key_points_[2].y;
           //线段部分的后退点
           processTwoPointsDifference(temp_1,
                                      temp_2,
                                      storageAllPath,
                                      ridge_index);
           //增加一个脚点前进倒退点
           point_1.x = key_points_[2].x;
           point_1.y = key_points_[2].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //最后一个线段点增加的点
           processDiff3Points(temp_1,
                              temp_2,
                              storageAllPath,
                              ridge_index);
           //前向插入一个点
           diff1PointToJudgeDirection(temp_1,
                                      temp_2,
                                      storageAllPath,
                                      ridge_index);

           //增加第二个弯道的终点，标志着第1垄的结束
           point_1.x = key_curve_points_[1][1].x;
           point_1.y = key_curve_points_[1][1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           return storageAllPath;
       }

       if(ridge_index == ridge_numbers_ &&
          backShapeMode_ ==
          pathRectanglePlan::baskShapeType::NORMAL_BACKSHAPE  ||
            (  (ridge_index == ridge_numbers_ - 1 &&
               backShapeMode_ ==
               pathRectanglePlan::baskShapeType::FONT_SHAPE_AND_BACKSHAPE )||
              ( ridge_index == ridge_numbers_ - 1 &&
               backShapeMode_ ==
               pathRectanglePlan::baskShapeType::L_SHAPE_AND_BACKSHAPE ))){
             int finalRidgeIndex_3 =   key_curve_points_.size() -1 -2 -1;


             auto finalRidgeIndex_2 = finalRidgeIndex_3 +1;
             startPoint[0] = key_points_[key_points_.size()-1-2].x;
             startPoint[1] = key_points_[key_points_.size()-1-2].y;
             startPoint[2] = 0;
             endPoint[0] = key_curve_points_[finalRidgeIndex_2][1].x;
             endPoint[1] = key_curve_points_[finalRidgeIndex_2][1].y;
             endPoint[2] = key_curve_points_[finalRidgeIndex_2][1].heading;
             r->reedsShepp(startPoint,endPoint);
             finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
             for(int i = 1;i  < finalpath.size();i++){
               //f1<<finalpath[i][0]<<" "<<finalpath[i][1]<<" "<<finalpath[i][2]<<endl;
               pathInterface::pathPoint   pathPointCurve;
               pathPointCurve.x = finalpath[i][0];
               pathPointCurve.y = finalpath[i][1];
               pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
               pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
               pathPointCurve.ridge_number = ridge_index;
               storageAllPath.push_back(pathPointCurve);
            }
             //到达弯道端点
             point_1.x = key_curve_points_[finalRidgeIndex_2][1].x;
             point_1.y =  key_curve_points_[finalRidgeIndex_2][1].y;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = ridge_index;
             storageAllPath.push_back(point_1);

           Point p1,p2;
           p2.x = key_curve_points_[finalRidgeIndex_2][1].x;
           p2.y = key_curve_points_[finalRidgeIndex_2][1].y;
           p1.x = key_points_[key_points_.size()-1].x;
           p1.y = key_points_[key_points_.size()-1].y;

           processDiff3Points(p1, p2,storageAllPath,ridge_index);

           //处理后退2的点
           Point temp_1,temp_2;
           temp_1.x = key_curve_points_[key_curve_points_.size()-3][1].x;
           temp_1.y = key_curve_points_[key_curve_points_.size()-3][1].y;
           temp_2.x = key_points_[key_points_.size()-2].x;
           temp_2.y = key_points_[key_points_.size()-2].y;
           //线段部分的后退点
           processTwoPointsDifference(temp_1,temp_2,storageAllPath,ridge_index);
           //增加一个脚点前进后退切换点
           point_1.x = key_points_[key_points_.size()-2].x;
           point_1.y = key_points_[key_points_.size()-2].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           //最后一个线段点增加的点
           processDiff3Points(temp_1,temp_2,storageAllPath,ridge_index);


           point_1.x = key_points_[key_points_.size()-1].x;
           point_1.y = key_points_[key_points_.size()-1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::JOB_DONE_PARKING;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);
           return storageAllPath;
       }
       if(ridge_index == ridge_numbers_ - 2 &&
               backShapeMode_ ==
               pathRectanglePlan::baskShapeType::NORMAL_BACKSHAPE  ||
                 (  (ridge_index == ridge_numbers_ - 2 &&
                    backShapeMode_ ==
                    pathRectanglePlan::baskShapeType::FONT_SHAPE_AND_BACKSHAPE) ||
                    (ridge_index == ridge_numbers_ - 2  &&
                    backShapeMode_ ==
                    pathRectanglePlan::baskShapeType::L_SHAPE_AND_BACKSHAPE))){
           //起点
           int startRidgePointIndex = 2*ridge_index -1;
           startRidgePointIndex = startRidgePointIndex - 2;
           point_1.x = key_curve_points_[startRidgePointIndex][1].x;
           point_1.y = key_curve_points_[startRidgePointIndex][1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);

           //第一个入弯的起点
           startRidgePointIndex = startRidgePointIndex + 1;
           point_1.x = key_curve_points_[startRidgePointIndex][0].x;
           point_1.y = key_curve_points_[startRidgePointIndex][0].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           point_1.ridge_number = ridge_index;
           storageAllPath.push_back(point_1);

           //第一个弯道
           double startCurvePoint[3],endCurvePoint[3];
           startCurvePoint[0] = key_curve_points_[startRidgePointIndex][0].x;
           startCurvePoint[1] = key_curve_points_[startRidgePointIndex][0].y;
           startCurvePoint[2] = key_curve_points_[startRidgePointIndex][0].heading;
           endCurvePoint[0] = key_curve_points_[startRidgePointIndex][1].x;
           endCurvePoint[1] = key_curve_points_[startRidgePointIndex][1].y;
           endCurvePoint[2] = key_curve_points_[startRidgePointIndex][1].heading;
           r->reedsShepp(startCurvePoint,endCurvePoint);
           finalpath = r->xingshensample(startCurvePoint,endCurvePoint,REEDSHEPP_SAMPLE_INTERVAL);
           for(int i = 1;i  < finalpath.size()-1;i++){
            pathInterface::pathPoint   pathPointCurve;
            pathPointCurve.x = finalpath[i][0];
            pathPointCurve.y = finalpath[i][1];
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number =ridge_index;
            storageAllPath.push_back(pathPointCurve);
            }
            //第一个弯道的终点
            point_1.x = key_curve_points_[startRidgePointIndex][1].x;
            point_1.y = key_curve_points_[startRidgePointIndex][1].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            //第一个弯道的终点增加的点
            Point p1,p2;
            p1.x = key_curve_points_[startRidgePointIndex+1][0].x;
            p1.y = key_curve_points_[startRidgePointIndex+1][0].y;
            p2.x = point_1.x;
            p2.y = point_1.y;

            processDiff3Points(p1,
                               p2,
                               storageAllPath,
                               ridge_index);

            //处理后退的点
            Point temp_1,temp_2;
            temp_1.x = key_curve_points_[startRidgePointIndex][1].x;
            temp_1.y = key_curve_points_[startRidgePointIndex][1].y;
            temp_2.x = key_points_[2*ridge_index-1].x;
            temp_2.y = key_points_[2*ridge_index-1].y;
            //线段部分的后退点
            processTwoPointsDifference(temp_1,temp_2,storageAllPath,ridge_index);
            //增加一个脚点前进后退切换点
            point_1.x = key_points_[2*ridge_index-1].x;
            point_1.y = key_points_[2*ridge_index-1].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            //最后一个线段点增加的点
            processDiff3Points(temp_1,temp_2,storageAllPath,ridge_index);
            //前向插入一个点
            diff1PointToJudgeDirection(temp_1,temp_2,storageAllPath,ridge_index);

            //处理下一个弯道
            //第二个弯道的起点
            point_1.x = key_curve_points_[startRidgePointIndex+1][0].x;
            point_1.y = key_curve_points_[startRidgePointIndex+1][0].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            //第二个弯道
            startCurvePoint[0] = key_curve_points_[startRidgePointIndex+1][0].x;
            startCurvePoint[1] = key_curve_points_[startRidgePointIndex+1][0].y;
            startCurvePoint[2] = key_curve_points_[startRidgePointIndex+1][0].heading;
            endCurvePoint[0] = key_curve_points_[startRidgePointIndex+1][1].x;
            endCurvePoint[1] = key_curve_points_[startRidgePointIndex+1][1].y;
            endCurvePoint[2] = key_curve_points_[startRidgePointIndex+1][1].heading;
            r->reedsShepp(startCurvePoint,endCurvePoint);
            finalpath = r->xingshensample(startCurvePoint,endCurvePoint,REEDSHEPP_SAMPLE_INTERVAL);
            for(int i = 1;i  < finalpath.size();i++){
            pathInterface::pathPoint   pathPointCurve;
            pathPointCurve.x = finalpath[i][0];
            pathPointCurve.y = finalpath[i][1];
            pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
            pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
            pathPointCurve.ridge_number = ridge_index;
            storageAllPath.push_back(pathPointCurve);
            }
            //第二个弯道的终点
            point_1.x = key_curve_points_[startRidgePointIndex+1][1].x;
            point_1.y = key_curve_points_[startRidgePointIndex+1][1].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            //第二个弯道终点增加的点
            p1.x = point_1.x;
            p1.y = point_1.y;
            p2.x = key_curve_points_[startRidgePointIndex+2][0].x;
            p2.y = key_curve_points_[startRidgePointIndex+2][0].y;
            processDiff3Points(p1, p2,storageAllPath,ridge_index);

            //第二个弯道倒退的点
            temp_1.x = key_curve_points_[startRidgePointIndex+1][1].x;
            temp_1.y = key_curve_points_[startRidgePointIndex+1][1].y;
            temp_2.x = key_points_[2*ridge_index].x;
            temp_2.y = key_points_[2*ridge_index].y;
            //线段部分的后退点
            processTwoPointsDifference(temp_1,temp_2,storageAllPath,ridge_index);
            //增加一个脚点前进后退点
            point_1.x =  key_points_[2*ridge_index].x;
            point_1.y =  key_points_[2*ridge_index].y;
            point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
            point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
            point_1.ridge_number = ridge_index;
            storageAllPath.push_back(point_1);
            //最后一个线段点增加的点
            processDiff3Points(temp_1,temp_2,storageAllPath,ridge_index);
            return storageAllPath;
       }
       //正常中间关键点处理
       //起点
       int startRidgePointIndex = 2*ridge_index -1;
       startRidgePointIndex = startRidgePointIndex - 2;
       point_1.x = key_curve_points_[startRidgePointIndex][1].x;
       point_1.y = key_curve_points_[startRidgePointIndex][1].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
       point_1.ridge_number = ridge_index;
       storageAllPath.push_back(point_1);

       //第一个入弯的起点
       startRidgePointIndex = startRidgePointIndex + 1;
       point_1.x = key_curve_points_[startRidgePointIndex][0].x;
       point_1.y = key_curve_points_[startRidgePointIndex][0].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
       point_1.ridge_number = ridge_index;
       storageAllPath.push_back(point_1);

       //第一个弯道
       double startCurvePoint[3],endCurvePoint[3];
       startCurvePoint[0] = key_curve_points_[startRidgePointIndex][0].x;
       startCurvePoint[1] = key_curve_points_[startRidgePointIndex][0].y;
       startCurvePoint[2] = key_curve_points_[startRidgePointIndex][0].heading;
       endCurvePoint[0] = key_curve_points_[startRidgePointIndex][1].x;
       endCurvePoint[1] = key_curve_points_[startRidgePointIndex][1].y;
       endCurvePoint[2] = key_curve_points_[startRidgePointIndex][1].heading;
       r->reedsShepp(startCurvePoint,endCurvePoint);
       finalpath = r->xingshensample(startCurvePoint,endCurvePoint,REEDSHEPP_SAMPLE_INTERVAL);
       for(int i = 1;i  < finalpath.size()-1;i++){
        pathInterface::pathPoint   pathPointCurve;
        pathPointCurve.x = finalpath[i][0];
        pathPointCurve.y = finalpath[i][1];
        pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        pathPointCurve.ridge_number =ridge_index;
        storageAllPath.push_back(pathPointCurve);
        }
        //第一个弯道的终点
        point_1.x = key_curve_points_[startRidgePointIndex][1].x;
        point_1.y = key_curve_points_[startRidgePointIndex][1].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        //第一个弯道的终点增加的点,这个bug是由于没有后续的key_curve_points_导致的
        Point p1,p2;
        p1.x = key_curve_points_[startRidgePointIndex+1][0].x;
        p1.y = key_curve_points_[startRidgePointIndex+1][0].y;
        p2.x = point_1.x;
        p2.y = point_1.y;

        processDiff3Points(p1, p2,storageAllPath,ridge_index);

        //处理后退的点
        Point temp_1,temp_2;
        temp_1.x = key_curve_points_[startRidgePointIndex][1].x;
        temp_1.y = key_curve_points_[startRidgePointIndex][1].y;
        temp_2.x = key_points_[2*ridge_index-1].x;
        temp_2.y = key_points_[2*ridge_index-1].y;
        //线段部分的后退点
        processTwoPointsDifference(temp_1,temp_2,storageAllPath,ridge_index);
        //增加一个脚点前进后退切换点
        point_1.x = key_points_[2*ridge_index-1].x;
        point_1.y = key_points_[2*ridge_index-1].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        //最后一个线段点增加的点
        processDiff3Points(temp_1,temp_2,storageAllPath,ridge_index);
        //前向插入一个点
        diff1PointToJudgeDirection(temp_1,temp_2,storageAllPath,ridge_index);

        //处理下一个弯道
        //第二个弯道的起点
        point_1.x = key_curve_points_[startRidgePointIndex+1][0].x;
        point_1.y = key_curve_points_[startRidgePointIndex+1][0].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        //第二个弯道
        startCurvePoint[0] = key_curve_points_[startRidgePointIndex+1][0].x;
        startCurvePoint[1] = key_curve_points_[startRidgePointIndex+1][0].y;
        startCurvePoint[2] = key_curve_points_[startRidgePointIndex+1][0].heading;
        endCurvePoint[0] = key_curve_points_[startRidgePointIndex+1][1].x;
        endCurvePoint[1] = key_curve_points_[startRidgePointIndex+1][1].y;
        endCurvePoint[2] = key_curve_points_[startRidgePointIndex+1][1].heading;
        r->reedsShepp(startCurvePoint,endCurvePoint);
        finalpath = r->xingshensample(startCurvePoint,endCurvePoint,REEDSHEPP_SAMPLE_INTERVAL);
        for(int i = 1;i  < finalpath.size();i++){
        pathInterface::pathPoint   pathPointCurve;
        pathPointCurve.x = finalpath[i][0];
        pathPointCurve.y = finalpath[i][1];
        pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
        pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
        pathPointCurve.ridge_number = ridge_index;
        storageAllPath.push_back(pathPointCurve);
        }
        //第二个弯道的终点
        point_1.x = key_curve_points_[startRidgePointIndex+1][1].x;
        point_1.y = key_curve_points_[startRidgePointIndex+1][1].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        //第二个弯道终点增加的点
        p1.x = point_1.x;
        p1.y = point_1.y;
        p2.x = key_curve_points_[startRidgePointIndex+2][0].x;
        p2.y = key_curve_points_[startRidgePointIndex+2][0].y;
        processDiff3Points(p1, p2,storageAllPath,ridge_index);

        //第二个弯道倒退的点
        temp_1.x = key_curve_points_[startRidgePointIndex+1][1].x;
        temp_1.y = key_curve_points_[startRidgePointIndex+1][1].y;
        temp_2.x = key_points_[2*ridge_index].x;
        temp_2.y = key_points_[2*ridge_index].y;
        //线段部分的后退点
        processTwoPointsDifference(temp_1,temp_2,storageAllPath,ridge_index);
        //增加一个脚点前进后退点
        point_1.x =  key_points_[2*ridge_index].x;
        point_1.y =  key_points_[2*ridge_index].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        //最后一个线段点增加的点
        processDiff3Points(temp_1,temp_2,storageAllPath,ridge_index);
        //前向插入一个点
        diff1PointToJudgeDirection(temp_1,temp_2,storageAllPath,ridge_index);
        //插入第二个弯道的终点
        point_1.x =  key_curve_points_[startRidgePointIndex+1][1].x;
        point_1.y =  key_curve_points_[startRidgePointIndex+1][1].y;
        point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point_1.ridge_number = ridge_index;
        storageAllPath.push_back(point_1);
        return storageAllPath;
}




//差值1个点用来判断方向
void pathRectanglePlan::diff1PointToJudgeDirection(Point p1,
                                                   Point p2,
                                                   std::vector<pathInterface::pathPoint> &path,
                                                   int ridge_index){
    Point temp;
    temp.x = (p1.x + p2.x)/2;
    temp.y = (p1.y + p2.y)/2;
    pathInterface::pathPoint point;
    point.x = temp.x;
    point.y = temp.y;
    point.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
    point.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
    point.ridge_number = ridge_index;
    path.push_back(point);
}


//功能是在点之间插入factor-1个点，是等间隔的点
std::vector<Point> pathRectanglePlan::densify(const std::vector<Point>& points,
                                              int factor) {
    std::vector<Point> densified_points;

    for (size_t i = 0; i < points.size() - 1; ++i) {
        densified_points.push_back(points[i]);

        for (int j = 1; j < factor; ++j) {
            double t = static_cast<double>(j) / factor;
            Point interpolated_point;
            interpolated_point.x = points[i].x + t * (points[i + 1].x - points[i].x);
            interpolated_point.y = points[i].y + t * (points[i + 1].y - points[i].y);
            densified_points.push_back(interpolated_point);
        }
    }

    densified_points.push_back(points.back());
    return densified_points;
}


//在m点，n点的线段上，n点之前扩展3个点
std::vector<Point>  pathRectanglePlan::getDiff3Points(Point&m,Point&n){
    //这里可以按照求出的k是无穷大还是0,将其分开处理按照哪个方向来进行扩展点的计算
    std::vector<Point> points;
    Point point_temp;
    switch(judge_increase_points_direction_)
    {
        case 0:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x ;
                point_temp.y = n.y + i * 0.2;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 1:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x ;
                point_temp.y = n.y - i * 0.2;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 2:
            for(int i = 1;i <= 3;i++){
                point_temp.x = m.x - i *0.2;
                point_temp.y = m.y ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 3:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x + i *0.2;
                point_temp.y = n.y ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 4:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x ;
                point_temp.y = n.y - i *0.2 ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 5:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x ;
                point_temp.y = n.y + i *0.2 ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 6:
            for(int i = 1;i <= 3;i++){
                point_temp.x = m.x + i *0.2;
                point_temp.y = m.y  ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        case 7:
            for(int i = 1;i <= 3;i++){
                point_temp.x = n.x - i *0.2;
                point_temp.y = n.y  ;
                points.push_back(point_temp);
            }
            judge_increase_points_direction_ +=1;
            if(judge_increase_points_direction_ == 8){
                judge_increase_points_direction_ = 0;
            }
            break;
        default:
            std::cout << "this is error !" << std::endl;
    }
    return points;
}



//在m点，n点的线段上，n点之前扩展3个点之前增加详细信息
void pathRectanglePlan::processDiff3Points(Point&m,
                                           Point&n,
                                           std::vector<pathInterface::pathPoint> & path,
                                           int ridge_number){
    pathInterface::pathPoint point;
    std::vector<Point>  temp_diff_points;
    temp_diff_points = getDiff3Points(m,n);
    for(auto it = temp_diff_points.begin();it != temp_diff_points.end();++it){
        point.x = it->x;
        point.y = it->y;
        point.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
        point.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
        point.ridge_number = ridge_number;
        path.push_back(point);
    }
}



//======================================================================
// 函数功能： 针对矩形短边不是垄宽整数倍，则额外添加一垄或两垄,用于获取关键点的信息
// 输入参数：

// 编辑日期：    2023/05/04
// 路径形状：
// |             ||
// |_____        ||______
//
//======================================================================
void pathRectanglePlan::processIncludeFirstRidgeKeyPoints(std::vector<Point>& points){
            switch(backShapeMode_){
               case baskShapeType::L_SHAPE_AND_BACKSHAPE:
                    deviation_number_L_ = transferd_points_[1].x -
                            (traversals_numbers_ - 1) * bow_spacing_setting_ * 2
                            - bow_spacing_setting_ + bow_spacing_setting_/2;
                    LOG(INFO) << "the rectangle horizontal deviation distance is : "
                              << deviation_number_L_ ;
                    LOG(INFO) << "use L shape compensate width is : "
                              << ( transferd_points_[1].x -
                                 (traversals_numbers_ - 1) * bow_spacing_setting_ * 2
                                 - bow_spacing_setting_ );
                    show_key_points_only_.push_back(points[3]);
                    show_key_points_only_.push_back(Point(points[0].x,points[0].y+2));
                    for(auto i : key_points_){
                        Point tempPoint;
                        tempPoint.x = i.x + deviation_number_L_;
                        tempPoint.y = i.y;
                        show_key_points_only_.push_back(tempPoint);
                    }
                    break;
               case  baskShapeType::FONT_SHAPE_AND_BACKSHAPE:
                    deviation_number_ = transferd_points_[1].x -
                            (traversals_numbers_ -1-1)* bow_spacing_setting_*2
                            - bow_spacing_setting_ + bow_spacing_setting_/2;
                    LOG(INFO) << "the rectangle horizontal deviation distance is :"
                              << deviation_number_ ;
                    LOG(INFO) << "the ji zi shape compensate width is :"
                              << (transferd_points_[1].x -
                                 (traversals_numbers_ -1-1) * bow_spacing_setting_* 2
                                 - bow_spacing_setting_ ) ;
                    show_key_points_only_.push_back(Point(points[0].x + 2, points[0].y));
                    show_key_points_only_.push_back(Point(points[3].x + 2, points[3].y - 2));
                    show_key_points_only_.push_back(Point(points[3].x + 6, points[2].y - 2));
                    show_key_points_only_.push_back(Point(points[3].x + 6, points[1].y + 2));
                    for(auto i : key_points_){
                        Point tempPoint;
                        tempPoint.x = i.x + deviation_number_;
                        tempPoint.y = i.y;
                        show_key_points_only_.push_back(tempPoint);
                    }
                    break;
              case  baskShapeType::NORMAL_BACKSHAPE:
                    for(auto i : key_points_){
                        Point tempPoint;
                        tempPoint.x = i.x + deviation_number_;
                        tempPoint.y = i.y;
                        show_key_points_only_.push_back(tempPoint);
                    }
                    break;
               default:
                    {
                      LOG(INFO) << "Nothing was done !";
                    }
            }
}


std::vector<Point>  pathRectanglePlan::getOriginalShowKeyPoints(){
    return original_show_key_points_;
}


//======================================================================
// 函数功能： 针对矩形短边不是垄宽整数倍，则额外添加一垄，用于生成下发的path点信息
// 输入参数：

// 编辑日期：    2023/04/28
// 路径形状：
// |             ||
// |_____        ||______
//
//======================================================================
std::vector<pathInterface::pathPoint>  pathRectanglePlan::processFirstRidge(){
    //在矩形短边的余数留下漏割小于垄宽的情况下处理
     ReedsSheppStateSpace   *r= new ReedsSheppStateSpace;
     std::vector<std::vector<double>> finalpath;


       //根据输入矩形得到第一垄对应的三个关键点
       if(transferd_points_.size() == 0){
           LOG(INFO) << "the  rectangle points is zero!" << std::endl;
       }
       first_road_.push_back(Point(transferd_points_[3].x + bow_spacing_setting_/2,
                             transferd_points_[3].y));
       first_road_.push_back(Point(transferd_points_[0].x + bow_spacing_setting_/2,
                             transferd_points_[0].y + bow_spacing_setting_/2));
       first_road_.push_back(Point(use_get_curve_point_ + bow_spacing_setting_/2,
                                   bow_spacing_setting_/2));
       LOG(INFO) << "L type first path horizontal distance(outer boudary) is :"
                 << transferd_points_[3].x + bow_spacing_setting_/2;
       pathInterface::pathPoint point_1;
       //起点
       point_1.x = first_road_[0].x;
       point_1.y = first_road_[0].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
       point_1.ridge_number = 1;
       first_road_points_.push_back(point_1);

       //弯道起点
       point_1.x =  first_road_[1].x ;
       point_1.y =  first_road_[1].y + use_get_curve_point_;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
       point_1.ridge_number = 1;
       first_road_points_.push_back(point_1);

       //处理整个弯道
       double startPoint[3],endPoint[3];
       startPoint[0] = first_road_[1].x ;
       startPoint[1] = first_road_[1].y + use_get_curve_point_;
       startPoint[2] = -3.1415926/2;
       endPoint[0] =   first_road_[1].x + use_get_curve_point_ ;
       endPoint[1] =   first_road_[1].y;
       endPoint[2] =  0;
       r->reedsShepp(startPoint,endPoint);
       finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
       for(int i = 0;i  < finalpath.size();i++){
           pathInterface::pathPoint   pathPointCurve;
           pathPointCurve.x = finalpath[i][0];
           pathPointCurve.y = finalpath[i][1];
           pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
           pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
           pathPointCurve.ridge_number = 1;
           first_road_points_.push_back(pathPointCurve);
       }
       //处理弯道的终点
       point_1.x =  first_road_[1].x + use_get_curve_point_ ;
       point_1.y =  first_road_[1].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
       point_1.ridge_number = 1;
       first_road_points_.push_back(point_1);
       //增加3个弯道转换点
       for(auto i  = 1;i <= 3; i++){
           point_1.x =  first_road_[1].x + use_get_curve_point_ +i * 0.2;
           point_1.y =  first_road_[1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = 1;
           first_road_points_.push_back(point_1);
       }

       //增加起点到弯道终点的后退点
       Point   temp_1,temp_2;
       temp_1.x = first_road_[1].x + use_get_curve_point_ ;
       temp_1.y = first_road_[1].y;
       temp_2.x  = first_road_[1].x;
       temp_2.y  = first_road_[1].y;
       processTwoPointsDifference(temp_1,temp_2,first_road_points_,1);

       //增加后退的终点信息
       point_1.x =  first_road_[1].x  ;
       point_1.y =  first_road_[1].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
       point_1.ridge_number = 1;
       first_road_points_.push_back(point_1);

       //针对后退的终点增加3个点
       for(int i  = 1;i <= 3; i++){
           point_1.x =  first_road_[1].x - i * 0.2 ;
           point_1.y =  first_road_[1].y;
           point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
           point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
           point_1.ridge_number = 1;
           first_road_points_.push_back(point_1);
       }

       //增加第一垄的终点信息（回字形的起点信息）
       point_1.x =  bow_spacing_setting_ * 2 * remainder_ ;            //垄宽 * 2 * 余数 = 需要割的遗漏的数
       point_1.y =  first_road_[1].y;
       point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
       point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
       point_1.ridge_number = 1;
       first_road_points_.push_back(point_1);


    return first_road_points_;
}



//======================================================================
// 函数功能： 针对矩形短边不是垄宽整数倍，则额外添加一垄(几字型)，用于生成下发的path点信息
// 输入参数：

// 编辑日期：    2023/05/11
// 路径形状：
//        ||
//        ||______
//
//======================================================================
std::vector<pathInterface::pathPoint>  pathRectanglePlan::processJiziShapeFirstRidge(){
    ReedsSheppStateSpace   *r=new ReedsSheppStateSpace;
    std::vector<std::vector<double>> finalpath;
    if(backShapeMode_ == baskShapeType::FONT_SHAPE_AND_BACKSHAPE){
         if(transferd_points_.size() == 0){
             LOG(INFO) << "the rectangle points is zero !" ;
         }
         first_road_.push_back(Point(transferd_points_[0].x + 2,
                               transferd_points_[0].y));
         first_road_.push_back(Point(transferd_points_[3].x + 2,
                               transferd_points_[3].y - 2));
         first_road_.push_back(Point(transferd_points_[3].x + 6,
                               transferd_points_[2].y - 2));
         first_road_.push_back(Point(transferd_points_[3].x + 6,
                               transferd_points_[1].y + 2));
         LOG(INFO) << "ji zi xing first path  horizontal distance(outer boudary) is: "
                   << transferd_points_[0].x + 2;
         pathInterface::pathPoint point_1;
         //起点
         point_1.x = first_road_[0].x ;
         point_1.y = first_road_[0].y;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //弯道起点
         point_1.x =  first_road_[1].x ;
         point_1.y =  first_road_[1].y - use_get_curve_point_;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //处理整个弯道
         double startPoint[3],endPoint[3];
         startPoint[0] = first_road_[1].x ;
         startPoint[1] = first_road_[1].y - use_get_curve_point_;
         startPoint[2] = PI/2;
         endPoint[0] =   first_road_[1].x + use_get_curve_point_ ;
         endPoint[1] =   first_road_[1].y;
         endPoint[2] =  0;
         r->reedsShepp(startPoint,endPoint);
         finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
         for(int i = 0;i  < finalpath.size();i++){
         //f1<<finalpath[i][0]<<" "<<finalpath[i][1]<<" "<<finalpath[i][2]<<endl;
             pathInterface::pathPoint   pathPointCurve;
             pathPointCurve.x = finalpath[i][0];
             pathPointCurve.y = finalpath[i][1];
             pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
             pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
             pathPointCurve.ridge_number = 1;
             first_road_points_.push_back(pathPointCurve);
         }
         //处理弯道的终点
         point_1.x =  first_road_[1].x + use_get_curve_point_ ;
         point_1.y =  first_road_[1].y;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //增加3个弯道转换点
         for(auto i  = 1;i <= 3; i++){
             point_1.x =  first_road_[1].x + use_get_curve_point_ + i * 0.2;
             point_1.y =  first_road_[1].y;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }


         //增加起点到弯道终点的后退点
         Point   temp_1,temp_2;
         temp_1.x = first_road_[1].x + use_get_curve_point_ ;
         temp_1.y = first_road_[1].y;
         temp_2.x = first_road_[1].x;
         temp_2.y = first_road_[1].y;
         processTwoPointsDifference(temp_1,
                                    temp_2,
                                    first_road_points_,
                                    1);

         //增加后退的终点信息
         point_1.x =  first_road_[1].x;
         point_1.y =  first_road_[1].y;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //针对后退的终点增加3个点
         for(int i  = 1;i <= 3; i++){
             point_1.x =  first_road_[1].x - i*0.2 ;
             point_1.y =  first_road_[1].y;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }

         //第二个弯道处理

         //处理整个弯道
         startPoint[0] = first_road_[1].x ;
         startPoint[1] = first_road_[1].y ;
         startPoint[2] = 0;
         endPoint[0] =   first_road_[2].x ;
         endPoint[1] =   first_road_[2].y - use_get_curve_point_;
         endPoint[2] =   -PI/2;
         r->reedsShepp(startPoint,endPoint);
         finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
         for(int i = 0;i  < finalpath.size();i++){
             pathInterface::pathPoint   pathPointCurve;
             pathPointCurve.x = finalpath[i][0];
             pathPointCurve.y = finalpath[i][1];
             pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
             pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
             pathPointCurve.ridge_number = 1;
             first_road_points_.push_back(pathPointCurve);
         }

         //处理弯道的终点
         point_1.x =  first_road_[2].x;
         point_1.y =  first_road_[2].y - use_get_curve_point_;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //增加3个弯道转换点
         for(auto i  = 1;i <= 3; i++){
             point_1.x =  first_road_[2].x  ;
             point_1.y =  first_road_[2].y - use_get_curve_point_ - i * 0.2;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }
          //处理后退点
         temp_1.x = first_road_[2].x ;
         temp_1.y = first_road_[2].y - use_get_curve_point_ ;
         temp_2.x = first_road_[2].x ;
         temp_2.y = first_road_[2].y ;
         processTwoPointsDifference(temp_1,temp_2,first_road_points_,1);

         //增加后退的终点信息
         point_1.x =  first_road_[2].x  ;
         point_1.y =  first_road_[2].y;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //针对后退的终点增加3个点
         for(int i  = 1;i <= 3; i++){
             point_1.x =  first_road_[2].x ;
             point_1.y =  first_road_[2].y +  i*0.2;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }

         //第三个弯道的起点
         point_1.x = first_road_[3].x;
         point_1.y = first_road_[3].y + use_get_curve_point_;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //处理整个弯道
         startPoint[0] = first_road_[3].x ;
         startPoint[1] = first_road_[3].y + use_get_curve_point_;
         startPoint[2] = -PI/2;
         endPoint[0] =   first_road_[3].x + use_get_curve_point_ ;
         endPoint[1] =   first_road_[3].y;
         endPoint[2] =  0;
         r->reedsShepp(startPoint,endPoint);
         finalpath = r->xingshensample(startPoint,endPoint,REEDSHEPP_SAMPLE_INTERVAL);
         for(int i = 0;i  < finalpath.size();i++){
             pathInterface::pathPoint   pathPointCurve;
             pathPointCurve.x = finalpath[i][0];
             pathPointCurve.y = finalpath[i][1];
             pathPointCurve.path_point_mode1 = pathInterface::pathPointMode1::TURNING_AREA;
             pathPointCurve.path_point_mode2 = pathInterface::pathPointMode2::FORWARD;
             pathPointCurve.ridge_number = 1;
             first_road_points_.push_back(pathPointCurve);
         }

         //处理弯道的终点
         point_1.x =  first_road_[3].x + use_get_curve_point_;
         point_1.y =  first_road_[3].y;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);
         //增加3个弯道转换点
         for(auto i  = 1;i <= 3; i++){
             point_1.x =  first_road_[3].x + use_get_curve_point_ +i * 0.2;
             point_1.y =  first_road_[3].y;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }

         //增加起点到弯道终点的后退点
         temp_1.x = first_road_[3].x + use_get_curve_point_ ;
         temp_1.y = first_road_[3].y;
         temp_2.x = first_road_[3].x;
         temp_2.y = first_road_[3].y;
         processTwoPointsDifference(temp_1,
                                    temp_2,
                                    first_road_points_,
                                    1);

         //增加后退的终点信息
         point_1.x =  first_road_[3].x ;
         point_1.y =  first_road_[3].y ;
         point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
         point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
         point_1.ridge_number = 1;
         first_road_points_.push_back(point_1);

         //针对后退的终点增加3个点
         for(int i  = 1;i <= 3; i++){
             point_1.x =  first_road_[3].x - i*0.2 ;
             point_1.y =  first_road_[3].y;
             point_1.path_point_mode1 = pathInterface::pathPointMode1::WORK_AREA;
             point_1.path_point_mode2 = pathInterface::pathPointMode2::SWITCH_BACK_FORWARD;
             point_1.ridge_number = 1;
             first_road_points_.push_back(point_1);
         }
    }
    return first_road_points_;
}

}
}
