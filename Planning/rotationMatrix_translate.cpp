//
// Created by zzm on 2023/4/25.
//
#include "rotationMatrix_translate.h"

//double rotationMatrixAndTranslate::angle = 0;
//Point  rotationMatrixAndTranslate::translate_point = Point(0,0);

rotationMatrixAndTranslate::rotationMatrixAndTranslate() {

}

rotationMatrixAndTranslate::~rotationMatrixAndTranslate() {

}


/*****
 atan2()函数返回值
 当 (x, y) 在第一象限，0 < θ < π/2
当 (x, y) 在第二象限，π/2 < θ ≤ π
当 (x, y) 在第三象限，-π < θ < -π/2
当 (x, y) 在第四象限，-π/2 < θ < 0
******/



/*对给定的矩形点位从左下角按照逆时针开始排序
  return： 返回逆时针的点位*/
std::vector<Point>  rotationMatrixAndTranslate::postProcess(std::vector<Point> & initial_points){
       if(initial_points.empty()){
           std::cout << "the points is empty!" << std::endl;
       }

       double x_min,y_min,x_max,y_max;

       std::vector<double> x_value;
       std::vector<double> y_value;
       for(auto i : initial_points){
           x_value.push_back(i.x);
       }
       for(auto j : initial_points){
           y_value.push_back(j.y);
       }

      double min_x_value = *std::min_element(x_value.begin(),x_value.end());
      double max_x_value = *std::max_element(x_value.begin(),x_value.end());
      double min_y_value = *std::min_element(y_value.begin(),y_value.end());
      double max_y_value = *std::max_element(y_value.begin(),y_value.end());

       std::vector<Point>  result;
       Point lb;
       lb.x = min_x_value;
       lb.y = min_y_value;
       result.push_back(lb);
       Point  br;
       br.x = max_x_value;
       br.y = min_y_value;
       result.push_back(br);
       Point  lt;
       lt.x = max_x_value;
       lt.y = max_y_value;
       result.push_back(lt);
       Point  tl;
       tl.x = min_x_value;
       tl.y = max_y_value;
       result.push_back(tl);
       return result;
}

//将矩形旋转到短边在x轴上，长边在y轴上
void rotationMatrixAndTranslate::process(std::vector<Point> & initial_points){
    LOG(INFO) << "start rotation matrix and translate !";
    double dist = sqrt(pow(initial_points[0].x - initial_points[1].x, 2)
            + pow(initial_points[0].y - initial_points[1].y, 2));
    double dist1 = sqrt(pow(initial_points[1].x - initial_points[2].x, 2)
            + pow(initial_points[1].y - initial_points[2].y, 2));
    if(dist < dist1){
        //短边是dist
        angle_ = atan2((initial_points[1].y-initial_points[0].y),
                (initial_points[1].x -initial_points[0].x));
        // 计算旋转矩阵
       angle_ = -angle_;
       LOG(INFO) << "choose short edge is dist";
       LOG(INFO) << "x-asix---The angle of matrix transformation rotation is: "
                 << angle_ ;
       LOG(INFO) << "--------------------------------------------------------";
       for(auto i=0 ;i < initial_points.size();i++){
            Point temp_point;
            temp_point.x = cos(angle_) *initial_points[i].x  -
                    sin(angle_) * initial_points[i].y;
            temp_point.y = sin(angle_) * initial_points[i].x  +
                    cos(angle_) * initial_points[i].y;
            if(i == 0){
                translate_point_.x = temp_point.x;
                translate_point_.y = temp_point.y;
                temp_point.x = 0;
                temp_point.y = 0;
            }else {
                temp_point.x = temp_point.x - translate_point_.x;
                temp_point.y = temp_point.y - translate_point_.y;
            }
            rotationedMartixTranslatePoints_.push_back(temp_point);
            LOG(INFO) << "after matrix rotation + translation number is :"
                      << i
                      << " point is : "
                      << "("
                      << temp_point.x
                      << ","
                      << temp_point.y
                      << ")";
        }
        LOG(INFO) << "------------------------------------------------------";
    }else{
        //短边是dist1
         LOG(INFO) << "choose short edge is dist1";
        angle_ = atan2((initial_points[2].y-initial_points[1].y),
                (initial_points[2].x -initial_points[1].x));
        // 计算旋转矩阵
        angle_ = -angle_;
        LOG(INFO) << "x-asix---The angle of matrix transformation rotation is: " << angle_;
        std::vector<Point>  tempStoragePoints;
        LOG(INFO) << "------------------------------------------------------";
        for(auto i = 0 ;i < initial_points.size();i++){
            Point temp_point;
            temp_point.x = cos(angle_) *initial_points[i].x
                    - sin(angle_) * initial_points[i].y;
            temp_point.y = sin(angle_) * initial_points[i].x
                    + cos(angle_) * initial_points[i].y;
            tempStoragePoints.push_back(temp_point);
            LOG(INFO) << "only for matrix rotated point rect  number is : "
                      << i
                      << " point is : "
                      << " ("
                      << temp_point.x
                      << ","
                      << temp_point.y
                      << ")";
        }
       LOG(INFO) << "-------------------------------------------------------";
        //对所有点进行平移变换
        translate_point_.x =  tempStoragePoints[1].x;
        translate_point_.y =  tempStoragePoints[1].y;
       LOG(INFO) << "-------------------------------------------------------";
       for(int  i = 0 ; i <tempStoragePoints.size();i++){
           Point temp_point;
           temp_point.x =  tempStoragePoints[i].x - tempStoragePoints[1].x;
           temp_point.y =  tempStoragePoints[i].y - tempStoragePoints[1].y;
           rotationedMartixTranslatePoints_.push_back(temp_point);
           LOG(INFO) << "after matrix rotation + translation number is :"
                     << i
                     << " point is : "
                     <<  " ("
                     << temp_point.x
                     << ","
                     << temp_point.y
                     << ")" ;
       }
       LOG(INFO) << "-------------------------------------------------------";
    }
}

//矩阵的逆变换
void rotationMatrixAndTranslate::processInverse(std::vector<Point> & initial_points){
    vector<Point>   temp_storage;
    for(auto i=0 ;i < initial_points.size();i++){
        Point temp_point;
        temp_point.x = initial_points[i].x  + translate_point_.x;
        temp_point.y = initial_points[i].y  + translate_point_.y;
        temp_storage.push_back(temp_point);
    }

    for(auto temp_point : temp_storage){
        Point  point;
        point.x = cos(angle_)  *   temp_point.x   +  sin(angle_) * temp_point.y ;
        point.y = -sin(angle_) *   temp_point.x    +  cos(angle_) * temp_point.y;
        originalMartixTranslatePoints_.push_back(point);
    }
}


//GCS坐标点的还原
std::vector<Point> rotationMatrixAndTranslate::processOtherInverse(std::vector<Point> & initial_points){
    vector<Point>    temp_storage;
    vector<Point>    originalGCSPoints;
    for(auto i=0 ;i < initial_points.size();i++){
        Point temp_point;
        temp_point.x = initial_points[i].x  + translate_point_.x;
        temp_point.y = initial_points[i].y  + translate_point_.y;
        temp_storage.push_back(temp_point);
    }

    for(auto temp_point : temp_storage){
        Point  point;
        point.x = cos(angle_)  *   temp_point.x   +  sin(angle_) * temp_point.y ;
        point.y = -sin(angle_) *   temp_point.x    +  cos(angle_) * temp_point.y;
        originalGCSPoints.push_back(point);
    }
    return originalGCSPoints;
}


std::vector<Point>  rotationMatrixAndTranslate::getRotationMartixTranslatePoints(){
    return rotationedMartixTranslatePoints_;
}


std::vector<Point>  rotationMatrixAndTranslate::getOriginalMartixTranslatePoints(){
    return originalMartixTranslatePoints_;
}
