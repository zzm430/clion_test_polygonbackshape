//
// Created by zzm on 2023/5/18.
//
#include "maxRectangle_opencv.h"


 maxRectangle::~maxRectangle(){

}



double maxRectangle::get_point_min_x( std::vector<cv::Point>& points ) {
    double min_value;

   std::vector<double> x_value;
   for(auto i : points){
          x_value.push_back(i.x);
   }
   min_value = *std::min_element(x_value.begin(),x_value.end());


   return min_value;
}


double maxRectangle::get_point_min_y( std::vector<cv::Point>& points ) {
    double min_value;

   std::vector<double> y_value;
   for(auto i : points){
          y_value.push_back(i.y);
   }
   min_value = *std::min_element(y_value.begin(),y_value.end());

   return min_value;
}



unsigned long long maxRectangle::getXGCSMin(){
    return x_GCS_min_;
}


unsigned long long maxRectangle::getYGCSMin(){
    return y_GCS_min_;
}


std::vector<cv::Point>  maxRectangle::getCoordinateNarrowingPoints(){
    return coordinateNarrowing_;
}


void maxRectangle::process(std::vector<cv::Point> &initialPoints) {

    unsigned long long  value_x = get_point_min_x(initialPoints);
    unsigned long long  value_y = get_point_min_y(initialPoints);
    x_GCS_min_ = value_x;
    y_GCS_min_ = value_y;

    for(auto i : initialPoints){
        cv::Point  tempPoint;
        tempPoint.x =  (i.x - value_x) * POLYGON_ZOOM_FACTOR;
        tempPoint.y =  (i.y - value_y) * POLYGON_ZOOM_FACTOR;
        coordinateNarrowing_.push_back(tempPoint);
    }

    LOG(INFO) << "----------------------------------------------------";
    for(auto i : coordinateNarrowing_){
         LOG(INFO) << "reduced ploygon points : " << i ;
    }
    LOG(INFO) << "----------------------------------------------------";


#ifdef DEBUG_MIDDLE_INFO
    std::ofstream  location_polygon;
    location_polygon.open("/home/zzm/Desktop/test_path_figure-main/src/location_polygon.txt",
                          std::ios::out);
    for(auto i : coordinateNarrowing_){
        location_polygon << i.x / POLYGON_ZOOM_FACTOR << " ";
    }
    location_polygon << std::endl;
    for(auto j : coordinateNarrowing_){
        location_polygon << j.y / POLYGON_ZOOM_FACTOR<< " ";
    }
    location_polygon << std::endl;
    location_polygon.close();
#endif

//#define DEBUG
#ifdef DEBUG
       ifstream   rec_file;
       rec_file.open("/home/zzm/Desktop/test_path_figure-main/src/rec_points.txt",std::ios::in);
       string line;
       std::vector<std::vector<double>>  storage_number;
       while(getline(rec_file,line)){
           istringstream  iss(line);
           double num;
           std::vector<double> temp;
           while(iss >> num){
               temp.push_back(num);
               std::cout << num << " ";
           }
           storage_number.push_back(temp);
           std::cout << std::endl;
       }

       storageRecPoints_.push_back(cv::Point(storage_number[0][0],
               storage_number[1][0]));
       storageRecPoints_.push_back(cv::Point(storage_number[0][1],
               storage_number[1][1]));
       storageRecPoints_.push_back(cv::Point(storage_number[0][2],
               storage_number[1][2]));
       storageRecPoints_.push_back(cv::Point(storage_number[0][3],
               storage_number[1][3]));

       rec_file.close();
#else
    createOpencvPng(coordinateNarrowing_);

    std::string png_path;
    png_path = "/home/zzm/Downloads/InnerRect-main/test_middle_result/test.png";
    cv::Mat img = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
    InnerRect   innerRectInstance;
    innerRectInstance.getInnerRect(img,
                                   storageRecPoints_,
                                   INNER_RECT_GRID_DENSITY);

    //绘出矩形并存储
   cv::Mat InnerRect_image = img.clone();
   cv::line(InnerRect_image,
            storageRecPoints_[0],
            storageRecPoints_[1],
            cv::Scalar(127));
   cv::line(InnerRect_image,
            storageRecPoints_[1],
            storageRecPoints_[2],
            cv::Scalar(127));
   cv::line(InnerRect_image,
            storageRecPoints_[2],
            storageRecPoints_[3],
            cv::Scalar(127));
   cv::line(InnerRect_image,
            storageRecPoints_[3],
            storageRecPoints_[0],
            cv::Scalar(127));
   std::string out_answer;
   out_answer = "/home/zzm/Downloads/InnerRect-main/test_middle_result/test_rec.png";
   cv::imwrite(out_answer,InnerRect_image);

#ifdef  DEBUG_MIDDLE_INFO
   std::ofstream   rec_file;
   rec_file.open("/home/zzm/Desktop/test_path_figure-main/src/rec_points.txt",std::ios::out);
   for(auto i : storageRecPoints_){
       rec_file << " " << i.x ;
   }
   rec_file << std::endl;
   for(auto j : storageRecPoints_){
       rec_file << " " << j.y ;
   }
   rec_file << std::endl;
   rec_file.close();
#endif

#endif

}

void  maxRectangle::createOpencvPng(std::vector<cv::Point> points){
   double max_number = -DBL_MAX;
   for(int i = 0; i < points.size();i++){
          if(points[i].x > max_number){
              max_number = points[i].x;
          }
          if(points[i].y > max_number){
              max_number = points[i].y;
          }
   }
   max_number = MAX_PNG_SIZE;
   cv::Mat img(max_number,max_number,CV_8UC3,cv::Scalar(0,0,0));

   polylines(img, points, true, cv::Scalar(255, 255, 255), 2);

   fillPoly(img, std::vector<std::vector<cv::Point>>{points},
            cv::Scalar(255, 255, 255));

   string out_str = "/home/zzm/Downloads/InnerRect-main/test_middle_result/test.png";

   imwrite(out_str, img);
}



std::vector<cv::Point2f>  maxRectangle::getMaxRecPoints(){
       return storageRecPoints_;
}
