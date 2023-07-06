#include "PreProcess/getMapData.h"

getMapData::getMapData() {

}

getMapData::~getMapData() {

}
void getMapData::loadMapOuter(){
//     std::ifstream infile( "/home/zzm/Downloads/middle/02.txt");
    std::ifstream infile("/home/zzm/Downloads/map_0511/hard/020.txt");
     // 检查文件是否成功打开
      if (!infile) {
          LOG(INFO) << "Failed to open  load map outter file.";
          exit(1);
      }
      std::string line;
      while (getline(infile, line)) {
          std::stringstream ss(line);
          std::vector<double> row;
          double value;
          while (ss >> value) {
              row.push_back(value);
          }
          Point temp_point;
          temp_point.x = row[1];
          temp_point.y = row[2];
          pointsOuter_.push_back(temp_point);
      }
}

void getMapData::loadMapOuter(std::string & loadPath) {
    std::cout << " fault is :" << loadPath;
    std::ifstream infile(loadPath);
    // 检查文件是否成功打开
    if (!infile) {
        LOG(INFO) << "Failed to open  load map outter file.";
        exit(1);
    }
    std::string line;
    while (getline(infile, line)) {
        std::stringstream ss(line);
        std::vector<double> row;
        double value;
        while (ss >> value) {
            row.push_back(value);
        }
        Point temp_point;
        temp_point.x = row[1];
        temp_point.y = row[2];
        pointsOuter_.push_back(temp_point);
    }
}

std::vector<Point> getMapData::getMapOuter(){
   return pointsOuter_;
}


//检查给出的多边形是否符合要求，不符合需要做更新处理
//默认为逆时针为正，顺时针不符合要求，需要修正为逆时针处理
void getMapData::updatepolygonSequence() {
    polygon_map  poly_temp;
    for(auto it : pointsOuter_){
        point_map temp;
        temp.x(it.x);
        temp.y(it.y);
        poly_temp.outer().push_back(temp);
    }
    double area_before = boost::geometry::area(poly_temp);
    if(area_before >=0){                      //顺时针翻转为逆时针
        LOG(INFO) << "the origin polygon is clockwise ！";
        boost::geometry::reverse(poly_temp);
        for(auto it = poly_temp.outer().begin();
                 it != poly_temp.outer().end();
                 it++){
            Point temp;
            temp.x = it->x();
            temp.y = it->y();
            updatedPointOuter_.push_back(temp);
        }
    }else{                                   //逆时针则保持不变
        LOG(INFO) << "the origin polygon is counterclockwise！";
        for(auto it : pointsOuter_){
            Point temp;
            temp.x = it.x;
            temp.y = it.y;
            updatedPointOuter_.push_back(temp);
        }
    }
}


std::vector<Point>  getMapData::getMapUpdatedOuter() {
    return updatedPointOuter_;
}