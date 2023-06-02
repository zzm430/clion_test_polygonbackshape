#include "PreProcess/getMapData.h"

getMapData::getMapData() {

}

getMapData::~getMapData() {

}
void getMapData::loadMapOuter(){

//     std::ifstream infile( "/home/zzm/Downloads/middle/025.txt");
    std::ifstream infile("/home/zzm/Downloads/map_0511/hard/040.txt");
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
