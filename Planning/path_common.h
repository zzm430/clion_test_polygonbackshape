//
// Created by zzm on 2023/4/12.
//

#ifndef UNTITLED3_PATH_COMMON_H
#define UNTITLED3_PATH_COMMON_H
#include <vector>

class Point {
public:
    Point():x(0),y(0),heading(0),flag(false){}
    Point(double a, double b):x(a),y(b){}
    long double x;
    long double y;
    double  heading;
    bool  flag;
};

class pathCommon{
public:
    pathCommon();
    ~pathCommon(){};

private:
    std::vector<std::vector<Point> > curve_path;
    std::vector<std::vector<Point>>  straight_path;
    std::vector<std::vector<Point>>  back_path;

};
#endif //UNTITLED3_PATH_COMMON_H
