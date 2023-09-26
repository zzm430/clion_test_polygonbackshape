//
// Created by zzm on 2023/4/12.
//

#ifndef UNTITLED3_PATH_COMMON_H
#define UNTITLED3_PATH_COMMON_H
#include <vector>
#include <cmath>
#include <unordered_map>
class Point {
public:
    Point():x(0),y(0),heading(0),flag(false){}

    Point( double a,  double b):x(a),y(b){}

    Point( double a, double b, double c):x(a),y(b),heading(c){}

     bool operator==(const Point& other) const{
        return (fabs(x - other.x) < 0.1 &&
                fabs(y -other.y) < 0.1);
    }

     bool operator<(const Point& other) const{
        if(x != other.x){
            return x < other.x;
        } else {
            return y < other.y;
        }
    }
    double x;
    double y;
    double  heading;
    bool  flag;
};

struct customHash{
       size_t operator()(const Point& s) const{
           std::hash<long double> h;
           return h(s.x) ^ h(s.y);
       }
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
