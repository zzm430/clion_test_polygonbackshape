//
// Created by zzm on 2023/9/5.
//

#ifndef POLYGONBACKSHAPE_PATH_POLYGONPOINT_H
#define POLYGONBACKSHAPE_PATH_POLYGONPOINT_H
#include "common/utilpath/path_common.h"

enum  class sideBisector :uint8_t{
    CONTOUR  = 0 ,   //轮廓边
    BISECTOR = 1,    //平分线
    INNER_BISECTOR = 2   //内部骨架边
};

class  polygonPoint: public Point{
public:
    polygonPoint() = default;

    virtual  ~polygonPoint(){};

    polygonPoint(long double f, long double g):Point(f,g){}

    polygonPoint(long double f, long double g,double heading):Point(f,g,heading){}

    bool operator == ( const polygonPoint& other) const{
        return (fabs(x - other.x) < 0.001 &&
                fabs(y -other.y) < 0.001);
    }

    bool operator != ( const polygonPoint& other) const {
        return !(*this == other);
    }

    // 定义小于运算符
    bool operator < (const polygonPoint& other) const {
        if (x != other.x) {
            return x < other.x;

        } else {
            return y < other.y;
        }
    }
    sideBisector judge_bisector_;              //判断是轮廓还是平分线
private:
    int origin_polygon_point_index_;           //对应多边形起始点的索引号
public:
    bool entrance_ = false;                    //回字形入口点
};

struct polyPointHash{
    size_t operator()(const polygonPoint& s) const{
        std::hash<long double> h;
        return h(s.x) ^ h(s.y);
    }
};


#endif //POLYGONBACKSHAPE_PATH_POLYGONPOINT_H
