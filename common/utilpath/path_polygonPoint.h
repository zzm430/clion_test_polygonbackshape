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


enum  class pathPtType :uint8_t{
    FORWARD  = 0 ,   //前进
    BACKWARD = 1,    //后退
    SWITCHPT = 2     //转换点
};

class  polygonPoint: public Point{
public:
    polygonPoint() = default;

    virtual  ~polygonPoint(){};

    polygonPoint( double f, double g):Point(f,g){}

    polygonPoint( double f,  double g,double heading):Point(f,g,heading){}

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
    pathPtType   pathPtType_;                  //路径点的前进方向

public:
    double distanceTo(const polygonPoint & other) const{
        return std::sqrt((x - other.x) * (x - other.x ) + (y - other.y) * (y - other.y));
    }

    void set_x(double value_x){
        x = value_x;
    }

    void set_y(double value_y){
        y = value_y;
    }

    void set_heading(double heading){
        heading_ = heading;
    }

    void set_theta(double theta){
        theta_ = theta;
    }

    void set_s(double s){
        s_ = s;
    }

    void set_kappa(double kappa){
        kappa_ = kappa;
    }

    void set_dkappa(double dkappa){
        dkappa_ = dkappa;
    }

    double get_theta(){
        return theta_;
    }

    double get_s(){
        return s_;
    }

    double get_kappa(){
        return kappa_;
    }

    double get_dkappa(){
        return dkappa_;
    }

    double s()  const {
        return s_;
    }

    double kappa() const {
        return kappa_;
    }

    double theta() const {
        return theta_;
    }

    double dkappa()  const {
        return dkappa_;
    }

    double ddkappa() const {
        return ddkappa_;
    }

    double heading() const {
        return heading_;
    }




private:

    double theta_;
    double s_;
    double kappa_;
    double dkappa_;
    double ddkappa_;
    double heading_;

};

struct polyPointHash{
    size_t operator()(const polygonPoint& s) const{
        std::hash<long double> h;
        return h(s.x) ^ h(s.y);
    }
};


#endif //POLYGONBACKSHAPE_PATH_POLYGONPOINT_H
