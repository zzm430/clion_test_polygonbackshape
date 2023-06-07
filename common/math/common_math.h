//
// Created by zzm on 2023/6/1.
//
#ifndef POLYGONBACKSHAPE_COMMON_MATH_H
#define POLYGONBACKSHAPE_COMMON_MATH_H
namespace common{
    class  commonMath{
    public:
//1.计算x范围内最小值
//2.计算y范围内最小值
//3.计算凹凸多边形内哪些点是凹点、凸点
//4.计算两点之间叉积
//5.计算两向量方向是否一致
//6.计算两条线段一条沿着另外一条移动固定距离d
//7.已知多边形中一个点，求其前一个点和后一个点按照逆时针
//8.已知一条线段，求其延长一边固定距离后的线段
//9.已知一条线段，求其延长两边固定距离后的线段
        commonMath() = default;
        virtual ~commonMath() = default;

        static  double get_point_min_x( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> x_value;
                for(auto i : points){
                    x_value.push_back(i.x);
                }
                min_value = *std::min_element(x_value.begin(),x_value.end());


                return min_value;
        }

        static  double get_point_min_y( std::vector<Point>& points ) {
                double min_value;

                std::vector<double> y_value;
                for(auto i : points){
                    y_value.push_back(i.y);
                }
                min_value = *std::min_element(y_value.begin(),y_value.end());

                return min_value;
        }
        //计算两个向量的叉积
        static double cross(const Point &v1, const Point &v2){
            return v1.x * v2.y - v1.y * v2.x;
        }
        //判断这三个点构成的三角形是否是凸三角形
        static bool  isConvex(const Point& p, const Point & prev, const Point & next){
            Point v1(prev.x - p.x, prev.y - p.y);
            Point v2(next.x - p.x,next.y-p.y);
            return cross(v1,v2) > 0.0;
        }
        //计算得到凹凸多边形的凸点或凹点
        static  std::vector<Point> IsHollow_vec(std::vector<Point> curveLoopPoints){
            //假设法向量判断凹凸性，检测多边形上上是否有凸点，每个顶点转向都应该一致，不一致则为凹点
            //假设给出的点是顺时针，则选择的是给出的是凸点，逆时针给出的是凹点
            std::vector<Point> hollowPoints;
            auto num = curveLoopPoints.size();
            Point p,prev,next;
            for(int i = 0; i < num ;i++){
                p = curveLoopPoints[i];
                prev = curveLoopPoints[(i + num -1) % num];
                next = curveLoopPoints[(i +1) % num];
                if(isConvex(p,prev,next)){
                    hollowPoints.push_back(p);
                }
            }
            return hollowPoints;
        }
        //计算一条线段沿着另外一条线段的方向移动固定的距离distance
        static std::vector<Point>   computeLineTranslationPoints(std::vector<Point> initialPoints,
                                                            std::vector<Point> directionPoints,
                                                            double distance){
                double Bx = directionPoints[1].x - directionPoints[0].x;
                double By = directionPoints[1].y - directionPoints[0].x;
                double length_B = sqrt(Bx * Bx + By * By);
                double u_Bx = Bx / length_B;
                double u_By = By / length_B;
                double p1x_new = initialPoints[0].x + distance * u_Bx;
                double p1y_new = initialPoints[0].y + distance * u_By;
                double p2x_new = initialPoints[1].x + distance * u_Bx;
                double p2y_new = initialPoints[1].y + distance * u_By;
                Point p1,p2;
                p1.x = p1x_new;
                p1.y = p1y_new;
                p2.x = p2x_new;
                p2.y = p2y_new;
                std::vector<Point>  result;
                result.push_back(p1);
                result.push_back(p2);
                return result;
        }
        //对于给定点，求出给定点的前向点和后向点
        static std::vector<Point>  computeForwardAndBackPoints(std::vector<Point> polyPoints,
                                                               Point topPoint){
            //要求polyPoints中没有重复点
            polyPoints.pop_back(); //因为最后一个点是起点，所以删除
            int num =  polyPoints.size();
            std::vector<Point> storagePoints;
            Point p, prev,next;
            for(int i = 0; i < num ;i++){
                p = polyPoints[i];
                prev = polyPoints[(i + num -1) % num];
                next = polyPoints[(i +1) % num];
                if(p == topPoint){
                    storagePoints.push_back(prev);
                    storagePoints.push_back(next);
                    return storagePoints;
                }
            }
        }

    };
}
#endif //POLYGONBACKSHAPE_COMMON_MATH_H
