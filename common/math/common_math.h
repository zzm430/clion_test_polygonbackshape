//
// Created by zzm on 2023/6/1.
//
#ifndef POLYGONBACKSHAPE_COMMON_MATH_H
#define POLYGONBACKSHAPE_COMMON_MATH_H
namespace common{
    class  commonMath{
    public:
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

    };
}
#endif //POLYGONBACKSHAPE_COMMON_MATH_H
