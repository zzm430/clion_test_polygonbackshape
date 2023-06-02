#include <Planning/path_baseplan.h>


namespace aiforce{
namespace Route_Planning
{



//获取关键点信息,这里的关键点信息主要用做回字形的内部处理
std::vector<Point>  pathBasePlan::getKeyPoints(){
    return key_points_;
}


//获取垄数
int  pathBasePlan::getRidges(){
       return  ridge_numbers_;
}


//std::unordered_map<int,std::vector<pathBasePlan::curvePoint>>  pathBasePlan::getCurvePoints(){
//    return key_curve_points_;
//}


}
}
