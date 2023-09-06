#ifndef PLOTTER_H
#define PLOTTER_H

#include "thirdParty/tabulate.hpp"
#include <easylogging++.h>
#include <sstream>
#include <chrono>
#include <common/utilpath/path_common.h>
#include <common/utilpath/path_polygonPoint.h>


//using namespace tabulate;


namespace  commonPlot{
    class plotter{
    public:
        plotter() = delete;
        ~plotter() = delete;

//    static void  computeAllRunTime(const double elapsed_time_secs,
//                                   const double elapsed_time_msecs){
//
//        Table format;
//
//        std::stringstream  string_s;
//        std::stringstream  string_ms;
//        string_s  << "all time s is : " << std::fixed << std::setprecision(3)
//              << elapsed_time_secs   << " s " ;
//        string_ms << "all time ms is : " << std::fixed << std::setprecision(3)
//               << elapsed_time_msecs  << " ms"  ;
//        format.add_row({string_s.str(),string_ms.str()});
//
//        format.format().border_color(Color::red);
//        format
//        .format()
//        .border_color(Color::red)
//        .font_color(Color::white)
//        .font_style({FontStyle::bold})
//        .padding_top(0)
//        .padding_bottom(0);
//        LOG(INFO) << "\n" << format << "\n";
//    }
//        static  void pyplotLine(std::vector<aiforce::Route_Planning::polygonPoint>& polylines){
//            //创建图形对象和子图
//            plt::figure();
//            auto ax = plt::subplot(2,2,1);
//
//            //绘制线段
//            for(size_t i = 0; i < polylines.size();i++){
//                if(i % 2 == 0){
//                    std::vector<double> x = {polylines[i].x,polylines[i].y};
//                    std::vector<double> y = {polylines[i+1].y,polylines[i+1].y};
//                    ax.plot(x,y);
//                    i++;
//                }
//
//            }
//            ax.set_title("线段的可视化");
//            ax.set_xlabel("X轴");
//            ax.set_ylabel("Y轴");
//
//            //显示图形
//            plt::show();
//
//        }

    };
}

#endif // PLOTTER_H
