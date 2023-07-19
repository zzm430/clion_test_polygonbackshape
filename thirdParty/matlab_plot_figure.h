//
// Created by zzm on 2023/7/18.
//

#ifndef POLYGONBACKSHAPE_MATLAB_PLOT_FIGURE_H
#define POLYGONBACKSHAPE_MATLAB_PLOT_FIGURE_H
#include <MatlabDataArray.hpp>
#include <MatlabEngine.hpp>
#include <iostream>
#include <Planning/path_polygonplan.h>
#include <vector>

namespace  mplot{
    static void  figure_line(std::vector<aiforce::Route_Planning::polygonPoint> & linepts){
        // 创建MATLAB引擎
        std::unique_ptr<matlab::engine::MATLABEngine> engine = matlab::engine::connectMATLAB();

//        int num = linepts.size();
//        for(int i = 0;i < num;i++){
//            matlab::data::TypedArray<double> xArray = matlab::data::TypedArray<double>({, x.end()});
//            matlab::data::TypedArray<double> yArray = matlab::data::TypedArray<double>({y.begin(), y.end()});
//            std::string command = "plot([" + std::to_string(x1) + ", " + std::to_string(x2) + "], [" +
//                                  std::to_string(y1) + ", " + std::to_string(y2) + "])";
//            // 执行MATLAB命令
//           eval(engine, command.c_str());
//        }
//        // 创建x和y坐标向量
//        std::vector<double> x = {1, 2, 3, 4, 5};
//        std::vector<double> y = {1, 4, 9, 16, 25};
//
//        // 将x和y转换为MATLAB数据数组
//        matlab::data::ArrayFactory factory;
//        auto xData = factory.createArray(2,x,y);
//        matlab::data::TypedArray<double> yData = matlab::data::TypedArray<double>(y);
//
//        // 绘制20条线段
//        for (int i = 0; i < 20; i++) {
//            // 在MATLAB中绘制线段
//            engine->eval(u"plot(" + xData.toAscii() + u", " + yData.toAscii() + u")");
//
//            // 更新x和y坐标向量（示例中简单地将每个元素加1）
//            for (auto& val : x) {
//                val += 1;
//            }
//            for (auto& val : y) {
//                val += 1;
//            }
//        }

        // 等待用户关闭绘图窗口
        std::cout << "Press any key to exit..." << std::endl;
        std::cin.get();
    }
}



#endif //POLYGONBACKSHAPE_MATLAB_PLOT_FIGURE_H
