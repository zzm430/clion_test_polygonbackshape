//
// Created by zzm on 23-9-13.
//
//决定路径的运行模式
#ifndef POLYGONBACKSHAPE_DIFFMODE_CHOOSE_H
#define POLYGONBACKSHAPE_DIFFMODE_CHOOSE_H

enum  class curveModeChoose{
    MODE_FISHNAIL = 0,                      //鱼尾弯道处理
    MODE_REEDSHEEP = 1                      //rs弯道处理
};

enum  class curveLocationChoose{
    MODE_CUSTOM_CURVE_START_END = 0,        //自定义弯道起始点和结束点
    MODE_HEADLANDS_CURVE__START_END = 1     //论文方法起始点和结束点
};


#endif //POLYGONBACKSHAPE_DIFFMODE_CHOOSE_H
