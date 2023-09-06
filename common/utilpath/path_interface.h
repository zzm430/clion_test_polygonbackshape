//
// Created by zzm on 2023/4/11.
//

#ifndef UNTITLED3_PATH_INTERFACE_H
#define UNTITLED3_PATH_INTERFACE_H
#include <stdint.h>
#include <vector>
class pathInterface{
public:
    pathInterface();
    virtual ~pathInterface();

    enum  class pathPointMode1 :uint8_t{
       NON_WORKING_AREA_WITHIN_BOUNDARY  = 0,   //边界内非工作区
       WORK_AREA = 1,                           //工作区
       TURNING_AREA = 2,                        //转弯区
       TRANSITION_ZONE = 3,                     //过渡区
       AVOIDANCE = 4,                           //避障
       NON_WORKING_AREAS_ADMISSION              //入场的非工作区
    };
    enum  class  pathPointMode2 :uint8_t {
        SWITCH_BACK_FORWARD = 0,                //后退前进切换
        FORWARD = 1,                            //前进
        BACK = 2,                               //后退
        JOB_DONE_PARKING = 3,                   //作业完成停车
        ACTIVELY_SWITCH_TO_MANUAL_MODE          //主动切换至人工模式点
    };
    struct  pathPoint {
        double x;
        double y;
        pathPointMode1 path_point_mode1;
        pathPointMode2 path_point_mode2;
        int            ridge_number;
    };
    std::vector<pathPoint>  storage_path_point;
private:
};
#endif //UNTITLED3_PATH_INTERFACE_H
