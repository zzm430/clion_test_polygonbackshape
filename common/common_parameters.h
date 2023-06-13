#ifndef COMMON_PARAMETERS_H
#define COMMON_PARAMETERS_H

//opencv  max rectangle
#define  POLYGON_ZOOM_FACTOR 10        //多边形缩放倍数
#define  MAX_PNG_SIZE 20000            //opencv处理的图片大小
#define  INNER_RECT_GRID_DENSITY 2     //利用柱状直方图计算最大内接矩形的精度 >=1




//debug
//#define  DEBUG                       //用于opencv相关dubeg

//矩形回字形规划相关
#define  USE_GCS true                  //使用大地坐标系
#define  BACKSHAPE_SPACE_SETTING  4    //回字形间距设置
#define  REEDSHEPP_SAMPLE_INTERVAL 0.2 //reedshepp算法采样间隔


//任意多边形回字形规划相关
#define RIDGE_WIDTH_LENGTH        4      //垄宽设置
#define MAX_TRAVERSALS_NUMBERS    500      //最大遍历次数
#define CHOOSE_NARROW_TYPE               //定义则选择点内缩，未定义选择边内缩


//LOG以及相关txt信息输出
#define DEBUG_MIDDLE_INFO              //控制输出的txt文件debug使用



//回字形规划方法选择
//#define  RECTANGLE_BACK_SHAPE          //矩形回字形规划
#define  POLYGON_BACK_SHAPE            //多边形回字形规划

#endif // COMMON_PARAMETERS_H
