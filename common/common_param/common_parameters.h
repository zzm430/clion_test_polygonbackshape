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
#define RIDGE_WIDTH_LENGTH        4       //垄宽设置
#define MAX_TRAVERSALS_NUMBERS    500     //最大遍历次数
#define CHOOSE_NARROW_TYPE                //定义则选择点内缩，未定义选择边内缩
#define JUDGE_CLOCKWISE            true   //判断回字形按照顺时针走还是逆时针，true为逆时针，false为顺时针
#define  SET_STARTTURN_DISTANCE   6       //设置起始转弯距离
#define  SET_ENDTURN_DISTANCE     6       //末尾转弯距离设置
#define  SET_CONVERTDIRECTION_DIST 0.1    //设置转换方向点的间距
#define  SET_CONVERTDIRECTION_COUNT 3     //设置转换方向点的个数
#define  SET_REVERSING_FLAG       true    //设置倒车标志位
#define  SET_BACK_DIS             20      //设置倒车间隔
#define  SET_VIRTUAL_LINE_LENGTH  2500    //设置虚拟线的长度
#define  SET_POLY_TRANSFER_THR     2      //设置开始走平行路线的阈值
#define  SET_FLAG_INNER_SKELETON_PATH  false //设置是否走内部直骨架路径
#define  SET_HEADLAND_WIDTH_WHL   4       //地头宽度(具体指的是CPA算法中的whl)

//LOG以及相关txt信息输出
#define DEBUG_MIDDLE_INFO                 //控制输出的txt文件debug使用

//回型部分弯道起始点和终点相关参数设置
#define WROBOT                    1.8       //widtth of robot
#define DCRF                      1.5        //distance center of rotation to front side
#define DCRR                      0.8        //distance center of rotation to rear hitch
#define DCRI                      0.8       //车体相关参数1,distance center of rotation to implement
#define DWA                       0.3       //车体相关参数2,distance to working area
#define LIM                       0.8       //implement length
#define LWORK                     0.5       //车体相关参数3, working length
#define WIM                       4         //Implement width
#define WWORK                     4         //垄宽,working width
#define OFFIM                     0         //implement offset
#define OFFWORK                   0         //偏移量农具, working offset
#define MINEFF                    0.975     //带有农具时农具有效工作宽度的系数,动态可调,能动态更改圆的半径
#define DIS_1                     1.1       //农具到车后轮中心的距离
#define DIS_2                     1.6       //包含农具到车后轮中心的距离
#define TRACTOR_WIDTH             1         //拖拉机车头宽度
#define TRACTOR_HEIGH             2         //拖拉机车头长度

//FT-CPA-CV
#define DNCZ                      0.15       //the distance for the no-crop zone，对于弯道属于part1、part2、part3起重要作用
#define DEXTRA                    0.0       //the distance an implement can cross the field border


//鱼尾弯道参数
//#define  F1                       1
//#define  F2                       1
//#define  F3                       1
#define  CIRCLE_RIDIS_R             6        //圆半径
#define  MAX_CIRCLE_RIDIS_R         12       //设置的最大圆半径
#define  FISHNail_DIFF_DIS          0.01      //间隔20cm

//回字形规划方法选择
//#define  RECTANGLE_BACK_SHAPE           //矩形回字形规划
#define  POLYGON_BACK_SHAPE               //多边形回字形规划

#endif // COMMON_PARAMETERS_H
