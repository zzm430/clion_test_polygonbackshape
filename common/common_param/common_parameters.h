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
#define JUDGE_CLOCKWISE           true   //判断回字形按照顺时针走还是逆时针，true为逆时针，false为顺时针
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
#define DEBUG_CPA_INFO                    //控制输出的txt文件debug使用
#define DEBUG_PJPO_INFO                   //控制PJPO相关算法的debug输出


//回型部分弯道起始点和终点相关参数设置
#define WROBOT                    1.8       //widtth of robot
#define DCRF                      1.5        //distance center of rotation to front side
#define DCRR                      0.8        //distance center of rotation to rear hitch
#define DCRI                      0.8       //车体相关参数1,distance center of rotation to implement
#define DWA                       0.3       //车体相关参数2,distance to working area
#define LIM                       3.8       //implement length
#define LWORK                     3.5       //车体相关参数3, working length
#define WIM                       4         //Implement width
#define WWORK                     4         //垄宽,working width
#define OFFIM                     0         //implement offset
#define OFFWORK                   0         //偏移量农具, working offset
#define MINEFF                    0.975     //带有农具时农具有效工作宽度的系数,动态可调,能动态更改圆的半径
#define DIS_1                     1.1       //农具到车后轮中心的距离
#define DIS_2                     1.6       //包含农具到车后轮中心的距离
#define TRACTOR_WIDTH             1         //拖拉机车头宽度
#define TRACTOR_HEIGH             2         //拖拉机车头长度

#define HEAD_WHEELBASE            2.1

//FT-CPA-CV
#define DNCZ                      0.1      //the distance for the no-crop zone，对于弯道属于part1、part2、part3起重要作用
#define DEXTRA                    0.0       //the distance an implement can cross the field border


//鱼尾弯道参数
//#define  F1                       1
//#define  F2                       1
//#define  F3                       1
#define  CIRCLE_RIDIS_R             6        //圆半径
#define  MAX_CIRCLE_RIDIS_R         12       //设置的最大圆半径
#define  FISHNail_DIFF_DIS          0.2      //间隔20cm
#define  CPA_DIFF_DIS               0.2      //CPA系列算法间隔

//回字形规划方法选择
//#define  RECTANGLE_BACK_SHAPE           //矩形回字形规划
#define  POLYGON_BACK_SHAPE               //多边形回字形规划

//将拐弯较小的弯道生成的弯道点替换为一个点
#define  REPLACE_CURVE_PATH_THR    10

//customFem算法相关参数设定
#define  APPLY_CURVATURE_CONSTRAINT  true
#define  WEIGHT_FEM_POS_DEVIATION    1e5
#define  WEIGHT_PATH_LEGNTH          1
#define  WEIGHT_REF_DEVIATION        1
#define  MAX_ITER_FEM                4000
#define  TIME_LIMIT                  3
#define  VERBOSE                     false
#define  SCALED_TERMINATION          true
#define  WARM_START_M                true
#define  BOUND                       1.7
#define  WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR 1e5
#define  SQP_SUB_MAX_ITER                      0.0
#define  SQP_FTOL                              0.0
#define  SQP_PEN_MAX_ITER                      0.0
#define  SQP_CTOL                              0.0


#define A1_DISTANCE        6
#define A3_DISTANCE        12
#define A2_DISTANCE        6
#define A4_DISTANCE        12
#define REFERENCE_DIFF_NUMBER 40

#define VIRTUAL_CENTROID_LINE 4
#define SIDE_PASS_CHOOSE   true     //true逆时针为正

#define DIFF_PTS           0.6

//static obstale manager
#define  DEBUG_STATIC_OBSTACLE
#define  SAFE_OBSTACLE_THR  0.3
#define  USE_REFERENCE_CENTER_LINE false

//PJPO算法
#define MAX_STEER_ANGLE   38            //最大转角
#define WHEEL_BASE        3.35          //前后轮轴距
#define PJPO_USE_SWITCH   true
#define DELTA_S           0.6
#define LOWER_BOUND       -5
#define UPPER_BOUND        5
#define MAX_TRANSVERSE_ACCELETATION  4   //最大侧向加速度
#define STEER_GEAR_RATIO   22.4          //传动比
#define CURVE_VEL          1             //转弯时参考速度


#endif // COMMON_PARAMETERS_H
