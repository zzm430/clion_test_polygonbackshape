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
#define  CIRCLE_RIDIS_R             10       //圆半径
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
#define  BOUND                       2.5
#define  WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR 1e5
#define  SQP_SUB_MAX_ITER                      0.0
#define  SQP_FTOL                              0.0
#define  SQP_PEN_MAX_ITER                      0.0
#define  SQP_CTOL                              0.0

#define A1_DISTANCE        4
#define A3_DISTANCE        14
#define A2_DISTANCE        10
#define A4_DISTANCE        14
#define REFERENCE_DIFF_NUMBER 50

#define VIRTUAL_CENTROID_LINE 7
#define SIDE_PASS_CHOOSE  false     //从障碍物左侧绕为正

#define DIFF_PTS           0.4

//static obstale manager
//#define  DEBUG_STATIC_OBSTACLE
#define  SAFE_OBSTACLE_THR_ZERO  0.0
#define  SAFE_OBSTACLE_THR_FIRST  1
#define  SAFE_OBSTACLE_THR_SECOND 0.6
#define  CONSIDER_CARBODY_SAFE_THR 4
#define  USE_REFERENCE_CENTER_LINE false

//PJPO算法
#define MAX_STEER_ANGLE   38            //最大转角
#define WHEEL_BASE        3.35          //前后轮轴距
#define PJPO_USE_SWITCH   true
#define DELTA_S           0.6
#define LOWER_BOUND       -10
#define UPPER_BOUND        10
#define MAX_TRANSVERSE_ACCELETATION  6   //最大侧向加速度
#define STEER_GEAR_RATIO   22.4          //传动比
#define CURVE_VEL          2             //转弯时参考速度
#define PJPO_SAFE_OBSTACLE_THR  1.5
#define PJPO_SAFE_OBSTACLE_THR_FIRST 2.3


//混合AStar算法
#define TRACTOR_FRONTEDGE_TO_CENTER 1.5
#define TRACTOR_BACKEDGE_TO_CENTER 1.5
#define TRACTOR_WHEEL_BASE 1.5
#define HEIGHT 4
#define WIDTH 4
#define MAX_FRONTWHEEL_STEERANGLE_DEG 30
#define MAX_FRONTWHEEL_STEERANGLE  0.52
#define IS_SHOW_RESULT_ANALYSIS false
#define MAXIMUM_SEARCHED_NUM 30000000
#define MAXIMUM_SEARCHED_TIME 200
#define XY_RESOLUTION 1.5
#define TRACTOR_PHI_RESOLUTION 5
#define XY_2D_RESOLUTION 1.5
#define OGM_RESOLUTION 0.4
#define NEXT_NODE_NUM 10
#define STEP_SIZE 0.5
#define DELTA_T 0.5
#define FORWARD_PENALTY 1
#define BACK_PENALTY 0.01
#define GEAR_SWITHCH_PENALTY 2
#define STEER_PENALTY 2
#define STEER_CHANGE_PENALTY 5
#define TRACTOR_PHI_RESOLUTION_IN_DEG 5
#define DEBUG_HYBRIDASTAR

//定制fem避障算法参数
#define EGO_LENGTH   2.8
#define EGO_WIDTH    4
#define BACK_EDGE_TO_CENTER 0.8
#define INTERPOLATED_DELTA_S 0.3
#define REANCHORING_TRAILS_NUM 500
#define REANCHORING_LENGTH_STDDEV 0.25
#define ESTIMATE_BOUND  false
#define DEFAULT_BOUND   3
#define VEHICLE_SHORTEST_DIMENSION 1.04
#define COLLISION_DECREASE_RATIO 0.8

//customFem避障算法相关参数设定
#define  APPLY_CURVATURE_CONSTRAINT_TWO  true
#define  WEIGHT_FEM_POS_DEVIATION_TWO    1
#define  WEIGHT_PATH_LEGNTH_TWO          0
#define  WEIGHT_REF_DEVIATION_TWO        0
#define  MAX_ITER_FEM_TWO                500
#define  TIME_LIMIT_TWO                  5
#define  VERBOSE_TWO                     false
#define  SCALED_TERMINATION_TWO          true
#define  WARM_START_M_TWO                true
#define  BOUND_TWO                       6
#define  WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR_TWO 1
#define  SQP_SUB_MAX_ITER_TWO                      0.0
#define  SQP_FTOL_TWO                              0.0
#define  SQP_PEN_MAX_ITER_TWO                      0.0
#define  SQP_CTOL_TWO                              0.0

//fem + qp算法参数设定
#define QP_WEIGHT_FEM_POS_DEVIATION 1
#define QP_WEIGHT_PATH_LENGTH 0
#define QP_WEIGTH_REF_DEVIATION 0
#define QP_MAX_ITER     400
#define QP_TIME_LIMIT   5
#define QP_VERBOSE     false
#define QP_SACLED_TERMINATION true
#define QP_WARM_START   true

//根据拖拉机的后轮中心点计算农具的中心点
//#define TRAILER_POSE_CHECK
#define TRACTOR_CENTER_TO_TRAILER_CENTER_DIS 2.5
#define TRAILER_LENGTH   3
#define TRAILER_WIDTH    4
#define TRAILER_CENTER_SHIFT_DISTANCE 0.7


//fem + ipopt求解参数
#define WEIGHT_FEM_POS_DEVIATION_IPOPT  10e3
#define WEIGHT_PATH_LENGTH_IPOPT  1
#define WEIGHT_REF_DEVIATION_IPOPT   1
#define WEIGHT_CURVATURE_CONSTRAINT_SLACK_VAR_IPOPT    10e8
#define CURVATURE_CONSTRAINT_IPOPT          0.1
#define PRINT_LEVEL_IPOPT                    3
#define MAX_NUM_OF_ITERATIONS_IPOPT         100
#define ACCEEPTABLE_NUM_OF_ITERATIONS_IPOPT  120
#define TOL_IPOPT    1e-3
#define ACCEPTABLE_TOL_IPOPT  1e-3

#endif    //COMMON_PARAMETERS_H
