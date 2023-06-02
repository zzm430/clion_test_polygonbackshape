#ifndef GEOMETRIC_ALGORITHM_H
#define GEOMETRIC_ALGORITHM_H
/*
计算几何
目录
㈠ 点的基本运算
1. 平面上两点之间距离 1
2. 判断两点是否重合 1
3. 矢量叉乘 1
4. 矢量点乘 2
5. 判断点是否在线段上 2
6. 求一点饶某点旋转后的坐标 2
7. 求矢量夹角 2

㈡ 线段及直线的基本运算
1. 点与线段的关系 3
2. 求点到线段所在直线垂线的垂足 4
3. 点到线段的最近点 4
4. 点到线段所在直线的距离 4
5. 点到折线集的最近距离 4
6. 判断圆是否在多边形内 5
7. 求矢量夹角余弦 5
8. 求线段之间的夹角 5
9. 判断线段是否相交 6
10.判断线段是否相交但不交在端点处 6
11.求线段所在直线的方程 6
12.求直线的斜率 7
13.求直线的倾斜角 7
14.求点关于某直线的对称点 7
15.判断两条直线是否相交及求直线交点 7
16.判断线段是否相交，如果相交返回交点 7
17.根据两点求直线斜率对应的角度 （0～Pi）

㈢ 多边形常用算法模块
1. 判断多边形是否简单多边形 8
2. 检查多边形顶点的凸凹性 9
3. 判断多边形是否凸多边形 9
4. 求多边形面积 9
5. 判断多边形顶点的排列方向，方法一 10
6. 判断多边形顶点的排列方向，方法二 10
7. 射线法判断点是否在多边形内 10
8. 判断点是否在凸多边形内 11
9. 寻找点集的graham算法 12
10.寻找点集凸包的卷包裹法 13
11.判断线段是否在多边形内 14
12.求简单多边形的重心 15
13.求凸多边形的重心 17
14.求肯定在给定多边形内的一个点 17
15.求从多边形外一点出发到该多边形的切线 18
16.判断多边形的核是否存在 19

㈣ 圆的基本运算
1 .点是否在圆内 20
2 .求不共线的三点所确定的圆 21

㈤ 矩形的基本运算
1.已知矩形三点坐标，求第4点坐标 22

㈥ 常用算法的描述 22

㈦ 补充
1．两圆关系： 24
2．判断圆是否在矩形内： 24
3．点到平面的距离： 25
4．点是否在直线同侧： 25
5．镜面反射线： 25
6．矩形包含： 26
7．两圆交点： 27
8．两圆公共面积： 28
9. 圆和直线关系： 29
10. 内切圆： 30
11. 求切点： 31
12. 线段的左右旋： 31
13．公式： 32

(八) 常用算法的样条曲线
1. BSpline: 1

*/
#include <QVector>
#include <QPointF>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <map>
#include <limits.h>
#include <vector>
#define MAXV 1000
#define	INF 1E200
#define	EP	1E-4
#define PI 3.1415926535897932384626433832795
#define PN 1000
using namespace std;
bool IsEqual(double a,double b);   //判断两值是否相等
struct PointF
{
    double x, y;
    PointF(double a, double b) { x=a; y=b;}
    PointF(){}
    PointF operator =(PointF temp)
    {
        this->x =temp.x;
        this->y =temp.y;
        return *this;
    }
    friend PointF operator - (PointF a, PointF b) //两点坐标相加
    {
        return PointF(a.x - b.x, a.y - b.y);
    }
    friend PointF operator + (PointF a, PointF b) //两点坐标相减
    {
        return PointF(a.x + b.x, a.y + b.y);
    }
    PointF operator * (const double k) //
    {
        return PointF(k*x, k*y);
    }
    PointF operator / (const double k)
    {
        return PointF(x/k, y/k);
    }
    double operator * (const PointF a) //点的点乘
    {
        return (this->x * a.x + this->y * a.y);
    }
    double operator ^ (const PointF a) //点的叉乘
    {
        return this->x * a.y - this->y * a.x;
    }
    bool operator == (PointF b) const {//判断两个点是否重合
        return IsEqual(x,b.x)&&IsEqual(y,b.y);
    }
};
bool IsClockwise(vector<PointF> pointList); //判断点列是否为顺时针
//1. 平面上两点之间距离 1
double Dist_pt(PointF p1,PointF p2);                // 返回两点之间欧氏距离

class DP_Point  //道格拉斯普克算法
{
public:
    int ID;
    //string Name;
    double x;
    double y;
    bool isRemoved = false;
};
struct LINESEG //线段
{
    PointF s;
    PointF e;
    double length;
    LINESEG(PointF a, PointF b) { s=a; e=b;}
    LINESEG(){}
    double get_SegLength(){length=Dist_pt(s,e);return length;}
    //~LINESEG(){}
};
struct Circle
{
    PointF O;
    double R;
    Circle(PointF o_,double r_) {O=o_;R=r_;}
    Circle() {}
};
struct LINE //直线一般式描述法
{
public:
    LINE (PointF p, PointF q)
    {
        a = p.y - q.y;
        b = q.x - p.x;
        c = - a * p.x - b * p.y;
        norm(); //
        get_slope();
        get_theta();  //(0 ~ pi)
    }
    void norm() { double z = sqrt(a*a + b*b); if ( std::abs(z)>EP ) { a /= z,  b /= z,  c /= z; } }
    double dist(PointF p) const { return a * p.x + b * p.y + c; }  //点到直线的距离，参数必须是归一化的
    void get_slope() {if(abs(a) < 1e-20) k=0;else if(abs(b) < 1e-20) k=INF;else k=-(a/b);}
    void get_theta()
    {
        if(abs(a)< EP)
            theta=0;
        else if(abs(b)< EP)
            theta=PI/2;
        else
        {
            if(k>0)
                theta=atan(k);
            else
                theta=PI+atan(k);
        }
    }
public:
    double a,b,c,k,theta;
};


struct Polygon //多边形点逆时针排列
{
    int n=0;
//    PointF V[PN];
//    LINESEG E[PN];
    vector<PointF> Vs{};
    vector<LINESEG> Es{};
    void input(int _n,std::vector<PointF> fdpts)
    {
        n = _n;
        Vs.clear();
        for (int i = 0; i < n; i ++ )
        {
            Vs.push_back(fdpts[i]);
        }
        if(IsClockwise(Vs))
            reverse(Vs.begin(), Vs.end());

    }
    void add(PointF q)
    {
        Vs.push_back(q);
        n++;
    }
    void getEdge()
    {
        Es.clear();
        for (int i = 0; i < n; i ++ )
        {
//            E[i] = LINESEG(V[i], V[(i+1)%n]);
//            LINESEG Seg_temp=LINESEG(V[i], V[(i+1)%n]);
//            Edges.push_back(Seg_temp);
            Es.emplace_back(LINESEG(Vs[i], Vs[(i+1)%n]));

        }
    }
    void init()
    {
        n=0;
        Vs.clear();
        Es.clear();
    }
};
double get_distQPointF(QPointF p1,QPointF p2);  //获取两点间的距离
QPointF getCrossQPointF(QPointF p1, QPointF p2, QPointF p3, QPointF p4);//求出交点,直线p1p2与直线p3p4
bool Is_PtOnSegment(PointF &k, PointF &a, PointF &b);              //点是否在线段上
bool Is_PtInPloygon(PointF *s, int n, PointF &k);                 //点是否在多边形内
bool Is_Seg1CrosSeg2(PointF &a, PointF &b, PointF &c, PointF &d);   //判断线段ab与线段cd是否相交
bool Is_SegInPolygon(PointF *v, int n, PointF &a, PointF &b);      //线段是否在多边形内
double get_PtAlpha(double dx,double dy);  //0~2PI

//㈠ 点的基本运算
//2. 判断两点是否重合 1
//3. 向量点积   <0:V02，V01夹角为锐角；  =0：V02，V01夹角为直角；  <0:V02，V01夹角为钝角；
double VectDot(PointF p0,PointF p1,PointF p2);  //向量p0p1,向量p0p2
//4. 向量叉乘   <0:V02在V01的顺时针方向； =0：三点共线，V01V02共线同向或者反向；  <0:V02在V01的逆时针方向；
// 叉乘的模表示以V01，V02组成的平行四边形的面积
double VectCross(PointF p0,PointF p1,PointF p2); //向量p0p1,向量p0p2
//5. 判断点是否在线段上(点在线段端点也算在线段上)，(1)三点共线 (2) 点p在以线段seg为对角线的矩形内
bool Is_PtonSegment(LINESEG seg,PointF p);
//5. 判断点是否在直线上，已知直线的a,b,c
bool Is_PtOnLine(LINE l,PointF p);
//6. 求一点饶某点旋转后的坐标，返回点p以点o为圆心逆时针旋转alpha(单位：弧度)后所在的位置
PointF rotate_aroundPt(PointF o,double alpha,PointF p);
/*7. 求矢量夹角  返回顶角在o点，起始边为os，终止边为oe的逆时针夹角(单位：弧度)
 * 	角度小于pi，返回正值   角度大于pi，返回负值 */
double Vect_angle(PointF o,PointF s,PointF e);


//㈡ 线段及直线的基本运算
/*1.判断点与线段的关系, 向量 seg.sP 在 seg.sseg.e上的投影点 与 seg关系
本函数是根据下面的公式写的，P是点C到segAB所在直线的垂足

                AC dot AB
        r =     ---------
                 ||AB||^2
             (Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
          = -------------------------------
                          L^2
    r has the following meaning:
        r=0      P = seg.s
        r=1      P = seg.e
        r<0		 P is on the backward extension of AB
        r>1      P is on the forward extension of AB
        0<r<1	 P is interior to AB
*/
double relation_PtSeg(PointF p,LINESEG seg);
//2. 求点p 到线段所在直线垂线的垂足
PointF perpendicular(PointF p,LINESEG seg);
/*3. 点到线段的最近点 求点p到线段seg的最短距离,并返回线段上距该点最近的点np
注意：np是线段seg上到点p最近的点，不一定是垂足*/
double PtoSegdist(PointF p,LINESEG seg,PointF &np);
//4. 点到线段所在直线的距离
double PtoLinedist(PointF p,LINESEG l);
//5. 计算点到折线集的最近距离,并返回最近点
double PtoPointset(vector<PointF> pointset,PointF p,PointF &q);
//6. 判断圆是否在多边形内
bool CircleInsidePolygon(PointF center,double radius,Polygon plg);
//7. 求矢量夹角余弦
double cos_vect(LINESEG l1,LINESEG l2);
//8. 求线段之间的夹角
double Seg_angle(LINESEG l1,LINESEG l2);
//9. 判断两线段是否相交   如果线段u和v相交(包括相交在端点处)时，返回true
//判断P1P2跨立Q1Q2的依据是：( P1 - Q1 ) × ( Q2 - Q1 ) * ( Q2 - Q1 ) × ( P2 - Q1 ) >= 0。
//判断Q1Q2跨立P1P2的依据是：( Q1 - P1 ) × ( P2 - P1 ) * ( P2 - P1 ) × ( Q2 - P1 ) >= 0。
bool intersect_seg2seg(LINESEG u,LINESEG v);
//10.判断两线段是否相交但不交在端点处  (线段u和v相交)&&(交点不是双方的端点) 时返回true
bool intersect_seg2seg_notendpt(LINESEG u,LINESEG v);
//11.线段v所在直线与线段u相交时返回true；方法：判断线段u是否跨立线段v
bool intersect_seg2line(LINESEG u,LINESEG v);
//12.求直线的斜率 根据直线解析方程返回直线的斜率k,水平线返回 0,竖直线返回 1e200
double line_slope(LINE l);
//13.求直线的倾斜角 返回直线的倾斜角alpha ( 0 - pi)
double line_alpha(LINE l);
//14.求点关于某直线的对称点 求点p关于直线l的对称点
PointF symmetry(LINE l,PointF p);
//15.判断两条直线是否相交及求直线交点  如果两条直线 l1(a1*x+b1*y+c1 = 0), l2(a2*x+b2*y+c2 = 0)相交，返回true，且返回交点p
bool line_intersect(LINE l1,LINE l2,PointF &p); // 是 L1，L2
//16.判断线段是否相交，如果相交返回交点 如果线段l1和l2相交，返回true且交点由(inter)返回，否则返回false
bool segment_intersect(LINESEG l1,LINESEG l2,PointF &inter);
//17.根据两点求直线斜率对应的角度 （0～Pi）
double get_theta_k(double k);
double get_theta_Pt(PointF pt1,PointF pt2);

//㈢ 多边形常用算法模块 | 如果无特别说明，输入多边形顶点要求按逆时针排列
//1. 判断多边形是否简单多边形 简单多边形定义:a.循环排序中相邻线段对的交是他们之间共有的单个点 b.不相邻的线段不相交
bool IsSimple_Polygon(Polygon plg); //每条边与不相邻的其他边是否没有交点
//2. 检查多边形顶点的凸凹性 第i个顶点是凸顶点则为true 用map存储
void IsConvex_Vertex(Polygon plg,std::map<int,bool> &table);
//3. 判断多边形是否凸多边形
bool IsConvex_Polygon(Polygon plg);
//4. 求多边形面积 逆时针面积为正 顺时针面积为负
double area_of_polygon(Polygon plg);
//5. 判断多边形顶点的排列方向，方法一 顶点按逆时针排列，返回true
bool IsConterClockwise(Polygon plg);
//6. 判断多边形顶点的排列方向，方法二 10
//7. 射线法判断点是否在多边形内,要求polygon为简单多边形(凹凸均可)，顶点逆时针排列
//  如果点在多边形内：返回0；     如果点在多边形边上：返回1；   如果点在多边形外：返回2
int Is_PtInPolygon(Polygon plg,PointF q);
//8. 判断点是否在凸多边形内，注意只适用于凸多边形
bool Is_PtInConvexPolygon(Polygon plg,PointF q); // 可用于三角形！
//9. 寻找点集凸包的graham算法
/*PointSet为输入的点集；pts为输出的凸包上的点集，按照逆时针方向排列;*/
vector<PointF> ConvexClosure_Graham(vector<PointF> PointSet);
//10.寻找点集凸包的卷包裹法
/*PointSet为输入的点集；pts为输出的凸包上的点集，按照逆时针方向排列;*/
//vector<PointF> ConvexClosure(vector<PointF> PointSet);
//11.判断线段是否在多边形内 简单多边形；线段或线段端点在多边形边上，视为线段不在多边形内
/*必要条件一：线段的两个端点都在多边形内； 必要条件二：线段和多边形的所有边都不内交；*/
bool Is_SegInsidePolygon(Polygon plg,LINESEG l);
//判断多边形plg1是否在多边形plg2内部
bool Is_Plg1InPlg2(Polygon plg1,Polygon plg2);
//12.求简单多边形的重心 15
//13.求凸多边形的重心 17
//14.求肯定在给定多边形内的一个点 17
//15.求从多边形外一点出发到该多边形的切线 18
//16.判断多边形的核是否存在 19

//㈣ 圆的基本运算
//1 .点是否在圆内 点p在圆内(包括边界)时，返回true
/*用途	： 因为圆为凸集，所以判断点集，折线，多边形是否在圆内时，	只需要逐一判断点是否在圆内即可。*/
bool Is_PtInCircle(Circle C,PointF p);
//2 .求不共线的三点所确定的圆 21
bool PtsGenerateCircle(PointF p1,PointF p2,PointF p3,Circle &c);

//㈤ 矩形的基本运算
//1.已知矩形三点坐标，求第4点坐标 22
// 已知矩形的三个顶点(a,b,c)，计算第四个顶点d的坐标. 注意：已知的三个顶点可以是无序的
PointF Rect4thPt(PointF a,PointF b,PointF c);
//㈥ 常用算法的描述 22

//㈦ 补充
//1．两圆关系： /* 两圆：相离：return 1； 外切：return 2； 相交：return 3； 内切：return 4； 内含：return 5；*/
int CircleRelation(Circle C1,Circle C2);
//2．判断圆是否在矩形内：
bool Is_CircleInRec(Circle C1,PointF pr1, PointF pr2, PointF pr3, PointF pr4);
//3．点到平面的距离：点到平面的距离,平面用一般式表示ax+by+cz+d=0
double P2planeDist(double x, double y, double z, double a, double b, double c, double d);
//4．点是否在直线同侧： 两个点是否在直线同侧，是则返回true
bool Is_PtsOnSameSide(PointF p1, PointF p2, LINE line);
//5．镜面反射线：
void LineReflect(LINE l1,LINE l2,LINE &l);
//6．矩形包含： 26
//7．两圆交点： 两圆已经相交（相切）
void  C1CrossC2(Circle c1,Circle c2,PointF &rp1,PointF &rp2);
//8．两圆公共面积： 28
//9. 圆和直线关系：返回直线与圆的交点个数
int CircleRelationLine(Circle C,LINE l,PointF &rp1,PointF &rp2);
//10. 内切圆： 30
//11. 求切点： 过圆外一点与圆相切的两条切线的切点
void CircleCutPt(Circle C,PointF sp,PointF &rp1,PointF &rp2);
//12. 线段的左右旋： 31
//13．公式： 32
//14. 对点的坐标按y从小到大排序，若y值相同则按x从小到大排序
bool compareValue(const PointF& pt1,const PointF& pt2);
bool Is_PtIsVertex(PointF pt,Polygon plg);
//15. 与起点相同的两向量的半径为R的圆  PtCut1为向量V01的切点，PtCut1为向量V02的切点
bool Circle_CutVector(PointF pt0,PointF pt1,PointF pt2,double theta,double R,Circle &circle,PointF &PtCut1,PointF &PtCut2);//与向量V01,向量V02 相切的圆
//16. C1C2相交，与C1C2都外切的圆， 外切圆在向量C1C2的左侧
bool get_CircleCutC1C2(Circle C1,Circle C2,LINESEG seg,double R,Circle &C);

//专供函数
//直线与多边形的交点，边在直线上，认为结束点为交点；多边形顶点在直线上，则认为线段结束点为交点，起始点不属于交点；交点集合按从y值从小到大；
bool LineCrossPolygon(LINE l,Polygon plg,vector<PointF> &cross_pts);

//有效线段长度大于0返回true；直线与多变形相交的在多变行内部的线段，直线与外边重合认为该线段不可用,线段距离小于阈值L_THR不可用，order_flag 为true按y升序
bool LineCrossPolygon_Seg(LINE l,Polygon plg,vector<LINESEG> &cross_seg,double L_THR,bool order_flag);

//多边形边上两点之间的多边形顶点，距离最短; 从p1到p2逆时针或者 从p1到p2逆时针
double Pt1VertexPt2_dist(Polygon plg,const PointF pt_start,const PointF pt_end,vector<PointF> &SortPts); //获得p1p2和之间的边界点，顺序和边界点的原始顺序一致
//多边形上两点Pt1和Pt2之间的多边形顶点(包含pt_start和pt_end)，Pt1为Pts第一个点，Pt2为Pts的最后一个点
bool PlgVertexBetPts(PointF Pt1,PointF Pt2,Polygon plg,vector<PointF> &Pts);
//点集是否能形成凸多边形
bool  pt_concave_convex(vector<PointF> pList_temp,int num);
//点是否在多边形的边上，
bool Is_PtOnPolygonSeg(Polygon plg,PointF Pt,LINESEG &seg,int &seg_num);
//多边形内缩外扩 dist正数为内缩， 负数为外扩；没有检测内缩外扩之后是否自交。
Polygon Plg_Expand(const Polygon plg,double dist);
void SelfIntersectedPolygon2NormalPolygon(Polygon plg1,Polygon &plg2);
//道格拉斯普克算法 点的稀疏
void DP(vector<DP_Point> &Points, int begin, int end, double threshold);
void Polygon2DP_Polygon(Polygon plg,Polygon &plg_re);

//(八) 样条曲线
class CBSpline
{
public:
    CBSpline(void);
    ~CBSpline(void);

    bool



    TwoOrderBSplineSmooth(vector<PointF> pts,vector<PointF> &final_pts);
    bool TwoOrderBSplineInterpolatePt(vector<PointF> pts,vector<PointF> &final_pts,vector<int> InsertNum);
    double F02(double t);
    double F12(double t);
    double F22(double t);

    bool ThreeOrderBSplineSmooth(vector<PointF> pts,vector<PointF> &final_pts);
    bool ThreeOrderBSplineInterpolatePt(vector<PointF> pts,vector<PointF> &final_pts,vector<int> InsertNum);
    double F03(double t);
    double F13(double t);
    double F23(double t);
    double F33(double t);
};
bool get_Bspline_pts(vector<PointF> key_pts,double gap,int k,vector<PointF> &final_pts); //k次B样条(K为2或者3)，生成点集点间距为gap
#endif // GEOMETRIC_ALGORITHM_H
