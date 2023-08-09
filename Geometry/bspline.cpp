#include "geometric_algorithm.h"
CBSpline::CBSpline(void)
{
}


CBSpline::~CBSpline(void)
{
}
//======================================================================
// 函数功能： 二次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： pts ：给定点序列
//          final_pts：bspline优化点
// 返回值：   bool
//======================================================================
bool CBSpline::TwoOrderBSplineSmooth(std::vector<PointF> pts,vector<PointF> &final_pts)
{
    vector<PointF> temp_pts; //  二次B样条不需要增加点数，需要将首尾点替换掉
    temp_pts=pts;
    final_pts=pts;
    int Num=pts.size();

    temp_pts[0].x=2*temp_pts[0].x-temp_pts[1].x;                  //  将折线两端点换成延长线上两点
    temp_pts[0].y=2*temp_pts[0].y-temp_pts[1].y;

    temp_pts[Num-1].x=2*temp_pts[Num-1].x-temp_pts[Num-2].x;
    temp_pts[Num-1].y=2*temp_pts[Num-1].y-temp_pts[Num-2].y;

    PointF NodePt1,NodePt2,NodePt3;
    double t;
    for(int i=0;i<Num-2;i++)
    {
        NodePt1=temp_pts[i]; NodePt2=temp_pts[i+1]; NodePt3=temp_pts[i+2];
        if(i==0)                                     //  第一段取t=0和t=0.5点
        {
            t=0;
            final_pts[i].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
            final_pts[i].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
            t=0.5;
            final_pts[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
            final_pts[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
        }else if(i==Num-3)                          //  最后一段取t=0.5和t=1点
        {
            t=0.5;
            final_pts[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
            final_pts[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
            t=1;
            final_pts[i+2].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
            final_pts[i+2].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
        }else                                      //  中间段取t=0.5点
        {
            t=0.5;
            final_pts[i+1].x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
            final_pts[i+1].y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
        }
    }

}

//================================================================
// 函数功能： 二次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： pts ：给定点序列
//          final_pts：bspline优化点
//          InsertNum: 节点之间需要插入的点个数指针
// 返回值：   bool
//
//=================================================================
bool CBSpline::TwoOrderBSplineInterpolatePt(vector<PointF> pts,vector<PointF> &final_pts,vector<int> InsertNum)
{
    if(pts.size()==0||InsertNum.size()==0)
        return false;

    PointF temp_pt;
    final_pts.clear();
    int Num=pts.size();
    int InsertNumSum=0;                               //  计算需要插入的点总数
    for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];


    vector<PointF> temp_pts; //  二次B样条不需要增加点数，需要将首尾点替换掉
    temp_pts=pts;
//    CPosition *temp=new CPosition[Num];               //  二次B样条不需要增加点数，需要将首尾点替换掉
//    for(int i=0;i<Num;i++)
//        temp[i]=pt[i];

    temp_pts[0].x=2*temp_pts[0].x-temp_pts[1].x;                  //  将折线两端点换成延长线上两点
    temp_pts[0].y=2*temp_pts[0].y-temp_pts[1].y;

    temp_pts[Num-1].x=2*temp_pts[Num-1].x-temp_pts[Num-2].x;
    temp_pts[Num-1].y=2*temp_pts[Num-1].y-temp_pts[Num-2].y;


    PointF NodePt1,NodePt2,NodePt3,NodePt4;        //  两节点间均匀插入点，需要相邻两段样条曲线,因此需要四个节点

    double t;
    for(int i=0;i<Num-1;i++)                          //  每条线段均匀插入点
    {
        if(i==0)                                      //  第一段只需计算第一条样条曲线，无NodePt1
        {
            NodePt2=temp_pts[i]; NodePt3=temp_pts[i+1]; NodePt4=temp_pts[i+2];

            double dt=0.5/(InsertNum[i]+1);
            for(int j=0;j<InsertNum[i]+1;j++)
            {
                t=0+dt*j;
                temp_pt.x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
                temp_pt.y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
                final_pts.push_back(temp_pt);
            }
        }else if(i==Num-2)                            //  最后一段只需计算最后一条样条曲线，无NodePt4
        {
            NodePt1=temp_pts[i-1]; NodePt2=temp_pts[i]; NodePt3=temp_pts[i+1];

            double dt=0.5/(InsertNum[i]+1);
            for(int j=0;j<InsertNum[i]+2;j++)
            {
                t=0.5+dt*j;
                temp_pt.x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
                temp_pt.y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
                final_pts.push_back(temp_pt);

            }
        }else
        {
            NodePt1=temp_pts[i-1],NodePt2=temp_pts[i]; NodePt3=temp_pts[i+1]; NodePt4=temp_pts[i+2];    // NodePt1,2,3计算第一条曲线，NodePt2,3,4计算第二条曲线

            int LeftInsertNum,RightInsertNum;          //  计算线段间左右曲线段上分别需要插入的点数
            double rightoffset=0;                      //  左边曲线段从t=0.5开始，又边曲线段从t=rightoffset开始
            double Leftdt=0,Rightdt=0;                 //  左右曲线取点t步长
            if(InsertNum[i]==0 )
            {
                LeftInsertNum=0;
                RightInsertNum=0;
            }else if(InsertNum[i]%2==1)                //  插入点数为奇数，左边曲线段插入点个数比右边多一点
            {
                RightInsertNum=InsertNum[i]/2;
                LeftInsertNum=RightInsertNum+1;
                Leftdt=0.5/(LeftInsertNum);
                Rightdt=0.5/(RightInsertNum+1);
                rightoffset=Rightdt;
            }else                                      //  插入点数为偶数，左右边曲线段插入个数相同
            {
                RightInsertNum=InsertNum[i]/2;
                LeftInsertNum=RightInsertNum;
                Leftdt=0.5/(LeftInsertNum+0.5);
                Rightdt=0.5/(RightInsertNum+0.5);
                rightoffset=Rightdt/2;
            }

            for(int j=0;j<LeftInsertNum+1;j++)
            {
                t=0.5+Leftdt*j;
                temp_pt.x=F02(t)*NodePt1.x+F12(t)*NodePt2.x+F22(t)*NodePt3.x;
                temp_pt.y=F02(t)*NodePt1.y+F12(t)*NodePt2.y+F22(t)*NodePt3.y;
                final_pts.push_back(temp_pt);
            }

            for(int j=0;j<RightInsertNum;j++)
            {
                t=rightoffset+Rightdt*j;
                temp_pt.x=F02(t)*NodePt2.x+F12(t)*NodePt3.x+F22(t)*NodePt4.x;
                temp_pt.y=F02(t)*NodePt2.y+F12(t)*NodePt3.y+F22(t)*NodePt4.y;
                final_pts.push_back(temp_pt);
            }
        }
    }
}
//================================================================
// 函数功能： 二次样条基函数
//
//================================================================
double CBSpline::F02(double t)
{
    return 0.5*(t-1)*(t-1);
}
double CBSpline::F12(double t)
{
    return 0.5*(-2*t*t+2*t+1);
}
double CBSpline::F22(double t)
{
    return 0.5*t*t;
}
//========================================================================
// 函数功能： 三次B样条平滑，把给定的点，平滑到B样条曲线上，不增加点的数目
// 输入参数： pts ：给定点序列
//          final_pts：bspline优化点
// 返回值：   bool
//========================================================================
bool CBSpline::ThreeOrderBSplineSmooth(vector<PointF> pts,vector<PointF> &final_pts)
{
    PointF temp_pt;
    final_pts.clear();
    int Num=pts.size();
    vector<PointF> temp_pts; //  二次B样条不需要增加点数，需要将首尾点替换掉
    temp_pts=pts;
    final_pts=pts;
    temp_pts.insert(temp_pts.begin(),temp_pt);
    temp_pts.push_back(temp_pt);

    temp_pts[0].x=2*temp_pts[1].x-temp_pts[2].x;                  //  将折线延长线上两点加入作为首点和尾点
    temp_pts[0].y=2*temp_pts[1].y-temp_pts[2].y;

    temp_pts[Num+1].x=2*temp_pts[Num].x-temp_pts[Num-1].x;
    temp_pts[Num+1].y=2*temp_pts[Num].y-temp_pts[Num-1].y;

    PointF NodePt1,NodePt2,NodePt3,NodePt4;
    double t;
    for(int i=0;i<Num-1;i++)
    {
        NodePt1=temp_pts[i]; NodePt2=temp_pts[i+1]; NodePt3=temp_pts[i+2]; NodePt4=temp_pts[i+3];

        if(i==Num-4)                          //  最后一段取t=0.5和t=1点
        {
            t=0;
            final_pts[i].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
            final_pts[i].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
            t=1;
            final_pts[i+1].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
            final_pts[i+1].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
        }else                                      //  中间段取t=0.5点
        {
            t=0;
            final_pts[i].x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
            final_pts[i].y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
        }
    }
}

//================================================================
// 函数功能： 三次B样条拟合,在节点之间均匀插入指定个数点
// 输入参数： pts ：给定点序列
//          final_pts：bspline优化点
//          InsertNum: 节点之间需要插入的点个数指针
// 返回值：   bool
//=================================================================
bool CBSpline::ThreeOrderBSplineInterpolatePt(vector<PointF> pts,vector<PointF> &final_pts,vector<int> InsertNum)
{
    if(pts.size()==0 || InsertNum.size()==0) return false;

    PointF temp_pt;
    final_pts.clear();
    int Num=pts.size();
    vector<PointF> temp_pts;                                   //二次B样条不需要增加点数，需要将首尾点替换掉
    temp_pts=pts;
    //final_pts=pts;
    temp_pts.insert(temp_pts.begin(),temp_pt);
    temp_pts.push_back(temp_pt);

    int InsertNumSum=0;                                        //计算需要插入的点总数
    for(int i=0;i<Num-1;i++)  InsertNumSum+=InsertNum[i];


    temp_pts[0].x=2*temp_pts[1].x-temp_pts[2].x;               //将折线延长线上两点加入作为首点和尾点
    temp_pts[0].y=2*temp_pts[1].y-temp_pts[2].y;

    temp_pts[Num+1].x=2*temp_pts[Num].x-temp_pts[Num-1].x;
    temp_pts[Num+1].y=2*temp_pts[Num].y-temp_pts[Num-1].y;

    PointF NodePt1,NodePt2,NodePt3,NodePt4;
    double t;
    for(int i=0;i<Num-1;i++)                          //每条线段均匀插入点
    {
        NodePt1=temp_pts[i]; NodePt2=temp_pts[i+1]; NodePt3=temp_pts[i+2]; NodePt4=temp_pts[i+3];
        double dt=1.0/(InsertNum[i]+1);

        for(int j=0;j<InsertNum[i]+1;j++)
        {
            t=dt*j;
            temp_pt.x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
            temp_pt.y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
            final_pts.push_back(temp_pt);
        }

        if(i==Num-2){              //  最后一个尾点
            t=1;
            temp_pt.x=F03(t)*NodePt1.x+F13(t)*NodePt2.x+F23(t)*NodePt3.x+F33(t)*NodePt4.x;
            temp_pt.y=F03(t)*NodePt1.y+F13(t)*NodePt2.y+F23(t)*NodePt3.y+F33(t)*NodePt4.y;
            final_pts.push_back(temp_pt);
        }
    }
    return true;
}

//================================================================
// 函数功能： 三次样条基函数
//================================================================
double CBSpline::F03(double t)
{
    return 1.0/6*(-t*t*t+3*t*t-3*t+1);
}
double CBSpline::F13(double t)
{
    return 1.0/6*(3*t*t*t-6*t*t+4);
}
double CBSpline::F23(double t)
{
    return 1.0/6*(-3*t*t*t+3*t*t+3*t+1);
}
double CBSpline::F33(double t)
{
    return 1.0/6*t*t*t;
}
