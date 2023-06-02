#include "geometric_algorithm.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) > (b) ? (b) : (a))
bool IsEqual(double a,double b)
{
    if(fabs(a-b)<=EP)
        return true;
    else
        return false;
}
bool IsClockwise(vector<PointF> pointList)
{
    double maxY = -INF;
    int index = 0;

    //找到Y值最大的点及其前一点和后一点
    for (unsigned int i = 0; i < pointList.size(); i++) {
        PointF pt = pointList[i];
        if (i == 0) {
            maxY = pt.y;
            index = 0;
            continue;
        }
        if (maxY < pt.y) {
            maxY = pt.y;
            index = i;
        }
    }
    int front = (index+pointList.size()-1)%pointList.size();
    int middle = index%pointList.size();
    int after = (index + 1)%pointList.size();
    PointF frontPt = pointList[front];
    PointF middlePt = pointList[middle];
    PointF afterPt = pointList[after];
    return (afterPt.x - frontPt.x)*(middlePt.y - frontPt.y)-(middlePt.x - frontPt.x)*(afterPt.y - frontPt.y)>0.0;
}

double Dist_pt(PointF p1,PointF p2)                // 返回两点之间欧氏距离
{
    return( sqrt( (p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y) ) );
}
double VectDot(PointF p0,PointF p1,PointF p2) //点乘
{
    return (p1-p0)*(p2-p0);
}
double VectCross(PointF p0,PointF p1,PointF p2) //叉乘
{
    return (p1-p0)^(p2-p0);
}
bool Is_PtonSegment(LINESEG seg,PointF p)
{
//    std::cout<<"  ggg  "<<std::setiosflags(std::ios::fixed)<<std::setprecision(6)<<VectCross(p,seg.e,seg.s)<<std::endl;
//    cout<<"  ggg1 "<<IsEqual(VectCross(p,seg.e,seg.s),0)<<endl;
//    cout<<"  ggg2 "<<( (p.x-seg.s.x)*(p.x-seg.e.x)<0||IsEqual((p.x-seg.s.x)*(p.x-seg.e.x),0))<<endl;
//    cout<<"  ggg3 "<<( (p.y-seg.s.y)*(p.y-seg.e.y)<0||IsEqual((p.y-seg.s.y)*(p.y-seg.e.y),0))<<endl;

    return( IsEqual(VectCross(p,seg.e,seg.s),0)&&( ( (p.x-seg.s.x)*(p.x-seg.e.x)<0||IsEqual((p.x-seg.s.x)*(p.x-seg.e.x),0))&&( (p.y-seg.s.y)*(p.y-seg.e.y)<0||IsEqual((p.y-seg.s.y)*(p.y-seg.e.y),0) ) ) );
}
bool Is_PtOnLine(LINE l,PointF p)
{
    return IsEqual((l.a*p.x+l.b*p.y+l.c),0);
}
PointF rotate_aroundPt(PointF o,double alpha,PointF p)
{
    PointF tp;
    p.x-=o.x;
    p.y-=o.y;
    tp.x=p.x*cos(alpha)-p.y*sin(alpha)+o.x;
    tp.y=p.y*cos(alpha)+p.x*sin(alpha)+o.y;
    return tp;
}
double Vect_angle(PointF o,PointF s,PointF e)
{
    double cosfi,fi,norm;
    double dsx = s.x - o.x;
    double dsy = s.y - o.y;
    double dex = e.x - o.x;
    double dey = e.y - o.y;
    std::cout<<"dsx "<<dsx<<std::endl;
    std::cout<<"dsy "<<dsy<<std::endl;
    std::cout<<"dex "<<dex<<std::endl;
    std::cout<<"dey "<<dey<<std::endl;
    cosfi=dsx*dex+dsy*dey;
    norm=(dsx*dsx+dsy*dsy)*(dex*dex+dey*dey);
    std::cout<<"cosfi "<<cosfi<<std::endl;
    std::cout<<"norm "<<norm<<std::endl;

    cosfi /= sqrt( norm );

    if (cosfi >=  1.0 ) return 0;
    if (cosfi <= -1.0 ) return -3.1415926;

    fi=acos(cosfi);
    std::cout<<"cosfi "<<cosfi<<std::endl;
    std::cout<<"fi "<<fi<<std::endl;

    if (dsx*dey-dsy*dex>0) return fi;      // 说明矢量os 在矢量 oe的顺时针方向
    return -fi;
}


double relation_PtSeg(PointF p,LINESEG seg) //p在seg上的投影点 与 seg的关系
{
    LINESEG tl;
    tl.s=seg.s;
    tl.e=p;
    return VectDot(seg.s,tl.e,seg.e)/(Dist_pt(seg.s,seg.e)*Dist_pt(seg.s,seg.e));
}
PointF perpendicular(PointF p,LINESEG seg)
{
    double r=relation_PtSeg(p,seg);
    PointF tp;
    tp.x=seg.s.x+r*(seg.e.x-seg.s.x);
    tp.y=seg.s.y+r*(seg.e.y-seg.s.y);
    return tp;
}
double dot(PointF p1, PointF p2) {
    return p1.x * p2.x + p1.y * p2.y;
}

double cross(PointF p1, PointF p2) {
    return p1.x * p2.y - p2.x * p1.y;
}
double PtoSegdist(PointF p,LINESEG seg,PointF &np)
{
//    double r=relation_PtSeg(p,seg);
//    if(r<0)
//    {
//        np=seg.s;
//        return Dist_pt(p,seg.s);
//    }
//    if(r>1)
//    {
//        np=seg.e;
//        return Dist_pt(p,seg.e);
//    }
//    np=perpendicular(p,seg);
//    return Dist_pt(p,np);

    PointF p1=seg.s;
    PointF p2=seg.e;
    double d1 = Dist_pt(p, p1);
    double d2 = Dist_pt(p, p2);
    double l = Dist_pt(p1, p2);
    double h = cross(PointF(p2.x - p1.x, p2.y - p1.y), PointF(p.x - p1.x, p.y - p1.y)) / l;

    if (dot(PointF(p.x - p1.x, p.y - p1.y), PointF(p2.x - p1.x, p2.y - p1.y)) < 0) {
        return d1;
    }
    if (dot(PointF(p.x - p2.x, p.y - p2.y), PointF(p1.x - p2.x, p1.y - p2.y)) < 0) {
        return d2;
    }
    return h;

}
double PtoLinedist(PointF p,LINESEG l) //
{   //abs(VectCross(l.s,p,l.e)) 是以 p,l.s,l.e组成的平行四边形的面积
    return abs(VectCross(l.s,p,l.e))/Dist_pt(l.s,l.e);
}
double PtoPointset(vector<PointF> pointset,PointF p,PointF &q)
{
    int vcount=pointset.size();
    int i;
    double cd=double(INF),td;
    LINESEG l;
    PointF tq,cq;

    for(i=0;i<vcount-1;i++)
    {
        l.s=pointset[i];

        l.e=pointset[i+1];
        td=PtoSegdist(p,l,tq);
        if(td<cd)
        {
            cd=td;
            cq=tq;
        }
    }
    q=cq;
    return cd;
}
bool CircleInsidePolygon(PointF center,double radius,Polygon plg)
{
//    PointF q;
//    double d;
//    q.x=0;
//    q.y=0;
//    vector<PointF> pointset;
//    for(int i=0;i<plg.n;i++)
//        pointset.push_back(plg.Vs[i]);
//    d=PtoPointset(pointset,center,q);
//    std::cout<<"ddd  "<<d<<std::endl;
//    if(d<radius||fabs(d-radius)<EP)
//    {
//        std::cout<<"ddddddd  "<<1<<std::endl;

//        return false;
//    }
//    else if(!Is_PtInPolygon(plg,center)==0)
//    {
//        std::cout<<"ddddddd  "<<2<<std::endl;

//        return false;
//    }
//    else
//    {
//        std::cout<<"ddddddd  "<<3<<std::endl;

//        return true;
//    }
    double min_dist=INF;
    PointF ptt;
    for(int i=0;i<plg.Es.size();i++)
    {
        double dist=PtoSegdist(center,plg.Es[i],ptt);
        if(dist<min_dist)
            min_dist=dist;
    }
    std::cout<<"ggg "<<Is_PtInPolygon(plg,center)<<std::endl;
    std::cout<<"kkk  "<<min_dist<<std::endl;

    if(Is_PtInPolygon(plg,center)==0&&min_dist>=radius)
    {
        std::cout<<"ddddddd  "<<2<<std::endl;

        return true;
    }
    else
    {
        std::cout<<"ddddddd  "<<3<<std::endl;

        return false;
    }
}
double cos_vect(LINESEG l1,LINESEG l2)
{
    return (((l1.e.x-l1.s.x)*(l2.e.x-l2.s.x)+(l1.e.y-l1.s.y)*(l2.e.y-l2.s.y))/(Dist_pt(l1.e,l1.s)*Dist_pt(l2.e,l2.s)));
}
double Seg_angle(LINESEG l1,LINESEG l2)
{
    PointF o,s,e;
    o.x=o.y=0;
    s.x=l1.e.x-l1.s.x;
    s.y=l1.e.y-l1.s.y;
    e.x=l2.e.x-l2.s.x;
    e.y=l2.e.y-l2.s.y;
    return Vect_angle(o,s,e);
}
bool intersect_seg2seg(LINESEG u,LINESEG v)
{
    return( (max(u.s.x,u.e.x)>=min(v.s.x,v.e.x))&&                     //排斥实验
            (max(v.s.x,v.e.x)>=min(u.s.x,u.e.x))&&
            (max(u.s.y,u.e.y)>=min(v.s.y,v.e.y))&&
            (max(v.s.y,v.e.y)>=min(u.s.y,u.e.y))&&
            (VectCross(u.s,v.s,u.e)*VectCross(u.s,u.e,v.e)>=0)&&         //跨立实验
            (VectCross(v.s,u.s,v.e)*VectCross(v.s,v.e,u.e)>=0));
}
bool intersect_seg2seg_notendpt(LINESEG u,LINESEG v)
{
    return	((intersect_seg2seg(u,v))&&
            (!Is_PtonSegment(u,v.s))&&
            (!Is_PtonSegment(u,v.e))&&
            (!Is_PtonSegment(v,u.e))&&
            (!Is_PtonSegment(v,u.s)));
}
bool intersect_seg2line(LINESEG u,LINESEG v)
{
    return VectCross(v.s,u.s,v.e)*VectCross(v.s,v.e,u.e)>=0;
}
double line_slope(LINE l)
{
    if(abs(l.a) < 1e-20)
        return 0;
    if(abs(l.b) < 1e-20)
        return INF;
    return -(l.a/l.b);
}
double line_alpha(LINE l)
{
    if(abs(l.a)< EP)
        return 0;
    if(abs(l.b)< EP)
        return PI/2;
    double k=line_slope(l);
    if(k>0)
        return atan(k);
    else
        return PI+atan(k);
}
PointF symmetry(LINE l,PointF p)
{
   PointF tp;
   tp.x=((l.b*l.b-l.a*l.a)*p.x-2*l.a*l.b*p.y-2*l.a*l.c)/(l.a*l.a+l.b*l.b);
   tp.y=((l.a*l.a-l.b*l.b)*p.y-2*l.a*l.b*p.x-2*l.b*l.c)/(l.a*l.a+l.b*l.b);
   return tp;
}
bool line_intersect(LINE l1,LINE l2,PointF &p) // 是 L1，L2
{
    double d=l1.a*l2.b-l2.a*l1.b;
    if(abs(d)<EP) // 不相交
        return false;
    p.x = (l2.c*l1.b-l1.c*l2.b)/d;
    p.y = (l2.a*l1.c-l1.a*l2.c)/d;
    return true;
}
bool segment_intersect(LINESEG l1,LINESEG l2,PointF &inter)
{
    LINE ll1(l1.s,l1.e),ll2(l2.s,l2.e);
    if(line_intersect(ll1,ll2,inter))
        return Is_PtonSegment(l1,inter);
    else
        return false;
}

double get_theta_k(double k)
{
    if(k<0)
        return PI+atan(k);
    return atan(k);
}
double get_theta_Pt(PointF pt1,PointF pt2)
{
    if(IsEqual(pt1.x,pt2.x))
        return PI/2;
    double k=(pt1.y-pt2.y)/(pt1.x-pt2.x);
    return get_theta_k(k);
}


bool IsSimple_Polygon(Polygon plg) //每条边与不相邻的其他边是否没有交点
{
    int i,cn;
    LINESEG l1,l2;
    for(i=0;i<plg.n;i++)
    {
        l1=plg.Es[i];
        cn=plg.n-3;
        while(cn)
        {
            l2=plg.Es[(i+2)%plg.n];
            if(intersect_seg2seg(l1,l2))
                break;
            cn--;
        }
        if(cn)
            return false;
    }
    return true;
}
void IsConvex_Vertex(Polygon plg,std::map<int,bool> &table)
{
    int vcount=plg.n;
    int i,index=0;
    PointF tp=plg.Vs[0];
    for(i=1;i<vcount;i++) // 寻找第一个凸顶点
    {
        if(plg.Vs[i].y<tp.y||(plg.Vs[i].y == tp.y&&plg.Vs[i].x<tp.x))
        {
            tp=plg.Vs[i];
            index=i;
        }
    }
    int count=vcount-1;
    table.insert(std::make_pair(index,1));
    while(count) // 判断凸凹性
    {
        if(VectCross(plg.Vs[index%vcount],plg.Vs[(index+1)%vcount],plg.Vs[(index+2)%vcount])>=0)
            table.insert(std::make_pair((index+1)%vcount,1));
        else
            table.insert(std::make_pair((index+1)%vcount,0));
        index++;
        count--;
    }
}
bool IsConvex_Polygon(Polygon plg)
{
    int vcount=plg.n;
    //bool bc[MAXV];
    std::map<int,bool> bc;
    IsConvex_Vertex(plg,bc);
    for(int i=0;i<vcount;i++) // 逐一检查顶点，是否全部是凸顶点
        if(!bc[i])
            return false;
    return true;
}
double area_of_polygon(Polygon plg)
{
    int vcount=plg.n;
    int i;
    double s;
    if (vcount<3)
        return 0;
    s=plg.Vs[0].y*(plg.Vs[vcount-1].x-plg.Vs[1].x);
    for (i=1;i<vcount;i++)
        s+=plg.Vs[i].y*(plg.Vs[(i-1)].x-plg.Vs[(i+1)%vcount].x);
    return s/2;
}
bool IsConterClockwise(Polygon plg) //多边形逆时针排列
{
    return area_of_polygon(plg)>0;
}
int Is_PtInPolygon(Polygon plg,PointF q)  //点在多边形内：0； 点在多边形边上：1； 点在多边形外：2
{
    int vcount=plg.n;
    int c=0,i,n;
    LINESEG l1,l2;
    bool bintersect_a,bonline1,bonline2,bonline3;
    double r1,r2;

    l1.s=q;
    l1.e=q;
    l1.e.x=double(INF);
    n=vcount;
    for (i=0;i<vcount;i++)
    {
        l2.s=plg.Vs[i];
        l2.e=plg.Vs[(i+1)%n];
        if(Is_PtonSegment(l2,q))
            return 1; // 如果点在边上，返回1
        if ( (bintersect_a=intersect_seg2seg_notendpt(l1,l2))|| // 相交且不在端点
        ( (bonline1=Is_PtonSegment(l1,plg.Vs[(i+1)%n]))&& // 第二个端点在射线上
        ( (!(bonline2=Is_PtonSegment(l1,plg.Vs[(i+2)%n])))&& /* 前一个端点和后一个端点在射线两侧 */
        ((r1=VectCross(l1.s,plg.Vs[i],plg.Vs[(i+1)%n])*VectCross(l1.s,plg.Vs[(i+1)%n],plg.Vs[(i+2)%n]))>0) ||
        (bonline3=Is_PtonSegment(l1,plg.Vs[(i+2)%n]))&&     /* 下一条边是水平线，前一个端点和后一个端点在射线两侧  */
            ((r2=VectCross(l1.s,plg.Vs[i],plg.Vs[(i+2)%n])*VectCross(l1.s,plg.Vs[(i+2)%n],
        plg.Vs[(i+3)%n]))>0)
                )
            )
        ) c++;
    }
    if(c%2 == 1)
        return 0;
    else
        return 2;
}
bool Is_PtInConvexPolygon(Polygon plg,PointF q) // 只适用于凸多边形
{
    int vcount=plg.n;
    PointF p;
    LINESEG l;
    int i;
    p.x=0;p.y=0;
    for(i=0;i<vcount;i++) // 寻找一个肯定在多边形polygon内的点p：多边形顶点平均值
    {
        p.x+=plg.Vs[i].x;
        p.y+=plg.Vs[i].y;
    }
    p.x /= vcount;
    p.y /= vcount;

    for(i=0;i<vcount;i++)
    {
        l.s=plg.Vs[i];l.e=plg.Vs[(i+1)%vcount];
        if(VectCross(l.s,p,l.e)*VectCross(l.s,q,l.e)<0) /* 点p和点q在边l的两侧，说明点q肯定在多边形外 */
        break;
    }
    return (i==vcount);
}
vector<PointF> ConvexClosure_Graham(vector<PointF> PointSet)
{
    int n=PointSet.size();
    vector<PointF> pts;
    int i,j,k=0;
    unsigned int top=2;
    PointF tmp;
    // 选取PointSet中y坐标最小的点PointSet[k]，如果这样的点有多个，则取最左边的一个
    for(i=1;i<n;i++)
        if ( PointSet[i].y<PointSet[k].y ||((PointSet[i].y==PointSet[k].y) && (PointSet[i].x<PointSet[k].x)) )
            k=i;
    tmp=PointSet[0];
    PointSet[0]=PointSet[k];
    PointSet[k]=tmp; // 现在PointSet中y坐标最小的点在PointSet[0]
    for (i=1;i<n-1;i++) /* 对顶点按照相对PointSet[0]的极角从小到大进行排序，极角相同的按照距离PointSet[0]从近到远进行排序 */
    {
        k=i;
        for (j=i+1;j<n;j++)
            if ( VectCross(PointSet[0],PointSet[j],PointSet[k])>0 ||  // 极角更小
                ((VectCross(PointSet[0],PointSet[j],PointSet[k])==0) && /* 极角相等，距离更短 */
                Dist_pt(PointSet[0],PointSet[j])<Dist_pt(PointSet[0],PointSet[k]))
               )
                k=j;
        tmp=PointSet[i];
        PointSet[i]=PointSet[k];
        PointSet[k]=tmp;
    }
    for(int i=0;i<3;i++)
        pts.push_back(PointSet[i]);
//    ch[0]=PointSet[0];
//    ch[1]=PointSet[1];
//    ch[2]=PointSet[2];
    for (i=3;i<n;i++)
    {
        while (VectCross(pts[top-1],PointSet[i],pts[top])>=0)
            top--;
        ++top;
        if(top<pts.size())
            pts[top]=PointSet[i];
        else
            pts.push_back(PointSet[i]);
    }
    return pts;
}
//vector<PointF> ConvexClosure(vector<PointF> PointSet)
//{
//    int n=PointSet.size();
//    vector<PointF> pts;
//    int i,index,first;
//    double curmax,curcos,curdis;
//    PointF tmp;
//    LINESEG l1,l2;
//    bool use[MAXV]={false};
//    tmp=PointSet[0];
//    index=0;
//    // 选取y最小点，如果多于一个，则选取最左点
//    for(i=1;i<n;i++)
//    {
//        if(PointSet[i].y<tmp.y||IsEqual(PointSet[i].y,tmp.y)&&PointSet[i].x<tmp.x)
//        {
//            index=i;
//        }
//        use[i]=false;
//    }
//    cout<<"indexx"<<index<<endl;

//    tmp=PointSet[index];
//    first=index;
//    use[index]=true;

//    index=-1;
//    pts.push_back(tmp);
//    tmp.x-=100;
//    l1.s=tmp;
//    l1.e=pts[0];
//    l2.s=pts[0];

//    while(index!=first)
//    {
//        curmax=-100;
//        curdis=0;
//        // 选取与最后一条确定边夹角最小的点，即余弦值最大者
//        for(i=0;i<n;i++)
//        {
//            if(use[i])continue;
//            l2.e=PointSet[i];
//            curcos=cos_vect(l1,l2); // 根据cos值求夹角余弦，范围在 （-1 -- 1 ）
//            if(curcos>curmax || fabs(curcos-curmax)<1e-6 && Dist_pt(l2.s,l2.e)>curdis)
//            {
//                curmax=curcos;
//                index=i;
//                curdis=Dist_pt(l2.s,l2.e);
//            }
//        }
//        cout<<"index"<<index<<endl;
//        use[first]=false;            //清空第first个顶点标志，使最后能形成封闭的hull
//        use[index]=true;
//        pts.push_back(PointSet[index]);
//        l1.s=pts[pts.size()-2];
//        l1.e=pts[pts.size()-1];
//        l2.s=pts[pts.size()-1];
//    }
//    return pts;
//}
bool Is_SegInsidePolygon(Polygon plg,LINESEG l)
{
    unsigned int vcount=plg.n;
    // 判断线端l的端点是否不都在多边形内
//    cout<<"    ("<<l.s.x<<", "<<l.s.y<<")   "<<"("<<l.e.x<<", "<<l.e.y<<")   "<<endl;
//    cout<<"    ("<<Is_PtInPolygon(plg,l.s)<<endl;
//    cout<<"    ("<<Is_PtInPolygon(plg,l.e)<<endl;

    if(Is_PtInPolygon(plg,l.s)!=0||Is_PtInPolygon(plg,l.e)!=0)
        return false;

    //cout<<"    (("<<Is_PtInPolygon(plg,l.e)<<endl;

    unsigned int i,j;
    PointF tmp;
    vector<PointF> PointSet;
    LINESEG s;
    cout<<"vcount    "<<vcount<<endl;

    for(i=0;i<vcount;i++)
    {
        s.s=plg.Vs[i];
        s.e=plg.Vs[(i+1)%vcount];
        if(Is_PtonSegment(s,l.s)) //线段l的起始端点在线段s上
            PointSet.push_back(l.s);
        else if(Is_PtonSegment(s,l.e)) //线段l的终止端点在线段s上
            PointSet.push_back(l.e);
        else
        {
            if(Is_PtonSegment(l,s.s)) //线段s的起始端点在线段l上
                PointSet.push_back(s.s);
            else if(Is_PtonSegment(l,s.e)) // 线段s的终止端点在线段l上
                PointSet.push_back(s.e);
            else
            {
                if(intersect_seg2seg(l,s)) // 这个时候如果相交，肯定是内交，返回false
                return false;
            }
        }
        cout<<"i    "<<i<<endl;

    }
    cout<<"PointSet    "<<PointSet.size()-1<<endl;

    if(PointSet.size()==0)
        return true;
    cout<<"PointSet    "<<PointSet.size()<<endl;

    for(i=0;i<PointSet.size()-1;i++) /* 冒泡排序，x坐标小的排在前面；x坐标相同者，y坐标小的排在前面 */
    {
//        cout<<"PointSett    "<<PointSet.size()<<endl;
//        cout<<"i    "<<i<<endl;

        for(j=i+1;j<PointSet.size();j++)
        {
//            cout<<"PointSettt    "<<PointSet.size()<<endl;
//            cout<<"j    "<<j<<endl;

            if( PointSet[i].x>PointSet[j].x || (fabs(PointSet[i].x-PointSet[j].x)<EP && PointSet[i].y>PointSet[j].y) )
            {
                tmp=PointSet[i];
                PointSet[i]=PointSet[j];
                PointSet[j]=tmp;
            }
        }
    }

    cout<<"PointSet    "<<PointSet.size()<<endl;

    for(i=0;i<PointSet.size()-1;i++)
    {
        cout<<"i+1    "<<i+1<<endl;

        tmp.x=(PointSet[i].x+PointSet[i+1].x)/2; //得到两个相邻交点的中点
        tmp.y=(PointSet[i].y+PointSet[i+1].y)/2;
        if(!Is_PtInPolygon(plg,tmp))
            return false;
    }
    return true;
}
bool Is_Plg1InPlg2(Polygon plg1,Polygon plg2)
{
    for(unsigned int i=0;i<plg1.Es.size();i++)
    {
        if(!Is_SegInsidePolygon(plg2,plg1.Es[i]))
            return false;
    }
    return true;
}

bool Is_PtInCircle(Circle C,PointF p)
{
    double d2=(p.x-C.O.x)*(p.x-C.O.x)+(p.y-C.O.y)*(p.y-C.O.y);
    double r2=C.R*C.R;
    return d2<r2||abs(d2-r2)<EP;
}
bool PtsGenerateCircle(PointF p1,PointF p2,PointF p3,Circle &c)
{
    double x12=p2.x-p1.x;
    double y12=p2.y-p1.y;
    double x13=p3.x-p1.x;
    double y13=p3.y-p1.y;
    double z2=x12*(p1.x+p2.x)+y12*(p1.y+p2.y);
    double z3=x13*(p1.x+p3.x)+y13*(p1.y+p3.y);
    double d=2.0*(x12*(p3.y-p2.y)-y12*(p3.x-p2.x));
    if(abs(d)<EP) //共线，圆不存在
        return false;
    c.O.x=(y13*z2-y12*z3)/d;
    c.O.y=(x12*z3-x13*z2)/d;
    c.R=Dist_pt(p1,c.O);
    return true;
}
PointF Rect4thPt(PointF a,PointF b,PointF c)
{
    PointF d;
    if(abs(VectDot(c,a,b))<EP) // 说明c点是直角拐角处
    {
        d.x=a.x+b.x-c.x;
        d.y=a.y+b.y-c.y;
    }
    if(abs(VectDot(b,a,c))<EP) // 说明b点是直角拐角处
    {
        d.x=a.x+c.x-b.x;
        d.y=a.y+c.y-b.x;
    }
    if(abs(VectDot(a,c,b))<EP) // 说明a点是直角拐角处
    {
        d.x=c.x+b.x-a.x;
        d.y=c.y+b.y-a.y;
    }
    return d;
}
int CircleRelation(Circle C1,Circle C2)
{
    double d = sqrt( (C1.O.x-C2.O.x)*(C1.O.x-C2.O.x)+(C1.O.y-C2.O.y)*(C1.O.y-C2.O.y) );

    if( fabs(d-C1.R-C2.R) < EP ) // 必须保证前两个if先被判定！
        return 2;
    if( fabs(d-fabs(C1.R-C2.R)) < EP )
        return 4;
    if( d > C1.R+C2.R )
        return 1;
    if( d < fabs(C1.R-C2.R) )
        return 5;
    if( fabs(C1.R-C2.R) < d && d < C1.R+C2.R )
        return 3;
    return 0; // indicate an error!
}
bool Is_CircleInRec(Circle C1,PointF pr1, PointF pr2, PointF pr3, PointF pr4)
{

    if( pr1.x < C1.O.x && C1.O.x < pr2.x && pr3.y < C1.O.y && C1.O.y < pr2.y )
    {
        LINESEG line1(pr1, pr2);
        LINESEG line2(pr2, pr3);
        LINESEG line3(pr3, pr4);
        LINESEG line4(pr4, pr1);
        if( C1.R<PtoLinedist(C1.O,line1) && C1.R<PtoLinedist(C1.O,line2) &&C1.R<PtoLinedist(C1.O,line3) && C1.R<PtoLinedist(C1.O,line4) )
            return true;
    }
    return false;
}
//3．点到平面的距离：点到平面的距离,平面用一般式表示ax+by+cz+d=0
double P2planeDist(double x, double y, double z, double a, double b, double c, double d)
{
    return fabs(a*x+b*y+c*z+d) / sqrt(a*a+b*b+c*c);
}
//4．点是否在直线同侧： 两个点是否在直线同侧，是则返回true
bool Is_PtsOnSameSide(PointF p1, PointF p2, LINE line)
{
    return (line.a * p1.x + line.b * p1.y + line.c) *
    (line.a * p2.x + line.b * p2.y + line.c) > 0;
}
//5．镜面反射线：
void LineReflect(LINE l1,LINE l2,LINE &l)
{
    double n,m;
    double tpb,tpa;
    tpb=l1.b*l2.b+l1.a*l2.a;
    tpa=l2.a*l1.b-l1.a*l2.b;
    m=(tpb*l1.b+tpa*l1.a)/(l1.b*l1.b+l1.a*l1.a);
    n=(tpa*l1.b-tpb*l1.a)/(l1.b*l1.b+l1.a*l1.a);
    if(fabs(l1.a*l2.b-l2.a*l1.b)<EP)
    {
        l.a=l2.a;l.b=l2.b;l.c=l2.c;
        return;
    }
    double xx,yy; //(xx,yy)是入射线与镜面的交点。
    xx=(l1.b*l2.c-l2.b*l1.c)/(l1.a*l2.b-l2.a*l1.b);
    yy=(l2.a*l1.c-l1.a*l2.c)/(l1.a*l2.b-l2.a*l1.b);
    l.a=n;
    l.b=-m;
    l.c=m*yy-xx*n;
}
//6．矩形包含： 26
//7．两圆交点： 两圆已经相交（相切）
void  C1CrossC2(Circle c1,Circle c2,PointF &rp1,PointF &rp2)
{
    double a,b,r;
    a=c2.O.x-c1.O.x;
    b=c2.O.y-c1.O.y;
    r=(a*a+b*b+c1.R*c1.R-c2.R*c2.R)/2;
    if(a==0&&b!=0)
    {
        rp1.y=rp2.y=r/b;
        rp1.x=sqrt(c1.R*c1.R-rp1.y*rp1.y);
        rp2.x=-rp1.x;
    }
    else if(a!=0&&b==0)
    {
        rp1.x=rp2.x=r/a;
        rp1.y=sqrt(c1.R*c1.R-rp1.x*rp2.x);
        rp2.y=-rp1.y;
    }
    else if(a!=0&&b!=0)
    {
        double delta;
        delta=b*b*r*r-(a*a+b*b)*(r*r-c1.R*c1.R*a*a);
        rp1.y=(b*r+sqrt(delta))/(a*a+b*b);
        rp2.y=(b*r-sqrt(delta))/(a*a+b*b);
        rp1.x=(r-b*rp1.y)/a;
        rp2.x=(r-b*rp2.y)/a;
    }

    rp1.x+=c1.O.x;
    rp1.y+=c1.O.y;
    rp2.x+=c1.O.x;
    rp2.y+=c1.O.y;
}
//8．两圆公共面积： 28
//9. 圆和直线关系：
int CircleRelationLine(Circle C,LINE l,PointF &rp1,PointF &rp2)
{

    int res=0;

    l.c=l.c+l.a*C.O.x+l.b*C.O.y;
    double tmp;
    if(l.a==0&&l.b!=0)
    {
        tmp=-l.c/l.b;
        if(C.R*C.R<tmp*tmp)
            res=0;
        else if(C.R*C.R==tmp*tmp)
        {
            res=1;
            rp1.y=tmp;
            rp1.x=0;
        }
        else
        {
            res=2;
            rp1.y=rp2.y=tmp;
            rp1.x=sqrt(C.R*C.R-tmp*tmp);
            rp2.x=-rp1.x;
        }
    }
    else if(l.a!=0&&l.b==0)
    {
        tmp=-l.c/l.a;
        if(C.R*C.R<tmp*tmp)
            res=0;
        else if(C.R*C.R==tmp*tmp)
        {
            res=1;
            rp1.x=tmp;
            rp1.y=0;
        }
        else
        {
            res=2;
            rp1.x=rp2.x=tmp;
            rp1.y=sqrt(C.R*C.R-tmp*tmp);
            rp2.y=-rp1.y;
        }
    }
    else if(l.a!=0&&l.b!=0)
    {
        double delta;
        delta=l.b*l.b*l.c*l.c-(l.a*l.a+l.b*l.b)*(l.c*l.c-l.a*l.a*C.R*C.R);
        if(delta<0)
            res=0;
        else if(delta==0)
        {
            res=1;
            rp1.y=-l.b*l.c/(l.a*l.a+l.b*l.b);
            rp1.x=(-l.c-l.b*rp1.y)/l.a;
        }
        else
        {
            res=2;
            rp1.y=(-l.b*l.c+sqrt(delta))/(l.a*l.a+l.b*l.b);
            rp2.y=(-l.b*l.c-sqrt(delta))/(l.a*l.a+l.b*l.b);
            rp1.x=(-l.c-l.b*rp1.y)/l.a;
            rp2.x=(-l.c-l.b*rp2.y)/l.a;
        }
    }
    rp1.x+=C.O.x;
    rp1.y+=C.O.y;
    rp2.x+=C.O.x;
    rp2.y+=C.O.y;
    return res;
}
//10. 内切圆： 30
//11. 求切点： 过圆外一点与圆相切的两条切线的切点
void CircleCutPt(Circle C,PointF sp,PointF &rp1,PointF &rp2)
{
    PointF p2;
    p2.x=(C.O.x+sp.x)/2;
    p2.y=(C.O.y+sp.y)/2;

    double dx2,dy2,r2;
    dx2=p2.x-C.O.x;
    dy2=p2.y-C.O.y;
    r2=sqrt(dx2*dx2+dy2*dy2);
    Circle C2(p2,r2);
    C1CrossC2(C,C2,rp1,rp2);
}

bool compareValue(const PointF& pt1, const PointF& pt2) //对点的坐标按y从小到大排序，若y值相同则按x从小到大排序
{
    if (pt1.y != pt2.y)
        return pt1.y < pt2.y;
    else
        return pt1.x < pt2.x;
}
bool Is_PtIsVertex(PointF pt,Polygon plg)
{
    for(int i=0;i<plg.n;i++)
        if(pt==plg.Vs[i])
            return true;
    return false;
}

double GetLength(double dx,double dy)
{
    return sqrt(pow(dx,2)+pow(dy,2));
}

PointF GetProportionPoint(PointF point, double segment,double length, double dx, double dy)
{
    double factor = segment / length;
    PointF temp;
    temp.x=point.x + dx * factor;
    temp.y=point.y + dy * factor;
    return temp;
}
bool Circle_CutVector(PointF pt0,PointF pt1,PointF pt2,double theta,double R,Circle &circle,PointF &PtCut1,PointF &PtCut2)//与向量V01,向量V02 相切的圆
{
    double dx1;
    double dy1;
    double dx2;
    double dy2;
    double length1;
    double length2;
    double segment;
    PointF p1Cut;
    PointF p2Cut;
    dx1 = pt1.x-pt0.x;
    dy1 = pt1.y-pt0.y;
    dx2 = pt2.x-pt0.x;
    dy2 = pt2.y-pt0.y;
    length1=GetLength(dx1,dy1);
    length2=GetLength(dx2,dy2);
    segment=R/tan(theta/2);
    p1Cut=GetProportionPoint(pt0, segment, length1, dx1, dy1);
    p2Cut=GetProportionPoint(pt0, segment, length2, dx2, dy2);
    PtCut1=p1Cut;
    PtCut2=p2Cut;
    /****获取pt0到圆心的向量***/
    double ddx1 = p1Cut.x-pt0.x;
    double ddy1 = p1Cut.y-pt0.y;
    double ddx2 = p2Cut.x-pt0.x;
    double ddy2 = p2Cut.y-pt0.y;
    double ddx=ddx1+ddx2;
    double ddy=ddy1+ddy2;
    double seg=R/sin(theta/2);
    double length=GetLength(ddx,ddy);
    circle.R=R;
    circle.O=GetProportionPoint(pt0, seg, length, ddx, ddy);
    return true;
}
bool get_CircleCutC1C2(Circle C1,Circle C2,LINESEG seg,double R,Circle &C)
{
//    cout<<"R   "<<R<<endl;

    double l_seg=seg.get_SegLength();
    double Dx=(seg.e.x-seg.s.x)/l_seg;
    double Dy=(seg.e.y-seg.s.y)/l_seg;
    double R_C3=R;

    double l_C1C2=sqrt(pow((C1.O.x-C2.O.x),2)+pow((C1.O.y-C2.O.y),2));
    double dx=(C1.O.x-C2.O.x)/l_C1C2;
    double dy=(C1.O.y-C2.O.y)/l_C1C2;
    double ddx=-dy;
    double ddy=dx;
//    cout<<"i   "<<" l_seg   "<<l_seg<<endl;
//    cout<<"i   "<<" l_C1C2   "<<l_C1C2<<endl;
    if((Dx*ddx+Dy*ddy)<0)
    {
        ddx=dy;
        ddy=-dx;
    }
    double theta=asin(l_C1C2/(2*(R+R_C3)));
    if(l_C1C2/2>(R+R_C3))
        return false;
    double dist=(R+R_C3)*cos(theta); //C到 C1C2的距离
    PointF pt_mid((C1.O.x+C2.O.x)/2,(C1.O.y+C2.O.y)/2);
    C.O.x=pt_mid.x+dist*ddx;
    C.O.y=pt_mid.y+dist*ddy;
    C.R=R_C3;
//    cout<<"i   "<<" pt_mid.x   "<<pt_mid.x<<endl;
//    cout<<"i   "<<" pt_mid.y   "<<pt_mid.y<<endl;
//    cout<<"i   "<<" dist   "<<dist<<endl;
//    cout<<"i   "<<" ddx   "<<ddx<<endl;
//    cout<<"i   "<<" ddy "<<ddy<<endl;
    return true;
}

bool LineCrossPolygon(LINE l,Polygon plg,vector<PointF> &cross_pts)
{
    vector<PointF> pts_temp;
    if(plg.Es.size()==0)
        plg.getEdge();
    //cout<<"l.a   "<<l.a<<"l.b   "<<l.b<<"l.c   "<<l.c<<endl;

    for(unsigned int i=0;i<plg.Es.size();i++)
    {
        LINESEG seg=plg.Es[i];
        if(Is_PtOnLine(l,seg.s)&&Is_PtOnLine(l,seg.e)) //线段在直线上，
            pts_temp.push_back(seg.e);
        else
        {
            LINE l2(seg.s,seg.e);
            PointF pt;
            bool cross_flag=line_intersect(l,l2,pt);
//            cout<<"i   "<<i<<" cross_flag   "<<cross_flag<<endl;
//            cout<<"i   "<<i<<" Is_PtonSeg   "<<(Is_PtonSegment(seg,pt))<<endl;
//            cout<<"i   "<<i<<" (pt==seg.s)   "<<(pt==seg.s)<<endl;

            if(cross_flag&&Is_PtonSegment(seg,pt)&&(!(pt==seg.s)))  //两直线的交点在线段上，并且和
                pts_temp.push_back(pt);

        }
    }
    //cout<<"pts_temp.size()    "<<pts_temp.size()<<endl;

    if(pts_temp.size()==0)
        return false;
    cross_pts=pts_temp;
    std::sort(cross_pts.begin(), cross_pts.end(), compareValue);
    return true;
}
bool LineCrossPolygon_Seg(LINE l,Polygon plg,vector<LINESEG> &cross_seg,double L_THR,bool order_flag)
{
    vector<PointF> cross_pts;

    LineCrossPolygon(l,plg,cross_pts);
    //cout<<"cross_pts.size()    "<<cross_pts.size()<<endl;
    //cout<<"cross_pts.size() "<<cross_pts.size()<<endl;

    if(cross_pts.size()<2)
        return false;
    if(!order_flag)
        std::reverse(std::begin(cross_pts), std::end(cross_pts));
//    for(int i=0;i<cross_pts.size();i++)
//        cout<<"iiii "<<i<<"    x "<<cross_pts[i].x<<"    y "<<cross_pts[i].y<<endl;
    for(unsigned int i=0;i<cross_pts.size()-1;i++)
    {
//        if(Is_PtIsVertex(cross_pts[i],plg)&&Is_PtIsVertex(cross_pts[i+1],plg))
//            continue;
//        else
//        {
//            PointF temp;
//            temp.x=(cross_pts[i].x+cross_pts[i+1].x)/2;
//            temp.y=(cross_pts[i].y+cross_pts[i+1].y)/2;
//            cout<<"ggggg "<<i<<"    x "<<temp.x<<"    y "<<temp.y<<endl;
//            cout<<"ggggg "<<Is_PtInPolygon(plg,temp)<<endl;

//            if(Is_PtInPolygon(plg,temp)==0&&LINESEG(cross_pts[i],cross_pts[i+1]).get_SegLength()>=L_THR)
//                cross_seg.push_back(LINESEG(cross_pts[i],cross_pts[i+1]));
//        }
        PointF temp;
        temp.x=(cross_pts[i].x+cross_pts[i+1].x)/2;
        temp.y=(cross_pts[i].y+cross_pts[i+1].y)/2;
//        cout<<"ggggg "<<i<<"    x "<<temp.x<<"    y "<<temp.y<<endl;
//        cout<<"ggggg "<<Is_PtInPolygon(plg,temp)<<endl;

//        cout<<"ggggg "<<LINESEG(cross_pts[i],cross_pts[i+1]).get_SegLength()<<endl;

        if(Is_PtInPolygon(plg,temp)==0&&LINESEG(cross_pts[i],cross_pts[i+1]).get_SegLength()>=L_THR)
            cross_seg.push_back(LINESEG(cross_pts[i],cross_pts[i+1]));
    }
    if(cross_seg.size()<1)
        return false;
    return true;
}
double Pt1VertexPt2_dist(Polygon plg,const PointF pt_start,const PointF pt_end,vector<PointF> &SortPts) //获得p1p2和之间的边界点，顺序和边界点的原始顺序一致
{
    int num_star=-1;  //初始点所在边号
    int num_end=-1;  //终止点所在边号
    if(plg.Es.size()==0)
        plg.getEdge();
    for(int i=0;i<plg.n;i++)  //点在线段上，并且点和线段终点不重合
        if((Is_PtonSegment(plg.Es[i],pt_start)&&(!(plg.Es[i].e==pt_start))))
        {
            num_star=i;
            break;
        }
    for(int i=0;i<plg.n;i++)  //点在线段上，并且点和线段终点不重合
        if((Is_PtonSegment(plg.Es[i],pt_end)&&(!(plg.Es[i].e==pt_end))))
        {
            num_end=i;
            break;
        }
//    cout<<"num_star "<<num_star<<endl;
//    cout<<"num_end "<<num_end<<endl;
    double dist=0;
    if(num_star>num_end)
        num_end=num_end+plg.n;
    if(pt_start==pt_end)
        num_end=num_end+plg.n;
    //cout<<"num_end "<<num_end<<endl;
    SortPts.push_back(pt_start);
    for(int i=num_star;i<num_end;i++)
    {
        int num=i%plg.n;
        if((!(plg.Es[num].e==pt_start))&&(!(plg.Es[num].e==pt_end)))
            SortPts.push_back(plg.Es[num].e);
    }
    SortPts.push_back(pt_end);
    //cout<<"SortPts "<<SortPts.size()<<endl;

    for(unsigned int i=0;i<SortPts.size()-1;i++)
        dist+=Dist_pt(SortPts[i],SortPts[i+1]);
    //cout<<"&&&&dist "<<dist<<endl;
    return dist;

}
bool PlgVertexBetPts(PointF Pt1,PointF Pt2,Polygon plg,vector<PointF> &Pts)
{
    //cout<<"pt   "<<Pt1.x<<" "<<Pt1.y<<endl;

    if(Is_PtInPolygon(plg,Pt1)!=1||Is_PtInPolygon(plg,Pt2)!=1)
        return false;
    vector<PointF> SortPts_p1p2;
    vector<PointF> SortPts_p2p1;

    double dist_p1p2=Pt1VertexPt2_dist(plg,Pt1,Pt2,SortPts_p1p2); //获得p1p2和之间的边界点，顺序和边界点的原始顺序一致
    double dist_p2p1=Pt1VertexPt2_dist(plg,Pt2,Pt1,SortPts_p2p1); //获得p2p1和之间的边界点，顺序和边界点的原始顺序一致

//    cout<<"dist_p2p1   "<<dist_p2p1<<endl;
//    for(int i=0;i<SortPts_p1p2.size();i++)
//        cout<<SortPts_p1p2[i].x<<"   "<<SortPts_p1p2[i].y<<endl;

//    cout<<"dist_p1p2   "<<dist_p1p2<<endl;
//    for(int i=0;i<SortPts_p2p1.size();i++)
//        cout<<SortPts_p2p1[i].x<<"   "<<SortPts_p2p1[i].y<<endl;


    std::reverse(SortPts_p2p1.begin(),SortPts_p2p1.end());
    if(dist_p2p1<dist_p1p2)
        Pts=SortPts_p2p1;
    else
        Pts=SortPts_p1p2;
    return true;
}
bool  pt_concave_convex(vector<PointF> pList_temp,int num)
{
    PointF v1;
    PointF v2;
    v1=pList_temp.at((num-1+pList_temp.size())%pList_temp.size())-pList_temp.at(num);
    v2=pList_temp.at((num+1)%pList_temp.size())-pList_temp.at(num);
    if((v1.x*v2.y-v1.y*v2.x)>=0)
        return true;
    else
        return false;
}
bool Is_PtOnPolygonSeg(Polygon plg,PointF Pt,LINESEG &seg,int &seg_num)
{
    for(int i=0;i<plg.n;i++)
    {
        if(Is_PtonSegment(plg.Es[i],Pt))
        {
            seg_num=i;
            seg=plg.Es[i];
            return true;
        }
    }
    return false;
}
Polygon Plg_Expand(const Polygon plg,double dist)
{
    Polygon plg_out;
    vector<PointF> pList;
    for(int i=0;i<plg.n;i++)
        pList.push_back(plg.Vs[i]);
    vector<PointF> out;
    // 2. edge set and normalize it
    vector<PointF> dpList, ndpList;
    int count = pList.size();
    int sig=1;
    if(!IsClockwise(pList))
        sig=-1;
    cout<<"边界点是否为顺时针   "<<IsClockwise(pList)<<endl;


    for(int i = 0; i < count; i++)
    {
        int next = (i==(count-1) ? 0: (i+1));
        //pt_temp.setX();
        dpList.push_back(pList.at(next)-pList.at(i));
        double unitLen = 1.0/sqrt(pow(dpList[i].x,2)+pow(dpList[i].y,2));
        ndpList.push_back(dpList.at(i) * unitLen);
        //cout<<"i="<<i<<",pList:"<<pList[i].x()<<","<<pList[i].y()<<"    "<<",dpList:"<<dpList[i].x()<<","<<dpList[i].y()<<"    "<<",ndpList:"<<ndpList[i].x()<<","<<ndpList[i].y()<<endl;
    }

    // 3. compute Line
    for(int i = 0; i < count; i++)
    {
        float SAFELINE = sig*dist;//负数为内缩， 正数为外扩。 需要注意算法本身并没有检测内缩多少后折线会自相交，那不是本代码的示范意图
        int startIndex = (i==0 ? (count-1):(i-1));
        int endIndex = i;
        double theta=acos((ndpList[startIndex].x*ndpList[endIndex].x + ndpList[startIndex].y*ndpList[endIndex].y)/ (sqrt(pow(ndpList[startIndex].x, 2) + pow(ndpList[startIndex].y, 2))*sqrt(pow(ndpList[endIndex].x, 2) + pow(ndpList[endIndex].y, 2))));
        double sinTheta =sin(theta);
        //cout<<"theta    "<<theta<<endl;
        //cout<<"sinTheta "<<sinTheta<<endl;
        if(!pt_concave_convex(pList,i))
            SAFELINE=-SAFELINE;
        PointF orientVector = ndpList.at(endIndex) - ndpList.at(startIndex);//i.e. PV2-V1P=PV2+PV1
        PointF temp_out;
//        temp_out.x = pList.at(i).x + SAFELINE/sinTheta * orientVector.x;
//        temp_out.y = pList.at(i).y + SAFELINE/sinTheta * orientVector.y;
        temp_out.x=pList[i].x + SAFELINE/sinTheta * orientVector.x;
        temp_out.y=pList[i].y + SAFELINE/sinTheta * orientVector.y;
        out.push_back(temp_out);
    }
    plg_out.input(out.size(),out);
    return plg_out;
}
bool get_Bspline_pts(vector<PointF> key_pts,double gap,int k,vector<PointF> &final_pts) //k次B样条(K为2或者3)，生成点集点间距为gap
{
    if(key_pts.size()<2||gap<EP||k>3||k<2)
        return false;
    final_pts.clear();
    CBSpline bspline;
    vector<int> Intnum;
    for(unsigned int i=0;i<key_pts.size()-1;i++){
        Intnum.push_back(int(sqrt(pow((key_pts[i+1].x-key_pts[i].x),2)+pow((key_pts[i+1].y-key_pts[i].y),2))/0.2));                 //  每一个样条曲线内插入10个点
    }
    if(k==2)
    {if(!bspline.TwoOrderBSplineInterpolatePt(key_pts,final_pts,Intnum)) return false;}        //  二次B样条曲线
    else
    {if(!bspline.ThreeOrderBSplineInterpolatePt(key_pts,final_pts,Intnum)) return false;}        //  二次B样条曲线
    return true;
}



double get_distQPointF(QPointF p1,QPointF p2)
{
    return sqrt(pow((p1.y()-p2.y()),2)+pow((p1.x()-p2.x()),2));
}

QPointF getCrossQPointF(QPointF p1, QPointF p2, QPointF p3, QPointF p4)//求出交点,直线p1p2与直线p3p4
{//t=lamta/(1+lamta)
    double x1 = p1.x(), y1 = p1.y();
    double x2 = p2.x(), y2 = p2.y();
    double x3 = p3.x(), y3 = p3.y();
    double x4 = p4.x(), y4 = p4.y();
    double t = ((x2 - x1)*(y3 - y1) - (x3 - x1)*(y2 - y1)) / ((x2 - x1)*(y3 - y4) - (x3 - x4)*(y2 - y1));
    QPointF temp;
    temp.setX(x3 + t*(x4 - x3));
    temp.setY(y3 + t*(y4 - y3));
    return temp;
}


// 判断线段 a-b 在不在多边形 v 中
// 首先线段的两个端点要在多边形中, 这是必要条件
// 因为多边形可能是凹集, 线段某一段可能在多边形外面
bool Is_SegInPolygon(PointF *v, int n, PointF &a, PointF &b)      //线段是否在多边形内
{
    int i, j, k;

    // 判断 a, b 两点在不在多边形中
    if (!Is_PtInPloygon(v, n, a)) return false;
    if (!Is_PtInPloygon(v, n, b)) return false;
    for (i=0; i<n; i++)		// 遍历所有边 v[i]-[j]
    {
        j = (i + 1) % n;
        // 不包括端点的相交
        if (Is_Seg1CrosSeg2(a, b, v[i], v[j])) return false;

        // 如果 i 点在线段 a-b 上, 判断是不是内切
        // 判断 i 点前后的两条线段是不是横跨 a-b
        if (Is_PtOnSegment(v[i], a, b))
        {
            k = (i - 1) % n;
            if (Is_Seg1CrosSeg2(v[j], v[k], a, b))
                return false;
        }
    }
    return true;
}

inline double fabs(double x)
{
    return x > 0 ? x : -x;
}

// 判断 k 在不在线段 a-b 上
bool Is_PtOnSegment(PointF &k, PointF &a, PointF &b)
{
    double x1 = MIN(a.x, b.x);
    double x2 = MAX(a.x, b.x);
    double y1 = MIN(a.y, b.y);
    double y2 = MAX(a.y, b.y);

    if (x1 <= k.x && k.x <= x2 && y1 <= k.y && k.y <= y2)
    {
        // a->k, a->b 两个向量做叉积, 判断是不是 0
        x1 = k.x - a.x;
        y1 = k.y - a.y;
        x2 = b.x - a.x;
        y2 = b.y - a.y;
        if (fabs(x1 * y2 - x2 * y1) < EP)
            return true;
    }
    return false;
}

// 判断 p1, p1 在不在 向量q1q2 的两端, 在返回 1
bool Is_PtOnVectorTwosides(PointF &p1, PointF &p2, PointF &q1, PointF &q2)
{
    double x1, y1, x2, y2, x3, y3;

    // q1->p1 (x1, y1), q1->p2 (x2, y2), q1->q2 (x3, y3)
    x1 = p1.x - q1.x;   y1 = p1.y - q1.y;
    x2 = p2.x - q1.x;   y2 = p2.y - q1.y;
    x3 = q2.x - q1.x;   y3 = q2.y - q1.y;
    if ((x1*y3 - x3*y1) * (x2*y3 - x3*y2) < 0) return true;
    return false;
}

// 判断线段 a-b, c-d 是否相交, 不判断两个端点
bool Is_Seg1CrosSeg2(PointF &a, PointF &b, PointF &c, PointF &d)
{
    if (Is_PtOnVectorTwosides(a, b, c, d) && Is_PtOnVectorTwosides(c, d, a, b))
        return true;
    return false;
}

// 判断 k 点在不在多边形 s 内
bool Is_PtInPloygon(PointF *s, int n, PointF &k)
{
    PointF t;		// 左水平射线 t <- k
    PointF tmp;
    int count = 0;
    int i, j;

    t.y = k.y;
    t.x = -0xFFFFF;
    for (i=0; i<n; i++)		// 遍历每一条边 s[i-1] - s[i]
    {
        j = (i + 1) % n;
        if (Is_PtOnSegment(k, s[i], s[j]))	// 点 k 在边上
            return true;
        else
        {
            // 判断线段是不是水平的, 水平线段忽略
            if (fabs(s[i].y - s[j].y) > EP)
            {
                // 找出线段中较高的点 tmp
                if (s[i].y < s[j].y) tmp = s[j];
                else tmp = s[i];
                // 判断 tmp 在不在射线上
                if (Is_PtOnSegment(tmp, k, t))
                    count++;
                // 或者射线与边相交
                else if (Is_Seg1CrosSeg2(k, t, s[i], s[j]))
                    count++;
            }
        }
    }
    if (count % 2 == 1) return true;
    else return false;
}

double get_PtAlpha(double dx,double dy)
{
    PointF PtO(0,0);
    PointF Pts(1,0);
    PointF Pte(dx,dy);
    double alpha=Vect_angle(PtO,Pts,Pte);
    if(alpha<0)
        alpha+=2*PI;
    return alpha;
//    if(IsEqual(dx,0))
//    {
//        if(dy>0||IsEqual(dy,0))
//            return PI/2;
//        else
//            return PI*3/2;
//    }
//    if(IsEqual(dy,0))
//    {
//        if(dx>0||IsEqual(dx,0))
//            return 0;
//        else
//            return PI;
//    }
//    if(dx>0&&dy>0)
//    {
//        return atan(dy/dx);
//    }
//    else if(dx<0&&dy>0)
//    {
//        return atan(dy/dx)+PI;
//    }
//    else if(dx<0&&dy<0)
//    {
//         return atan(dy/dx)+PI;
//    }
//    else
//    {
//        return atan(dy/dx)+PI*2;
//    }

}
void SelfIntersectedPolygon2NormalPolygon(Polygon plg1,Polygon &plg2)
{
    std::cout<<"SelfIntersectedPolygon2NormalPolygon  "<<plg1.Vs.size()<<endl;
    int i=0;
    int origin_n=plg1.n;
    vector<LINESEG> segs;
    bool flag=false;
    plg2.init();
    while(i<plg1.Vs.size())
    {
        int j=(i+1)%plg1.Vs.size();
        segs.push_back(LINESEG(plg1.Vs[i],plg1.Vs[j]));
        i++;
        if(i<3) continue;
        for(unsigned int k=0;k<segs.size()-2;k++)
        {
            if(intersect_seg2seg(segs[k],segs[segs.size()-1]))
            {
                //std::cout<<"k "<<k<<"  segs.size() "<<segs.size()<<endl;
                if(k==0&&segs[segs.size()-1].e==segs[0].s)
                    continue;
                //std::cout<<"kk "<<k<<endl;
                PointF inter;
                if((segs.size()-k)>origin_n/2)
                {
                    //std::cout<<"s "<<plg1.Vs.size()<<endl;
                    segment_intersect(segs[k],segs[segs.size()-1],inter);
                    plg1.Vs.erase(plg1.Vs.begin(),plg1.Vs.begin()+k+1);
                    //std::cout<<"s "<<plg1.Vs.size()<<endl;
                    plg1.Vs.erase(plg1.Vs.begin()+(segs.size()-k-1),plg1.Vs.end());
                    //std::cout<<"s "<<plg1.Vs.size()<<endl;
                    plg1.Vs.insert(plg1.Vs.begin(),inter);
                    for(unsigned int n=0;n<plg1.Vs.size();n++)
                        plg2.add(plg1.Vs[n]);
                    flag=true;
                    break;
                }
                //std::cout<<"kkk "<<k<<endl;
                segment_intersect(segs[k],segs[segs.size()-1],inter);
                plg1.Vs.erase(plg1.Vs.begin()+k+1,plg1.Vs.begin()+segs.size());
                plg1.Vs.insert(plg1.Vs.begin()+k+1,inter);
                segs.erase(segs.begin()+k,segs.end());
                i=k;
                break;
            }
        }
        if(flag==true)
            break;
    }
    if(plg2.n==0)
        for(unsigned int j=0;j<segs.size();j++)
            plg2.add(segs[j].s);
}


