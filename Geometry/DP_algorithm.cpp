#include "geometric_algorithm.h"
double PointToLine(DP_Point p1, DP_Point p2, DP_Point p3)
{
    double dist;
    double A, B, C;

    if(p1.x==p2.x)
        return fabs(p3.x-p1.x);

    A = -(p2.y - p1.y) / (p2.x - p1.x);
    B = 1.0;
    C = -A * p1.x - p1.y;
    dist = abs(A * p3.x + B * p3.y + C) / sqrt(A * A + B * B);
    return dist;
}
double getMaxDist(vector<DP_Point> &Points, int begin, int end)
{
    vector<double> dists;
    double maxdist;
    for (int i = begin; i <= end; i++)
    {
        dists.push_back(PointToLine(Points[begin], Points[end], Points[i]));
    }
    auto max = max_element(dists.begin(), dists.end());
    return *max;
}
int getMaxDistIndex(vector<DP_Point> &Points, int begin, int end)
{
    vector<double> dists;
    int index;
    for (int i = begin; i <= end; i++)
    {
        dists.push_back(PointToLine(Points[begin], Points[end], Points[i]));
    }
    auto max = max_element(dists.begin(), dists.end());
    index = Points[begin].ID + distance(dists.begin(),max);
    return index;
}
void DP(vector<DP_Point> &Points, int begin, int end, double threshold)
{
    int mid;
    if (end - begin > 1)
    {
        if (getMaxDist(Points, begin, end) > threshold)
        {
            mid = getMaxDistIndex(Points, begin, end);
            DP(Points, begin, mid, threshold);
            DP(Points, mid, end, threshold);
        }
        else
        {
            for (int i = begin + 1; i < end; i++)
            {
                Points[i].isRemoved = true;
            }
        }
    }
    else
    {
        return;
    }
}
void Polygon2DP_Polygon(Polygon plg,Polygon &plg_re)
{
    std::cout<<"******fieldpts2dppts_fdPts.size************ "<<plg.n<<std::endl;
    plg_re.init();
    DP_Point dp_pt;
    vector<DP_Point> dp_pts;
    double threshold=1;
    if(plg.Vs[0]==plg.Vs[plg.Vs.size()-1])
    {
        plg.Vs.pop_back();
        plg.n--;
    }
    if(plg.n<=3)
    {
        plg_re=plg;
        return;
    }

    for(int i=0;i<plg.n;i++)
    {
        dp_pt.ID=i;
        dp_pt.x=plg.Vs[i].x;
        dp_pt.y=plg.Vs[i].y;
        dp_pts.push_back(dp_pt);
    }
    DP(dp_pts, 0, plg.n-1, threshold);
    for(int i=0;i<dp_pts.size();i++)
    {
        if(dp_pts[i].isRemoved==false)
            plg_re.add(PointF(dp_pts[i].x,dp_pts[i].y));
    }
    std::cout<<"******_DP_pts.size************ "<<plg_re.n<<std::endl;
    for(int i=0;i<plg_re.Vs.size();i++)
        std::cout<<setiosflags(ios::fixed)<<std::setprecision(4)<<"DP_"<<i<<"_x "<<plg_re.Vs[i].x<<"    DP_"<<i<<"_y "<<plg_re.Vs[i].y<<std::endl;
    return ;
}
