#ifndef PATH_POLYGONPLAN_H
#define PATH_POLYGONPLAN_H

#include "path_baseplan.h"
namespace aiforce{
namespace Route_Planning
{
 class pathPolygonPlan : public pathBasePlan{
 public:
     pathPolygonPlan(){}
     virtual ~pathPolygonPlan(){}

     virtual void initiate();

private:


 };
}
}

#endif // PATH_POLYGONPLAN_H
