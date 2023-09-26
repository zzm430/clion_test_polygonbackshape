//
// Created by zzm on 23-9-25.
//

#ifndef POLYGONBACKSHAPE_COMMON_TYPEDEF_H
#define POLYGONBACKSHAPE_COMMON_TYPEDEF_H
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/intersections.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/create_straight_skeleton_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/partition_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/property_map.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

//圆相关
#include <CGAL/Circular_kernel_intersections.h>
#include <CGAL/Exact_circular_kernel_2.h>

struct  Kernel : public CGAL::Cartesian<double> {};
typedef Kernel::Point_2                           Point_2;
typedef Kernel::Line_2                            Line_2;
typedef Kernel::Segment_2                         Segment_2;
typedef CGAL::Polygon_2<Kernel>                   Polygon_2;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K ;
typedef K::FT                             cgal_FT ;
typedef K::Point_2                        cgal_Point ;
typedef CGAL::Polygon_2<K>                cgal_Polygon_2 ;
typedef CGAL::Straight_skeleton_2<K>      cgal_Ss ;
typedef boost::shared_ptr<cgal_Ss>        cgal_SsPtr ;
typedef K::Segment_2                      cgal_Segment_2;
typedef Kernel::Bounded_side              cgal_Bounded_side;
typedef CGAL::Convex_hull_traits_adapter_2<K,
CGAL::Pointer_property_map<cgal_Point>::type > Convex_hull_traits_2;

typedef boost::shared_ptr<cgal_Polygon_2> cgal_PolygonPtr ;
typedef std::vector<cgal_PolygonPtr>      cgal_PolygonPtrVector ;

typedef CGAL::Partition_traits_2<K>                         Traits;
typedef Traits::Point_2                               partition_Point_2;
typedef Traits::Polygon_2                             partition_Polygon_2;      // a polygon of indices
typedef std::list<partition_Polygon_2>                partition_Polygon_list;
//半边数据结构
typedef CGAL::HalfedgeDS_default<K> cgal_HalfedgeDS;

//圆相关
typedef CGAL::Exact_circular_kernel_2             Circular_k;
typedef CGAL::Point_2<Circular_k>                 Point1_2;
typedef CGAL::Circle_2<Circular_k>                Circle1_2;
typedef CGAL::Circular_arc_2<Circular_k>          Circular_arc_2;
typedef CGAL::CK2_Intersection_traits<Circular_k, Circle1_2, Circle1_2>::type Intersection_result;

using boostRetVal = std::pair<CGAL::Circular_arc_point_2<CGAL::Filtered_bbox_circular_kernel_2<CGAL::Circular_kernel_2<CGAL::Cartesian<CGAL::Gmpq>, CGAL::Algebraic_kernel_for_circles_2_2<CGAL::Gmpq> > > > , unsigned>;

#endif //POLYGONBACKSHAPE_COMMON_TYPEDEF_H
