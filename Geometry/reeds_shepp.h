//
// Created by zzm on 2023/4/7.
//

#ifndef RSPATH_RS_H
#define RSPATH_RS_H


#include <boost/math/constants/constants.hpp>
#include <cassert>
#include <typeinfo>
#include <vector>


class ReedsSheppStateSpace
{
public:

    enum ReedsSheppPathSegmentType { RS_NOP=0, RS_LEFT=1, RS_STRAIGHT=2, RS_RIGHT=3 };
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    class ReedsSheppPath
    {
    public:
        ReedsSheppPath(const ReedsSheppPathSegmentType* type=reedsSheppPathType[0],
                       double t=std::numeric_limits<double>::max(), double u=0., double v=0.,
                       double w=0., double x=0.);

        double length() const { return totalLength_; }
        const ReedsSheppPathSegmentType* type_;
        double length_[5];
        double totalLength_;
    };
    double rho_= 6;//TURNNING RADIUS

    double distance(double q0[3], double q1[3]);

    std::vector <int> xingshentype(double q0[3], double q1[3]);

    std::vector<std::vector<double> > xingshensample(double q0[3], double q1[3], double step_size);

    ReedsSheppPath reedsShepp(double q0[3], double q1[3]);

public:
    void interpolate(double q0[3], ReedsSheppPath &path, double seg, double q[3]);
};




#endif //RSPATH_RS_H
