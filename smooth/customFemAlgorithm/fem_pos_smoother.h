//
// Created by zzm on 23-11-24.
//

#ifndef POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
#define POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
#include <utility>
#include <vector>

namespace  smoothalgorithm{

    class femSmoothManager{
    public:
        femSmoothManager() = default;
        virtual ~femSmoothManager() = default;

        femSmoothManager(
               const std::vector<double>  bounds,
               const std::vector<std::pair<double,double>>  xy_pts,
               std::vector<double>& opt_x,
               std::vector<double>& opt_y);


    private:


    };

}


#endif //POLYGONBACKSHAPE_FEM_POS_SMOOTHER_H
