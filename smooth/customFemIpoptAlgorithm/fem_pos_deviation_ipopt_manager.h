//
// Created by zzm on 24-1-22.
//

#ifndef POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_MANAGER_H
#define POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_MANAGER_H
#include "fem_pos_deviation_ipopt_interface.h"
#include "fem_pos_deviation_ipopt_param.h"
#include "easylogging++.h"
namespace smoothalgorithm{

    class femPosIpoptSmoother{
    public:
        femPosIpoptSmoother() = default;
        femPosIpoptSmoother(femPosDeviationIpoptParam femIpoptParam);
        bool NlpWithIpopt(const std::vector<std::pair<double, double>>& raw_point2d,
                          const std::vector<double>& bounds,
                          std::vector<double>* opt_x, std::vector<double>* opt_y);
    private:
        femPosDeviationIpoptParam fem_ipopt_param_;
    };
}



#endif //POLYGONBACKSHAPE_FEM_POS_DEVIATION_IPOPT_MANAGER_H
