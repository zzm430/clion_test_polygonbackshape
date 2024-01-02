//
// Created by zzm on 23-12-22.
//

#ifndef POLYGONBACKSHAPE_NODE_HYBRIDA_H
#define POLYGONBACKSHAPE_NODE_HYBRIDA_H
#include <memory>
#include <string>
#include <vector>
#include "hybridA_param.h"
#include "common/math/box2d.h"


namespace searchAlgorithm {

class nodeTractor{
public:
    nodeTractor() = default;
    virtual ~nodeTractor() = default;

    nodeTractor(const double x, const double y, const double phi);
    nodeTractor(const double x, const double y, const double phi,
           const std::vector<double>& XYbounds,
           const coarseSearchParam& open_space_conf);

    nodeTractor(const std::vector<double>& traversed_x,
           const std::vector<double>& traversed_y,
           const std::vector<double>& traversed_phi,
           const std::vector<double>& XYbounds,
           const coarseSearchParam& open_space_conf);

    static math::Box2d GetBoundingBox(
            const vehicleParam& vehicle_param_, const double x,
            const double y, const double phi);

    double GetCost() const { return traj_cost_ + heuristic_cost_; }
    double GetTrajCost() const { return traj_cost_; }
    double GetHeuCost() const { return heuristic_cost_; }
    int GetGridX() const { return x_grid_; }
    int GetGridY() const { return y_grid_; }
    int GetGridPhi() const { return phi_grid_; }
    double GetX() const { return x_; }
    double GetY() const { return y_; }
    double GetPhi() const { return phi_; }
    bool operator==(const nodeTractor& right) const;
    const std::string& GetIndex() const { return index_; }
    size_t GetStepSize() const { return step_size_; }
    bool GetDirec() const { return direction_; }
    double GetSteer() const { return steering_; }
    std::shared_ptr<nodeTractor> GetPreNode() const { return pre_node_; }
    const std::vector<double>& GetXs() const { return traversed_x_; }
    const std::vector<double>& GetYs() const { return traversed_y_; }
    const std::vector<double>& GetPhis() const { return traversed_phi_; }
    void SetPre(std::shared_ptr<nodeTractor> pre_node) { pre_node_ = pre_node; }
    void SetDirec(bool direction) { direction_ = direction; }
    void SetTrajCost(double cost) { traj_cost_ = cost; }
    void SetHeuCost(double cost) { heuristic_cost_ = cost; }
    void SetSteer(double steering) { steering_ = steering; }

private:
    static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid);

private:
    double x_ = 0.0;
    double y_ = 0.0;
    double phi_ = 0.0;
    size_t step_size_ = 1;
    std::vector<double> traversed_x_;
    std::vector<double> traversed_y_;
    std::vector<double> traversed_phi_;
    int x_grid_ = 0;
    int y_grid_ = 0;
    int phi_grid_ = 0;
    std::string index_;
    double traj_cost_ = 0.0;
    double heuristic_cost_ = 0.0;
    double cost_ = 0.0;
    std::shared_ptr<nodeTractor> pre_node_ = nullptr;
    double steering_ = 0.0;
    // true for moving forward and false for moving backward
    bool direction_ = true;
    };
}

#endif //POLYGONBACKSHAPE_NODE_HYBRIDA_H
