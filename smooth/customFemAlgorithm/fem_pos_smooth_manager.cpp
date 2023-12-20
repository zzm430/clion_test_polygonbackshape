//
// Created by zzm on 23-11-29.
//

#include "smooth/customFemAlgorithm/fem_pos_smooth_manager.h"

namespace  smoothalgorithm{

    femSmoothManager::femSmoothManager(
                                      const std::vector<std::pair<double, double>> xy_pts):xy_pts_(xy_pts){

    }

    void femSmoothManager::initiate(){

        //设置anchor_pts
        auto size_achor_pts = xy_pts_.size();
        std::vector<AnchorPoint> anchor_pts;
        for(int i = 0;i < size_achor_pts;i++){
            AnchorPoint temp;
            temp.lateral_bound_ = BOUND;
            temp.path_point_ = polygonPoint(xy_pts_[i].first,xy_pts_[i].second);
            anchor_pts.push_back(temp);
        }
        anchor_pts.front().lateral_bound_ = 0.0;
        anchor_pts.back().lateral_bound_ = 0.0;

        FemPosDeviationSmoother   smoothFem;
        normalizePoints(&xy_pts_);

        std::vector<double> bounds;
        std::vector<double> opt_x;
        std::vector<double> opt_y;
        for(const auto& pt : anchor_pts){
            bounds.emplace_back(pt.lateral_bound_);
        }
        smoothFem.Solve(xy_pts_,bounds,&opt_x, &opt_y);
        std::vector<std::pair<double, double>> smoothed_point2d;

        for(int i = 0;i <opt_x.size();i++){
            smoothed_point2d.emplace_back(opt_x[i],opt_y[i]);
        }

        deNormalizePoints(&smoothed_point2d);

        std::vector<polygonPoint> last_pts;
        setPathProfile(smoothed_point2d,last_pts);

#ifdef  DEBUG_STATIC_OBSTACLE
        std::ofstream   testfemSmooth;
        testfemSmooth.open("/home/zzm/Desktop/test_path_figure-main/src/testfemSmooth.txt",std::ios::out);
        for(auto i : last_pts){
            testfemSmooth << " " << i.x ;
        }
        testfemSmooth << std::endl;
        for(auto i : last_pts){
            testfemSmooth << " " << i.y;
        }
        testfemSmooth << std::endl;
        std::ofstream   testCurve;
        testCurve.open("/home/zzm/Desktop/test_path_figure-main/src/testCurve.txt",std::ios::out);
        for(auto i: last_pts){
            testCurve << " " << i.get_kappa();
        }
        testCurve << std::endl;

//        for(int  im = 1 ;im < last_pts.size();im++) {
//            tractorPolygonShow tractorPolygonShowInstance(im,
//                                                          last_pts);
//            auto tractorHeadPts = tractorPolygonShowInstance.getTractorPolygonHeadPts();
//            auto tractorTailPts = tractorPolygonShowInstance.getTractorPolygonTailPts();
//            std::string test1 =  "/home/zzm/Desktop/test_path_figure-main/src/tractorObstacles.txt";
//            auto & tractorHeadPtsStream = common::Singleton::GetInstance<tractorPolyPrint>(test1);
//            tractorHeadPtsStream.writePts(tractorHeadPts,tractorTailPts);
//        }

#endif

        for(auto i : last_pts){
            last_smoothed_pts_.push_back(i);
        }
    }

    void femSmoothManager::normalizePoints(
            std::vector<std::pair<double, double>>* xy_points) {
        x_ref_ = xy_points->front().first;
        y_ref_ = xy_points->front().second;
        std::for_each(xy_points->begin(),
                      xy_points->end(),
                      [this](std::pair<double, double>& point) {
                          auto curr_x = point.first;
                          auto curr_y = point.second;
                          std::pair<double, double> xy(curr_x - x_ref_,
                                                       curr_y - y_ref_);
                          point = std::move(xy);
                      });
    }

    void femSmoothManager::deNormalizePoints(
            std::vector<std::pair<double, double>>* xy_points) {
        std::for_each(xy_points->begin(),
                      xy_points->end(),
                      [this](std::pair<double, double>& point) {
                          auto curr_x = point.first;
                          auto curr_y = point.second;
                          std::pair<double, double> xy(curr_x + x_ref_,
                                                       curr_y + y_ref_);
                          point = std::move(xy);
                      });
    }

    bool femSmoothManager::setPathProfile(
            const std::vector<std::pair<double,double>>& point2d,
            std::vector<polygonPoint>& raw_path_points){

        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulated_s;

        if (!computePathProfileInfo::ComputePathProfile(
                point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
            return false;
        }

        // Load into path point
        size_t points_size = point2d.size();
        for (size_t i = 0; i < points_size; ++i) {
            polygonPoint  path_pt;
            path_pt.set_x(point2d[i].first);
            path_pt.set_y(point2d[i].second);
            path_pt.set_theta(headings[i]);
            path_pt.set_s(accumulated_s[i]);
            path_pt.set_kappa(kappas[i]);
            path_pt.set_dkappa(dkappas[i]);
            raw_path_points.push_back(path_pt);
        }
        return true;
    }

    std::vector<polygonPoint> femSmoothManager::get_smoothed_pts(){
        return last_smoothed_pts_;
    }

}