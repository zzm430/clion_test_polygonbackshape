//
// Created by zzm on 23-12-22.
//
#ifndef POLYGONBACKSHAPE_DUBIN_PATH_H
#define POLYGONBACKSHAPE_DUBIN_PATH_H
#include <omp.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "HybridAStar/hybridA_param.h"
#include "HybridAStar/node_HybridA.h"
#include "common/utilpath/path_dubins.h"

namespace searchAlgorithm{

    const double EPSILON = 10e-10;

    typedef enum { L_SEG = 0, S_SEG = 1, R_SEG = 2 } SegmentType;

    /* The segment types for each of the Path types */
    const SegmentType DIRDATA[][3] = {{L_SEG, S_SEG, L_SEG},
                                      {L_SEG, S_SEG, R_SEG},
                                      {R_SEG, S_SEG, L_SEG},
                                      {R_SEG, S_SEG, R_SEG},
                                      {R_SEG, L_SEG, R_SEG},
                                      {L_SEG, R_SEG, L_SEG}};

    struct DubinParam {
        bool flag = false;
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
    };

    struct DubinsIntermediateResults {
        double alpha;
        double beta;
        double d;
        double sa;
        double sb;
        double ca;
        double cb;
        double c_ab;
        double d_sq;
    };


    class Dubin {
    public:
        Dubin(const vehicleParam& vehicle_param,
              const coarseSearchParam& coarse_search_param);
        Dubin() = default;

        virtual ~Dubin() = default;

        bool AllDPs(const std::shared_ptr<nodeTractor> start_node,
                    const std::shared_ptr<nodeTractor> end_node,
                    std::vector<dubinPath>* all_paths);

        bool ShortestPath(const std::shared_ptr<nodeTractor> start_node,
                          const std::shared_ptr<nodeTractor> end_node,
                          dubinPath* optimal_path,
                          double* optimal_cost);

        double ShortestPathLength(const std::shared_ptr<nodeTractor> start_node,
                                  const std::shared_ptr<nodeTractor> end_node);

        enum class DubinsPathType {
            LSL = 0,
            LSR = 1,
            RSL = 2,
            RSR = 3,
            RLR = 4,
            LRL = 5
        };

    protected:
        struct DubinsPath {
            /* the initial configuration */
            double qi[3];
            /* the lengths of the three segments */
            double param[3];
            /* model forward velocity / model angular velocity */
            double rho;
            /* the path type described */
            DubinsPathType type;
        };

        /**
         * Generate a path from an initial configuration to
         * a target configuration, with a specified maximum turning
         * radii
         *
         * A configuration is (x, y, theta), where theta is in radians, with zero
         * along the line x = 0, and counter-clockwise is positive
         *
         * @param path  - the resultant path
         * @param q0    - a configuration specified as an array of x, y, theta
         * @param q1    - a configuration specified as an array of x, y, theta
         * @param rho   - turning radius of the vehicle (forward velocity divided by
         * maximum angular velocity)
         * @return      - non-zero on error
         */
        int dubins_shortest_path(DubinsPath* path,
                                 double q0[3],
                                 double q1[3],
                                 double rho);

        /**
         * Calculate the length of an initialised path
         *
         * @param path - the path to find the length of
         */
        double dubins_path_length(DubinsPath* path);

        /**
         * Return the length of a specific segment in an initialized path
         *
         * @param path - the path to find the length of
         * @param i    - the segment you to get the length of (0-2)
         */
        double dubins_segment_length(DubinsPath* path, int i);

        /**
         * Calculate the configuration along the path, using the parameter t
         *
         * @param path - an initialised path
         * @param t    - a length measure, where 0 <= t < dubins_path_length(path)
         * @param q    - the configuration result
         * @returns    - non-zero if 't' is not in the correct range
         */
        void dubins_path_sample(DubinsPath* path, double t, double q[3]);

        /**
         * Walk along the path at a fixed sampling interval, calling the
         * callback function at each interval
         *
         * The sampling process continues until the whole path is sampled, or the
         * callback returns a non-zero value
         *
         * @param path      - the path to sample
         * @param stepSize  - the distance along the path for subsequent samples
         * @param cb        - the callback function to call for each sample
         * @param user_data - optional information to pass on to the callback
         *
         * @returns - zero on successful completion, or the result of the callback
         */
        int dubins_path_sample_many(DubinsPath* path,
                                    double stepSize,
                                    dubinPath& discret_path);

        DubinsIntermediateResults dubins_intermediate_results(
                const std::shared_ptr<nodeTractor> start_node,
                const std::shared_ptr<nodeTractor> end_node,
                double rho);

        void dubins_segment(double t, double qi[3], double qt[3], SegmentType type);

        bool dubins_LSL(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);
        bool dubins_RSR(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);
        bool dubins_LSR(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);
        bool dubins_RSL(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);
        bool dubins_RLR(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);
        bool dubins_LRL(const DubinsIntermediateResults& in,
                        std::vector<double>* seg_lengths);

    protected:
        vehicleParam vehicle_param_;
        coarseSearchParam planner_open_space_config_;
        double turing_radius_ = 0.0;
        double start_x_ = 0.0;
        double start_y_ = 0.0;
        double start_phi_ = 0.0;
        double step_size_ = 0.0;
    };


}


#endif //POLYGONBACKSHAPE_DUBIN_PATH_H
