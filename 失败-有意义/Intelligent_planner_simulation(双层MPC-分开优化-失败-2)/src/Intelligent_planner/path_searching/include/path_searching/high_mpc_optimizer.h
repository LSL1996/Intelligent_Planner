#ifndef _HIGH_MPC_OPTIMIZER_H
#define _HIGH_MPC_OPTIMIZER_H

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include "plan_env/edt_environment.h"
#include <math.h>
#include "uav_utils/geometry_utils.h"

#define inf 1000000.0
namespace Intelligent_planner{

    class high_mpc_optimizer
    {
    private:
        /* record data */
        EDTEnvironment::Ptr edt_map_;
        Eigen::MatrixXd start_state_;
        Eigen::Vector3d external_force_remain_;

        Eigen::Matrix3d As_;
        Eigen::Vector3d Bs_, Cs_;

        Eigen::VectorXd input_x_, input_y_, input_z_, input_f_;
        Eigen::VectorXd state_x_, state_y_, state_z_;
        Eigen::Vector3d map_min_, map_max_;
        Eigen::VectorXd Gradient_f_;

        /* mpc setting */
        int N_;                 // prediction horizon number
        std::vector<Eigen::Vector3d> mpc_traj_, mpc_vel_, mpc_acc_;
        std::vector<double> mpc_input_;
        double Ts_;             // time step
        double dist_0_;         // obstacle distance threshold
        double K_;
        Eigen::MatrixXd A_;     // state equation (S = A_ * U + B_ * S0)
        Eigen::MatrixXd B_;
        Eigen::MatrixXd C_;
        double f_;
        /* optimization parameters */
        double alpha1_;                  // the cost of similarity
        double alpha2_;                  // smooth control input weight
        double alpha3_;                  // avoid collisions
        double alpha4_;                  // the penalty of vel acc
        double alpha5_;                  // minimum input jerk
        double amiga1_;
        double amiga2_;

        int max_iteration_num_, iter_num_;
        double max_iteration_time_;
        double min_cost_;

        int dim_, variable_num_;

        bool is_init_system_finish_;
        /* useful function */
        void setInitialState(Eigen::MatrixXd start_state,
                             std::vector <Eigen::Vector3d> mpc_traj,
                             std::vector <Eigen::Vector3d> mpc_vel,
                             std::vector <Eigen::Vector3d> mpc_acc,
                             std::vector<double> mpc_input,
                             Eigen::Vector3d external_force_remain) ;
        void setInitialSystem();

        void optimize();

        /* cost function */
        /* calculate each part of cost function with control points q as input */
        static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
        void combineCost(const std::vector<double>& x, std::vector<double>& grad,  double& f_combine);
        void stateEquations();
        void calCostFunctionandGradient();
        void getExtforceGardCost(double x, double u, double grad, double cost);

    public:
        high_mpc_optimizer(){};
        ~high_mpc_optimizer(){};

        Eigen::Vector3d  odom_pos_;
        Eigen::Vector3d  end_pt_;
        Eigen::Quaterniond odom_orient_;
        std::vector<double> best_variable_;
        Eigen::Vector3d external_acc_;
        /* main API */
        void resetInputInital();
        void setEnvironment(const EDTEnvironment::Ptr& env);
        void setParam(ros::NodeHandle& nh);
        void getEnvGardandDistPos(double dist, Eigen::Vector3d& grad, Eigen::Vector3d& pos);

        std::vector<std::vector<Eigen::Vector3d>> highmpcOptimizeTraj(Eigen::MatrixXd start_state,
                                                                       std::vector <Eigen::Vector3d> mpc_traj,
                                                                       std::vector <Eigen::Vector3d> mpc_vel,
                                                                       std::vector <Eigen::Vector3d> mpc_acc,
                                                                       std::vector<double> mpc_input,
                                                                       Eigen::Vector3d external_force_remain);

        typedef unique_ptr<high_mpc_optimizer> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };



}    // namespace Intelligent_planner











#endif
