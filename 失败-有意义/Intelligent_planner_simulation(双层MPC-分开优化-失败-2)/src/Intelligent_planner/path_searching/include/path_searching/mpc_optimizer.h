#ifndef _MPC_OPTIMIZER_H
#define _MPC_OPTIMIZER_H

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

    class mpc_optimizer
    {
    private:
        /* record data */
        EDTEnvironment::Ptr edt_map_;
        Eigen::Vector3d start_pt_;
        Eigen::MatrixXd start_state_;
        Eigen::Vector3d external_force_,A_external_force_;

        Eigen::VectorXd path_x_, path_y_, path_z_;     // reference path
        //PolynomialTraj  traj_ref_;
        Eigen::Matrix3d As_;
        Eigen::Vector3d Bs_, Cs_;

        double f_;
        Eigen::VectorXd input_x_, input_y_, input_z_; // input (snap)
        Eigen::VectorXd state_x_, state_y_, state_z_;  // state of mpc system and reference theta system
        Eigen::Vector3d map_min_, map_max_;
        Eigen::VectorXd Gradient_x_, Gradient_y_, Gradient_z_;

        /* mpc setting */
        int N_;                 // prediction horizon number

        bool if_external_force_adaptive_;
        bool if_environment_adaptive_;

        std::vector<Eigen::Vector3d> local_result_path_;
        double Ts_;             // time step
        double dist_0_, dist_1_;         // obstacle distance threshold
        double Ke_,Kf_;              // penalty coefficient
        double vel_min_;
        double yaw_limit_;     // max yaw under have external force
        Eigen::MatrixXd A_;     // state equation (S = A_ * U + B_ * S0)
        Eigen::MatrixXd B_;
        Eigen::MatrixXd C_;

        /* optimization parameters */
        double alpha1_;                  // the cost of similarity
        double alpha2_;                  // smooth control input weight
        double alpha3_;                  // avoid collisions
        double alpha4_;                  // the penalty of vel acc
        double alpha5_;                  // minimum input jerk
        double alpha6_;                  // external force adaptive
        double alpha7_;                  // dir guide

        double vel_lower_, vel_upper_, acc_lower_, acc_upper_, jerk_lower_,jerk_upper_;
        double external_force_limit_, vel_limit_,external_force_limit_replan_;
        int max_iteration_num_, iter_num_;
        double max_iteration_time_;
        double min_cost_;
        int dim_, variable_num_;

        int M_, Ma_;
        double K_;

        bool is_init_system_finish_;
        /* useful function */
        void setInitialState( Eigen::MatrixXd start_pt, std::vector<Eigen::Vector3d> local_result_path,Eigen::Vector3d external_force);
        void setInitialSystem();


        Eigen::Vector3d getPosPoly( Eigen::MatrixXd polyCoeff, int k, double t );

        void optimize();

        /* cost function */
        /* calculate each part of cost function with control points q as input */
        static double costFunction(const std::vector<double>& x, std::vector<double>& grad, void* func_data);
        void combineCost(const std::vector<double>& x, std::vector<double>& grad,  double& f_combine);
        void stateEquations();
        void calCostFunctionandGradient();
        double calRangeCost(double value, double lower,double upper);
        double calRangeGrad(double value, double lower,double upper);

        double calRangeCostThree(double value, double lower,double upper);
        double calRangeGradThree(double value, double lower,double upper);


    public:
        mpc_optimizer(){};
        ~mpc_optimizer(){};

        Eigen::Vector3d  last_optimized_acc_;
        Eigen::Vector3d  odom_pos_;
        Eigen::Vector3d  end_pt_;
        Eigen::Quaterniond odom_orient_;
        std::vector<double> best_variable_;
        /* main API */
        void resetInputInital();
        void setEnvironment(const EDTEnvironment::Ptr& env);
        void setParam(ros::NodeHandle& nh);
        void getEnvGardandDistPos(double dist, Eigen::Vector3d& grad, Eigen::Vector3d& pos);

        std::vector<std::vector<Eigen::Vector3d>> mpcOptimizeTraj( Eigen::MatrixXd start_state,
                                                                   std::vector<Eigen::Vector3d> local_result_path,
                                                                   bool if_external_force_adaptive,
                                                                   Eigen::Vector3d external_force);

        typedef unique_ptr<mpc_optimizer> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };



}    // namespace Intelligent_planner

#endif