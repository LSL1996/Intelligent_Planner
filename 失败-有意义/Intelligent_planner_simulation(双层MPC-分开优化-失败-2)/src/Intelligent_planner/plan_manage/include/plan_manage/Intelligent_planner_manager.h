#ifndef _INTELLIGENT_PLANNER_MANAGER_H_
#define _INTELLIGENT_PLANNER_MANAGER_H_

#include <ros/ros.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/kinodynamic_astar.h>
#include <path_searching/mpc_optimizer.h>
#include <path_searching/high_mpc_optimizer.h>

namespace Intelligent_planner{

    class IntelligentPlannerManager
    {
    public:
        IntelligentPlannerManager();
        ~IntelligentPlannerManager();

        /* main planning interface */
        double safety_dist_;
        bool pathsearching(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,Eigen::Vector3d end_pt);
        bool highMpc(Eigen::MatrixXd start_state, bool if_external_force_adaptive, Eigen::Vector3d external_force);
        bool highhighMpc(Eigen::MatrixXd start_state,  Eigen::Vector3d external_force_remain);

        void resetMPCinitial();
        bool safeCheck();

        void initPlanModules(ros::NodeHandle& nh);

        void setodompos(void);
        void getEnvGardandDistPos(double dist, Eigen::Vector3d& grad, Eigen::Vector3d& pos);
        void getOptimizedExtforce(Eigen::Vector3d& optimizedforce);
        //PolynomialTraj traj_ref_;
        //double dt_;
        std::vector<Eigen::Vector3d> global_path_ , local_path_, visited_nodes_;
        std::vector<Eigen::Vector3d> local_result_path_;
        std::vector<Eigen::Vector3d> local_safe_result_path_;
        std::vector<std::vector<Eigen::Vector3d>> mpc_traj_, high_mpc_traj_;
        std::vector<Eigen::Vector3d> mpc_traj_pos_, mpc_traj_vel_, mpc_traj_acc_, mpc_traj_ref_;
        std::vector<double> mpc_best_variable_;
        std::vector<Eigen::Vector3d> high_mpc_traj_pos_, high_mpc_traj_vel_, high_mpc_traj_acc_, high_mpc_traj_ref_;
        Eigen::Vector3d              local_goal_;
        bool need_replan_;

        Eigen::Vector3d external_acc_, external_acc_remain_;
        Eigen::Vector3d last_optimized_acc_;
        Eigen::Vector3d odom_pos_;
        Eigen::Vector3d end_pt_;
        Eigen::Quaterniond odom_orient_;

    private:
        /* main planning algorithms & modules */
        SDFMap::Ptr sdf_map_;
        EDTEnvironment::Ptr edt_environment_;
        unique_ptr<KinodynamicAstar> path_finder_;

        unique_ptr<mpc_optimizer> mpc_optimizer_;

        unique_ptr<high_mpc_optimizer> high_mpc_optimizer_;

    public:
        typedef unique_ptr<IntelligentPlannerManager> Ptr;
    };

}

#endif