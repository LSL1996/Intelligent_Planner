#ifndef INTELLIGENT_REPLAN_FSM_H_
#define INTELLIGENT_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/Gains.h"
#include "uav_utils/geometry_utils.h"
#include <plan_env/sdf_map.h>
#include <plan_manage/Intelligent_planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include "plan_env/edt_environment.h"

using std::vector;

namespace Intelligent_planner{
// This class is the Finite Systems Machine of adaptive replanner,
// which charges the several states in the planning process.
    class IntelligentReplanFsm
    {
    private:
        /* ---------- flag ---------- */
        enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, EXEC_TRAJ, REPLAN_TRAJ};

        enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };
        int flight_type_;

        ros::Time t_start_;   // start_point start
        ros::Time t_end_;     // end_point end
        /* planning utils */
        IntelligentPlannerManager::Ptr planner_manager_;
        PlanningVisualization::Ptr visualization_;

        /* planning data */
        bool have_trigger_, have_target_, have_odom_, have_low_traj_, have_traj_, near_goal_,have_external_force_;
        FSM_EXEC_STATE exec_state_;
        double mpc_delta_T_;
        double mpc_opt_;
        double yaw_dot_max_;
        double noise_boundary_;
        double last_yaw_, last_yaw_dot_;
        double time_forward_;
        ros::Time start_time_;

        ros::Time tMpc1_ , tMpc2_, tMpc1_lst_;
        bool if_external_force_adaptive_;
        std::vector<Eigen::Vector3d> high_mpc_traj_pos_, high_mpc_traj_vel_, high_mpc_traj_acc_;

        double T_max_path_search_, T_max_mpc_;
        bool show_path_search_time_, show_mpc_time_;

        Eigen::Vector2d yaw_0_;
        double yaw_angle_;
        int waypoint_num_, waypoint_flag_;

        double waypoints_[50][3];
        double v_min_, v_max_;

        /* visualization setting */
        double line_width_;
        int have_times;
        std::vector<Eigen::Vector3d> history_pos_, history_vel_;

        Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
        Eigen::Quaterniond odom_orient_;
        Eigen::Vector3d start_force_pos_;
        //--------------------------------------//
        Eigen::Vector3d external_force_, last_external_force_;                    // external force
        Eigen::Vector3d external_force_remain_, external_force_remain_optimized;
        std::vector<double> use_temp_arr_;
        double external_force_research_;
        bool if_external_force_limit_, if_external_force_limit_replan_;
        //-------------------------------------//

        Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_jerk_, start_yaw_;  // start state
        Eigen::Vector3d last_optimized_acc_;
        Eigen::Vector3d end_pt_;                              // target state
        Eigen::Vector3d imu_acc_;

        //-------------------------------------------//
        Eigen::Vector3d dist_grad_, comp_acc_, vel_temp_, pos_temp_;
        double z1_,z2_,z3_,z4_,z5_;
        double acos1_, acos2_;
        //---------------------------------------------//
        EDTEnvironment::Ptr edt_map_;
        /* ROS utils */
        ros::NodeHandle node_;
        ros::Timer cmd_timer_, path_search_timer_,  mpc_timer_, safety_check_timer_, external_force_timer_;
        ros::Timer exec_timer;
        ros::Subscriber trigger_sub_, waypoint_sub_, odom_sub_, extforce_sub_, imu_sub_;
        ros::Publisher pos_cmd_pub_, pos_cmd_vis_pub_, exected_traj_pub_;
        ros::Publisher env_grad_pub_, end_pos_pub_, external_force_pub_, external_force_comp_pub_;
        double wight_use_extforce_;

        /* ROS Timer Function */
        void cmdCallback(const ros::TimerEvent& e);
        void trajSafetyCallback(const ros::TimerEvent& e);
        void execFSMCallback(const ros::TimerEvent& e);
        void externalforceGenCallback(const ros::TimerEvent& e);
        void calculateExtForceAssignWeight();
        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        bool kinoNMPCtraj();
        /* ROS functions */
        void waypointCallback(const nav_msgs::PathConstPtr& msg);
        void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
        //void extforceCallback(const serial_node::extforce& exf);
        void extforceCallback(const geometry_msgs::Vector3::ConstPtr& exf);
        void imu_callback(const sensor_msgs::Imu& imu);

        void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
                     const Eigen::Vector4d& color);
        void draw_history_traj();
        std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last);
        /* helper functions */
        bool callPathSearch();
        bool callMpc(Eigen::MatrixXd start_state);

    public:
        IntelligentReplanFsm(){};
        ~IntelligentReplanFsm(){};

        void init(ros::NodeHandle& nh);

        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}	// namespace Intelligent_planner

#endif