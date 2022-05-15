#include <plan_manage/Intelligent_replan_fsm.h>
#include <random>
#include <math.h>

namespace Intelligent_planner{
void IntelligentReplanFsm::init(ros::NodeHandle& nh){
    exec_state_    = FSM_EXEC_STATE::INIT;
    have_target_   = false;
    have_low_traj_ = false;
    have_odom_     = false;
    have_traj_     = false;
    near_goal_     = false;
    have_trigger_  = false;
    have_external_force_ = false;

    external_force_ << 0,0,0;
    external_force_remain_ << 0,0,0;
    last_external_force_ << 0, 0, 0;
    imu_acc_ << 0,0,0;

    nh.param("visual/line_width",               line_width_, -1.0);
    nh.param("multi_mpc/Ts",                    mpc_delta_T_, -1.0);
    nh.param("multi_mpc/mpc_opt",               mpc_opt_, -1.0);  // 10ms
    nh.param("optimization/vel_upper",          v_max_,-1.0);
    nh.param("multi_mpc/show_path_search_time", show_path_search_time_, true);
    nh.param("multi_mpc/show_mpc_time",         show_mpc_time_, false);
    nh.param("fsm/flight_type",                 flight_type_, 0);
    nh.param("fsm/time_forward",                time_forward_, -1.0);
    nh.param("optimization/external_force_research",  external_force_research_, -1.0);
    nh.param("optimization/noise_boundary",     noise_boundary_, -1.0);
    nh.param("search/yaw_dot_max",              yaw_dot_max_, 5.0);

    v_min_ = 0.0;
    have_times = 0;
    // get the goal waypoints
    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
    if (waypoint_num_>=1)
        waypoint_flag_ = 0;     // if there are some waypoints, then initial the first goal

    yaw_0_ << 1.0,0.0;
    /* initialize main modules */
    planner_manager_.reset(new IntelligentPlannerManager);
    planner_manager_->initPlanModules(nh);
    visualization_.reset(new PlanningVisualization(nh));

    /* callback */
    cmd_timer_            = nh.createTimer(ros::Duration(0.01), &IntelligentReplanFsm::cmdCallback, this);  /// cmd 10ms
    external_force_timer_ = nh.createTimer(ros::Duration(0.04), &IntelligentReplanFsm::externalforceGenCallback, this); ///  50ms
    exec_timer            = nh.createTimer(ros::Duration(0.01), &IntelligentReplanFsm::execFSMCallback, this);
    safety_check_timer_   = nh.createTimer(ros::Duration(0.05), &IntelligentReplanFsm::trajSafetyCallback, this);  ///  50ms
    ///
    odom_sub_           = nh.subscribe("/odom_world", 1, &IntelligentReplanFsm::odometryCallback, this);
    imu_sub_            = nh.subscribe("/quadrotor_simulator_so3/imu", 10, &IntelligentReplanFsm::imu_callback, this, ros::TransportHints().tcpNoDelay());

    pos_cmd_pub_             = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    pos_cmd_vis_pub_         = nh.advertise<visualization_msgs::Marker> ("/position_cmd_vis", 50);
    exected_traj_pub_        = nh.advertise<visualization_msgs::Marker> ("/exected_traj", 50);
    end_pos_pub_             = nh.advertise<geometry_msgs::Vector3>("/end_pos", 1);
    external_force_pub_      = nh.advertise<geometry_msgs::Vector3>("/extforce", 50);
    external_force_comp_pub_ = nh.advertise<geometry_msgs::Vector3>("/extforce_comp", 50);

    if_external_force_adaptive_ = false;
    T_max_mpc_ = 0.0;

    if (flight_type_ == TARGET_TYPE::MANUAL_TARGET){
        waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &IntelligentReplanFsm::waypointCallback, this);
    } else if (flight_type_ == TARGET_TYPE::PRESET_TARGET){
        trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &IntelligentReplanFsm::triggerCallback, this);
        ros::Duration(1.0).sleep();
        ROS_WARN("Waiting for trigger from [n3ctrl] from RC");
        while (ros::ok() && (!have_odom_ || !have_trigger_))
        {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }
    } else
      cout << "Wrong target_type_ value! target_type_=" << flight_type_ << endl;
}

void IntelligentReplanFsm::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
    string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "EXEC_TRAJ","REPLAN_TRAJ" };
    int pre_s           = int(exec_state_);
    exec_state_         = new_state;
   // cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void IntelligentReplanFsm::execFSMCallback(const ros::TimerEvent& e){
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100) {
        if (!have_odom_)
            cout << "no odom." << endl;
        if (!have_target_)
            cout << "wait for goal." << endl;
        fsm_num = 0;
    }

    switch(exec_state_)
    {
        case INIT:{
            if(!have_odom_)
                return;
            if(have_target_)
                changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }
        case WAIT_TARGET:{
            if(!have_target_)
                return;
            else
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            break;
        }
        case GEN_NEW_TRAJ:{
            bool NMPC_success  = kinoNMPCtraj();
            if(NMPC_success)
                changeFSMExecState(EXEC_TRAJ, "FSM");
            else
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            break;
        }
        case EXEC_TRAJ:{
            if(near_goal_)
                return;
            else
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            break;
        }
        case REPLAN_TRAJ:{
            bool success = kinoNMPCtraj();
            if (success) {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            } else {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }
    }
}

bool IntelligentReplanFsm::kinoNMPCtraj()
{
    start_pt_  = odom_pos_;
    start_vel_ = odom_vel_;
    Eigen::Vector3d start_acc;

    if(high_mpc_traj_acc_.size() != 0){
        double t_cur = (ros::Time::now() -tMpc1_).toSec();
        int k = floor(t_cur/mpc_delta_T_);
        Eigen::Vector3d acc_l = high_mpc_traj_acc_[k];
        Eigen::Vector3d acc_r = high_mpc_traj_acc_[k+1];
        t_cur = t_cur - k * mpc_delta_T_;
        start_acc(0) = acc_l(0) + (acc_r(0) - acc_l(0)) * t_cur / mpc_delta_T_;
        start_acc(1) = acc_l(1) + (acc_r(1) - acc_l(1)) * t_cur / mpc_delta_T_;
        start_acc(2) = acc_l(2) + (acc_r(2) - acc_l(2)) * t_cur / mpc_delta_T_;
        //start_acc << 0,0,0;
    }

    if((abs(external_force_remain_(0)) || abs(external_force_remain_(1)) ||  abs(external_force_remain_(2)))> external_force_research_){
        if_external_force_adaptive_ = true;
        planner_manager_->need_replan_ = true;
        last_optimized_acc_ = start_acc;
        planner_manager_->last_optimized_acc_ = start_acc;
        start_acc << 0,0,0;
    }else {
        planner_manager_->need_replan_ = false;
        if_external_force_adaptive_ = false;
    }

    planner_manager_->external_acc_remain_ = external_force_remain_;
    last_external_force_ = external_force_remain_;

    ros::Time T_start_search = ros::Time::now();
    bool search_success = planner_manager_->pathsearching(start_pt_, start_vel_, start_acc, end_pt_);
    double search_duration = (ros::Time::now() - T_start_search).toSec();
    if (show_path_search_time_)
        cout << "path search time:           " <<  search_duration << endl;

    if(search_success){
        // optimize start
        double dist = (odom_pos_ - end_pt_).norm();
        if (dist < 0.5){
            cout << "near the goal " << endl;
            planner_manager_->resetMPCinitial();
            draw_history_traj();
            have_traj_     = false;
            have_target_   = false;
            near_goal_     = true;
            return true;
        }

        ros::Time T_optimize_start = ros::Time::now();
        Eigen::Matrix3d start_state;
        double t;
        if (have_traj_){
            tMpc2_ = ros::Time::now();
            t = (tMpc2_ - tMpc1_).toSec();
            int k = floor(t/mpc_delta_T_);
            t = t - k * mpc_delta_T_;
            Eigen::Vector3d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R, jerk_L, jerk_R;
            /*
            acc_L = planner_manager_->high_mpc_traj_acc_[k];
            acc_R = planner_manager_->high_mpc_traj_acc_[k+1];
            vel_L = planner_manager_->high_mpc_traj_vel_[k];
            vel_R = planner_manager_->high_mpc_traj_vel_[k+1];
            pos_L = planner_manager_->high_mpc_traj_pos_[k];
            pos_R = planner_manager_->high_mpc_traj_pos_[k+1];
            */
            acc_L = high_mpc_traj_acc_[k];
            acc_R = high_mpc_traj_acc_[k+1];
            vel_L = high_mpc_traj_vel_[k];
            vel_R = high_mpc_traj_vel_[k+1];
            pos_L = high_mpc_traj_pos_[k];
            pos_R = high_mpc_traj_pos_[k+1];

            start_state(2,0) = acc_L(0) + (acc_R(0) - acc_L(0)) * t / mpc_delta_T_;
            start_state(2,1) = acc_L(1) + (acc_R(1) - acc_L(1)) * t / mpc_delta_T_;
            start_state(2,2) = acc_L(2) + (acc_R(2) - acc_L(2)) * t / mpc_delta_T_;
            start_state(1,0) = vel_L(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
            start_state(1,1) = vel_L(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
            start_state(1,2) = vel_L(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
            start_state(0,0) = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
            start_state(0,1) = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
            start_state(0,2) = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;

            // record the last mpc
            tMpc1_lst_ = tMpc1_;
            tMpc1_ = tMpc2_;

            //high_mpc_traj_pos_ = planner_manager_->high_mpc_traj_pos_;
            //high_mpc_traj_vel_ = planner_manager_->high_mpc_traj_vel_;
            //high_mpc_traj_acc_ = planner_manager_->high_mpc_traj_acc_;

            bool mpc_plan_success = planner_manager_->highMpc(start_state, if_external_force_adaptive_, external_force_remain_);
            if (mpc_plan_success){
                if(have_times == 0){
                    have_times = 1;
                }
                draw_history_traj();
                visualization_->drawMpcTraj(planner_manager_->mpc_traj_pos_, 0.2,Eigen::Vector4d(1, 0, 0, 1.0));
                visualization_->drawMpcRefTraj(planner_manager_->mpc_traj_ref_, 0.1,Eigen::Vector4d(0, 0, 1, 1.0));
                if(if_external_force_adaptive_){
                    bool high_mpc_plan_success = planner_manager_->highhighMpc(start_state, external_force_remain_);
                    if(high_mpc_plan_success){
                        high_mpc_traj_pos_ = planner_manager_->high_mpc_traj_pos_;
                        high_mpc_traj_vel_ = planner_manager_->high_mpc_traj_vel_;
                        high_mpc_traj_acc_ = planner_manager_->high_mpc_traj_acc_;
                        have_traj_   = true;
                        visualization_->drawHighMpcTraj(planner_manager_->high_mpc_traj_pos_, 0.1,Eigen::Vector4d(1, 1, 0, 1.0));
                        //------------------------------//
                        planner_manager_->getOptimizedExtforce(external_force_remain_optimized);
                        cout << "external_force_remain_   \n" << external_force_remain_ << endl;
                        cout << "external_force_remain_optimized   \n" << external_force_remain_optimized << endl;
                        geometry_msgs::Vector3 extforce_comp;
                        extforce_comp.x = external_force_(0) - external_force_remain_optimized(0);
                        extforce_comp.y = external_force_(1) - external_force_remain_optimized(1);
                        extforce_comp.z = external_force_(2) - external_force_remain_optimized(2);
                        external_force_comp_pub_.publish(extforce_comp);
                        //------------------------------//
                        return true;
                    }else{
                        return false;
                    }
                }else{
                    high_mpc_traj_pos_     = planner_manager_-> mpc_traj_pos_;
                    high_mpc_traj_vel_     = planner_manager_-> mpc_traj_vel_;
                    high_mpc_traj_acc_     = planner_manager_-> mpc_traj_acc_;
                    have_traj_   = true;
                }
                return true;
            } else {
                return false;
            }
        } else{
            start_state(0,0) = odom_pos_(0);
            start_state(0,1) = odom_pos_(1);
            start_state(0,2) = odom_pos_(2);

            start_state(1,0) = odom_vel_(0);
            start_state(1,1) = odom_vel_(1);
            start_state(1,2) = odom_vel_(2);

            start_state(2,0) = 0.0;
            start_state(2,1) = 0.0;
            start_state(2,2) = 0.0;

            //cout << " low mpc start" << endl;
            tMpc1_ = ros::Time::now();
            bool mpc_plan_success = planner_manager_->highMpc(start_state, if_external_force_adaptive_, external_force_remain_);
            if (mpc_plan_success){
                if(have_times == 0){
                    have_times =1;
                }
                draw_history_traj();
                visualization_->drawMpcTraj(planner_manager_->mpc_traj_pos_, 0.2,Eigen::Vector4d(1, 0, 0, 1.0));
                visualization_->drawMpcRefTraj(planner_manager_->mpc_traj_ref_, 0.1,Eigen::Vector4d(0, 0, 1, 1.0));

                if(if_external_force_adaptive_){
                    planner_manager_->external_acc_ = external_force_;
                    bool high_mpc_plan_success = planner_manager_->highhighMpc(start_state, external_force_remain_);
                    if(high_mpc_plan_success){
                        high_mpc_traj_pos_ = planner_manager_->high_mpc_traj_pos_;
                        high_mpc_traj_vel_ = planner_manager_->high_mpc_traj_vel_;
                        high_mpc_traj_acc_ = planner_manager_->high_mpc_traj_acc_;
                        have_traj_   = true;
                        visualization_->drawHighMpcTraj(planner_manager_->high_mpc_traj_pos_, 0.1,Eigen::Vector4d(1, 1, 0, 1.0));

                        //------------------------------//
                        planner_manager_->getOptimizedExtforce(external_force_remain_optimized);
                        cout << "external_force_remain_   \n" << external_force_remain_ << endl;
                        cout << "external_force_remain_optimized   \n" << external_force_remain_optimized << endl;
                        geometry_msgs::Vector3 extforce_comp;
                        extforce_comp.x = external_force_(0) - external_force_remain_optimized(0);
                        extforce_comp.y = external_force_(1) - external_force_remain_optimized(1);
                        extforce_comp.z = external_force_(2) - external_force_remain_optimized(2);
                        external_force_comp_pub_.publish(extforce_comp);
                        //------------------------------//
                        return true;
                    }else{
                        return false;
                    }
                }else{
                    //cout << " high mpc = low mpc " << endl;
                    high_mpc_traj_pos_     = planner_manager_->mpc_traj_pos_;
                    high_mpc_traj_vel_     = planner_manager_->mpc_traj_vel_;
                    high_mpc_traj_acc_     = planner_manager_->mpc_traj_acc_;
                    have_traj_   = true;
                }
                return true;
            } else {
                return false;
            }
        }
        if (show_mpc_time_){
            cout << "Time of mpc:      " << (ros::Time::now() -T_optimize_start).toSec() << endl;
        }
    }else{
        return false;
    }
}

void IntelligentReplanFsm::imu_callback(const sensor_msgs::Imu& imu){

    const Eigen::Vector3d imu_acc(imu.linear_acceleration.x,
                          imu.linear_acceleration.y,
                          imu.linear_acceleration.z);
    imu_acc_ = imu_acc;
}

void IntelligentReplanFsm::calculateExtForceAssignWeight(){
    Eigen::Vector3d comp_acc, vel_temp, pos_temp, exf_temp;
    int opt_index;
    wight_use_extforce_ = 0;
    std::vector <Eigen::Vector3d> pos_t, vel_t, acc_t;
    Eigen::Vector3d dist_grad;
    double safety_dist = 1.0;
    double min_dist = std::numeric_limits<double>::max();
    double dist;
    double cos1, z1, z2, z3, z4, z5, cos2, cos3, cos4;
    double acos1, acos2, acos3, acos4, acos5;
    double v_acos1, v_acos2, v_acos3, v_acos4;
    z1 = z2 = z3 = z4 = z5 = 0;
    double w1, w2, w3, w4, w5;
    double use_temp;
    Eigen::Vector3d yaw_guide;
    w1 = 0.40;
    w2 = 0.20;
    w3 = 0.15;
    w4 = 0.20;
    w5 = 0.30;

    double w[5] = {0.1,0.2,0.3,0.4,0.2};
    //-------------------------init----------------//
    std::vector<Eigen::Vector3d>::iterator it;
    if(!pos_t.empty())
        for (it = pos_t.begin(); it != pos_t.end(); )
            it = pos_t.erase(it);
    if(!vel_t.empty())
        for (it = vel_t.begin(); it != vel_t.end(); )
            it = vel_t.erase(it);
    if(!acc_t.empty())
        for (it = acc_t.begin(); it != acc_t.end(); )
            it = acc_t.erase(it);

    if(!use_temp_arr_.empty())
        use_temp_arr_.clear();
    //-----------------------cal-----------------=----//
    Eigen::Vector3d start_acc;
    double t_cur = (ros::Time::now() -tMpc1_).toSec();
    int k = floor(t_cur/mpc_delta_T_);
    Eigen::Vector3d acc_l = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_r = Eigen::Vector3d::Zero();
    if(high_mpc_traj_acc_.size()!=0){
        acc_l = high_mpc_traj_acc_[k];
        acc_r = high_mpc_traj_acc_[k+1];
        t_cur = t_cur - k * mpc_delta_T_;
        start_acc(0) = acc_l(0) + (acc_r(0) - acc_l(0)) * t_cur / mpc_delta_T_;
        start_acc(1) = acc_l(1) + (acc_r(1) - acc_l(1)) * t_cur / mpc_delta_T_;
        start_acc(2) = acc_l(2) + (acc_r(2) - acc_l(2)) * t_cur / mpc_delta_T_;
    }
    //------------------------------------------------//
    if((abs(external_force_(0)) || abs(external_force_(1)) || abs(external_force_(2)))> noise_boundary_){
        start_acc(2) = 0.0;
        comp_acc = start_acc + external_force_;
        exf_temp = external_force_;

        // consider delay
        vel_temp = odom_vel_;
        pos_temp = odom_pos_;
        pos_temp = pos_temp + vel_temp * mpc_delta_T_ + 1/2 * comp_acc * mpc_delta_T_ * mpc_delta_T_;
        vel_temp = vel_temp + comp_acc * mpc_delta_T_;
        pos_temp = pos_temp + vel_temp * mpc_delta_T_ + 1/2 * comp_acc * 0.02 * 0.02;
        vel_temp = vel_temp + comp_acc * 0.02;
        // predict
        for(int i = 0; i < 5; i++){    // predicet 3 step
            pos_temp = pos_temp + vel_temp * 0.1 + 1/2 * comp_acc * 0.1 * 0.1;
            // cout << "pos_temp   " << pos_temp << endl;
            pos_t.push_back(pos_temp);
            vel_temp = vel_temp + comp_acc * 0.1;
            vel_t.push_back(vel_temp);

            for(int k = 0; k < 3; k++){
                if((1 - abs(pos_temp(k) - odom_pos_(k)) * 0.3) > 0)
                    exf_temp(k) = (1 - abs(pos_temp(1) - odom_pos_(1)) * 0.3) * external_force_(k);
                else
                    exf_temp(k) = 0;
            }
            acc_t.push_back(exf_temp);
        }

        for(int j = 0; j < 5; j++) {
            vel_temp = vel_t[j];
            planner_manager_->getEnvGardandDistPos(dist, dist_grad, pos_t[j]);
            comp_acc = acc_t[j] + start_acc;
            pos_temp = pos_t[j];
            //cout << " pos_t[2] - odom_pos_     "<<pos_t[2] - odom_pos_ << endl;
            // ---------------------------------use-----------------------------//
            comp_acc(2) = 0;
            dist_grad(2) = 0;
            vel_temp(2) = 0;
            pos_temp(2) = 0;

            vel_temp_ = vel_temp;
            pos_temp_ = pos_t[2];
            dist_grad_ = dist_grad;
            comp_acc_ = comp_acc;
            //cout << "dist_grad \n " << dist_grad << endl;

            if (dist_grad.norm() > 0.01 && vel_temp.norm() > 0.01) {
                cos1 = vel_temp.dot(dist_grad) / dist_grad.norm() / vel_temp.norm(); // env info
                acos1 = acos(cos1);
                v_acos1 = vel_temp(0) * dist_grad(1) - vel_temp(1) * dist_grad(0);
                if (v_acos1 < 0)
                    acos1 = -acos1;

                cos2 = comp_acc.dot(dist_grad) / dist_grad.norm() / comp_acc.norm();
                acos2 = acos(cos2);
                v_acos2 = comp_acc(0) * dist_grad(1) - comp_acc(1) * dist_grad(0);
                if (v_acos2 < 0)
                    acos2 = -acos2;

                acos1_ = acos1;
                acos2_ = acos2;
                //cout << " max_acc  "<< max_acc  << endl;
                // ---------------------------------use-----------------------------//
                if (acos1 > -3.1415926 / 2 && acos1 < 3.1415926 / 2) {    //[-90,90] safe
                    z1 = 2 / (1 + exp(-3 * cos1));    // > 1
                    if (acos1 < 0) {
                        if (acos2 < 0 && acos2 > -3.1415926 / 2) {  // safe a little   //[-90 0]
                            z2 = 1 / (2 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (4 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));
                            }
                        } else if (acos2 > 0 && acos2 < 3.1415926 / 2) {
                            if (acos2 < 3.1415926 / 2 - abs(acos1))  // further to v
                                z2 = 1 / (4 + exp(3 * cos2));
                            else
                                z2 = -1 / (4 + exp(3 * cos2));

                            if (dist < safety_dist)
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            else
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));

                        } else {  // danger
                            z2 = -1 / (1 + exp(3 * cos2));

                            if (dist < safety_dist) {
                                z3 = -1 / (4 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (2 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 - abs(acos2));
                            double t2 = vel_temp.norm() * cos(acos1) - 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;
                        }
                    }
                    if (acos1 > 0) {
                        if (acos2 > 0 && acos2 < 3.1415926 / 2) {  // safe a little
                            z2 = 1 / (2 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (4 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));
                            }
                        } else if (acos2 < 0 && acos2 > -3.1415926 / 2) {
                            if (abs(acos2) < 3.1415926 / 2 - acos1) // further to v
                                z2 = 1 / (4 + exp(3 * cos2));
                            else
                                z2 = -1 / (4 + exp(3 * cos2));

                            if (dist < safety_dist)
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            else
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));

                        } else {  // danger
                            z2 = -1 / (1 + exp(3 * cos2));

                            if (dist < safety_dist) {
                                z3 = -1 / (4 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (2 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 - abs(acos2));
                            double t2 = vel_temp.norm() * cos(acos1) - 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;
                        }
                    }
                } else if ((acos1 > -3.1415926 && acos1 < -3.1415926 / 2) ||
                           (acos1 < 3.1415926 && acos1 > 3.1415926 / 2)) {  // danger  [-180 -90] [90 180]
                    z1 = 2 / (1 + exp(3 * cos1));  // < 1
                    if (acos1 < 0) {
                        if (acos2 < 0 && abs(acos2) < abs(acos1)) {  // safe a little
                            z2 = 1 / (2 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));  // 0.3 - 0.5
                            } else {
                                z3 = 1 / (2 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 + acos2);
                            double t2 = vel_temp.norm() * cos(3.1415926 + acos1) + 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;

                        } else if (acos2 > 0 && acos2 < 3.1415926 / 2) {
                            acos5 = 3.1415926 + acos1;
                            if (acos2 < acos5)    //  further to v
                                z2 = 1 / (4 + exp(3 * cos2));
                            else
                                z2 = -1 / (4 + exp(3 * cos2));

                            if (dist < safety_dist)
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            else
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));

                            double t1 = comp_acc.norm() * cos(acos2);
                            double t2 = vel_temp.norm() * cos(3.1415926 + acos1) - 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;

                        } else {  // danger
                            z2 = -1 / (1 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 - abs(acos2));
                            double t2 = vel_temp.norm() * cos(3.1415926 - acos1) + 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;
                        }
                    }
                    if (acos1 > 0) {
                        if (acos2 > 0 && acos2 < acos1) {  // safe a little
                            z2 = 1 / (2 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 - acos2);
                            double t2 = vel_temp.norm() * cos(3.1415926 - acos1) + 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;

                        } else if (acos2 < 0 && acos2 > -3.1415926 / 2) {
                            acos5 = -3.1415926 + acos1;
                            if (abs(acos2) < abs(acos5))   // further to v
                                z2 = 1 / (4 + exp(3 * cos2));
                            else
                                z2 = -1 / (4 + exp(3 * cos2));

                            if (dist < safety_dist)
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            else
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));

                            double t1 = comp_acc.norm() * cos(acos2);
                            double t2 = vel_temp.norm() * cos(3.1415926 - acos1) - 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;

                        } else {  // danger
                            z2 = -1 / (2 + exp(3 * cos2));
                            if (dist < safety_dist) {
                                z3 = -1 / (2 + exp(4 * (dist - safety_dist)));
                            } else {
                                z3 = 1 / (4 + exp(4 * (safety_dist - dist)));
                            }

                            double t1 = comp_acc.norm() * cos(3.1415926 - abs(acos2));
                            double t2 = vel_temp.norm() * cos(3.1415926 - acos1) + 0.5 * t1 * 0.1 * 0.1;
                            if (t2 > dist)
                                z4 = -1 / (2 + exp(3 * (dist - t2)));
                            else if (t2 > 0.5 * dist)
                                z4 = -1 / (4 + exp(3 * (dist - t2)));
                            else
                                z4 = 0.0;
                        }
                    }

                }
                //----------------------------------------------------//
                yaw_guide(0) = end_pt_(0) - pos_t[2](0);
                yaw_guide(1) = end_pt_(1) - pos_t[2](1);
                yaw_guide(2) = 0;
                cos3 = yaw_guide.dot(dist_grad) / dist_grad.norm() / yaw_guide.norm();
                cos4 = vel_temp.dot(dist_grad) / dist_grad.norm() / vel_temp.norm();
                acos3 = acos(cos3);
                v_acos3 = yaw_guide(0) * dist_grad(1) - yaw_guide(1) * dist_grad(0);
                acos4 = acos(cos4);
                v_acos4 = vel_temp(0) * dist_grad(1) - vel_temp(1) * dist_grad(0);

                if (v_acos3 < 0)
                    acos3 = -acos3;
                if (v_acos4 < 0)
                    acos4 = -acos4;

                if (acos3 > 0 && acos3 < 3.1415926 && acos4 > -3.1415926 / 2 && acos4 < 3.1415926 / 2)
                    z5 = -1 / (1 + exp(3 * cos3)) * 1 / (2 + exp(1 * cos4));
                else if (acos3 > 0 && acos3 < 3.1415926 && (acos4 > -3.1415926 && acos4 < -3.1415926 / 2) ||
                         (acos4 > 3.1415926 / 2 && acos4 < 3.1415926))
                    z5 = 1 / (1 + exp(3 * cos3)) * 1 / (1 + exp(3 * cos4));
                else if (acos3 < 0 && acos3 > -3.1415926 && acos4 > -3.1415926 / 2 && acos4 < 3.1415926 / 2)
                    z5 = -1 / (1 + exp(3 * cos3)) * 1 / (1 + exp(3 * cos4));
                else
                    z5 = 1 / (1 + exp(3 * cos3)) * 1 / (2 + exp(1 * cos4));
                //---------------------------------------------------------//

                z1_ = z1;
                z2_ = z2;
                z3_ = z3;
                z4_ = z4;
                z5_ = z5;
                use_temp = w1 * z1 + w2 * z2 + w3 * z3 + w4 * z4 + w5 * z5;

                if (use_temp < 0) {
                    use_temp = 0.0;
                }
                use_temp_arr_.push_back(use_temp);
            }
        }

        for(int i = 0;i < 5; i++){
            wight_use_extforce_ += w[i]*use_temp_arr_[i];
        }

        if (wight_use_extforce_ > 1.0)
            wight_use_extforce_ = 1.0;
        else if (wight_use_extforce_ > 0)
            wight_use_extforce_ = wight_use_extforce_;
        else
            wight_use_extforce_ = 0.0;
    }
}

void IntelligentReplanFsm::externalforceGenCallback(const ros::TimerEvent& e){

    static int times = 0;
    if(have_odom_ && odom_pos_(0) > -1.5 && odom_pos_(0) < 3.5){
        if(times == 0){
            start_force_pos_ = odom_pos_;
            external_force_ << 1.0,-1.5,0.0;
            times++;
        }
        if(times > 0){
            double y_dist = abs(odom_pos_(1) - start_force_pos_(1));
            double x_dist = abs(odom_pos_(0) - start_force_pos_(0));
            external_force_(0) = 1.0 - x_dist * 0.3;
            external_force_(1) = -1.5 + y_dist * 0.3;
        }
    }else {
        external_force_ << 0, 0, 0;
    }
    wight_use_extforce_ = 0.0;
    // external_force_pub
    geometry_msgs::Vector3 extforce;
    extforce.x = external_force_(0) * 0.98;
    extforce.y = external_force_(1) * 0.98;
    extforce.z = external_force_(2) * 0.98;
    external_force_pub_.publish(extforce);

    if(have_target_ && have_odom_)
         calculateExtForceAssignWeight();

    external_force_remain_ = wight_use_extforce_ * external_force_;
}

void IntelligentReplanFsm::waypointCallback(const nav_msgs::PathConstPtr& msg){
    if (msg->poses[0].pose.position.z < -0.1) return;
    cout << "Get waypoint!" << endl;
    start_pt_ = odom_pos_;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    planner_manager_->end_pt_ = end_pt_;
   // cout << "end_pt_ "<< end_pt_ <<endl;
    //-----------------------------------------------//
    geometry_msgs::Vector3 end_pt;
    end_pt.x = end_pt_(0);
    end_pt.y = end_pt_(1);
    end_pt.z = end_pt_(2);
    end_pos_pub_.publish(end_pt);
    //----------------------------------------------//

    visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
    planner_manager_->resetMPCinitial();
    have_target_ = true;
    //t_start_ = ros::Time::now();
    near_goal_   = false;
}

void IntelligentReplanFsm::triggerCallback(const geometry_msgs::PoseStampedPtr &msg){
    have_trigger_ = true;
    cout << "Triggered!" << endl;

    if (waypoint_flag_ == 0){
        start_pt_ = odom_pos_;
        end_pt_ << waypoints_[0][0], waypoints_[0][1], waypoints_[0][2];
        visualization_->drawLocalGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
        planner_manager_->resetMPCinitial();
    }
    have_target_ = true;
    near_goal_   = false;
}

void IntelligentReplanFsm::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;
    // cout << "cur pos   "<< odom_pos_ <<endl;
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // record history pos and vel
    if (have_target_){
        history_pos_.push_back(odom_pos_);
        history_vel_.push_back(odom_vel_);
    }
    have_odom_ = true;
}

void IntelligentReplanFsm::cmdCallback(const ros::TimerEvent& e){

    //cout << "have_traj_   "<< have_traj_ << endl;
    if (have_traj_)
    {
        geometry_msgs::Point tmpPos;
        geometry_msgs::Vector3 tmpVel, tmpAcc, tmpJerk;

        quadrotor_msgs::PositionCommand cmdMsg;
        cmdMsg.header.frame_id = "world";
        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();
        static ros::Time time_last = ros::Time::now();

        double t;
        int k;
        Eigen::Vector3d pos_L, pos_R, vel_L, vel_R, acc_L, acc_R, jerk_L, jerk_R;
        //cout << "high_mpc_traj_pos_.size()  " << high_mpc_traj_pos_.size()  << endl;
        if (high_mpc_traj_pos_.size() == 0 ) {
            return ;
        }else{
            t = (ros::Time::now() - tMpc1_).toSec() ;
            k = floor(t/mpc_delta_T_);
            acc_L = high_mpc_traj_acc_[k];
            acc_R = high_mpc_traj_acc_[k+1];
            vel_L = high_mpc_traj_vel_[k];
            vel_R = high_mpc_traj_vel_[k+1];
            pos_L = high_mpc_traj_pos_[k];
            pos_R = high_mpc_traj_pos_[k+1];
        }

        /*
        if (high_mpc_traj_pos_.size() == 0){
            t = (ros::Time::now() - tMpc1_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[0]  k:  "<< k << "  t:  " << t << endl;
            acc_L = planner_manager_->high_mpc_traj_acc_[k];
            acc_R = planner_manager_->high_mpc_traj_acc_[k+1];
            vel_L = planner_manager_->high_mpc_traj_vel_[k];
            vel_R = planner_manager_->high_mpc_traj_vel_[k+1];
            pos_L = planner_manager_->high_mpc_traj_pos_[k];
            pos_R = planner_manager_->high_mpc_traj_pos_[k+1];
        }else if ((ros::Time::now()-tMpc2_).toSec() - mpc_opt_ < 0 ){
            t = (ros::Time::now() - tMpc1_lst_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[1]  k: " << k << "  t:  " << t << endl;
            acc_L = high_mpc_traj_acc_[k];
            acc_R = high_mpc_traj_acc_[k+1];
            vel_L = high_mpc_traj_vel_[k];
            vel_R = high_mpc_traj_vel_[k+1];
            pos_L = high_mpc_traj_pos_[k];
            pos_R = high_mpc_traj_pos_[k+1];
        }else {
            t = (ros::Time::now() - tMpc1_).toSec();
            k = floor(t/mpc_delta_T_);
            // cout <<"[2]  k: " << k << "  t:  " << t << endl;
            acc_L = planner_manager_->high_mpc_traj_acc_[k];
            acc_R = planner_manager_->high_mpc_traj_acc_[k+1];
            vel_L = planner_manager_->high_mpc_traj_vel_[k];
            vel_R = planner_manager_->high_mpc_traj_vel_[k+1];
            pos_L = planner_manager_->high_mpc_traj_pos_[k];
            pos_R = planner_manager_->high_mpc_traj_pos_[k+1];
        }
         */

        t = t - k * mpc_delta_T_;

        tmpAcc.x = acc_L(0) + (acc_R(0) - acc_L(0)) * t / mpc_delta_T_;
        tmpAcc.y = acc_L(1) + (acc_R(1) - acc_L(1)) * t / mpc_delta_T_;
        tmpAcc.z = acc_L(2) + (acc_R(2) - acc_L(2)) * t / mpc_delta_T_;
        cmdMsg.acceleration = tmpAcc;
        tmpVel.x = vel_L(0) + acc_L(0) * t + (acc_R(0) - acc_L(0)) * t * t / mpc_delta_T_ / 2;
        tmpVel.y = vel_L(1) + acc_L(1) * t + (acc_R(1) - acc_L(1)) * t * t / mpc_delta_T_ / 2;
        tmpVel.z = vel_L(2) + acc_L(2) * t + (acc_R(2) - acc_L(2)) * t * t / mpc_delta_T_ / 2;
        cmdMsg.velocity = tmpVel;
        tmpPos.x = pos_L(0) + vel_L(0) * t + acc_L(0) * t * t / 2 + (acc_R(0) - acc_L(0)) * t * t * t / mpc_delta_T_ / 6;
        tmpPos.y = pos_L(1) + vel_L(1) * t + acc_L(1) * t * t / 2 + (acc_R(1) - acc_L(1)) * t * t * t / mpc_delta_T_ / 6;
        tmpPos.z = pos_L(2) + vel_L(2) * t + acc_L(2) * t * t / 2 + (acc_R(2) - acc_L(2)) * t * t * t / mpc_delta_T_ / 6;
        cmdMsg.position = tmpPos;
        //cout << "tmpPos \n " << tmpPos << endl;
        /*
        if(odom_pos_(0) > -8.0 && odom_pos_(0) < -3.0){
            cout << "tmpPos "<< tmpPos << endl;
            cout << "tmpVel "<< tmpVel << endl;
            cout << "tmpAcc "<< tmpAcc << endl;
        }


        if(odom_pos_(0) > 3.5 ){
            cout << "tmpPos "<< tmpPos << endl;
            cout << "tmpVel "<< tmpVel << endl;
            cout << "tmpAcc "<< tmpAcc << endl;
        }
*/
        // yaw

        Eigen::Vector2d temp_vel;
        temp_vel << tmpVel.x, tmpVel.y;
        double yaw_angle_cos = yaw_0_.dot(temp_vel) / yaw_0_.norm() / temp_vel.norm();
        double yaw_angle;
        if (temp_vel(1) < 0) {
            yaw_angle = -acos(yaw_angle_cos);
        } else{
            yaw_angle = acos(yaw_angle_cos);
        }
        yaw_angle_ = yaw_angle;

        Eigen::Vector2d temp_acc;
        temp_acc << tmpAcc.x, tmpAcc.y;
        double yaw_dot;
        if (tmpVel.x * tmpAcc.y >= tmpVel.y * tmpAcc.x){
            yaw_dot = temp_acc.norm();
        } else{
            yaw_dot = -temp_acc.norm();
        }
        cmdMsg.yaw = yaw_angle;
        cmdMsg.yaw_dot = yaw_dot;

        // draw cmd
        Eigen::Vector3d pos,vel,acc,jerk;
        pos << tmpPos.x, tmpPos.y, tmpPos.z;
        vel << tmpVel.x, tmpVel.y, tmpVel.z;
        acc << tmpAcc.x, tmpAcc.y, tmpAcc.z;

        drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
        drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
        Eigen::Vector3d dir(cos(yaw_angle), sin(yaw_angle), 0.0);
        drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

        // draw exected_traj with vel color
        // ADD
        cmdMsg.header.stamp = ros::Time::now();
        pos_cmd_pub_.publish(cmdMsg);

    }
    if (near_goal_){
        geometry_msgs::Point tmpPos;
        geometry_msgs::Vector3 tmpVel, tmpAcc, tmpJerk;
        quadrotor_msgs::PositionCommand cmdMsg;
        cmdMsg.header.frame_id = "world";
        cmdMsg.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        tmpAcc.x = tmpAcc.y = tmpAcc.z = 0;
        cmdMsg.acceleration = tmpAcc;
        tmpVel.x = tmpVel.y = tmpVel.z = 0;
        cmdMsg.velocity = tmpVel;

        tmpPos.x = end_pt_(0);
        tmpPos.y = end_pt_(1);
        tmpPos.z = end_pt_(2);
        cmdMsg.position = tmpPos;

        // yaw
        double yaw_angle;
        yaw_angle = last_yaw_;
        cmdMsg.yaw = last_yaw_;
        cmdMsg.yaw_dot = 0;

        // draw cmd
        Eigen::Vector3d dir(cos(yaw_angle), sin(yaw_angle), 0.0);
        drawCmd(end_pt_, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));

        cmdMsg.header.stamp = ros::Time::now();
        pos_cmd_pub_.publish(cmdMsg);

    }

}

void IntelligentReplanFsm::trajSafetyCallback(const ros::TimerEvent& e){
    if (!have_odom_)
        return;
    if (!have_target_)
        return;
    if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
        bool if_safe = planner_manager_->safeCheck();
        if (if_safe){
             return; // safe
        } else {
            cout << "low traj is in obs, replan right now!" << endl;
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
    }
}

/*
void IntelligentReplanFsm::extforceCallback(const serial_node::extforce& exf){
    external_force_(0) = exf.exf_x;
    external_force_(1) = exf.exf_y;
    external_force_(2) = exf.exf_z;
}
*/

void IntelligentReplanFsm::drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id, const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = "world";
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  pos_cmd_vis_pub_.publish(mk_state);

}

void IntelligentReplanFsm::draw_history_traj(){
    unsigned int sp = history_pos_.size(), sv = history_vel_.size();
    int size = sp<=sv ? sp : sv;
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::LINE_STRIP;
    // mk.action          = visualization_msgs::Marker::DELETE;

    mk.action             = visualization_msgs::Marker::ADD;
    mk.scale.x = 0.15;
    for (int i = 0; i < size; i++){
        Eigen::Vector3d v = history_vel_[i];
        double v_norm = v.norm(); 
        // cout << "v  :  " << v_norm << endl;
        double color_num;
        if(v_norm < v_min_)
            color_num = 0.0;
        else if(v_norm > v_max_)
            color_num = 1.0;
        else{
            color_num = (v_norm - v_min_) / (v_max_ - v_min_);
        } 

        std_msgs::ColorRGBA color;    
        color.r = color_num>0.5 ? 2*color_num-1 : 0.0;
        color.g = color_num>0.5 ? -2*color_num+2 : 2*color_num;
        color.b = color_num>0.5 ? 0.0 : -2*color_num+1;
        color.a = 1.0;
        mk.colors.push_back(color);

        geometry_msgs::Point pt;
        pt.x = history_pos_[i](0);
        pt.y = history_pos_[i](1);
        pt.z = history_pos_[i](2);
        mk.points.push_back(pt);
    }
    exected_traj_pub_.publish(mk);

}

}	// namespace