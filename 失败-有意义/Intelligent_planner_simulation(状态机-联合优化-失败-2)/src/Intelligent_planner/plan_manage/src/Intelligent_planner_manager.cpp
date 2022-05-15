#include <plan_manage/Intelligent_planner_manager.h>
#include <thread>

namespace Intelligent_planner{

IntelligentPlannerManager::IntelligentPlannerManager(){}

IntelligentPlannerManager::~IntelligentPlannerManager(){ std::cout << "des manager" << std::endl; }


void IntelligentPlannerManager::initPlanModules(ros::NodeHandle& nh){
    nh.param("fsm/safety_dist",   safety_dist_, 0.1);

    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setParam(nh);
    edt_environment_->setMap(sdf_map_);

    path_finder_.reset(new KinodynamicAstar);
    path_finder_->setParam(nh);
    path_finder_->setEnvironment(sdf_map_);
    path_finder_->init();

    mpc_optimizer_.reset(new mpc_optimizer);
    mpc_optimizer_->setParam(nh);
    mpc_optimizer_->setEnvironment(edt_environment_);
}

bool IntelligentPlannerManager::pathsearching(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,Eigen::Vector3d start_acc,Eigen::Vector3d end_pt){
    path_finder_->reset();
    Eigen::Vector3d end_vel(0,0,0);
    //std::cout << "start search dya_a"<< std::endl;
    path_finder_->updateExternalAcc(external_acc_remain_);

    int status = path_finder_->search(start_pt,start_vel, start_acc,end_pt,end_vel,true);
    //std::cout << "end search dya_a"<< std::endl;
    if (status == KinodynamicAstar::NO_PATH) {
        path_finder_->reset();
        status = path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
        if (status == KinodynamicAstar::NO_PATH) {
            return false;
        }
    }
    //cout << "need_replan_     "<< need_replan_ << endl;
    if(need_replan_)
        local_result_path_ =  path_finder_->getKinoTraj(0.04);
    else
        local_result_path_ =  path_finder_->getKinoTraj(0.05);
    return true;
}

bool IntelligentPlannerManager::highMpc(Eigen::MatrixXd start_state, bool if_external_force_adaptive, Eigen::Vector3d external_force){

    mpc_optimizer_-> external_acc_ = external_acc_;
    mpc_traj_ = mpc_optimizer_->mpcOptimizeTraj(start_state, local_result_path_, if_external_force_adaptive, external_force);

    mpc_traj_pos_     = mpc_traj_[0];
    mpc_traj_vel_     = mpc_traj_[1];
    mpc_traj_acc_     = mpc_traj_[2];
    mpc_traj_ref_     = mpc_traj_[3];

    return true;
}

void IntelligentPlannerManager::resetMPCinitial(){
    mpc_optimizer_->resetInputInital();
    mpc_optimizer_->end_pt_ = end_pt_;
}


bool IntelligentPlannerManager::safeCheck(){
    std::vector<Eigen::Vector3d> checked_tarj = mpc_traj_pos_;  ///  local_result_path_
    int num = checked_tarj.size();
    for (int i=0;i<num;i++){
        Eigen::Vector3d pos = checked_tarj[i];
        double dist = sdf_map_->getDistance(pos);  ///
        if(dist < 0.1)
            return false;
    }
    return true;
}

void IntelligentPlannerManager::setodompos(void){
    mpc_optimizer_->odom_pos_ = odom_pos_;
}

void IntelligentPlannerManager::getEnvGardandDistPos(double dist, Eigen::Vector3d& grad, Eigen::Vector3d& pos){
    mpc_optimizer_->getEnvGardandDistPos(dist,grad,pos);
}

void IntelligentPlannerManager::getOptimizedExtforce(Eigen::Vector3d& optimizedforce){
    for(int i = 3 * (mpc_optimizer_->N_ - 1); i < 3 * (mpc_optimizer_->N_ - 1) + 3; i++) {
        optimizedforce(i - 3 * (mpc_optimizer_->N_ - 1)) = mpc_optimizer_->best_variable_[i];
    }
}

}	// namespace Intelligent_planner