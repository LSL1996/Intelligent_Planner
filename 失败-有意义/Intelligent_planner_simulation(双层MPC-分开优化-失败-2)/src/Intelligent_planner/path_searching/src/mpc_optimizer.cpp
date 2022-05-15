#include <path_searching/mpc_optimizer.h>
#include <nlopt.hpp>
#include <math.h>

using namespace std;
using namespace Eigen;

namespace Intelligent_planner {

void mpc_optimizer::setParam(ros::NodeHandle &nh) {
    //nh.param("optimization/N", N_, -1);
    nh.param("optimization/Ke", Ke_, -1.0);
    nh.param("optimization/Kf", Kf_, -1.0);
    nh.param("optimization/alpha1", alpha1_, -1.0);
    nh.param("optimization/alpha2", alpha2_, -1.0);
    nh.param("optimization/alpha3", alpha3_, -1.0);
    nh.param("optimization/alpha4", alpha4_, -1.0);
    nh.param("optimization/alpha5", alpha5_, -1.0);
    nh.param("optimization/alpha6", alpha6_, -1.0);
    nh.param("optimization/alpha7", alpha7_, -1.0);


    nh.param("optimization/yaw_limit", yaw_limit_, -1.0);
    nh.param("optimization/vel_min", vel_min_, -1.0);
    nh.param("optimization/K", K_, -1.0);
    nh.param("optimization/M", M_, 15);
    nh.param("optimization/Ma", Ma_, 50);

    nh.param("multi_mpc/Ts", Ts_, -1.0);

    nh.param("optimization/dist_0", dist_0_, -1.0);
    nh.param("optimization/dist_1", dist_1_, -1.0);

    nh.param("optimization/max_iteration_num", max_iteration_num_, -1);
    nh.param("optimization/max_iteration_time", max_iteration_time_, -1.0);

    nh.param("optimization/vel_lower", vel_lower_, -1.0);
    nh.param("optimization/vel_upper", vel_upper_, -1.0);
    nh.param("optimization/acc_lower", acc_lower_, -1.0);
    nh.param("optimization/acc_upper", acc_upper_, -1.0);
    nh.param("optimization/jerk_lower", jerk_lower_, -1.0);
    nh.param("optimization/jerk_upper", jerk_upper_, -1.0);

    nh.param("optimization/external_force_limit", external_force_limit_, 2.0);  ///
    nh.param("optimization/external_force_limit_replan", external_force_limit_replan_, 4.0);  ///
    nh.param("optimization/vel_limit", vel_limit_, 2.0);   ///

    /// build A_ and B_
    As_ = Matrix3d::Identity();
    Bs_ = Vector3d::Zero();
    Cs_ = Vector3d::Zero();
    As_(0,1) = Ts_;
    As_(0,2) = Ts_ * Ts_ / 2 ;
    As_(1,2) = Ts_;

    Bs_(0)   = Ts_ * Ts_ * Ts_ / 6 ;
    Bs_(1)   = Ts_ * Ts_ / 2 ;
    Bs_(2)   = Ts_;

    Cs_(0)   = Ts_ * Ts_ / 2 ;
    Cs_(1)   = Ts_ ;
    Cs_(2)   = 1 ;

    if_environment_adaptive_ = false;
    is_init_system_finish_ = false;

}


void mpc_optimizer::setEnvironment(const EDTEnvironment::Ptr &env) {
    this->edt_map_ = env;
}


std::vector <std::vector<Eigen::Vector3d>> mpc_optimizer::mpcOptimizeTraj(Eigen::MatrixXd start_state,
                                                                          std::vector <Eigen::Vector3d> local_result_path,
                                                                          bool if_external_force_adaptive,
                                                                          Eigen::Vector3d external_force) {
    if_external_force_adaptive_ = if_external_force_adaptive;
    if (if_external_force_adaptive_) {
        A_external_force_ = K_ * external_force / external_force.norm();
        alpha6_ = alpha6_;
    } else {
        A_external_force_ << 0.0, 0.0, 0.0;
        alpha6_ = 0.0;
    }

    setInitialState(start_state, local_result_path, external_force);

    setInitialSystem();

    is_init_system_finish_ = true;
    // use the NLopt optimizer
    optimize();

    vector <vector<Vector3d>> traj;
    vector <Vector3d> traj_pos;
    vector <Vector3d> traj_vel;
    vector <Vector3d> traj_acc;
    //vector <Vector3d> traj_jerk;
    vector <Vector3d> traj_reference;

    for (int i = 0; i < N_; i++) {
        Vector3d pos, vel, acc, pos_ref;
        pos << state_x_(3 * i), state_y_(3 * i), state_z_(3 * i);
        vel << state_x_(3 * i + 1), state_y_(3 * i + 1), state_z_(3 * i + 1);
        acc << state_x_(3 * i + 2), state_y_(3 * i + 2), state_z_(3 * i + 2);

        pos_ref = local_result_path[i];

        traj_pos.push_back(pos);
        traj_vel.push_back(vel);
        traj_acc.push_back(acc);
        traj_reference.push_back(pos_ref);
    }

    //last_optimized_acc_ = traj_acc[N_ -1];

    traj.push_back(traj_pos);
    traj.push_back(traj_vel);
    traj.push_back(traj_acc);
    traj.push_back(traj_reference);

    return traj;
}


void mpc_optimizer::optimize() {
    /* initialize solver */
    iter_num_ = 0;
    min_cost_ = std::numeric_limits<double>::max();

    variable_num_ = dim_ * (N_ - 1);

    /* do optimization using NLopt slover */
    nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_); // use the LD_LBFGS optimization method
    opt.set_min_objective(mpc_optimizer::costFunction, this);
    opt.set_maxeval(max_iteration_num_);
    opt.set_maxtime(max_iteration_time_);
    opt.set_ftol_rel(1e-3);  // stop condition
    opt.set_vector_storage(16);  //

    /// initial objective variables
    vector<double> q(variable_num_);
    for (int i = 0; i < (N_ - 1); i++) {
        q[i] = input_x_(i);
    }
    for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
        q[i] = input_y_(i - (N_ - 1));
    }
    for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
        q[i] = input_z_(i - 2 * (N_ - 1));
    }

    double final_cost;
    try {
        nlopt::result result = opt.optimize(q, final_cost);
    }
    catch (std::exception &e) {
        ROS_WARN("[Optimization]: nlopt exception mpc");
        cout << e.what() << endl;
    }
}


double mpc_optimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data) {
    mpc_optimizer *opt = reinterpret_cast<mpc_optimizer *>(func_data);
    double cost;
    opt->combineCost(x, grad, cost);

    opt->iter_num_++;
    /* save the min cost result */
    if (cost < opt->min_cost_) {
        opt->min_cost_ = cost;
        opt->best_variable_ = x;
    }
    return cost;
}

void mpc_optimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad, double &f_combine) {
    /* convert the NLopt format vector to control inputs. */
    for (int i = 0; i < (N_ - 1); i++) {
        input_x_(i) = x[i];
    }
    for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
        input_y_(i - (N_ - 1)) = x[i];
    }
    for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
        input_z_(i - 2 * (N_ - 1)) = x[i];
    }

    /*  evaluate costs and their gradient  */
    f_combine = 0.0;
    grad.resize(variable_num_);
    fill(grad.begin(), grad.end(), 0.0);
    // calculate the initial system state
    stateEquations();
    // calculate the initial cost function and gradient
    calCostFunctionandGradient();
    f_combine = f_;

    // convert the gradient to the NLopt format
    for (int i = 0; i < (N_ - 1); i++) {
        grad[i] = Gradient_x_(i);
    }
    for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
        grad[i] = Gradient_y_(i - (N_ - 1));
    }
    for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
        grad[i] = Gradient_z_(i - 2 * (N_ - 1));
    }
}


void mpc_optimizer::stateEquations() {
    state_x_ = A_ * input_x_ + B_ * start_state_.col(0) + C_ * A_external_force_(0);
    state_y_ = A_ * input_y_ + B_ * start_state_.col(1) + C_ * A_external_force_(1);
    state_z_ = A_ * input_z_ + B_ * start_state_.col(2) + C_ * A_external_force_(2);

}


void mpc_optimizer::setInitialState(Eigen::MatrixXd start_state, std::vector <Eigen::Vector3d> local_result_path,
                                    Eigen::Vector3d external_force) {

    // initial start state and system
    local_result_path_ = local_result_path;
    start_state_ = start_state;
    external_force_ = external_force;
    dim_ = 3;        // x, y, z,
    N_ = local_result_path.size();
    //cout<< "N_ "<< N_ << endl;
    MatrixPower <Matrix3d> Apow(As_);

    A_ = MatrixXd::Zero(3 * N_, N_ - 1);
    B_ = MatrixXd::Zero(3 * N_, 3);
    C_ = MatrixXd::Zero(3 * N_, 1);
    B_.block(0, 0, 3, 3) = Matrix3d::Identity();

    for (int i = 1; i < N_; i++) {
        for (int j = 0; j < i; j++) {
            A_.block(3 * i, j, 3, 1) = Apow(i - j - 1) * Bs_;
        }
        B_.block(3 * i, 0, 3, 3) = Apow(i);

        C_.block(3 * i, 0, 3, 1) = Apow(i - 1) * Cs_;
    }

    resetInputInital();

    // initial reference path
    path_x_ = VectorXd::Zero(N_);
    path_y_ = VectorXd::Zero(N_);
    path_z_ = VectorXd::Zero(N_);
    for (int i = 0; i < N_; i++) {
        Vector3d pos = local_result_path[i];
        path_x_(i) = pos(0);
        path_y_(i) = pos(1);
        path_z_(i) = pos(2);
    }
}


void mpc_optimizer::resetInputInital() {

    if (local_result_path_.size() > 1) {
        N_ = local_result_path_.size();
        //cout<< "N_ "<< N_ << endl;
        input_x_ = VectorXd::Zero(N_ - 1);
        input_y_ = VectorXd::Zero(N_ - 1);
        input_z_ = VectorXd::Zero(N_ - 1);
    }

}


void mpc_optimizer::setInitialSystem() {
    // initial map size
    Vector3d map_origin, map_size;
    edt_map_->getMapRegion(map_origin, map_size);
    map_min_ = map_origin;
    map_max_ = map_origin + map_size;
}


Vector3d mpc_optimizer::getPosPoly(MatrixXd polyCoeff, int k, double t) {
    Vector3d ret;
    int _poly_num1D = (int) polyCoeff.cols() / 3;
    for (int dim = 0; dim < 3; dim++) {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = 0; j < _poly_num1D; j++)
            if (j == 0)
                time(j) = 1.0;
            else
                time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
    }
    return ret;
}


void mpc_optimizer::calCostFunctionandGradient() {
    // initial intermediate variables
    double ft, fu, fc, fvelacc, fjerk, fef, fdir;
    ft = fu = fc = fvelacc = fjerk = fef = fdir = 0.0;
    Vector3d total_force;
    double fef_acc,fef_vel;
    Vector3d pos, vel, acc, jerk, path_pos, dist_grad, dist_grad2;
    double dist;
    double fef_acc_tmp, fef_vel_temp;

    double beta, f_beta, p_beta_vx, p_beta_vy, p_beta_vz, p_beta_cx, p_beta_cy, p_beta_cz,p_f_beta_beta;

    double beta1, f_beta1, p_f_beta1_beta, p_beta1_ax, p_beta1_ay, p_beta1_az;
    double beta_f;
    double f_beta1_1,f_beta1_2;
    f_beta1_1 = f_beta1_2 = 0.0;

    double f_beta1_2_x,f_beta1_2_y,f_beta1_2_z;

    Vector3d input, input_next;
    VectorXd Gradient_tx, Gradient_ty, Gradient_tz, Gradient_ttheta;
    Gradient_tx = Gradient_ty = Gradient_tz = Gradient_ttheta = VectorXd::Zero(N_ - 1);
    VectorXd Gradient_ux, Gradient_uy, Gradient_uz;
    Gradient_ux = Gradient_uy = Gradient_uz = VectorXd::Zero(N_ - 1);
    VectorXd Gradient_cx, Gradient_cy, Gradient_cz;
    Gradient_cx = Gradient_cy = Gradient_cz = VectorXd::Zero(N_ - 1);
    VectorXd Gradient_velx, Gradient_vely, Gradient_velz, Gradient_accx, Gradient_accy, Gradient_accz, Gradient_jerkx, Gradient_jerky, Gradient_jerkz;
    Gradient_velx = Gradient_vely = Gradient_velz = VectorXd::Zero(N_ - 1);
    Gradient_accx = Gradient_accy = Gradient_accz = VectorXd::Zero(N_ - 1);
    Gradient_jerkx = Gradient_jerky = Gradient_jerkz = VectorXd::Zero(N_ - 1);
    //---------------------------------------------------------//
    VectorXd Gradient_fvx, Gradient_fvy, Gradient_fvz, Gradient_fax, Gradient_fay, Gradient_faz;
    Gradient_fvx = Gradient_fvy = Gradient_fvz = Gradient_fax = Gradient_fay = Gradient_faz = VectorXd::Zero(N_ - 1);
    VectorXd Gradient_fdirx,Gradient_fdiry;
    Gradient_fdirx = Gradient_fdiry = VectorXd::Zero(N_-1);
    VectorXd Gradient_fdirx_t,Gradient_fdiry_t;
    Gradient_fdirx_t = Gradient_fdiry_t = VectorXd::Zero(N_);
    //-------------------------------------------------------//
    VectorXd d_f_beta1_2_x,d_f_beta1_2_y,d_f_beta1_2_z;
    d_f_beta1_2_x = d_f_beta1_2_y = d_f_beta1_2_z = VectorXd::Zero(N_ - 1);
    // calculate the CostFunction and Gradient
    for (int i = 0; i < N_; i++) {
        if (i > 0) {
            /* similarity between the traj and reference traj */
            //  ft
            pos      << state_x_(3 * i), state_y_(3 * i), state_z_(3 * i);
            path_pos << path_x_(i), path_y_(i), path_z_(i);
            // gradient of ft
            ft += pow((pos - path_pos).norm(), 2);
            Gradient_tx += 2 * (pos(0) - path_pos(0)) * A_.row(3 * i);
            Gradient_ty += 2 * (pos(1) - path_pos(1)) * A_.row(3 * i);
            Gradient_tz += 2 * (pos(2) - path_pos(2)) * A_.row(3 * i);

            /* collision cost */
            // fc and gradient of fc
            edt_map_->evaluateEDT(pos, dist);  ///
            //cout << "dist     "<< dist << endl;
            edt_map_->evaluateFirstGrad(pos, dist_grad);   ///

            /*
            if (abs(dist) < dist_0_) {
                fc += pow(dist - dist_0_, 2);

                Gradient_cx += 2 * (dist - dist_0_) * dist_grad(0) * A_.row(3 * i);
                Gradient_cy += 2 * (dist - dist_0_) * dist_grad(1) * A_.row(3 * i);
                Gradient_cz += 2 * (dist - dist_0_) * dist_grad(2) * A_.row(3 * i);
            }
             */

            if (abs(dist) < dist_0_) {
                fc += pow(- dist + dist_0_ , 3);

                Gradient_cx += 3 * pow(-dist + dist_0_, 2) * (-1) * dist_grad(0) * A_.row(3 * i);
                Gradient_cy += 3 * pow(-dist + dist_0_, 2) * (-1) * dist_grad(1) * A_.row(3 * i);
                Gradient_cz += 3 * pow(-dist + dist_0_, 2) * (-1) * dist_grad(2) * A_.row(3 * i);
            }

            /* the penalty of vel and acc jerk, which is used to prevent vel and acc  jerk from exceeding the range */
            // fvelaccjerk
            vel  << state_x_(3 * i + 1), state_y_(3 * i + 1), state_z_(3 * i + 1);
            acc  << state_x_(3 * i + 2), state_y_(3 * i + 2), state_z_(3 * i + 2);

            fvelacc += calRangeCost(vel(0), vel_lower_, vel_upper_);
            fvelacc += calRangeCost(vel(1), vel_lower_, vel_upper_);
            fvelacc += calRangeCost(vel(2), vel_lower_, vel_upper_);

            fvelacc += calRangeCost(acc(0), acc_lower_, acc_upper_);
            fvelacc += calRangeCost(acc(1), acc_lower_, acc_upper_);
            fvelacc += calRangeCost(acc(2), acc_lower_, acc_upper_);

            // gradient of fvelacc
            Gradient_velx += calRangeGrad(vel(0), vel_lower_, vel_upper_) * A_.row(3 * i + 1);
            Gradient_vely += calRangeGrad(vel(1), vel_lower_, vel_upper_) * A_.row(3 * i + 1);
            Gradient_velz += calRangeGrad(vel(2), vel_lower_, vel_upper_) * A_.row(3 * i + 1);

            Gradient_accx += calRangeGrad(acc(0), acc_lower_, acc_upper_) * A_.row(3 * i + 2);
            Gradient_accy += calRangeGrad(acc(1), acc_lower_, acc_upper_) * A_.row(3 * i + 2);
            Gradient_accz += calRangeGrad(acc(2), acc_lower_, acc_upper_) * A_.row(3 * i + 2);

            /* minimum input jerk */
            // fjerk
            if (i < N_ - 1) {
                input << input_x_(i - 1), input_y_(i - 1), input_z_(i - 1);
                fjerk += calRangeCost(input(0), jerk_lower_, jerk_upper_);
                fjerk += calRangeGrad(input(1), jerk_lower_, jerk_upper_);
                fjerk += calRangeGrad(input(2), jerk_lower_, jerk_upper_);
                Gradient_jerkx(i - 1) = calRangeGrad(input(0), jerk_lower_, jerk_upper_);
                Gradient_jerky(i - 1) = calRangeGrad(input(1), jerk_lower_, jerk_upper_);
                Gradient_jerkz(i - 1) = calRangeGrad(input(2), jerk_lower_, jerk_upper_);
            }

            /* external force adaptive */
            /// fef  根据外力大小约束速度加速度 以及增加避障性能

            //cout << "if_external_force_adaptive_   " << if_external_force_adaptive_ << endl;
            if (if_external_force_adaptive_) {
                if (abs(dist) < dist_1_) {
                    //beta1 = -last_optimized_acc_.dot(external_force_) / last_optimized_acc_.norm() / external_force_.norm();
                    //cout << "beta1 \n"<<beta1 <<endl;
                    //f_beta1_1 = 2 / (1 + exp(Kf_ * beta1));
                    f_beta1_1 = 0.0;

                    if (abs(acc(0)) > acc_upper_) {
                        f_beta1_2_x = 1.0 * (abs(acc(0)) - acc_upper_) + 1.0;
                    } else if (abs(acc(0)) > 3.0) {
                        f_beta1_2_x = 1.0 * exp(abs(acc(0)) - acc_upper_);
                    } else {
                        f_beta1_2_x = 0.0;
                    }

                    if (abs(acc(1)) > acc_upper_) {
                        f_beta1_2_y = 1.0 * (abs(acc(1)) - acc_upper_) + 1.0;
                    } else if (abs(acc(1)) > 3.0) {
                        f_beta1_2_y = 1.0 * exp(abs(acc(1)) - acc_upper_);
                    } else {
                        f_beta1_2_y = 0.0;
                    }

                    if (abs(acc(2)) > 0.8) {
                        f_beta1_2_z = 0.5 * exp(abs(acc(2)) - 0.8);
                    } else {
                        f_beta1_2_z = 0.0;
                    }

                    f_beta1_2 = f_beta1_2_x + f_beta1_2_y + f_beta1_2_z;
                    f_beta1 = f_beta1_1 + f_beta1_2;

                    //cout << "f_beta1_2 \n "<<f_beta1_2<<endl;
                    //cout << "f_beta1 \n "<<f_beta1<<endl;
                    fef_acc_tmp += pow(acc.norm() - sqrt(acc_upper_ * acc_upper_ * 3), 2);
                    //cout << "fef_acc_tmp \n" << fef_acc_tmp <<endl;
                    fef_acc += log(1 + fef_acc_tmp) / log(Ma_);
                    //cout << "Ma_       " << Ma_ <<endl;
                    //cout << "fef_acc \n" << fef_acc << endl;
                    fef_vel_temp = pow(vel.norm() - vel_limit_, 2);
                    fef_vel = log(1 + fef_vel_temp) / log(M_);
                    //cout << "fef_vel \n"  << fef_vel <<endl;
                    fef += f_beta1 * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                    //cout << "fef\n" << fef <<endl;

                    if (abs(acc(0)) > acc_upper_) {
                        d_f_beta1_2_x = 1.0 * A_.row(3 * i + 2) * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                        d_f_beta1_2_x = acc(0) > 0 ? d_f_beta1_2_x : -d_f_beta1_2_x;
                    } else if (abs(acc(0)) > 3.0) {
                        d_f_beta1_2_x = 1.0 * exp(abs(acc(0)) - acc_upper_) * A_.row(3 * i + 2) * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                        d_f_beta1_2_x = acc(0) > 0 ? d_f_beta1_2_x : -d_f_beta1_2_x;
                    }

                    if (abs(acc(1)) > acc_upper_) {
                        d_f_beta1_2_y = 1.0 * A_.row(3 * i + 2) * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                        d_f_beta1_2_y = acc(1) > 0 ? d_f_beta1_2_y : -d_f_beta1_2_y;
                    } else if (abs(acc(1)) > 3.0) {
                        d_f_beta1_2_y = 1.0 * exp(abs(acc(1)) - acc_upper_) * A_.row(3 * i + 2) * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                        d_f_beta1_2_y = acc(1) > 0 ? d_f_beta1_2_y : -d_f_beta1_2_y;
                    }

                    if (abs(acc(2)) > 0.8) {
                        d_f_beta1_2_z = 0.5 * exp(abs(acc(2)) - 0.8) * A_.row(3 * i + 2) * pow(dist - dist_1_, 2) * fef_vel * fef_acc;
                        d_f_beta1_2_z = acc(2) > 0 ? d_f_beta1_2_z : -d_f_beta1_2_z;
                    }

                    //cout << "d_f_beta1_2_y  \n "<<d_f_beta1_2_y << endl;

                    Gradient_fvx += f_beta1 * fef_acc * 2 * (dist - dist_1_) * dist_grad(0) * A_.row(3 * i) * fef_vel +
                                    f_beta1 * fef_acc * pow(dist - dist_1_, 2) * vel(0) * (vel.norm() - vel_limit_) /
                                    vel.norm() * A_.row(3 * i + 1) / (1 + fef_vel_temp) / log(M_);
                    Gradient_fvy += f_beta1 * fef_acc * 2 * (dist - dist_1_) * dist_grad(1) * A_.row(3 * i) * fef_vel +
                                    f_beta1 * fef_acc * pow(dist - dist_1_, 2) * vel(1) * (vel.norm() - vel_limit_) /
                                    vel.norm() * A_.row(3 * i + 1) / (1 + fef_vel_temp) / log(M_);
                    Gradient_fvz += f_beta1 * fef_acc * 2 * (dist - dist_1_) * dist_grad(2) * A_.row(3 * i) * fef_vel +
                                    f_beta1 * fef_acc * pow(dist - dist_1_, 2) * vel(2) * (vel.norm() - vel_limit_) /
                                    vel.norm() * A_.row(3 * i + 1) / (1 + fef_vel_temp) / log(M_);

                    Gradient_fax += f_beta1 * pow(dist - dist_1_, 2) * fef_vel * acc(0) *
                                    (acc.norm() - sqrt(acc_upper_ * acc_upper_ * 3)) / acc.norm() * A_.row(3 * i + 2) /
                                    (1 + fef_acc_tmp) / log(Ma_);
                    //cout << "Gradient_fax_1 \n  "<< Gradient_fax <<endl;
                    Gradient_fax += d_f_beta1_2_x;
                    //cout << "Gradient_fax  result\n  "<<endl;
                    Gradient_fay += f_beta1 * pow(dist - dist_1_, 2) * fef_vel * acc(1) *
                                    (acc.norm() - sqrt(acc_upper_ * acc_upper_ * 3)) / acc.norm() * A_.row(3 * i + 2) /
                                    (1 + fef_acc_tmp) / log(Ma_);
                    //cout << "Gradient_fay_1 \n  "<< Gradient_fay <<endl;
                    Gradient_fay += d_f_beta1_2_y;
                    //cout << "Gradient_fay  result\n  "<<endl;
                    Gradient_faz += f_beta1 * pow(dist - dist_1_, 2) * fef_vel * acc(2) *
                                    (acc.norm() - sqrt(acc_upper_ * acc_upper_ * 3)) / acc.norm() * A_.row(3 * i + 2) /
                                    (1 + fef_acc_tmp) / log(Ma_);
                    // cout << "Gradient_faz_1 \n  "<< Gradient_faz <<endl;
                    Gradient_faz += d_f_beta1_2_z;
               }
            }
        }
        /// fdir 引导搜索轨迹指向目标，以最短的缓冲距离向目标点移动
        if(if_external_force_adaptive_) {
            if (i > 0) {
                Eigen::Vector2d v1, v2;
                v1(1) = end_pt_(1) - state_y_(3 * i);
                v1(0) = end_pt_(0) - state_x_(3 * i);
                v2(1) = state_y_(3 * i) - state_y_(3 * (i - 1));
                v2(0) = state_x_(3 * i) - state_x_(3 * (i - 1));
                double the = (v1(0) * v2(0) + v1(1) * v2(1)) / sqrt(v1(0) * v1(0) + v1(1) * v1(1)) /
                             sqrt(v2(0) * v2(0) + v2(1) * v2(1));
                double ac = acos(the);
                if (ac > 3.1415926 / 2) {
                    fdir += pow(ac - 3.1415926 / 2, 2);
                    double t1, t2, t3;
                    t1 = (v1(0) * v1(0) + v1(1) * v1(1)) * (v2(0) * v2(0) + v2(1) * v2(1));
                    Gradient_fdirx_t(i) +=
                            1 / (sqrt(1 - pow(the, 2))) * (sqrt(t1) * (end_pt_(0) + state_x_(i - 1) - 2 * state_x_(i)) +
                                                           (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                           sqrt(v1(0) * v1(0) + v1(1) * v1(1)) * v2(0) /
                                                           sqrt(v2(0) * v2(0) + v2(1) * v2(1)) -
                                                           (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                           sqrt(v2(0) * v2(0) + v2(1) * v2(1)) * v1(0) /
                                                           sqrt(v1(0) * v1(0) + v1(1) * v1(1))) / t1;

                    Gradient_fdiry_t(i) +=
                            1 / (sqrt(1 - pow(the, 2))) * (sqrt(t1) * (end_pt_(1) + state_y_(i - 1) - 2 * state_y_(i)) +
                                                           (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                           sqrt(v1(0) * v1(0) + v1(1) * v1(1)) * v2(1) /
                                                           sqrt(v2(0) * v2(0) + v2(1) * v2(1)) -
                                                           (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                           sqrt(v2(0) * v2(0) + v2(1) * v2(1)) * v1(1) /
                                                           sqrt(v1(0) * v1(0) + v1(1) * v1(1))) / t1;

                    Gradient_fdirx_t(i - 1) += 1 / (sqrt(1 - pow(the, 2))) * (-sqrt(t1) * (end_pt_(0) - state_x_(i)) -
                                                                              (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                                              sqrt(v1(0) * v1(0) + v1(1) * v1(1)) *
                                                                              v2(0) /
                                                                              sqrt(v2(0) * v2(0) + v2(1) * v2(1))) / t1;

                    Gradient_fdiry_t(i - 1) += 1 / (sqrt(1 - pow(the, 2))) * (-sqrt(t1) * (end_pt_(1) - state_y_(i)) -
                                                                              (v1(0) * v2(0) + v1(1) * v2(1)) *
                                                                              sqrt(v1(0) * v1(0) + v1(1) * v1(1)) *
                                                                              v2(1) /
                                                                              sqrt(v2(0) * v2(0) + v2(1) * v2(1))) / t1;
                }
            }
            if (i == N_ - 1) {
                for (int j = 0; j < N_ ; j++) {
                    Gradient_fdirx += Gradient_fdirx_t(i) * A_.row(3 * j);
                    Gradient_fdiry += Gradient_fdiry_t(i) * A_.row(3 * j);
                }
            }
        }

        /* the cost of smooth control input */
        // fu
        if (i < N_ - 1) {
            if (i < N_ - 2) {
                fu += pow((input_x_(i + 1) - input_x_(i)), 2)
                      + pow((input_y_(i + 1) - input_y_(i)), 2)
                      + pow((input_z_(i + 1) - input_z_(i)), 2);
            }
            if (i == 0) {
                Gradient_ux(i) = 2 * (input_x_(i) - input_x_(i + 1));
                Gradient_uy(i) = 2 * (input_y_(i) - input_y_(i + 1));
                Gradient_uz(i) = 2 * (input_z_(i) - input_z_(i + 1));
            } else if (i == N_ - 2) {
                Gradient_ux(i) = 2 * (input_x_(i) - input_x_(i - 1));
                Gradient_uy(i) = 2 * (input_y_(i) - input_y_(i - 1));
                Gradient_uz(i) = 2 * (input_z_(i) - input_z_(i - 1));
            } else {
                Gradient_ux(i) = 4 * input_x_(i) - 2 * (input_x_(i - 1) + input_x_(i + 1));
                Gradient_uy(i) = 4 * input_y_(i) - 2 * (input_y_(i - 1) + input_y_(i + 1));
                Gradient_uz(i) = 4 * input_z_(i) - 2 * (input_z_(i - 1) + input_z_(i + 1));
            }
        }
    }

    // f_ mix and gradient mix
    f_ = alpha1_ * ft + alpha2_ * fu + alpha3_ * fc + alpha4_ * fvelacc + alpha5_ * fjerk  + alpha6_ * fef + alpha7_ * fdir;
    Gradient_x_ = alpha1_ * Gradient_tx +
                  alpha2_ * Gradient_ux +
                  alpha3_ * Gradient_cx +
                  alpha4_ * (Gradient_velx + Gradient_accx ) +
                  alpha5_ * Gradient_jerkx +
                  alpha6_ * (Gradient_fvx + Gradient_fax)+
                  alpha7_ * Gradient_fdirx;

    Gradient_y_ = alpha1_ * Gradient_ty +
                  alpha2_ * Gradient_uy +
                  alpha3_ * Gradient_cy +
                  alpha4_ * (Gradient_vely + Gradient_accy) +
                  alpha5_ * Gradient_jerky +
                  alpha6_ * (Gradient_fvy + Gradient_fay)+
                  alpha7_ * Gradient_fdiry;

    Gradient_z_ = alpha1_ * Gradient_tz +
                  alpha2_ * Gradient_uz +
                  alpha3_ * Gradient_cz +
                  alpha4_ * (Gradient_velz + Gradient_accz ) +
                  alpha5_ * Gradient_jerkz +
                  alpha6_ * (Gradient_fvz + Gradient_faz);
}


double mpc_optimizer::calRangeCost(double value, double lower, double upper) {
    double range_cost;
    if (value < lower) {
        range_cost = pow((value - lower), 2);
    } else if (value > upper) {
        range_cost = pow((value - upper), 2);
    } else {
        range_cost = 0.0;
    }
    return range_cost;
}


double mpc_optimizer::calRangeGrad(double value, double lower, double upper) {
    double range_grad;
    if (value < lower) {
        range_grad = 2 * (value - lower);
    } else if (value > upper) {
        range_grad = 2 * (value - upper);
    } else {
        range_grad = 0.0;
    }
    return range_grad;
}

double mpc_optimizer::calRangeCostThree(double value, double lower, double upper) {
    double range_cost;
    if (value < lower) {
        range_cost = -pow((value - lower), 3);
    } else if (value > upper) {
        range_cost = pow((value - upper), 3);
    } else {
        range_cost = 0.0;
    }
    return range_cost;
}

double mpc_optimizer::calRangeGradThree(double value, double lower,double upper) {
    double range_grad;
    if (value < lower){
         range_grad = -3 * pow((value - lower),2);
     } else if (value > upper){
         range_grad = 3 * pow((value - upper),2);
     } else {
         range_grad = 0.0;
     }
    return range_grad;
}

void mpc_optimizer::getEnvGardandDistPos(double dist, Eigen::Vector3d& grad, Eigen::Vector3d& pos){
    if(is_init_system_finish_){
        edt_map_->evaluateEDT(pos, dist);
        edt_map_->evaluateFirstGrad(pos, grad);
    }else{
        grad << 0,0,0;
        dist = std::numeric_limits<double>::max();
    }
}

}