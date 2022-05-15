#include <path_searching/high_mpc_optimizer.h>
#include <nlopt.hpp>
#include <math.h>

using namespace std;
using namespace Eigen;

namespace Intelligent_planner {

    void high_mpc_optimizer::setParam(ros::NodeHandle &nh) {

        nh.param("high_optimization/alpha1", alpha1_, -1.0);
        nh.param("high_optimization/alpha2", alpha2_, -1.0);
        nh.param("high_optimization/alpha3", alpha3_, -1.0);
        nh.param("high_optimization/alpha4", alpha4_, -1.0);
        nh.param("high_optimization/alpha5", alpha5_, -1.0);

        nh.param("high_optimization/amiga1", amiga1_, -1.0);
        nh.param("high_optimization/amiga2", amiga2_, -1.0);

        nh.param("high_optimization/K", K_, -1.0);

        nh.param("high_optimization/Ts", Ts_, -1.0);

        nh.param("high_optimization/dist_0", dist_0_, -1.0);

        nh.param("high_optimization/max_iteration_num", max_iteration_num_, -1);
        nh.param("high_optimization/max_iteration_time", max_iteration_time_, -1.0);


        /// build A_ and B_
        As_ = Matrix3d::Identity();
        Bs_ = Vector3d::Zero();
        Cs_ = Vector3d::Zero();
        As_(0, 1) = Ts_;
        As_(0, 2) = Ts_ * Ts_ / 2;
        As_(1, 2) = Ts_;

        Bs_(0) = Ts_ * Ts_ * Ts_ / 6;
        Bs_(1) = Ts_ * Ts_ / 2;
        Bs_(2) = Ts_;

        Cs_(0) = Ts_ * Ts_ / 2;
        Cs_(1) = Ts_;
        Cs_(2) = 1;

    }


    void high_mpc_optimizer::setEnvironment(const EDTEnvironment::Ptr &env) {
        this->edt_map_ = env;
    }


    std::vector <std::vector<Eigen::Vector3d>> high_mpc_optimizer::highmpcOptimizeTraj(Eigen::MatrixXd start_state,
                                                                                   std::vector <Eigen::Vector3d> mpc_traj,
                                                                                   std::vector <Eigen::Vector3d> mpc_vel,
                                                                                   std::vector <Eigen::Vector3d> mpc_acc,
                                                                                   std::vector<double> mpc_input,
                                                                                   Eigen::Vector3d external_force_remain) {
        setInitialState(start_state, mpc_traj, mpc_vel, mpc_acc, mpc_input, external_force_remain);

        setInitialSystem();

        is_init_system_finish_ = true;
        // use the NLopt optimizer
        optimize();

        vector <vector<Vector3d>> traj;
        vector <Vector3d> traj_pos;
        vector <Vector3d> traj_vel;
        vector <Vector3d> traj_acc;
        //vector <Vector3d> traj_jerk;
        //vector <Vector3d> traj_reference;

        for (int i = 0; i < N_; i++) {
            Vector3d pos, vel, acc, pos_ref;
            pos << state_x_(3 * i), state_y_(3 * i), state_z_(3 * i);
            vel << state_x_(3 * i + 1), state_y_(3 * i + 1), state_z_(3 * i + 1);
            acc << state_x_(3 * i + 2), state_y_(3 * i + 2), state_z_(3 * i + 2);

            //pos_ref = mpc_traj[i];

            traj_pos.push_back(pos);
            traj_vel.push_back(vel);
            traj_acc.push_back(acc);
            //traj_reference.push_back(pos_ref);
        }

        //last_optimized_acc_ = traj_acc[N_ -1];

        traj.push_back(traj_pos);
        traj.push_back(traj_vel);
        traj.push_back(traj_acc);
        //traj.push_back(traj_reference);

        return traj;
    }


    void high_mpc_optimizer::optimize() {
        /* initialize solver */
        iter_num_ = 0;
        min_cost_ = std::numeric_limits<double>::max();

        variable_num_ = 3;

        /* do optimization using NLopt slover */
        nlopt::opt opt(nlopt::algorithm::LD_LBFGS, variable_num_); // use the LD_LBFGS optimization method
        opt.set_min_objective(high_mpc_optimizer::costFunction, this);
        opt.set_maxeval(max_iteration_num_);
        opt.set_maxtime(max_iteration_time_);
        opt.set_ftol_rel(1e-5);  // stop condition
        opt.set_vector_storage(16);  //

        /*
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
        */
        vector<double> q(variable_num_);
        for (int i = 0; i < variable_num_; i++) {
            q[i] = input_f_(i);
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


    double high_mpc_optimizer::costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data) {
        high_mpc_optimizer *opt = reinterpret_cast<high_mpc_optimizer *>(func_data);
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

    void high_mpc_optimizer::combineCost(const std::vector<double> &x, std::vector<double> &grad, double &f_combine) {
        /* convert the NLopt format vector to control inputs. */
        /*
        for (int i = 0; i < (N_ - 1); i++) {
            input_x_(i) = x[i];
        }
        for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
            input_y_(i - (N_ - 1)) = x[i];
        }
        for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
            input_z_(i - 2 * (N_ - 1)) = x[i];
        }
        */
        for (int i = 0; i < 3; i++) {
            input_f_(i) = x[i];
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
        /*
        for (int i = 0; i < (N_ - 1); i++) {
            grad[i] = Gradient_x_(i);
        }
        for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
            grad[i] = Gradient_y_(i - (N_ - 1));
        }
        for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
            grad[i] = Gradient_z_(i - 2 * (N_ - 1));
        }
         */
        for (int i = 0; i < 3; i++) {
            grad[i] = Gradient_f_(i);
        }
    }


    void high_mpc_optimizer::stateEquations() {
        state_x_ = A_ * input_x_ + B_ * start_state_.col(0) + C_ * input_f_(0);
        state_y_ = A_ * input_y_ + B_ * start_state_.col(1) + C_ * input_f_(1);
        state_z_ = A_ * input_z_ + B_ * start_state_.col(2) + C_ * input_f_(2);
    }


    void high_mpc_optimizer::setInitialState(Eigen::MatrixXd start_state,
                                             std::vector <Eigen::Vector3d> mpc_traj,
                                             std::vector <Eigen::Vector3d> mpc_vel,
                                             std::vector <Eigen::Vector3d> mpc_acc,
                                             std::vector<double> mpc_input,
                                             Eigen::Vector3d external_force_remain) {

        // initial start state and system
        mpc_traj_ = mpc_traj;
        mpc_vel_ = mpc_vel;
        mpc_acc_ = mpc_acc;
        mpc_input_ = mpc_input;

        //cout << "mpc_input_.size()   "<< mpc_input_.size() << endl;
        start_state_ = start_state;
        external_force_remain_ = external_force_remain;
        dim_ = 3;        // x, y, z,
        N_ = mpc_traj.size();
        //cout << "N_   "<< N_ << endl;

        input_x_ = VectorXd::Zero(N_ - 1);
        input_y_ = VectorXd::Zero(N_ - 1);
        input_z_ = VectorXd::Zero(N_ - 1);

        for (int i = 0; i < (N_ - 1); i++) {
            input_x_(i) = mpc_input_[i];
        }
        for (int i = (N_ - 1); i < 2 * (N_ - 1); i++) {
            input_y_(i - (N_ - 1)) = mpc_input_[i];
        }
        for (int i = 2 * (N_ - 1); i < 3 * (N_ - 1); i++) {
            input_z_(i - 2 * (N_ - 1)) = mpc_input_[i];
        }
        //cout << "N_  end "<< endl;
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
    }


    void high_mpc_optimizer::resetInputInital() {
        if (mpc_traj_.size() > 1) {
            N_ = mpc_traj_.size();
            //cout<< "N_ "<< N_ << endl;
            //input_f_ = VectorXd::Zero(3);
            input_f_ = external_force_remain_;
        }else{
            input_f_ = VectorXd::Zero(3);
        }
    }


    void high_mpc_optimizer::setInitialSystem() {
        // initial map size
        Vector3d map_origin, map_size;
        edt_map_->getMapRegion(map_origin, map_size);
        map_min_ = map_origin;
        map_max_ = map_origin + map_size;
    }

    void high_mpc_optimizer::calCostFunctionandGradient() {
        // initial intermediate variables
        double fsp, fsv, fsa, fexf, fc;
        fsp = fsv = fsa = fexf = fc = 0.0;
        VectorXd Gradient_fsp, Gradient_fsv, Gradient_fsa, Gradient_fexf, Gradient_fc;
        Gradient_fsp = Gradient_fsa = Gradient_fsv = Gradient_fexf = Gradient_fc = VectorXd::Zero(3);
        Vector3d pos, path_pos, dist_grad, vel, path_vel, acc, path_acc;
        double dist;
        // calculate the CostFunction and Gradient
        for (int i = 0; i < N_; i++) {
            if (i > 0) {
                /* similarity between the traj and reference traj */
                pos << state_x_(3 * i), state_y_(3 * i), state_z_(3 * i);
                path_pos = mpc_traj_[i];
                fsp += pow((pos - path_pos).norm(), 2);
                Vector3d temp ;
                temp << 2 * (pos(0) - path_pos(0)) * C_.row(3 * i),
                        2 * (pos(1) - path_pos(1)) * C_.row(3 * i),
                        2 * (pos(2) - path_pos(2)) * C_.row(3 * i);
                Gradient_fsp += temp;
                //cout << "Gradient_fsp  \n" << Gradient_fsp  << endl;
                /* collision cost */
                edt_map_->evaluateEDT(pos, dist);
                //cout << "dist      " << dist << endl;
                if (abs(dist) < dist_0_) {
                    edt_map_->evaluateFirstGrad(pos, dist_grad);
                    //cout << "dist_grad      \n" << dist_grad << endl;
                    fc += pow(dist - dist_0_, 2);
                    temp << 2 * (dist - dist_0_) * dist_grad(0) * C_.row(3 * i),
                            2 * (dist - dist_0_) * dist_grad(1) * C_.row(3 * i),
                            2 * (dist - dist_0_) * dist_grad(2) * C_.row(3 * i);
                    Gradient_fc += temp;
                    //cout << "Gradient_fc  \n" << Gradient_fc  << endl;
                }

                /* fsv*/
                vel << state_x_(3 * i + 1), state_y_(3 * i + 1), state_z_(3 * i + 1);
                path_vel = mpc_vel_[i];
                //cout << " path_vel\n  " << path_vel  << endl;
                fsv += pow((vel - path_vel).norm(), 2);
                //cout << " C_.row(3 * i + 1)  "<< C_.row(3 * i + 1)  << endl;
                temp << 2 * (vel(0) - path_vel(0)) * C_.row(3 * i + 1),
                        2 * (vel(1) - path_vel(1)) * C_.row(3 * i + 1),
                        2 * (vel(2) - path_vel(2)) * C_.row(3 * i + 1);
                //cout << "temp \n" << temp << endl;
                Gradient_fsv += temp;
                //cout << "Gradient_fsv  \n" << Gradient_fsv  << endl;
                /* fsa */
                acc << state_x_(3 * i + 2), state_y_(3 * i + 2), state_z_(3 * i + 2);
                path_acc = mpc_acc_[i];
                //cout << " path_acc \n  " << path_acc  << endl;
                fsv += pow((acc - path_acc).norm(), 2);
                temp << 2 * (acc(0) - path_acc(0)) * C_.row(3 * i + 2),
                        2 * (acc(1) - path_acc(1)) * C_.row(3 * i + 2),
                        2 * (acc(2) - path_acc(2)) * C_.row(3 * i + 2);
                Gradient_fsa += temp;
                //cout << "Gradient_fsa  \n" << Gradient_fsa  << endl;

            }
        }
        /* fexf */
        for (int i = 0; i < 3; i++) {
            double cost, grad;
            getExtforceGardCost(external_acc_(i), input_f_(i), grad, cost);
            //cout << "grad  " << grad  << endl;
            //cout << "cost  " << cost  << endl;
            fexf += amiga1_ * pow((K_ * input_f_(i) - K_ * external_force_remain_(i)), 2) + amiga2_ * cost;
            Gradient_fexf(i) = amiga1_ * 2 * (K_ * input_f_(i) - K_ * external_force_remain_(i)) * K_ + amiga2_ * grad;
           // fexf += amiga1_ * pow((K_ * input_f_(i) - external_force_remain_(i)), 2);
            //Gradient_fexf(i) = amiga1_ * 2 * (K_ * input_f_(i) - external_force_remain_(i)) * K_ ;
        }
        //cout << "Gradient_fexf  \n" << Gradient_fexf  << endl;
        // f_ mix and gradient mix
        f_ = alpha1_ * fsp + alpha2_ * fc + alpha3_ * fsv + alpha4_ * fsa + alpha5_ * fexf;
        Gradient_f_ = alpha1_ * Gradient_fsp +
                      alpha2_ * Gradient_fc +
                      alpha3_ * Gradient_fsv +
                      alpha4_ * Gradient_fsa +
                      alpha5_ * Gradient_fexf;
    }

    void high_mpc_optimizer::getEnvGardandDistPos(double dist, Eigen::Vector3d &grad, Eigen::Vector3d &pos) {
        if (is_init_system_finish_) {
            edt_map_->evaluateEDT(pos, dist);
            edt_map_->evaluateFirstGrad(pos, grad);
        } else {
            grad << 0, 0, 0;
            dist = std::numeric_limits<double>::max();
        }
    }

    void high_mpc_optimizer::getExtforceGardCost(double x, double u, double grad, double cost) {
        double lx, hx;
        if (x > 0) {
            lx = 0.0;
            hx = x;
        } else {
            lx = x;
            hx = 0.0;
        }
        if (u < lx) {
            cost = pow(u - lx, 2);
            grad = 2 * (u - lx);
        } else if (u > hx) {
            cost = pow(hx - u, 2);
            grad = 2 * (hx - u) * (-1);
        } else {
            cost = 0.0;
            grad = 0.0;
        }
    }
}