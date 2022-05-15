#include <iostream>
#include <so3_control/HFControl.h>
#include <boost/bind.hpp>
#include <ros/ros.h>

using namespace std;

HFControl::HFControl()
  : mass_(0.98), g_(9.81), L_(0.3)
{
  acc_.setZero();

  last_z_.setZero();

  //beta_comp_force_ = 0.0 ;

  //last_ext_force_acc_.setZero();

}

void HFControl::setMass(const double mass)
{
  mass_ = mass;
}

void HFControl::setGravity(const double g)
{
  g_ = g;
}

void HFControl::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void HFControl::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void HFControl::setAcc(const Eigen::Vector3d& acc)
{
    acc_ = acc;
}

void HFControl::setEnvGrad(const Eigen::Vector3d& envgrad)
{
    envgrad_ = envgrad;
}

void HFControl::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const Eigen::Vector3d& des_jerk,
                             const double des_yaw,
                             const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{

  Eigen::Vector3d totalError =
    (des_pos - pos_) + (des_vel - vel_) + (des_acc - acc_);   ///

  Eigen::Vector3d ka(fabs(totalError[0]) > 4 ? 0 : (fabs(totalError[0]) * 0.2),
                     fabs(totalError[1]) > 4 ? 0 : (fabs(totalError[1]) * 0.2),
                     fabs(totalError[2]) > 4 ? 0 : (fabs(totalError[2]) * 0.2));


  //cout <<"ext_force_acc_comp_   "<< ext_force_acc_comp_  <<endl;
    force_.noalias() = mass_ *
                       (kx.asDiagonal() * (des_pos - pos_) +
                        kv.asDiagonal() * (des_vel - vel_) +
                        des_acc +
                        ka.asDiagonal() * (des_acc - acc_) +   /// 等同于对积分环节 ka * (des_acc - acc_) * dt
                        g_ * Eigen::Vector3d(0, 0, 1) - ext_force_acc_comp_);  ///

  // Limit control angle to 45 degree
  double          theta = M_PI / 2;
  double          c     = cos(theta);
  Eigen::Vector3d f;

    f.noalias() = mass_ *
                  ( kx.asDiagonal() * (des_pos - pos_) +
                    kv.asDiagonal() * (des_vel - vel_) + //
                    des_acc +                    //
                    ka.asDiagonal() * (des_acc - acc_) - ext_force_acc_comp_);

  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);

    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  }
/*
  for(int i=0;i<3;i++)
        std::cout<< "force_("<<i<<")   "<<force_(i)<<std::endl;
  // Limit control angle to 45 degree
*/
  Eigen::Vector3d f_m, abc, abc_dot;
  Eigen::Vector4d q_abc, temp, q_yaw, R_q;
  f_m = force_ / mass_;
  abc = f_m.normalized();
/*
  for(int i=0;i<3;i++)
        std::cout<< "abc("<<i<<")   "<<abc(i)<<std::endl;
*/
  Eigen::Vector3d tf_1 = (f_m.transpose() * f_m)* Eigen::MatrixXd::Identity(3,3) * des_jerk * (1 / pow(f_m.norm(),3));
  Eigen::Vector3d tf_2 = f_m * f_m.transpose() * des_jerk * (1 / pow(f_m.norm(),3));
  abc_dot = tf_1 - tf_2;
/*
    for(int i=0;i<3;i++)
        std::cout<< "abc_dot("<<i<<")   "<<abc_dot(i)<<std::endl;
   std::cout<<"des_yaw  "<<des_yaw<<std::endl;
    std::cout<<"des_yaw_dot  "<<des_yaw_dot<<std::endl;
*/
  if(abc(2)+1 > 0){
      omega_b_(0) = sin(des_yaw)*abc_dot(0) - cos(des_yaw)*abc_dot(1) -
              (sin(des_yaw)*abc(0) - cos(des_yaw)*abc(1) ) * abc_dot(2) / (abc(2)+1);
      omega_b_(1) = cos(des_yaw)*abc_dot(0) + sin(des_yaw)*abc_dot(1) -
              (cos(des_yaw)*abc(0) + sin(des_yaw)*abc(1) )* abc(2) / (abc(2)+1);
      omega_b_(2) = (abc(1)*abc_dot(0) - abc(0)*abc_dot(1) ) / (1+abc(2)) + des_yaw_dot;
      /*
      for(int i=0;i<3;i++)
          std::cout<< "omega_b_("<<i<<")   "<<omega_b_(i)<<std::endl;
          */

      temp(0) = 1+ abc(2);
      temp(1) = -abc(1);
      temp(2) = abc(0);
      temp(3) = 0;
      q_abc = (1/ sqrt(2*temp(0)))* temp;
      q_yaw(0) = cos(des_yaw/2);
      q_yaw(1) = 0;
      q_yaw(2) =0;
      q_yaw(3) = sin(des_yaw/2);
  }else{
      double temp_yaw;
      temp_yaw = std::atan2(abc(0), abc(1)) + des_yaw;
      omega_b_(0) = sin(des_yaw)*abc_dot(0) + cos(des_yaw)*abc_dot(1) -
              (sin(des_yaw)*abc(0) + cos(des_yaw)*abc(1))* abc_dot(2) / (abc(2)-1);
      omega_b_(1) = cos(des_yaw)*abc_dot(0) - sin(des_yaw)*abc_dot(1) -
              (cos(des_yaw)*abc(0) - sin(des_yaw)*abc(1))* abc_dot(2) / (abc(2)-1);
      omega_b_(2) = (abc(1)*abc_dot(0) - abc(0)*abc_dot(1)) / (-1+abc(2)) + temp_yaw;

      temp(0) = -abc(1);
      temp(1) = 1 - abc(2);
      temp(2) = 0;
      temp(3) = abc(0);
      q_abc = (1/ sqrt(2*temp(1)))* temp;
      q_yaw(0) = cos(temp_yaw/2);
      q_yaw(1) = 0;
      q_yaw(2) =0;
      q_yaw(3) = sin(temp_yaw/2);
  }
  R_q(0) = q_abc(1)*q_yaw(0) + q_abc(0)*q_yaw(1) - q_abc(3)*q_yaw(2) + q_abc(2)*q_yaw(3);   // x
  R_q(1) = q_abc(2)*q_yaw(0) + q_abc(3)*q_yaw(1) + q_abc(0)*q_yaw(2) - q_abc(1)*q_yaw(3);   // y
  R_q(2) = q_abc(3)*q_yaw(0) - q_abc(2)*q_yaw(1) + q_abc(1)*q_yaw(2) + q_abc(0)*q_yaw(3);   // z
  R_q(3) = q_abc(0)*q_yaw(0) - q_abc(1)*q_yaw(1) - q_abc(2)*q_yaw(2) - q_abc(3)*q_yaw(3);   // w

  /*
  for(int i=0;i<4;i++)
      std::cout<< "R_q("<<i<<")   "<<R_q(i)<<std::endl;
*/
  orientation_ = Eigen::Quaterniond(R_q);

}

const Eigen::Vector3d& HFControl::getComputedForce(void)
{
  return force_;
}
const Eigen::Quaterniond& HFControl::getComputedOrientation(void)
{
  return orientation_;
}
const Eigen::Vector3d& HFControl::getComputedOmega_b(void)
{
  return omega_b_;
}

/*
void HFControl::calculateExternalForce(double dt)
{
    Eigen::Vector3d z(0,0,0), dz(0,0,0);
    dz = -L_ * last_z_ + L_ * (L_ * vel_ + force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1));

    z = last_z_ + dz * dt;

    ext_force_acc_ = (last_z_ + L_ * vel_)/mass_;

    last_z_ = z;
}
*/

