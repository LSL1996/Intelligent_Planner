#ifndef __HFCONTROL_H__
#define __HFCONTROL_H__

#include <Eigen/Geometry>


class HFControl
{
public:
  HFControl();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);
  void setEnvGrad(const Eigen::Vector3d& envgrad);

  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc,
                        const Eigen::Vector3d& des_jerk,
                        const double des_yaw,
                        const double des_yaw_dot,
                        const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv);

  void calculateExternalForce(double dt);

  const Eigen::Vector3d&    getComputedForce(void);
  const Eigen::Quaterniond& getComputedOrientation(void);
  const Eigen::Vector3d&    getComputedOmega_b(void);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /// Inputs for the controller
  double          mass_;
  double          g_;
  double          L_;  /// disturbace obv gain

  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;   /// 实际的值
  Eigen::Vector3d envgrad_;

  /// Outputs of the controller
  Eigen::Vector3d    force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d    omega_b_;

public:
    Eigen::Vector3d    ext_force_acc_,ext_force_acc_comp_;
    Eigen::Vector3d    last_z_;

    /// external_force_comp_
    double beta_comp_force_,last_beta_comp_force_;
};

#endif
