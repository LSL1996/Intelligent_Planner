#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/SO3Command.h>
#include <quadrotor_simulator/Quadrotor.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_utils/geometry_utils.h>

typedef struct _Control
{
  double rpm[4];
} Control;

typedef struct _Command
{
  float force[3];
  float qx, qy, qz, qw;
  float w_bx, w_by, w_bz;
  float kR[3];
  float kOm[3];
  float corrections[3];
  float current_yaw;
  bool  use_external_yaw;
} Command;

typedef struct _Disturbance
{
  Eigen::Vector3d f;
  Eigen::Vector3d m;
} Disturbance;

static Command     command;
static Disturbance disturbance;
Eigen::Vector3d last_z_;
Eigen::Vector3d ext_moment_;
double L_ = 0.3;

void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state,nav_msgs::Odometry& odom);
void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu&  imu);
void calculateExternalTorque(Eigen::Vector3d I, Eigen::Vector3d w, Eigen::Vector3d torque, double dt);
void extTorque();

/// 姿态环控制 返回电机的转速
static Control getControl(const QuadrotorSimulator::Quadrotor& quad, const Command& cmd)
{
  const double _kf = quad.getPropellerThrustCoefficient();
  const double _km = quad.getPropellerMomentCoefficient();
  const double kf  = _kf - cmd.corrections[0];
  const double km  = _km / _kf * kf;

  const double          d       = quad.getArmLength();
  const Eigen::Matrix3f J       = quad.getInertia().cast<float>();
  const float           I[3][3] = { { J(0, 0), J(0, 1), J(0, 2) },
                          { J(1, 0), J(1, 1), J(1, 2) },
                          { J(2, 0), J(2, 1), J(2, 2) } };
  const QuadrotorSimulator::Quadrotor::State state = quad.getState();

  // Rotation, may use external yaw
  Eigen::Vector3d _ypr = uav_utils::R_to_ypr(state.R);   ///
  Eigen::Vector3d ypr  = _ypr;
  if (cmd.use_external_yaw)
    ypr[0] = cmd.current_yaw;

  /// 实际的旋转矩阵
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX());   /// 从角度值获得旋转矩阵  注意乘的顺序
  float R11 = R(0, 0);
  float R12 = R(0, 1);
  float R13 = R(0, 2);
  float R21 = R(1, 0);
  float R22 = R(1, 1);
  float R23 = R(1, 2);
  float R31 = R(2, 0);
  float R32 = R(2, 1);
  float R33 = R(2, 2);

  /*
    float R11 = state.R(0,0);
    float R12 = state.R(0,1);
    float R13 = state.R(0,2);
    float R21 = state.R(1,0);
    float R22 = state.R(1,1);
    float R23 = state.R(1,2);
    float R31 = state.R(2,0);
    float R32 = state.R(2,1);
    float R33 = state.R(2,2);
  */

  float Om1 = state.omega(0);  /// 机体系下的实际角速度
  float Om2 = state.omega(1);
  float Om3 = state.omega(2);

  /// 期望的旋转矩阵
  float Rd11 = cmd.qw * cmd.qw + cmd.qx * cmd.qx - cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd12 = 2 * (cmd.qx * cmd.qy - cmd.qw * cmd.qz);
  float Rd13 = 2 * (cmd.qx * cmd.qz + cmd.qw * cmd.qy);
  float Rd21 = 2 * (cmd.qx * cmd.qy + cmd.qw * cmd.qz);
  float Rd22 = cmd.qw * cmd.qw - cmd.qx * cmd.qx + cmd.qy * cmd.qy - cmd.qz * cmd.qz;
  float Rd23 = 2 * (cmd.qy * cmd.qz - cmd.qw * cmd.qx);
  float Rd31 = 2 * (cmd.qx * cmd.qz - cmd.qw * cmd.qy);
  float Rd32 = 2 * (cmd.qy * cmd.qz + cmd.qw * cmd.qx);
  float Rd33 = cmd.qw * cmd.qw - cmd.qx * cmd.qx - cmd.qy * cmd.qy + cmd.qz * cmd.qz;
  /*
 //------------------------------------------------//
    std::cout<< "Rd11  "<< Rd11<< std::endl;
    std::cout<< "Rd12  "<< Rd12<< std::endl;
    std::cout<< "Rd13  "<< Rd13<< std::endl;
    std::cout<< "Rd21  "<< Rd21<< std::endl;
    std::cout<< "Rd22  "<< Rd22<< std::endl;
    std::cout<< "Rd23  "<< Rd23<< std::endl;
    std::cout<< "Rd31  "<< Rd31<< std::endl;
    std::cout<< "Rd32  "<< Rd32<< std::endl;
    std::cout<< "Rd33  "<< Rd33<< std::endl;
  //-----------------------------------------------//
   */
  float Psi = 0.5f * (3.0f - (Rd11 * R11 + Rd21 * R21 + Rd31 * R31 +
                              Rd12 * R12 + Rd22 * R22 + Rd32 * R32 +
                              Rd13 * R13 + Rd23 * R23 + Rd33 * R33));

  float force = 0;
  if (Psi < 1.0f)   /// Position control stability guaranteed only when Psi < 1
    force = cmd.force[0] * R13 + cmd.force[1] * R23 + cmd.force[2] * R33;

  /// 旋转矩阵误差
  float eR1 = 0.5f * (R12 * Rd13 - R13 * Rd12 + R22 * Rd23 - R23 * Rd22 +
                      R32 * Rd33 - R33 * Rd32);
  float eR2 = 0.5f * (R13 * Rd11 - R11 * Rd13 - R21 * Rd23 + R23 * Rd21 -
                      R31 * Rd33 + R33 * Rd31);
  float eR3 = 0.5f * (R11 * Rd12 - R12 * Rd11 + R21 * Rd22 - R22 * Rd21 +
                      R31 * Rd32 - R32 * Rd31);
  /*
    std::cout<< "eR1  "<< eR1<< std::endl;
    std::cout<< "eR2  "<< eR2<< std::endl;
    std::cout<< "eR3  "<< eR3<< std::endl;
*/

  float eOm1 = -command.w_bx + Om1;
  float eOm2 = -command.w_bx + Om2;
  float eOm3 = -command.w_bx + Om3;

  //float eOm1 = Om1;
  //float eOm2 = Om2;
  //float eOm3 = Om3;
   /*
    std::cout<< "eOm1  "<< eOm1<< std::endl;
    std::cout<< "eOm2  "<< eOm2<< std::endl;
    std::cout<< "eOm3  "<< eOm3<< std::endl;
    */

  float in1 = Om2 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3) -
              Om3 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3);
  float in2 = Om3 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3) -
              Om1 * (I[2][0] * Om1 + I[2][1] * Om2 + I[2][2] * Om3);
  float in3 = Om1 * (I[1][0] * Om1 + I[1][1] * Om2 + I[1][2] * Om3) -
              Om2 * (I[0][0] * Om1 + I[0][1] * Om2 + I[0][2] * Om3);


    // ----------------------------------------------------------//
   // std::cout<< "in1  "<< in1<< std::endl;
   // std::cout<< "in2  "<< in2<< std::endl;
   // std::cout<< "in3  "<< in3<< std::endl;

    // ----------------------------------------------------------//

  /*
    // Robust Control --------------------------------------------
    float c2       = 0.6;
    float epsilonR = 0.04;
    float deltaR   = 0.1;
    float eA1 = eOm1 + c2 * 1.0/I[0][0] * eR1;
    float eA2 = eOm2 + c2 * 1.0/I[1][1] * eR2;
    float eA3 = eOm3 + c2 * 1.0/I[2][2] * eR3;
    float neA = sqrt(eA1*eA1 + eA2*eA2 + eA3*eA3);
    float muR1 = -deltaR*deltaR * eA1 / (deltaR * neA + epsilonR);
    float muR2 = -deltaR*deltaR * eA2 / (deltaR * neA + epsilonR);
    float muR3 = -deltaR*deltaR * eA3 / (deltaR * neA + epsilonR);
    // Robust Control --------------------------------------------
  */
  float M1 = -cmd.kR[0] * eR1 - cmd.kOm[0] * eOm1 + in1 - ext_moment_(0); // - I[0][0]*muR1;
  float M2 = -cmd.kR[1] * eR2 - cmd.kOm[1] * eOm2 + in2 - ext_moment_(1);// - I[1][1]*muR2;
  float M3 = -cmd.kR[2] * eR3 - cmd.kOm[2] * eOm3 + in3 - ext_moment_(2);// - I[2][2]*muR3;
    // ----------------------------------------------------------//
       // std::cout<< "M1  "<< M1<< std::endl;
      //  std::cout<< "M2  "<< M2<< std::endl;
      //  std::cout<< "M3  "<< M3<< std::endl;

    // ----------------------------------------------------------//

    ///------------------------------------------------------------///
    Eigen::Vector3d II(J(0,0),J(1,1),J(2,2));
    Eigen::Vector3d torque(M1,M2,M3);
    calculateExternalTorque(II,state.omega,torque,0.01);   /// 计算外力矩
    ext_moment_ << 0,0,0;
    extTorque();  /// 发布外力矩信息
    //std::cout << ext_moment_ << std::endl;

    ///-----------------------------------------------------------///

    /// 由期望的力和力矩反解电机转速
  float w_sq[4];
  w_sq[0] = force / (4 * kf) - M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[1] = force / (4 * kf) + M2 / (2 * d * kf) + M3 / (4 * km);
  w_sq[2] = force / (4 * kf) + M1 / (2 * d * kf) - M3 / (4 * km);
  w_sq[3] = force / (4 * kf) - M1 / (2 * d * kf) - M3 / (4 * km);

  // ----------------------------------------------------------//
  //for(int i=0;i<3;i++)
      //  std::cout<< "w_sq["<<i<<"]  "<< w_sq[i]<< std::endl;
  // ----------------------------------------------------------//
  Control control;
  for (int i = 0; i < 4; i++)
  {
    if (w_sq[i] < 0)
      w_sq[i] = 0;

    control.rpm[i] = sqrtf(w_sq[i]);
  }
  return control;
}

///
static void cmd_callback(const quadrotor_msgs::SO3Command::ConstPtr& cmd)
{
  command.force[0]         = cmd->force.x;   /// 是从控制器的位置控制环而来总升力
  command.force[1]         = cmd->force.y;
  command.force[2]         = cmd->force.z;

  command.qx               = cmd->orientation.x;   /// 是从控制器的位置控制环而来的期望的旋转矩阵
  command.qy               = cmd->orientation.y;
  command.qz               = cmd->orientation.z;
  command.qw               = cmd->orientation.w;

  command.w_bx             = cmd->omega_b.x;
  command.w_by             = cmd->omega_b.y;
  command.w_bz             = cmd->omega_b.z;

  command.kR[0]            = cmd->kR[0];
  command.kR[1]            = cmd->kR[1];
  command.kR[2]            = cmd->kR[2];

  command.kOm[0]           = cmd->kOm[0];
  command.kOm[1]           = cmd->kOm[1];
  command.kOm[2]           = cmd->kOm[2];

  command.corrections[0]   = cmd->aux.kf_correction;
  command.corrections[1]   = cmd->aux.angle_corrections[0];
  command.corrections[2]   = cmd->aux.angle_corrections[1];

  command.current_yaw      = cmd->aux.current_yaw;
  command.use_external_yaw = cmd->aux.use_external_yaw;

}

void calculateExternalTorque(Eigen::Vector3d I, Eigen::Vector3d w, Eigen::Vector3d torque, double dt)
{
    Eigen::Vector3d z(0,0,0), dz(0,0,0);
    Eigen::Vector3d temp;
    temp(0) = (torque(0) - (w(1) * I(2) * w(2) - w(2) * I(1) * w(1)))/I(0);
    temp(1) = (torque(1) - (w(2) * I(0) * w(0) - w(0) * I(2) * w(2)))/I(1);
    temp(2) = (torque(2) - (w(0) * I(1) * w(1) - w(2) * I(0) * w(0)))/I(2);

    dz(0) = -L_ * last_z_(0)/I(0) - L_ * (L_ * w(0)/I(0) + temp(0));
    dz(1) = -L_ * last_z_(1)/I(1) - L_ * (L_ * w(1)/I(1) + temp(1));
    dz(2) = -L_ * last_z_(2)/I(2) - L_ * (L_ * w(2)/I(2) + temp(2));

    z = last_z_ + dz * dt;

    ext_moment_ = last_z_ + L_ * w;

    last_z_ = z;
}
///
void force_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& f)
{
  disturbance.f(0) = f->x;
  disturbance.f(1) = f->y;
  disturbance.f(2) = f->z;
  //std::cout<< "disturbance.f\n"<< disturbance.f <<std::endl;
}

///
static void moment_disturbance_callback(const geometry_msgs::Vector3::ConstPtr& m)
{
  disturbance.m(0) = m->x;
  disturbance.m(1) = m->y;
  disturbance.m(2) = m->z;
}

void extTorque()
{
    disturbance.m(0) = ext_moment_(0);
    disturbance.m(1) = ext_moment_(1);
    disturbance.m(2) = ext_moment_(2);
}

///
int main(int argc, char** argv)
{
  ros::init(argc, argv, "quadrotor_simulator_so3");

  ros::NodeHandle n("~");

  ros::Publisher  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);  /// 实际就是VIO
  ros::Publisher  imu_pub  = n.advertise<sensor_msgs::Imu>("imu", 10);      /// 自驾仪的imu数据

  ros::Subscriber cmd_sub  = n.subscribe("cmd", 100, &cmd_callback, ros::TransportHints().tcpNoDelay());  ///
  //ros::Subscriber f_sub    = n.subscribe("/so3_control/force_disturbance_comp", 100, &force_disturbance_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber f_sub    = n.subscribe("/planner/extforce", 100, &force_disturbance_callback, ros::TransportHints().tcpNoDelay());

  ros::Subscriber m_sub    = n.subscribe("moment_disturbance_comp", 100, &moment_disturbance_callback, ros::TransportHints().tcpNoDelay());

  ///
  QuadrotorSimulator::Quadrotor quad;
  double  _init_x, _init_y, _init_z;
  n.param("simulator/init_state_x", _init_x, 0.0);
  n.param("simulator/init_state_y", _init_y, 0.0);
  n.param("simulator/init_state_z", _init_z, 1.0);

  Eigen::Vector3d position = Eigen::Vector3d(_init_x, _init_y, _init_z);
  quad.setStatePos(position);
  ///

  double simulation_rate; /// 1000hz
  n.param("rate/simulation", simulation_rate, 1000.0);
  ROS_ASSERT(simulation_rate > 0);

  double odom_rate;
  n.param("rate/odom", odom_rate, 100.0);   /// 100hz
  const ros::Duration odom_pub_duration(1 / odom_rate);

  std::string quad_name;
  n.param("quadrotor_name", quad_name, std::string("quadrotor"));

  QuadrotorSimulator::Quadrotor::State state = quad.getState();  /// 无人机当前状态

  ros::Rate    r(simulation_rate);
  const double dt = 1 / simulation_rate;

  Control control;  ///

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "/simulator";
  odom_msg.child_frame_id  = "/" + quad_name;

  sensor_msgs::Imu imu;
  imu.header.frame_id = "/simulator";

  /*
  command.force[0] = 0;
  command.force[1] = 0;
  command.force[2] = quad.getMass()*quad.getGravity() + 0.1;
  command.qx = 0;
  command.qy = 0;
  command.qz = 0;
  command.qw = 1;
  command.kR[0] = 2;
  command.kR[1] = 2;
  command.kR[2] = 2;
  command.kOm[0] = 0.15;
  command.kOm[1] = 0.15;
  command.kOm[2] = 0.15;
  */

  ros::Time next_odom_pub_time = ros::Time::now();
  while (n.ok())
  {
    ros::spinOnce();

    auto last = control;   ///
    control   = getControl(quad, command);  ///
    for (int i = 0; i < 4; ++i)
    {
      //! @bug might have nan when the input is legal
      if (std::isnan(control.rpm[i]))
        control.rpm[i] = last.rpm[i];  /// 上一次的控制
    }
    quad.setInput(control.rpm[0], control.rpm[1], control.rpm[2],
                  control.rpm[3]);

    quad.setExternalForce(disturbance.f);    /// 通过观测器观测出的/传感器融合的
    quad.setExternalMoment(disturbance.m);
    quad.step(dt);   ///

    ros::Time tnow = ros::Time::now();

    if (tnow >= next_odom_pub_time)
    {
      next_odom_pub_time += odom_pub_duration;
      odom_msg.header.stamp = tnow;
      state = quad.getState();   ///

      stateToOdomMsg(state, odom_msg);
      quadToImuMsg(quad, imu);

      odom_pub.publish(odom_msg);
      imu_pub.publish(imu);

    }

    r.sleep();
  }

  return 0;
}

///
void stateToOdomMsg(const QuadrotorSimulator::Quadrotor::State& state, nav_msgs::Odometry& odom)
{
  odom.pose.pose.position.x = state.x(0);
  odom.pose.pose.position.y = state.x(1);
  odom.pose.pose.position.z = state.x(2);

  Eigen::Quaterniond q(state.R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = state.v(0);
  odom.twist.twist.linear.y = state.v(1);
  odom.twist.twist.linear.z = state.v(2);

  odom.twist.twist.angular.x = state.omega(0);  /// 机体系下的角速度
  odom.twist.twist.angular.y = state.omega(1);
  odom.twist.twist.angular.z = state.omega(2);
}


///
void quadToImuMsg(const QuadrotorSimulator::Quadrotor& quad, sensor_msgs::Imu& imu)
{
  QuadrotorSimulator::Quadrotor::State state = quad.getState();
  Eigen::Quaterniond  q(state.R);
  imu.orientation.x = q.x();
  imu.orientation.y = q.y();
  imu.orientation.z = q.z();
  imu.orientation.w = q.w();

  imu.angular_velocity.x = state.omega(0);
  imu.angular_velocity.y = state.omega(1);
  imu.angular_velocity.z = state.omega(2);

  imu.linear_acceleration.x = quad.getAcc()[0];
  imu.linear_acceleration.y = quad.getAcc()[1];
  imu.linear_acceleration.z = quad.getAcc()[2];
}
