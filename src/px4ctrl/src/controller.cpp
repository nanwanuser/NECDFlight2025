#include "controller.h"

using namespace std;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  resetThrustMapping();
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
Px4ctrlDebug_t
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu,
    Controller_Output_t &u)
{
  /* WRITE YOUR CODE HERE */
  //compute disired acceleration
  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
  Eigen::Vector3d Kp,Kv;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  des_acc = des.a + Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);
  des_acc += Eigen::Vector3d(0,0,param_.gra);

  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  double roll,pitch;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin_yaw = std::sin(yaw_odom);
  double cos_yaw = std::cos(yaw_odom);
  roll = (des_acc(0) * sin_yaw - des_acc(1) * cos_yaw )/ param_.gra;
  pitch = (des_acc(0) * cos_yaw + des_acc(1) * sin_yaw )/ param_.gra;

  // Keep current yaw angle (no yaw control)
  double yaw_imu = fromQuaternion2yaw(imu.q);

  Eigen::Quaterniond q = Eigen::AngleAxisd(yaw_imu, Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q;

  /* WRITE YOUR CODE HERE */

  //used for debug
  debug_msg_.des_thrust = u.thrust;
  debug_msg_.u_thrust = u.thrust;
  
  // Convert quaternion to RPY for debug
  Eigen::Vector3d des_rpy = u.q.toRotationMatrix().eulerAngles(0, 1, 2);
  debug_msg_.des_q_rpy = des_rpy;
  debug_msg_.u_q_rpy = des_rpy;
  
  debug_msg_.des_acc = des_acc;
  debug_msg_.u_acc = des_acc;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage
*/
double
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);

  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}
