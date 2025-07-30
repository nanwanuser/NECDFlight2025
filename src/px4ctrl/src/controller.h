/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64MultiArray.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

// Simple debug structure to replace quadrotor_msgs::Px4ctrlDebug
struct Px4ctrlDebug_t
{
  double des_thrust;
  double u_thrust;
  Eigen::Vector3d des_q_rpy;
  Eigen::Vector3d u_q_rpy;
  Eigen::Vector3d des_acc;
  Eigen::Vector3d u_acc;
  
  // Convert to Float64MultiArray for publishing
  std_msgs::Float64MultiArray toMsg() const
  {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(12);
    msg.data[0] = des_thrust;
    msg.data[1] = u_thrust;
    msg.data[2] = des_q_rpy.x();
    msg.data[3] = des_q_rpy.y();
    msg.data[4] = des_q_rpy.z();
    msg.data[5] = u_q_rpy.x();
    msg.data[6] = u_q_rpy.y();
    msg.data[7] = u_q_rpy.z();
    msg.data[8] = des_acc.x();
    msg.data[9] = des_acc.y();
    msg.data[10] = des_acc.z();
    msg.data[11] = u_acc.x();
    return msg;
  }
};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class LinearControl
{
public:
	LinearControl(Parameter_t &);
Px4ctrlDebug_t calculateControl(const Desired_State_t &des,
const Odom_Data_t &odom,
const Imu_Data_t &imu,
Controller_Output_t &u);
	bool estimateThrustModel(const Eigen::Vector3d &est_a,
		const Parameter_t &param);
	void resetThrustMapping(void);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
Parameter_t param_;
Px4ctrlDebug_t debug_msg_;
std::queue<std::pair<ros::Time, double>> timed_thrust_;
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

	// Thrust-accel mapping params
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;

	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
	double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif