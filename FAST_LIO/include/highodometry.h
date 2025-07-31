#ifndef _highodometry_
#define _highodometry_
#include <ros/ros.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "common_lib.h"
#include "eigen3/Eigen/Dense"
#include <cmath>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <fast_lio/optimization.h>

class highodometry
{
private:
    sensor_msgs::Imu imu_data;
    nav_msgs::Odometry odo_in, odo_out;
    fast_lio::optimization optvar;

    ros::Subscriber vbagrSub, imuSub, odoSub;

    ros::Publisher odoPub;

    Eigen::Vector3d latest_P; 
    Eigen::Vector3d old_P; 
    Eigen::Vector3d last_v, latest_V, latest_Ba, latest_Bg, g;
    Eigen::Vector3d latest_acc_0, latest_gry_0, linear_acc, angular_vel;
    Eigen::Quaterniond latest_Q;
    geometry_msgs::Quaternion odoQ, old_odoQ;
    std::vector<double>v_nums, z_nums;
    double v_sum, z_sum, z, v_mea, z_mea;
public:
    highodometry(ros::NodeHandle &nh);
    ~highodometry(){};
    void pubOptData(const ros::TimerEvent& event);

    void imuDtaCallback(const sensor_msgs::ImuConstPtr &msg);
    void odoDataCallback(const nav_msgs::OdometryConstPtr &msg);
    void optvarDtatCallback(const fast_lio::optimizationConstPtr &msg);

    template<typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta)
    {
        typedef typename Derived::Scalar Scalar_t;
        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);

        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();

        return dq;
    }

    template<typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived>&ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        Scalar_t y = ypr(0) / 180 * M_PI;
        Scalar_t p = ypr(1) / 180 * M_PI;
        Scalar_t r = ypr(2) / 180 * M_PI;

        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
              sin(y), cos(y), 0,
              0, 0, 1;

        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0.0, sin(p),
              0.0, 1.0, 0.0,
              -sin(p), 0.0, cos(p);

        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1.0, 0.0, 0.0,
              0.0, cos(r), -sin(r),
              0.0, sin(r), cos(r);
        
        return Rz * Ry * Rx;
    }
};
#endif