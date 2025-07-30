#include "highodometry.h"
highodometry::highodometry(ros::NodeHandle& nh)
{
    imuSub = nh.subscribe("/livox/imu", 1, &highodometry::imuDtaCallback, this);
    odoSub = nh.subscribe("/Odometry", 1, &highodometry::odoDataCallback, this);
    vbagrSub = nh.subscribe("OptVars", 1, &highodometry::optvarDtatCallback, this);

    odoPub = nh.advertise<nav_msgs::Odometry>("mavros/odometry/out", 10);
}

void highodometry::imuDtaCallback(const sensor_msgs::ImuConstPtr &msg){
    imu_data = *msg;
    linear_acc << imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z;
    angular_vel << imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z;
}

void highodometry::optvarDtatCallback(const fast_lio::optimizationConstPtr &msg){
    optvar = *msg;
}

void highodometry::odoDataCallback(const nav_msgs::OdometryConstPtr &msg){
    odo_in = *msg;
}

void highodometry::pubOptData(const ros::TimerEvent& event){    
    if(odo_in.header.stamp.toSec() > imu_data.header.stamp.toSec()){
        std::cout<<"time stamp is error!"<<std::endl;
        return;
    }
    latest_P << odo_in.pose.pose.position.x, odo_in.pose.pose.position.y, odo_in.pose.pose.position.z;
    z = odo_in.pose.pose.position.z;
    latest_V << optvar.v[0], optvar.v[1], optvar.v[2];//odo_v
    latest_Ba << optvar.ba[0], optvar.ba[1], optvar.ba[2];
    latest_Bg << optvar.bg[0], optvar.bg[1], optvar.bg[2];
    g << optvar.g[0], optvar.g[1], optvar.g[2];
    
    double dt = imu_data.header.stamp.toSec() - odo_in.header.stamp.toSec();

    latest_Q = highodometry::ypr2R(Eigen::Vector3d{optvar.R[2], optvar.R[1], optvar.R[0]});

    Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (latest_gry_0 + angular_vel) - latest_Bg;//odo_w

    latest_Q = latest_Q * highodometry::deltaQ(un_gyr * dt).toRotationMatrix();

    Eigen::Vector3d un_acc_1 = latest_Q * (linear_acc - latest_Ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    latest_P(0) = latest_P(0) + dt * latest_V(0) + 0.5 * dt * dt * un_acc(0);
    latest_P(1) = latest_P(1) + dt * latest_V(1) + 0.5 * dt * dt * un_acc(1);
    //std::cout<<"latest_P02"<<std::endl<<latest_P(0)<<'\t'<<latest_P(1)<<'\t'<<latest_P(2)<<std::endl;

    z_nums.push_back(latest_P(2));
    v_nums.push_back(latest_V(2));

    if(z_nums.size()>5 && v_nums.size()>5){
        std::vector<double> v_res(v_nums.end()-6, v_nums.end()-1);
        std::vector<double> z_res(z_nums.end()-6, z_nums.end()-1);

        for(int i = 0; i < 5; i++){
            v_sum += v_res[i];
            z_sum += z_res[i]; 
        }
        v_mea = v_sum * 0.2;
        z_mea = z_sum * 0.2;

        latest_P(2) = z_mea + dt * v_mea;

        v_nums.erase(v_nums.begin());
        z_nums.erase(z_nums.begin());
        v_sum = 0;
        z_sum = 0;
    }else{
        latest_P(2) = z;
    }

    latest_acc_0 = linear_acc;
    latest_gry_0 = angular_vel;

    //smooth odometry
    latest_P = old_P * (1 - 0.62) + latest_P * 0.62;

    odoQ.x = latest_Q.coeffs()[0];
    odoQ.y = latest_Q.coeffs()[1];
    odoQ.z = latest_Q.coeffs()[2];
    odoQ.w = latest_Q.coeffs()[3];

    odoQ.x = old_odoQ.x * (1- 0.62) + odoQ.x * 0.62;
    odoQ.y = old_odoQ.y * (1- 0.62) + odoQ.y * 0.62;
    odoQ.z = old_odoQ.z * (1- 0.62) + odoQ.z * 0.62;
    odoQ.w = old_odoQ.w * (1- 0.62) + odoQ.w * 0.62;

    //publish odometry
    odo_out.pose.pose.position.x = latest_P(0);
    odo_out.pose.pose.position.y = latest_P(1);
    odo_out.pose.pose.position.z = latest_P(2);
    odo_out.pose.pose.orientation.x = odoQ.x;
    odo_out.pose.pose.orientation.y = odoQ.y;
    odo_out.pose.pose.orientation.z = odoQ.z;
    odo_out.pose.pose.orientation.w = odoQ.w;

    odo_out.twist.twist.linear.x = latest_V(0);
    odo_out.twist.twist.linear.y = latest_V(1);
    odo_out.twist.twist.linear.z = latest_V(2);
    odo_out.twist.twist.angular.z = un_gyr(2);
    odo_out.twist.twist.angular.y = un_gyr(1);
    odo_out.twist.twist.angular.x = un_gyr(0);
    odo_out.header.stamp = ros::Time::now();

    odo_out.header.frame_id = "odom";
    odo_out.child_frame_id = "base_link";
    odoPub.publish(odo_out);
    old_P = latest_P;
    old_odoQ = odoQ;


    //tf tree
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odo_out.pose.pose.orientation.x, \
                                    odo_out.pose.pose.orientation.y, \
                                    odo_out.pose.pose.orientation.z));
    q.setW(odo_out.pose.pose.orientation.w);
    q.setX(odo_out.pose.pose.orientation.x);
    q.setY(odo_out.pose.pose.orientation.y);
    q.setZ(odo_out.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odo_out.header.stamp, "odom", "base_link"));

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "highFreodo");
    ros::NodeHandle n;//ros::NodeHandle nh("~")就不会执行ros::Timer里面的东西，或者没有ros::Timer

    highodometry odometer(n);
    
    ros::Timer timer = n.createTimer(ros::Duration(0.008), &highodometry::pubOptData, &odometer);

    ros::spin();
    return 0;
}