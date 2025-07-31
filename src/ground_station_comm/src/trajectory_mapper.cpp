/**
  ******************************************************************************
  * @file    trajectory_mapper.cpp
  * @brief   轨迹映射器实现
  * @version V1.0.0
  * @date    2025-07-31
  ******************************************************************************
  */

#include "ground_station_comm/trajectory_mapper.hpp"
#include <ros/ros.h>

namespace GroundStation {

/**
 * @brief 将网格标号映射到ROS坐标
 * 网格布局（7行×9列）：
 *
 * 第7行: 55 56 57 58 59 60 61 62 63  (x=3.0)
 * 第6行: 46 47 48 49 50 51 52 53 54  (x=2.5)
 * 第5行: 37 38 39 40 41 42 43 44 45  (x=2.0)
 * 第4行: 28 29 30 31 32 33 34 35 36  (x=1.5)
 * 第3行: 19 20 21 22 23 24 25 26 27  (x=1.0)
 * 第2行: 10 11 12 13 14 15 16 17 18  (x=0.5)
 * 第1行: 1  2  3  4  5  6  7  8  9   (x=0.0)
 *       y=0                        y=4
 *
 * 标号1在原点(0,0,1.2)，标号2在(0,0.5,1.2)，标号9在(0,4,1.2)，标号55在(3,0,1.2)
 */
bool TrajectoryMapper::gridNumberToPose(uint8_t grid_number, geometry_msgs::PoseStamped& pose) {
    // 检查标号有效性
    if (grid_number < 1 || grid_number > 63) {
        ROS_ERROR("Invalid grid number: %d. Must be between 1 and 63.", grid_number);
        return false;
    }

    // 将标号转换为数组索引（0-62）
    int index = grid_number - 1;

    // 计算网格坐标
    int row = index / GRID_WIDTH;     // 行号（0-6），从下往上
    int col = index % GRID_WIDTH;     // 列号（0-8），从右往左

    // 根据网格布局计算实际坐标
    // x方向：row=0对应x=0，row=6对应x=3.0，间隔0.5米
    // y方向：col=0对应y=0，col=8对应y=4.0，间隔0.5米
    double x = row * GRID_SIZE;
    double y = col * GRID_SIZE;
    double z = HEIGHT;

    // 设置位姿
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // 设置姿态（默认朝向）
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    ROS_DEBUG("Grid %d (row=%d, col=%d) -> Pose: (%.2f, %.2f, %.2f)",
              grid_number, row, col, x, y, z);

    return true;
}

/**
 * @brief 解析轨迹数据
 */
std::vector<geometry_msgs::PoseStamped> TrajectoryMapper::parseTrajectory(const uint8_t* data, uint16_t len) {
    waypoints_.clear();

    // 每个网格标号占1字节
    for (uint16_t i = 0; i < len; i++) {
        geometry_msgs::PoseStamped pose;
        if (gridNumberToPose(data[i], pose)) {
            waypoints_.push_back(pose);
        } else {
            ROS_WARN("Skipping invalid grid number at index %d", i);
        }
    }

    ROS_INFO("Parsed trajectory with %zu waypoints", waypoints_.size());

    return waypoints_;
}

} // namespace GroundStation