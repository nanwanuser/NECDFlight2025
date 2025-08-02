/**
  ******************************************************************************
  * @file    trajectory_mapper.cpp
  * @brief   轨迹映射器实现
  * @version V1.2.1
  * @date    2025-08-01
  ******************************************************************************
  */

#include "ground_station_comm/trajectory_mapper.hpp"
#include <ros/ros.h>
#include <cmath>

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
 * 标号1在原点(0,0,0.9)，标号2在(0,0.5,0.9)，标号9在(0,4,0.9)，标号55在(3,0,0.9)
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
    double z = HEIGHT;  // 固定高度0.9米

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

    ROS_INFO("Parsed trajectory with %zu waypoints (height=%.1fm)", waypoints_.size(), HEIGHT);

    return waypoints_;
}

/**
 * @brief 根据当前位置找到当前所在的网格标号
 */
uint8_t TrajectoryMapper::positionToGridNumber(const geometry_msgs::Point& position) {
    // 检查是否在网格范围内
    // 修正：使用正确的边界值
    // x范围：0 到 3.5米（7个0.5米的格子）
    // y范围：0 到 4.5米（9个0.5米的格子）
    if (position.x < 0 || position.x >= GRID_HEIGHT * GRID_SIZE ||
        position.y < 0 || position.y >= GRID_WIDTH * GRID_SIZE) {
        ROS_DEBUG("Position (%.2f, %.2f) is outside grid bounds [0, %.1f] x [0, %.1f]",
                  position.x, position.y,
                  GRID_HEIGHT * GRID_SIZE, GRID_WIDTH * GRID_SIZE);
        return 0;  // 不在网格内
    }

    // 计算行列索引
    int row = static_cast<int>(position.x / GRID_SIZE);
    int col = static_cast<int>(position.y / GRID_SIZE);

    // 边界检查（防止浮点数精度问题导致的越界）
    if (row >= GRID_HEIGHT) row = GRID_HEIGHT - 1;
    if (col >= GRID_WIDTH) col = GRID_WIDTH - 1;
    if (row < 0) row = 0;
    if (col < 0) col = 0;

    // 计算网格标号
    uint8_t grid_number = static_cast<uint8_t>(row * GRID_WIDTH + col + 1);

    ROS_DEBUG("Position (%.2f, %.2f) -> Grid %d (row=%d, col=%d)",
              position.x, position.y, grid_number, row, col);

    return grid_number;
}

/**
 * @brief 获取网格中心位置
 */
geometry_msgs::Point TrajectoryMapper::getGridCenter(uint8_t grid_number) {
    geometry_msgs::Point center;

    if (grid_number < 1 || grid_number > 63) {
        center.x = center.y = center.z = 0;
        return center;
    }

    int index = grid_number - 1;
    int row = index / GRID_WIDTH;
    int col = index % GRID_WIDTH;

    // 网格中心位置
    center.x = row * GRID_SIZE + GRID_SIZE / 2.0;
    center.y = col * GRID_SIZE + GRID_SIZE / 2.0;
    center.z = HEIGHT;  // 固定高度0.9米

    return center;
}

/**
 * @brief 检查位置是否接近网格边界
 */
bool TrajectoryMapper::isNearGridBoundary(const geometry_msgs::Point& position, double threshold) {
    // 获取当前网格的边界
    double x_mod = fmod(position.x, GRID_SIZE);
    double y_mod = fmod(position.y, GRID_SIZE);

    // 检查是否接近任何边界
    return (x_mod < threshold || x_mod > (GRID_SIZE - threshold) ||
            y_mod < threshold || y_mod > (GRID_SIZE - threshold));
}

} // namespace GroundStation