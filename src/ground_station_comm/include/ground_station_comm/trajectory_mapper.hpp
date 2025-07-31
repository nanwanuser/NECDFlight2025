/**
******************************************************************************
  * @file    trajectory_mapper.hpp
  * @brief   轨迹映射器 - 将网格标号映射到ROS坐标系
  * @version V1.0.0
  * @date    2025-07-31
  ******************************************************************************
  */

#ifndef TRAJECTORY_MAPPER_HPP
#define TRAJECTORY_MAPPER_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <cstdint>

namespace GroundStation {

    /**
     * @brief 轨迹映射器类
     * 将地面站发送的网格标号映射到ROS坐标系中的实际位置
     */
    class TrajectoryMapper {
    private:
        static constexpr double GRID_SIZE = 0.5;  // 网格大小（米）
        static constexpr double HEIGHT = 1.2;     // 固定高度（米）
        static constexpr int GRID_WIDTH = 9;      // 网格宽度
        static constexpr int GRID_HEIGHT = 7;     // 网格高度

        std::vector<geometry_msgs::PoseStamped> waypoints_;  // 航点列表

    public:
        TrajectoryMapper() = default;
        ~TrajectoryMapper() = default;

        /**
         * @brief 将网格标号映射到ROS坐标
         * @param grid_number 网格标号(1-63)
         * @param pose 输出的位姿
         * @return 映射是否成功
         */
        bool gridNumberToPose(uint8_t grid_number, geometry_msgs::PoseStamped& pose);

        /**
         * @brief 解析轨迹数据
         * @param data 轨迹数据（网格标号数组）
         * @param len 数据长度
         * @return 解析得到的航点列表
         */
        std::vector<geometry_msgs::PoseStamped> parseTrajectory(const uint8_t* data, uint16_t len);

        /**
         * @brief 清空航点列表
         */
        void clearWaypoints() { waypoints_.clear(); }

        /**
         * @brief 获取航点列表
         */
        const std::vector<geometry_msgs::PoseStamped>& getWaypoints() const { return waypoints_; }

        /**
         * @brief 根据当前位置找到当前所在的网格标号
         * @param position 当前位置
         * @return 网格标号(1-63)，如果不在网格内返回0
         */
        uint8_t positionToGridNumber(const geometry_msgs::Point& position);

        /**
         * @brief 获取网格中心位置
         * @param grid_number 网格标号
         * @return 网格中心位置
         */
        geometry_msgs::Point getGridCenter(uint8_t grid_number);

        /**
         * @brief 检查位置是否接近网格边界
         * @param position 当前位置
         * @param threshold 边界阈值（米）
         * @return 是否接近边界
         */
        bool isNearGridBoundary(const geometry_msgs::Point& position, double threshold);

        /**
         * @brief 获取网格大小
         */
        static double getGridSize() { return GRID_SIZE; }
    };

} // namespace GroundStation

#endif /* TRAJECTORY_MAPPER_HPP */