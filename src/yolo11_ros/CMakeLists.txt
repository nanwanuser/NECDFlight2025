cmake_minimum_required(VERSION 3.10)
project(yolo11_ros)

# 添加C++14支持
add_compile_options(-std=c++14)

# 查找catkin组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  
)

# 查找OpenCV包
find_package(OpenCV REQUIRED)

# 添加消息文件
add_message_files(
  FILES
  AnimalData.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

# catkin特定配置
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    geometry_msgs 
    std_msgs 
    cv_bridge 
    image_transport 
    message_runtime
)

# 设置头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
  src/yolo11_ros_node.cpp
)

# 添加消息生成依赖
add_dependencies(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_generate_messages_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

# 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)