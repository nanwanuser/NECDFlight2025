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
find_package(PkgConfig REQUIRED)

# 查找RKNN库（可能需要根据实际安装路径调整）
set(RKNN_API_PATH /usr/lib)
set(RKNN_RT_LIB ${RKNN_API_PATH}/librknnrt.so)

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
  include
  /usr/include/rknn  # RKNN头文件路径
)

# 添加YOLO11相关源文件
set(YOLO11_SOURCES
  src/yolo11.cc
  src/postprocess.cc
)

# 创建可执行文件
add_executable(${PROJECT_NAME}_node
  src/yolo11_ros_node.cpp
  ${YOLO11_SOURCES}
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
  ${RKNN_RT_LIB}
  pthread
  dl
)

# 设置编译选项
target_compile_definitions(${PROJECT_NAME}_node PRIVATE
  -DRKNN_API_PATH="${RKNN_API_PATH}"
)