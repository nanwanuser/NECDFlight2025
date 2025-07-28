cmake_minimum_required(VERSION 3.0.2)
project(stm32_comm)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin and required components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

## Find system dependencies
# 移除 serial 依赖，因为我们自己实现了串口通信
# find_package(serial REQUIRED)
find_package(Threads REQUIRED)  # 添加线程支持

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${serial_INCLUDE_DIRS}  # 移除serial include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/data_comm_protocol.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  # ${serial_LIBRARIES}  # 移除serial库
  ${CMAKE_THREAD_LIBS_INIT}  # 添加pthread支持
)

## Declare a C++ executable
add_executable(stm32_node src/stm32_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(stm32_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link
target_link_libraries(stm32_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}  # 添加pthread支持
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS stm32_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)