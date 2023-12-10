cmake_minimum_required(VERSION 3.5)
project(moveo_moveit)

set(CMAKE_C_STANDARD 99)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  Eigen3
)



################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
)

###########
## Build ##
###########

## Declare a C++ executable
add_executable(moveit_convert src/moveit_convert.cpp)
add_executable(move_group_1 src/move_group_interface_coor_1.cpp)

target_include_directories(move_group_1 PUBLIC ${EIGEN_INCLUDE_DIRS})

ament_target_dependencies(moveit_convert ${dependencies})
ament_target_dependencies(move_group_1 ${dependencies})

## Specify libraries to link a library or executable target against
install(PROGRAMS scripts/moveo_objrec_publisher.py
  DESTINATION lib)




