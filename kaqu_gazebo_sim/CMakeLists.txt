cmake_minimum_required(VERSION 3.5)
project(kaqu_gazebo_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kaqu_msgs REQUIRED)

add_executable(Kaqu_gazebo_joint_ctrl_node src/kaqu_gazebo_joint_controller.cpp)
ament_target_dependencies(Kaqu_gazebo_joint_ctrl_node
 rclcpp std_msgs)

install(TARGETS
  Kaqu_gazebo_joint_ctrl_node 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
