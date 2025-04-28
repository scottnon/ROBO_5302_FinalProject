cmake_minimum_required(VERSION 3.8)
project(robot_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# Install Python nodes - add names of new python nodes here
install(PROGRAMS
  robot_car/bicycle_model_fake_odom_publisher.py
  # robot_car/new_python_node.py  -- this is the format for adding new python files (just add another line above this line)
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, URDF, RViz configs
install(
  DIRECTORY launch urdf config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
 
