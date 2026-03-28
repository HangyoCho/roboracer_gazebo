cmake_minimum_required(VERSION 3.0)
project(unicorn_model)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  ackermann_msgs
  sensor_msgs
  tf
  urdf
  xacro
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs ackermann_msgs sensor_msgs tf urdf xacro gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(ackermann_to_twist_converter_node src/ackermann_to_cmd_vel.cpp)
target_link_libraries(ackermann_to_twist_converter_node ${catkin_LIBRARIES})

install(TARGETS ackermann_to_twist_converter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/twist_to_joints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf launch rviz world meshes config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
