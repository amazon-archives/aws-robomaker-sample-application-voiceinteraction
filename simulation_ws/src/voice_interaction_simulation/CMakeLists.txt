cmake_minimum_required(VERSION 2.8.3)
project(voice_interaction_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  turtlebot3_description
)

catkin_package(DEPENDS gazebo_ros)

include_directories(${catkin_INCLUDE_DIRS})

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Copy the rviz model for easier access in B9 RViz
install(FILES ${turtlebot3_description_DIR}/../rviz/model.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  RENAME turtlebot3_model.rviz
)
