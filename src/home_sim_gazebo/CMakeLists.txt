cmake_minimum_required(VERSION 3.8)
project(home_sim_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_adapter
  scripts/ground_truth_odom
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
