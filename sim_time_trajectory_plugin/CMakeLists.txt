cmake_minimum_required(VERSION 3.16)
project(swarm_sim_time_trajectory_plugin LANGUAGES CXX)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(swarm_sim_time_trajectory SHARED SimTimeTrajectorySystem.cc)
target_compile_features(swarm_sim_time_trajectory PRIVATE cxx_std_17)
target_link_libraries(swarm_sim_time_trajectory PRIVATE
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
