cmake_minimum_required(VERSION 2.8)
project(robot_model)

find_package(catkin REQUIRED COMPONENTS
  rbdl_urdfreader
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  CATKIN_DEPENDS rbdl_urdfreader
)

set (SOURCES
  src/robot_model/robot_model.cpp
)

add_library(robot_model ${SOURCES})
target_link_libraries(robot_model ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ./include)

add_executable(RobotModel src/main.cpp)
target_link_libraries(RobotModel robot_model)