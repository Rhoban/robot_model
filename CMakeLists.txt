cmake_minimum_required(VERSION 2.8)
project(robot_model)

find_package(catkin REQUIRED COMPONENTS
  rbdl_urdfreader ZeroMQ rhoban_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  CATKIN_DEPENDS rbdl_urdfreader ZeroMQ rhoban_utils
)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/humanoid_model.proto
)

set (SOURCES
  src/robot_model/robot_model.cpp
  src/robot_model/humanoid_model.cpp
  src/robot_model/humanoid_server.cpp
  src/robot_model/leg_ik.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS} 
)

add_library(robot_model ${SOURCES})
target_link_libraries(robot_model ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ./include)

add_executable(RobotModel src/main.cpp)
target_link_libraries(RobotModel robot_model)