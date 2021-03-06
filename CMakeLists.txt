cmake_minimum_required(VERSION 2.8)
project(robot_model)

find_package(catkin REQUIRED COMPONENTS
  rbdl_urdfreader ZeroMQ rhoban_geometry
)

set(OpenCV_DIR "/usr/local/include/")
find_package(OpenCV 3.2.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_model
  CATKIN_DEPENDS rbdl_urdfreader ZeroMQ rhoban_geometry
)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS
    proto/humanoid_model.proto
)

set (SOURCES
  src/robot_model/robot_model.cpp
  src/robot_model/humanoid_model.cpp
  src/robot_model/humanoid_server.cpp
  src/robot_model/camera_model.cpp
  src/robot_model/leg_ik.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS} 
)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_library(robot_model ${SOURCES})
target_link_libraries(robot_model ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS})

include_directories(${catkin_INCLUDE_DIRS} ./include)

add_executable(RobotModel src/main.cpp)
target_link_libraries(RobotModel robot_model)

enable_testing()
 
set(TESTS
  CameraModel
  # HumanoidModel
)

if (CATKIN_ENABLE_TESTING)
  # create an executable for each test
  foreach(test ${TESTS})
    string(REPLACE "/" "_" id ${test})
    catkin_add_gtest(${id} automated_tests/${test}.cpp)
    target_link_libraries(${id} robot_model)
  endforeach(test)
endif()
