cmake_minimum_required(VERSION 3.16.3)
project(robot_model)

find_package(OpenCV 4.2.0 REQUIRED)

set (SOURCES
  src/robot_model/robot_model.cpp
  src/robot_model/humanoid_model.cpp
  src/robot_model/humanoid_server.cpp
  src/robot_model/camera_model.cpp
  src/robot_model/leg_ik.cpp
)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

add_library(robot_model ${SOURCES})
target_link_libraries(robot_model PUBLIC 
  rbdl_urdfreader 
  cppzmq
  rhoban_geometry 
  sigmaban_model
  ${OpenCV_LIBS}
)

target_include_directories(robot_model PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ./include
)

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
