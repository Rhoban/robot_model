#include <iostream>
#include "robot_model/robot_model.h"

int main()
{
  rhoban::RobotModel robot;

  // Printing robot DOFs
  std::cout << "Robot DOFs: ";
  for (auto entry : robot.getDofNames()) {
    std::cout << entry << " ";
  }
  std::cout << std::endl;

  // Turning left hip yaw and printing left foot orientation in trunk frame
  // robot.resetDofs();
  // for (double alpha = 0; alpha < M_PI / 2; alpha += 0.1)
  // {
  //   robot.setDof("left_hip_yaw", alpha);
  //   std::cout << robot.orientationYaw("left_foot", "trunk") << std::endl;
  // }

  // std::cout << robot.position("left_knee_1", "trunk") << std::endl;

  std::cout << robot.position("left_foot", "trunk") << std::endl;
  std::cout << robot.jointPosition("left_ankle_pitch", "trunk") << std::endl;
}