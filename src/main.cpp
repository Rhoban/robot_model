#include <unistd.h>
#include <iostream>
#include "robot_model/humanoid_model.h"

int main()
{
  rhoban::HumanoidModel robot;

  robot.startServer();

  robot.worldToRobot = robot.worldToRobot * Eigen::Translation3d(0.5, 0, -0.5);

  double t = 0;
  while (true)
  {
    usleep(10000);
    t += 0.01;
    robot.setDof("left_knee", sin(t));
    robot.setDof("left_hip_roll", sin(t));
    robot.setDof("left_ankle_pitch", sin(t));
    robot.worldToRobot = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ())*robot.worldToRobot;
    robot.publishModel();
  }
}