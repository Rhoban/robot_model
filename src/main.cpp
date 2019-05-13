#include <unistd.h>
#include <iostream>
#include "robot_model/humanoid_model.h"

int main()
{
  rhoban::HumanoidModel robot;

  robot.startServer();

  double t = 0;
  while (true)
  {
    usleep(10000);
    t += 0.01;
    robot.setDof("left_knee", sin(t)*0.5);
    robot.setDof("left_hip_roll", sin(t)*0.5);
    robot.setDof("left_hip_yaw", sin(t)*0.5);
    // robot.worldToSupport = Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()) * robot.worldToSupport;
    robot.publishModel();
  }
}