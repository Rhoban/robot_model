#include <unistd.h>
#include <iostream>
#include "robot_model/humanoid_model.h"
#include "rhoban_utils/timing/benchmark.h"

int main()
{
  rhoban::HumanoidModel robot;

  robot.startServer();

  rhoban_utils::Benchmark b(NULL, "robot_model");

  double t = 0;
  while (true)
  {
    usleep(10000);
    t += 0.01;

    robot.setDof("left_hip_pitch", sin(t));
    robot.setImu(0.8, 0, 0);

    robot.publishModel();
  }
}