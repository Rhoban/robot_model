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

    Eigen::Vector3d leftPos;
    leftPos.x() = sin(t)*0.1;
    leftPos.y() = robot.distFootYOffset;
    leftPos.z() = (robot.supportFoot == robot.Right ? 0.002 : 0) + 0.03 - robot.distHipToGround;
    robot.computeLegIK(robot.Left, leftPos, Eigen::AngleAxisd(sin(t), Eigen::Vector3d::UnitZ()).toRotationMatrix());

    Eigen::Vector3d rightPos;
    rightPos.x() = -sin(t)*0.1;
    rightPos.y() = -robot.distFootYOffset;
    rightPos.z() = (robot.supportFoot == robot.Left ? 0.002 : 0) + 0.03 - robot.distHipToGround;
    robot.computeLegIK(robot.Right, rightPos, Eigen::AngleAxisd(-sin(t), Eigen::Vector3d::UnitZ()).toRotationMatrix());

    double diff = rightPos.x() - leftPos.x();
    if (robot.supportFoot == robot.Right) {
      diff = -diff;
    }
    if (diff >= 0.1) {
      robot.setSupportFoot(robot.supportFoot == robot.Left ? robot.Right : robot.Left, true);
    }

    robot.publishModel();
  }
}