#include <unistd.h>
#include <iostream>
#include "robot_model/humanoid_model.h"
#include "robot_model/humanoid_server.h"
#include "rhoban_utils/timing/benchmark.h"

int main()
{
  rhoban::HumanoidModel robot;
  rhoban::HumanoidServer server;
  rhoban_utils::Benchmark b(NULL, "robot_model");

  robot.supportToWorld.translation().x() = 1;

  double t = 0;
  while (true)
  {
    usleep(10000);
    t += 0.01;

    std::map<std::string, double> angles;

    Eigen::Vector3d leftPos;
    leftPos.x() = sin(t)*0.1;
    leftPos.y() = robot.distFootYOffset;
    leftPos.z() = (robot.supportFoot == robot.Right ? 0.002 : 0) + 0.03 - robot.distHipToGround;
    robot.computeLegIK(angles, robot.Left, leftPos, Eigen::AngleAxisd(sin(t)*0, Eigen::Vector3d::UnitZ()).toRotationMatrix());

    Eigen::Vector3d rightPos;
    rightPos.x() = -sin(t)*0.1;
    rightPos.y() = -robot.distFootYOffset;
    rightPos.z() = (robot.supportFoot == robot.Left ? 0.002 : 0) + 0.03 - robot.distHipToGround;
    robot.computeLegIK(angles, robot.Right, rightPos, Eigen::AngleAxisd(-sin(t)*0, Eigen::Vector3d::UnitZ()).toRotationMatrix());

    robot.setDofs(angles);

    double diff = rightPos.x() - leftPos.x();
    if (robot.supportFoot == robot.Right) {
      diff = -diff;
    }
    if (diff >= 0.1) {
      robot.setSupportFoot(robot.supportFoot == robot.Left ? robot.Right : robot.Left, true);
    }

    server.publishModel(robot, true);
  }
}