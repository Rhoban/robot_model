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

    Eigen::Vector3d pos;
    pos.x() = 0;
    pos.y() = -robot.distFootYOffset;
    pos.z() = 0.05 * sin(t) + 0.05 - robot.distHipToGround;

    Eigen::Matrix3d rot = Eigen::AngleAxisd(sin(t) * 0.3, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();

    if (!robot.computeLegIK(rhoban::HumanoidModel::Right, pos, rot))
    {
      std::cout << "Error in IK" << std::endl;
    }

    robot.publishModel();
  }
}