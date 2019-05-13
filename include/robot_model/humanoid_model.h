#pragma once

#include <iostream>
#include <map>
#include <zmq.hpp>

#include "robot_model/robot_model.h"

namespace rhoban
{
class HumanoidModel : public RobotModel
{
public:
  HumanoidModel(std::string filename = "robot.urdf");

  void startServer();
  void publishModel();

  void setSupportFoot(bool left);

  // Getting given frame to world
  Eigen::Affine3d frameToWorld(const std::string &frame);

  // World to support foot
  bool isLeftSupport;
  Eigen::Affine3d worldToSupport;

  // Name of expected DOFs and frames in robot
  std::vector<std::string> dofs;
  std::vector<std::string> frames;

  // ZeroMQ PUB/SUB
  zmq::context_t context;
  zmq::socket_t socket;
  bool serverStarted;
};
}  // namespace rhoban