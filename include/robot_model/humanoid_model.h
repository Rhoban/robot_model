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

  Eigen::Affine3d worldToRobot;

  std::vector<std::string> dofs;
  std::vector<std::string> frames;

  // ZeroMQ PUB/SUB
  zmq::context_t context;
  zmq::socket_t socket;
  bool serverStarted;
};
}  // namespace rhoban