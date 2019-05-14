#pragma once

#include "robot_model/humanoid_model.h"
#include <zmq.hpp>

namespace rhoban
{
class HumanoidServer
{
public:
  HumanoidServer();
  void start();
  void publishModel(rhoban::HumanoidModel& model, bool flatFoot = true);

  // ZeroMQ PUB/SUB
  zmq::context_t context;
  zmq::socket_t socket;
  bool serverStarted;
};
}  // namespace rhoban