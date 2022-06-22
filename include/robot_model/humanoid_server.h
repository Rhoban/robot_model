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
  void publishModel(rhoban::HumanoidModel& model, bool flatFoot = true,
                    Eigen::Affine3d correction = Eigen::Affine3d::Identity());

  void setBallPosition(Eigen::Vector3d ballPosition);
  Eigen::Vector3d ballPosition;
  bool hasBall;

  void setFieldPose(Eigen::Affine3d fieldPose);
  Eigen::Affine3d fieldPose;

  void addOpponentPosition(Eigen::Vector3d opponentPosition);
  std::vector<Eigen::Vector3d> opponentPositions;

  void addMatePose(Eigen::Affine3d matePose);
  std::vector<Eigen::Affine3d> matePoses;

  void addDebugPosition(Eigen::Vector3d debugPosition);
  std::vector<Eigen::Vector3d> debugPositions;

  // ZeroMQ PUB/SUB
  zmq::context_t context;
  zmq::socket_t socket;
  bool serverStarted;
};
}  // namespace rhoban