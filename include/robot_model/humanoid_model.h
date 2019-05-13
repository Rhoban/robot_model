#pragma once

#include <iostream>
#include <map>
#include <zmq.hpp>

#include "robot_model/robot_model.h"
#include "robot_model/leg_ik.h"

namespace rhoban
{
class HumanoidModel : public RobotModel
{
public:
  HumanoidModel(std::string filename = "robot.urdf");
  virtual ~HumanoidModel();

  void startServer();
  void publishModel();

  enum Side
  {
    Left = 0,
    Right
  };

  // Compute the leg IK and update the model accordingly
  // Target is in trunk frame
  bool computeLegIK(Side side, const Eigen::Vector3d& footPos,
                    const Eigen::Matrix3d& footRotation = Eigen::Matrix3d::Identity());

  // Set support foot
  void setSupportFoot(Side side);

  // Getting given frame to world
  Eigen::Affine3d frameToWorld(const std::string& frame);

  // World to support foot
  Side supportFoot;
  Eigen::Affine3d worldToSupport;

  // Name of expected DOFs and frames in robot
  std::vector<std::string> dofs;
  std::vector<std::string> frames;

  // Y offset distance from trunk to leg [m]
  double distFootYOffset;

  // Z offset distance from hip joint to knee [m]
  double distHipToKnee;

  // Z offset distance from knee to ankle [m]
  double distKneeToAnkle;

  // Z offset distance from ankle to ground [m]
  double distAnkleToGround;

  // Z offset distance from hip to ground [m]
  double distHipToGround;

protected:
  // Leg IK
  LegIK::IK* legIK;

  // ZeroMQ PUB/SUB
  zmq::context_t context;
  zmq::socket_t socket;
  bool serverStarted;
};
}  // namespace rhoban