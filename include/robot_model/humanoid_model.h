#pragma once

#include <iostream>

#include "robot_model/robot_model.h"
#include "robot_model/leg_ik.h"

namespace rhoban
{
class HumanoidModel : public RobotModel
{
public:
  HumanoidModel(std::string filename = "robot.urdf");
  virtual ~HumanoidModel();

  enum Side
  {
    Left = 0,
    Right
  };

  // Resets the world frame
  void resetWorldFrame();

  // Compute the leg IK and update the model accordingly
  // Target is in trunk frame
  // Output will be set in the reference map
  bool computeLegIK(std::map<std::string, double> &angles, Side side, const Eigen::Vector3d& footPos,
                    const Eigen::Matrix3d& footRotation = Eigen::Matrix3d::Identity());

  // Set support foot
  // If updateWorldPosition is true, the supportToWorld matrix is updated, resulting in
  // odometry
  void setSupportFoot(Side side, bool updateWorldPosition = false);

  // Sets the IMU matrix
  void updateImu();
  void setImu(bool present, double yaw = 0, double pitch = 0, double roll = 0);

  // Getting given frame to world
  Eigen::Affine3d frameToWorld(const std::string& frame, bool flatFoot = true);

  // Current support foot
  Side supportFoot;

  // World to support foot, flat on the ground
  Eigen::Affine3d supportToWorld;

  // World to support foot, including IMU pitch/roll
  Eigen::Affine3d supportToWorldPitchRoll;

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
  rhoban_leg_ik::IK* legIK;

  // Has IMU constraint
  bool hasImu;
  double imuYaw, imuPitch, imuRoll;
};
}  // namespace rhoban