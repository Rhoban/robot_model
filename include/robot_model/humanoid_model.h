#pragma once

#include <iostream>

#include "robot_model/robot_model.h"
#include "robot_model/leg_ik.h"
#include "rhoban_utils/history/history.h"

namespace rhoban
{
void makeParallelToFloor(Eigen::Affine3d& frame);

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
  bool computeLegIK(std::map<std::string, double>& angles, Side side, const Eigen::Vector3d& footPos,
                    const Eigen::Matrix3d& footRotation = Eigen::Matrix3d::Identity());

  // Set support foot
  // If updateWorldPosition is true, the supportToWorld matrix is updated, resulting in
  // odometry
  void setSupportFoot(Side side, bool updateWorldPosition = false);

  // Sets the IMU matrix
  void updateImu();
  void setImu(bool present, double yaw = 0, double pitch = 0, double roll = 0);

  // Flying foot frame, flatenned on the ground in the
  Eigen::Affine3d flyingFootFlattenedToWorld();

  // Getting given frame to world
  Eigen::Affine3d frameToWorld(const std::string& frame, bool flatFoot = true);

  // Gets the self frame in world
  // This frame is the weighted average of the support foot and the flatenned flying foot
  Eigen::Affine3d selfToWorld();

  // Get the pan and tilt target for the camera to look at a position target
  bool cameraLookAt(double& panDOF, double& tiltDOF, const Eigen::Vector3d& posTarget);

  // Current support foot
  Side supportFoot;

  // World to support foot, flat on the ground
  Eigen::Affine3d supportToWorld;

  // World to support foot, including IMU pitch/roll
  Eigen::Affine3d supportToWorldPitchRoll;

  // Name of expected DOFs and frames in robot
  std::vector<std::string> dofNames;
  std::vector<std::string> legDofNames;
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

  // Head yaw to pitch delta Z
  double distHeadYawToPitchZ;
  double distHeadPitchToCameraZ;
  double distHeadPitchToCameraX;

  // Updates the supportToWorld and supportToWorldPitchRoll to match the given trunkFrame
  // If forceOnFloor is true, the robot will be put back on the floor (the delta z between support foot and
  // the floor is zero'd)
  void setPositionFromFrame(const std::string& frame, const Eigen::Affine3d& frameToWorld, bool forceOnFloor = true);

  // Update the current model, reading values from histories logged
  void readFromHistories(rhoban_utils::HistoryCollection& histories, double timestamp, bool readSupport = true);

  // Computes the 12x12 Jacobian matrix, in which all the following DOFs will be diff'd with
  // respect to the following:
  // - Position of CoM
  // - Orientation of frame
  // - Position of flyingFrame
  // - Orientation of flyingFrame
  // Everything is expressed in RBDL origin frame
  // Typically, this can be used to control the robot CoM, the trunk orientation, along with the flying foot
  // position and orientation
  Eigen::Matrix<double, 12, 12> computeJacobian(const std::string& frame, const std::string& orientationFrame,
                                                const std::string& flyingFrame);

  // Computes the error to reach the given com frame (position of the com frame and orientation of the trunk)
  // and flyingFrame
  Eigen::Matrix<double, 12, 1> getError(const std::string& frame, Eigen::Affine3d com, const std::string& flyingFrame,
                                        Eigen::Affine3d flying, const std::string& orientationFrame = "trunk");

protected:
  // Leg IK
  rhoban_leg_ik::IK* legIK;

  // Has IMU constraint
  bool hasImu;
  double imuYaw, imuPitch, imuRoll;
};
}  // namespace rhoban