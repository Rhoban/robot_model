#pragma once

#include <iostream>
#include <map>
#include <rbdl/rbdl.h>

namespace rhoban
{
double frameYaw(Eigen::Matrix3d rotation);

class RobotModel
{
public:
  RobotModel(std::string filename = "robot.urdf");
  virtual ~RobotModel();

  // Checks if a dof is present in the model
  bool hasDof(const std::string& name);

  // Return all DOF names
  std::vector<std::string> getDofNames(bool include_frames = false);

  // Is this name a frame DOF ?
  bool isFrameDof(const std::string& name);

  // Sets the value of a dof [rad]
  void setDof(const std::string& name, double value);
  void setDofs(const std::map<std::string, double> angles);
  double getDof(const std::string& name);

  // Reset all DOFs to 0 position
  void resetDofs();

  // Gets the id of a RBDL body, raise exception if not found
  unsigned int getBodyId(const std::string& name);
  unsigned int getJointId(const std::string& name);

  // Gets the position transformation from frame "src" to frame "dest"
  virtual Eigen::Vector3d position(const std::string& srcFrame, const std::string& dstFrame,
                                   const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
  Eigen::Vector3d jointPosition(const std::string& jointName, const std::string& frame);

  // Gets the orientation of frame src in frame dst
  virtual Eigen::Matrix3d orientation(const std::string& srcFrame, const std::string& dstFrame);
  double orientationYaw(const std::string& srcFrame, const std::string& dstFrame);

  // Gets the robot center of mass in the given frame
  Eigen::Vector3d centerOfMass(const std::string& frame = "origin");

  // Gets the transformation from frame src to frame dst
  Eigen::Affine3d transformation(const std::string& srcFrame, const std::string& dstFrame);

  RigidBodyDynamics::Model model;

  // Degrees of freedom
  RigidBodyDynamics::Math::VectorNd dofs;
  std::map<std::string, int> dofToId;

  // Body name aliases
  std::map<std::string, std::string> bodyAliases;

  void update();

  bool isDirty;
};
}  // namespace rhoban