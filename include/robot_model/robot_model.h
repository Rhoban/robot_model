#pragma once

#include <iostream>
#include <map>
#include <rbdl/rbdl.h>

namespace rhoban
{
class RobotModel
{
public:
  RobotModel(std::string filename = "robot.urdf");

  // Checks if a dof is present in the model
  bool hasDof(const std::string &name);

  // Return all DOF names
  std::vector<std::string> getDofNames();

  // Sets the value of a dof [rad]
  void setDof(const std::string &name, double value);
  double getDof(const std::string &name);

  // Reset all DOFs to 0 position
  void resetDofs();

  // Gets the id of a RBDL body, raise exception if not found
  unsigned int getBodyId(const std::string &name);
  unsigned int getJointId(const std::string &name);

  // Gets the position transformation from frame "src" to frame "dest"
  virtual Eigen::Vector3d position(const std::string &srcFrame, const std::string &dstFrame,
                           const Eigen::Vector3d& point = Eigen::Vector3d::Zero());
  Eigen::Vector3d jointPosition(const std::string &jointName, const std::string &frame);

  // Gets the orientation of frame src in frame dst
  virtual Eigen::Matrix3d orientation(const std::string& srcFrame, const std::string& dstFrame);
  double orientationYaw(const std::string& srcFrame, const std::string& dstFrame);

  // Gets the transformation from frame src to frame dst
  Eigen::Affine3d transformation(const std::string& srcFrame, const std::string& dstFrame);

  RigidBodyDynamics::Model model;

  // Degrees of freedom
  RigidBodyDynamics::Math::VectorNd dofs;
  std::map<std::string, int> dofToId;

  // Body name aliases
  std::map<std::string, std::string> bodyAliases;
};
}  // namespace rhoban