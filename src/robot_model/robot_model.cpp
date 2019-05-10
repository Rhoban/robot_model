#include "robot_model/robot_model.h"
#include <urdfreader/urdfreader.h>

namespace rhoban
{
RobotModel::RobotModel(std::string filename)
{
  std::vector<std::string> jointNames;

  // Loading URDF model
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), &model, false, NULL, NULL, false, NULL, NULL,
                                                   false, &jointNames))
  {
    throw std::runtime_error("RobotModel: unable to load URDF file: " + filename);
  }

  // for (int k = 0; k < model.mBodies.size(); k++)
  // {
  //   std::cout << model.GetBodyName(k) << std::endl;
  //   std::cout << model.mBodies[k].mMass << std::endl;
  //   if (k > 0) {
  //     std::cout << "#" << k << " Joint is: " << jointNames[k-1] << std::endl;
  //   }
  // }
  // std::cout << jointNames.size() << " joints" << std::endl;

  for (int k = 0; k < jointNames.size(); k++)
  {
    dofToId[jointNames[k]] = k;
  }

  // Initializing dofs vectors with zeros
  resetDofs();
}

bool RobotModel::hasDof(std::string name)
{
  return dofToId.count(name);
}

std::vector<std::string> RobotModel::getDofNames()
{
  std::vector<std::string> names;
  for (auto& entry : dofToId)
  {
    names.push_back(entry.first);
  }

  return names;
}

void RobotModel::resetDofs()
{
  dofs = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
}

unsigned int RobotModel::getJointId(std::string name)
{
  if (dofToId.count(name)) {
    return dofToId[name];
  } else {
    std::ostringstream oss;
    oss << "RobotModel: Can't find joint with name \"" << name << "\"";
    
    throw std::runtime_error(oss.str());
  }
}

void RobotModel::setDof(std::string name, double value)
{
  dofs[getJointId(name)] = value;
}

unsigned int RobotModel::getBodyId(std::string name)
{
  unsigned int id = model.GetBodyId(name.c_str());

  if (id == std::numeric_limits<unsigned int>::max())
  {
    std::ostringstream oss;
    oss << "RobotModel: asking id for unknown body \"" << name << "\"" << std::endl;

    throw std::runtime_error(oss.str());
  }

  return id;
}

Eigen::Vector3d RobotModel::position(std::string srcFrame, std::string dstFrame, const Eigen::Vector3d& point)
{
  RigidBodyDynamics::Math::Vector3d ptBase;
  RigidBodyDynamics::Math::Vector3d ptBody;

  ptBase = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, dofs, getBodyId(srcFrame), point, true);
  ptBody = RigidBodyDynamics::CalcBaseToBodyCoordinates(model, dofs, getBodyId(dstFrame), ptBase, true);

  return ptBody;
}

Eigen::Vector3d RobotModel::jointPosition(std::string jointName, std::string frame)
{
  std::string jointParentBody = model.GetBodyName(getJointId(jointName) + 1);

  return position(jointParentBody, frame);
}

Eigen::Matrix3d RobotModel::orientation(const std::string& srcFrame, const std::string& dstFrame)
{
  RigidBodyDynamics::Math::Matrix3d transform1;
  transform1 = RigidBodyDynamics::CalcBodyWorldOrientation(model, dofs, getBodyId(srcFrame), true);
  RigidBodyDynamics::Math::Matrix3d transform2;
  transform2 = RigidBodyDynamics::CalcBodyWorldOrientation(model, dofs, getBodyId(dstFrame), true);

  return transform1 * transform2.transpose();
}

double RobotModel::orientationYaw(const std::string& srcFrame, const std::string& dstFrame)
{
  Eigen::Matrix3d rotation = orientation(srcFrame, dstFrame);

  return atan2(rotation(0, 1), rotation(0, 0));
}

Eigen::Affine3d RobotModel::transformation(const std::string& srcFrame, const std::string& dstFrame)
{
  Eigen::Affine3d rotation(orientation(srcFrame, dstFrame));
  Eigen::Translation3d translation(-position(srcFrame, dstFrame));

  return rotation * translation;
}

}  // namespace rhoban