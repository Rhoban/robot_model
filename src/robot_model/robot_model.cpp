#include "robot_model/robot_model.h"
#include <urdfreader/urdfreader.h>

namespace rhoban
{
double frameYaw(Eigen::Matrix3d rotation)
{
  return -atan2(rotation(0, 1), rotation(0, 0));
}

RobotModel::RobotModel(std::string filename)
{
  std::vector<std::string> jointNames;

  // Loading URDF model
  if (!RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), &model, false, NULL, NULL, false, NULL, NULL,
                                                   false, &jointNames))
  {
    throw std::runtime_error("RobotModel: unable to load URDF file: " + filename);
  }

  for (int k = 0; k < jointNames.size(); k++)
  {
    dofToId[jointNames[k]] = k;
  }

  // Initializing dofs vectors with zeros
  resetDofs();

  isDirty = true;
  bodyAliases["origin"] = "ROOT";
}

RobotModel::~RobotModel()
{
}

void RobotModel::update()
{
  if (isDirty)
  {
    isDirty = false;
    RigidBodyDynamics::UpdateKinematicsCustom(model, &dofs, nullptr, nullptr);
  }
}

bool RobotModel::hasDof(const std::string& name)
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
  isDirty = true;
}

unsigned int RobotModel::getJointId(const std::string& name)
{
  if (dofToId.count(name))
  {
    return dofToId[name];
  }
  else
  {
    std::ostringstream oss;
    oss << "RobotModel: Can't find joint with name \"" << name << "\"";

    throw std::runtime_error(oss.str());
  }
}

void RobotModel::setDof(const std::string& name, double value)
{
  isDirty = true;
  dofs[getJointId(name)] = value;
}

void RobotModel::setDofs(const std::map<std::string, double> angles)
{
  isDirty = true;
  for (auto& entry : angles)
  {
    dofs[getJointId(entry.first)] = entry.second;
  }
}

double RobotModel::getDof(const std::string& name)
{
  return dofs[getJointId(name)];
}

unsigned int RobotModel::getBodyId(const std::string& name)
{
  if (bodyAliases.count(name))
  {
    return getBodyId(bodyAliases[name]);
  }

  unsigned int id = model.GetBodyId(name.c_str());

  if (id == std::numeric_limits<unsigned int>::max())
  {
    std::ostringstream oss;
    oss << "RobotModel: asking id for unknown body \"" << name << "\"" << std::endl;

    throw std::runtime_error(oss.str());
  }

  return id;
}

Eigen::Vector3d RobotModel::position(const std::string& srcFrame, const std::string& dstFrame,
                                     const Eigen::Vector3d& point)
{
  update();

  RigidBodyDynamics::Math::Vector3d ptBase;
  RigidBodyDynamics::Math::Vector3d ptBody;

  ptBase = RigidBodyDynamics::CalcBodyToBaseCoordinates(model, dofs, getBodyId(srcFrame), point, false);
  ptBody = RigidBodyDynamics::CalcBaseToBodyCoordinates(model, dofs, getBodyId(dstFrame), ptBase, false);

  return ptBody;
}

Eigen::Vector3d RobotModel::jointPosition(const std::string& jointName, const std::string& frame)
{
  std::string jointParentBody = model.GetBodyName(getJointId(jointName) + 1);

  return position(jointParentBody, frame);
}

Eigen::Matrix3d RobotModel::orientation(const std::string& srcFrame, const std::string& dstFrame)
{
  update();

  RigidBodyDynamics::Math::Matrix3d transform1;
  transform1 = RigidBodyDynamics::CalcBodyWorldOrientation(model, dofs, getBodyId(dstFrame), false);
  RigidBodyDynamics::Math::Matrix3d transform2;
  transform2 = RigidBodyDynamics::CalcBodyWorldOrientation(model, dofs, getBodyId(srcFrame), false);

  return transform1 * transform2.transpose();
}

double RobotModel::orientationYaw(const std::string& srcFrame, const std::string& dstFrame)
{
  Eigen::Matrix3d rotation = orientation(srcFrame, dstFrame);

  return frameYaw(rotation);
}

Eigen::Affine3d RobotModel::transformation(const std::string& srcFrame, const std::string& dstFrame)
{
  Eigen::Affine3d rotation(orientation(srcFrame, dstFrame));
  Eigen::Translation3d translation(-position(dstFrame, srcFrame));

  return rotation * translation;
}

}  // namespace rhoban