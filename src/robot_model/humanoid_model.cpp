#include <algorithm>
#include "robot_model/humanoid_model.h"
#include <urdfreader/urdfreader.h>
#include "humanoid_model.pb.h"

void eigenToProtobuf(Eigen::Affine3d pose, HumanoidModelPosition* msgPosition)
{
  auto translation = pose.translation();
  msgPosition->set_x(translation.x());
  msgPosition->set_y(translation.y());
  msgPosition->set_z(translation.z());
}

void eigenToProtobuf(Eigen::Affine3d pose, HumanoidModelQuaternion* msgOrientation)
{
  auto rotation = Eigen::Quaterniond(pose.rotation());
  msgOrientation->set_qw(rotation.w());
  msgOrientation->set_qx(rotation.x());
  msgOrientation->set_qy(rotation.y());
  msgOrientation->set_qz(rotation.z());
}

void eigenToProtobuf(Eigen::Affine3d pose, HumanoidModelPose* msgPose)
{
  eigenToProtobuf(pose, msgPose->mutable_position());
  eigenToProtobuf(pose, msgPose->mutable_orientation());
}

namespace rhoban
{
HumanoidModel::HumanoidModel(std::string filename)
  : RobotModel(filename), context(1), socket(context, ZMQ_PUB), serverStarted(false), legIK(nullptr)
{
  worldToSupport = Eigen::Affine3d::Identity();
  worldToSupportPitchRoll = worldToSupport;

  // Degrees of freedom
  dofs = { "head_yaw",        "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll",
           "left_elbow",      "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
           "left_hip_yaw",    "left_hip_roll",        "left_hip_pitch",      "left_knee",
           "left_ankle_roll", "left_ankle_pitch",     "right_hip_yaw",       "right_hip_roll",
           "right_hip_pitch", "right_knee",           "right_ankle_roll",    "right_ankle_pitch" };

  // Frames
  frames = { "trunk", "left_foot", "right_foot" };

  // Sorting
  std::sort(dofs.begin(), dofs.end());
  std::sort(frames.begin(), frames.end());

  // Checking DOFs presence
  for (auto& dof : dofs)
  {
    if (!hasDof(dof))
    {
      std::ostringstream oss;
      oss << "HumanoidModel: Missing DOF: " << dof;
      throw std::runtime_error(oss.str());
    }
  }

  // Checking frames presence
  for (auto& frame : frames)
  {
    // Note: this will raise an exception if the frame is not found
    getBodyId(frame);
  }

  // Resetting DOFs
  resetDofs();

  // Measuring some distances
  distFootYOffset = fabs(jointPosition("left_hip_yaw", "trunk").y());
  distHipToKnee = fabs(jointPosition("left_hip_pitch", "trunk").z() - jointPosition("left_knee", "trunk").z());
  distKneeToAnkle = fabs(jointPosition("left_knee", "trunk").z() - jointPosition("left_ankle_pitch", "trunk").z());
  distAnkleToGround = fabs(jointPosition("left_ankle_pitch", "left_foot").z());
  distHipToGround = (distHipToKnee + distKneeToAnkle + distAnkleToGround);
  legIK = new LegIK::IK(distHipToKnee, distKneeToAnkle, distAnkleToGround);

  // Initializing right support foot
  setSupportFoot(Side::Left);
}

HumanoidModel::~HumanoidModel()
{
  if (legIK != nullptr)
  {
    delete legIK;
  }
}

bool HumanoidModel::computeLegIK(HumanoidModel::Side side, const Eigen::Vector3d& footPos,
                                 const Eigen::Matrix3d& rotation)
{
  LegIK::Vector3D legIKTarget;
  legIKTarget[0] = footPos.x();
  legIKTarget[1] = footPos.y();
  legIKTarget[2] = footPos.z();

  if (side == Side::Left)
  {
    legIKTarget[1] -= distFootYOffset;
  }
  else
  {
    legIKTarget[1] += distFootYOffset;
  }

  LegIK::Frame3D legIKMatrix;
  for (int a = 0; a < 3; a++)
  {
    for (int b = 0; b < 3; b++)
    {
      legIKMatrix[a][b] = rotation(a, b);
    }
  }

  LegIK::Position result;
  if (!legIK->compute(legIKTarget, legIKMatrix, result))
  {
    return false;
  }

  std::string prefix = (side == Side::Left ? "left" : "right");
  setDof(prefix + "_hip_yaw", result.theta[0]);
  setDof(prefix + "_hip_roll", result.theta[1]);
  setDof(prefix + "_hip_pitch", -result.theta[2]);
  setDof(prefix + "_knee", result.theta[3]);
  setDof(prefix + "_ankle_pitch", -result.theta[4]);
  setDof(prefix + "_ankle_roll", result.theta[5]);

  return true;
}

void HumanoidModel::setSupportFoot(Side side, bool updateWorldPosition)
{
  if (updateWorldPosition)
  {
    worldToSupport = frameToWorld("flying_foot").inverse();
    worldToSupport.translation().z() = 0;
  }

  // Initializing aliases
  supportFoot = side;

  if (side == Side::Left)
  {
    bodyAliases["support_foot"] = "left_foot";
    bodyAliases["flying_foot"] = "right_foot";
  }
  else
  {
    bodyAliases["support_foot"] = "right_foot";
    bodyAliases["flying_foot"] = "left_foot";
  }
}

void HumanoidModel::setImu(double yaw, double pitch, double roll)
{
  // We update world to support, that suppose that foot is flat on ground
  Eigen::Matrix3d imuMatrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  Eigen::Affine3d newWorldToSupport(imuMatrix.inverse());
  newWorldToSupport.translation() = worldToSupport.translation();
  worldToSupport = newWorldToSupport;

  // We update worldToSupportPitchRoll, that take in account the pitch and roll from the IMU
  imuMatrix = imuMatrix * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
              Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Affine3d newWorldToSupportPitchRoll(orientation("trunk", "support_foot") * imuMatrix.inverse());
  newWorldToSupportPitchRoll.translation() = worldToSupport.translation();
  worldToSupportPitchRoll = newWorldToSupportPitchRoll;
}

Eigen::Affine3d HumanoidModel::frameToWorld(const std::string& frame, bool pitchRoll)
{
  if (pitchRoll) {
    return worldToSupportPitchRoll.inverse() * transformation(frame, "support_foot");
  } else {
    return worldToSupport.inverse() * transformation(frame, "support_foot");
  }
}

void HumanoidModel::startServer()
{
  if (!serverStarted)
  {
    socket.bind("tcp://*:7332");
    serverStarted = true;
  }
}

void HumanoidModel::publishModel()
{
  if (!serverStarted)
  {
    throw std::runtime_error("HumanoidModel: trying to publish model where server is not started");
  }

  HumanoidModelMsg msg;

  // Adding DOFs, the order is alphabetical (because dofs was orted out)
  for (auto& dof : dofs)
  {
    msg.add_dofs(getDof(dof));
  }

  // Adding robot Pose
  eigenToProtobuf(frameToWorld("origin"), msg.mutable_robottoworld());

  // Debugging frame positions
  // eigenToProtobuf(frameToWorld("trunk"), msg.add_debugpositions());
  // eigenToProtobuf(frameToWorld("left_foot"), msg.add_debugpositions());
  // eigenToProtobuf(frameToWorld("right_foot"), msg.add_debugpositions());

  // Sending it through PUB/SUB
  zmq::message_t packet(msg.ByteSize());
  msg.SerializeToArray(packet.data(), packet.size());

  // Sending packet
  socket.send(packet);
}
}  // namespace rhoban