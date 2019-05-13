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
  : RobotModel(filename), context(1), socket(context, ZMQ_PUB), serverStarted(false)
{
  worldToRobot = Eigen::Affine3d::Identity();

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

  auto robotToWorld = worldToRobot.inverse();

  // Adding robot Pose
  eigenToProtobuf(robotToWorld, msg.mutable_robottoworld());

  // Debugging frame positions
  eigenToProtobuf(robotToWorld * transformation("trunk", "origin"), msg.add_debugpositions());
  eigenToProtobuf(robotToWorld * transformation("left_foot", "origin"), msg.add_debugpositions());
  eigenToProtobuf(robotToWorld * transformation("right_foot", "origin"), msg.add_debugpositions());

  // Sending it through PUB/SUB
  zmq::message_t packet(msg.ByteSize());
  msg.SerializeToArray(packet.data(), packet.size());

  // Sending packet
  socket.send(packet);
}
}  // namespace rhoban