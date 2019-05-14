#include "robot_model/humanoid_server.h"
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
HumanoidServer::HumanoidServer() : context(1), socket(context, ZMQ_PUB), serverStarted(false)
{
}

void HumanoidServer::start()
{
  if (!serverStarted)
  {
    socket.bind("tcp://*:7332");
    serverStarted = true;
  }
}

void HumanoidServer::publishModel(rhoban::HumanoidModel& model, bool flatFoot)
{
  if (!serverStarted)
  {
    start();
  }

  HumanoidModelMsg msg;

  // Adding DOFs, the order is alphabetical (because dofs was orted out)
  for (auto& dof : model.dofs)
  {
    msg.add_dofs(model.getDof(dof));
  }

  // Adding robot Pose
  eigenToProtobuf(model.frameToWorld("origin", flatFoot), msg.mutable_robottoworld());

  // Debugging frame positions can be added to the message
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