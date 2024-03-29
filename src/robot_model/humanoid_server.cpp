#include "robot_model/humanoid_server.h"
#include "humanoid_model.pb.h"

void eigenToProtobuf(Eigen::Vector3d translation, HumanoidModelPosition* msgPosition)
{
  msgPosition->set_x(translation.x());
  msgPosition->set_y(translation.y());
  msgPosition->set_z(translation.z());
}

void eigenToProtobuf(Eigen::Affine3d pose, HumanoidModelPosition* msgPosition)
{
  eigenToProtobuf(pose.translation(), msgPosition);
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
HumanoidServer::HumanoidServer() : context(1), socket(context, ZMQ_PUB), serverStarted(false), hasBall(false)
{
  fieldPose = Eigen::Affine3d::Identity();
}

void HumanoidServer::start()
{
  if (!serverStarted)
  {
    int water_mark = 10;

    socket.setsockopt(ZMQ_SNDHWM, &water_mark, sizeof(int));
    socket.bind("tcp://*:7332");
    serverStarted = true;
  }
}

void HumanoidServer::setBallPosition(Eigen::Vector3d ballPosition_)
{
  ballPosition = ballPosition_;
  hasBall = true;
}

void HumanoidServer::setFieldPose(Eigen::Affine3d fieldPose_)
{
  fieldPose = fieldPose_;
}

void HumanoidServer::addOpponentPosition(Eigen::Vector3d opponentPosition)
{
  opponentPositions.push_back(opponentPosition);
}

void HumanoidServer::addMatePose(Eigen::Affine3d matePose)
{
  matePoses.push_back(matePose);
}

void HumanoidServer::addDebugPosition(Eigen::Vector3d debugPosition)
{
  debugPositions.push_back(debugPosition);
}

void HumanoidServer::publishModel(rhoban::HumanoidModel& model, bool flatFoot, Eigen::Affine3d correction)
{
  if (!serverStarted)
  {
    start();
  }

  HumanoidModelMsg msg;

  // Adding DOFs, the order is alphabetical (because dofs was orted out)
  for (auto& dof : model.dofNames)
  {
    msg.add_dofs(model.getDof(dof));
  }

  // Adding robot Pose
  eigenToProtobuf(correction * model.frameToWorld("trunk", flatFoot), msg.mutable_robottoworld());

  // Debugging frame positions can be added to the message
  for (auto debugPosition : debugPositions)
  {
    eigenToProtobuf(debugPosition, msg.add_debugpositions());
  }
  debugPositions.clear();

  // Opponents positions
  for (auto opponentPosition : opponentPositions)
  {
    eigenToProtobuf(opponentPosition, msg.add_opponentspositions());
  }
  opponentPositions.clear();

  // Mates poses
  for (auto matePose : matePoses)
  {
    eigenToProtobuf(matePose, msg.add_alliesposes());
  }
  matePoses.clear();

  // Ball pose
  if (hasBall)
  {
    eigenToProtobuf(ballPosition, msg.mutable_ballposition());
  }

  // Adding field pose
  eigenToProtobuf(fieldPose, msg.mutable_fieldtoworld());

  // Sending it through PUB/SUB
  zmq::message_t packet(msg.ByteSize());
  msg.SerializeToArray(packet.data(), packet.size());

  // Sending packet
  socket.send(packet);
}
}  // namespace rhoban