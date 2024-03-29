#include <algorithm>
#include "robot_model/humanoid_model.h"
#include "rhoban_utils/history/history.h"
#include <urdfreader/urdfreader.h>

namespace rhoban
{
// Makes a frame parallel to XY plane (only keep the yaw of a frame)
void makeParallelToFloor(Eigen::Affine3d& frame)
{
  frame.linear() = Eigen::AngleAxisd(frameYaw(frame.rotation()), Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

HumanoidModel::HumanoidModel(std::string filename)
  : RobotModel(filename), legIK(nullptr), imuYawOffset(0), hasImu(false), supportFoot(Side::Left)
{
  // Degrees of freedom
  dofNames = { "head_yaw",        "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll",
               "left_elbow",      "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
               "left_hip_yaw",    "left_hip_roll",        "left_hip_pitch",      "left_knee",
               "left_ankle_roll", "left_ankle_pitch",     "right_hip_yaw",       "right_hip_roll",
               "right_hip_pitch", "right_knee",           "right_ankle_roll",    "right_ankle_pitch" };

  // Frames
  frames = { "trunk", "left_foot", "right_foot", "camera", "head_base" };

  // Sorting
  std::sort(dofNames.begin(), dofNames.end());
  std::sort(frames.begin(), frames.end());

  // Checking DOFs presence
  for (auto& dof : dofNames)
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
  distHeadYawToPitchZ =
      jointPosition("head_pitch", "trunk").z() - transformation("head_base", "trunk").translation().z();
  distHeadPitchToCameraZ = jointPosition("head_pitch", "camera").y();
  distHeadPitchToCameraX = -jointPosition("head_pitch", "camera").z();
  legIK = new rhoban_leg_ik::IK(distHipToKnee, distKneeToAnkle, distAnkleToGround);

  // Initializing right support foot
  setSupportFoot(Side::Right);
  setSupportFoot(Side::Left);

  // Initalizing with no IMU constraint
  hasImu = false;
  imuYaw = imuPitch = imuRoll = 0;

  resetWorldFrame();

  if (model.GetParentBodyId(getBodyId("torso")) != 0)
  {
    throw std::runtime_error("Error in URDF: torso should be the root of the robot");
  }

  // Example of code hacking one frame
  // auto frame = model.GetJointFrame(getJointId("left_foot_frame"));
  // frame.r.z() += 0.5;
  // model.SetJointFrame(getJointId("left_foot_frame"), frame);
}

void HumanoidModel::resetWorldFrame()
{
  supportToWorld = Eigen::Affine3d::Identity();
  supportToWorld.translation().y() += supportFoot == Left ? distFootYOffset : -distFootYOffset;
  supportToWorldPitchRoll = supportToWorld;
  imuYawOffset = imuYaw;
}

HumanoidModel::~HumanoidModel()
{
  if (legIK != nullptr)
  {
    delete legIK;
  }
}

bool HumanoidModel::computeLegIK(std::map<std::string, double>& angles, HumanoidModel::Side side,
                                 const Eigen::Vector3d& footPos, const Eigen::Matrix3d& rotation)
{
  rhoban_leg_ik::Vector3D legIKTarget;
  legIKTarget[0] = footPos.x();
  legIKTarget[1] = footPos.y();
  legIKTarget[2] = footPos.z();

  // Applying foot Y offset because IK works in the hip motor intersections frame
  if (side == Side::Left)
  {
    legIKTarget[1] -= distFootYOffset;
  }
  else
  {
    legIKTarget[1] += distFootYOffset;
  }

  // Copying rotation to the leg IK matrix
  rhoban_leg_ik::Frame3D legIKMatrix;
  for (int a = 0; a < 3; a++)
  {
    for (int b = 0; b < 3; b++)
    {
      legIKMatrix[a][b] = rotation(a, b);
    }
  }

  rhoban_leg_ik::Position result;
  if (!legIK->compute(legIKTarget, legIKMatrix, result))
  {
    return false;
  }

  std::string prefix = (side == Side::Left ? "left" : "right");
  angles[prefix + "_hip_yaw"] = result.theta[0];
  angles[prefix + "_hip_roll"] = result.theta[1];
  angles[prefix + "_hip_pitch"] = -result.theta[2];
  angles[prefix + "_knee"] = result.theta[3];
  angles[prefix + "_ankle_pitch"] = -result.theta[4];
  angles[prefix + "_ankle_roll"] = result.theta[5];

  return true;
}

void HumanoidModel::setSupportFoot(Side side, bool updateWorldPosition, bool flatFoot)
{
  if (side != supportFoot)
  {
    if (updateWorldPosition)
    {
      // Assigning new supportToWorld matrix
      supportToWorld = flyingFootFlattenedToWorld(flatFoot);
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

    updateImu();
  }
}

void HumanoidModel::updateImu()
{
  // hasImu = false;
  if (hasImu)
  {
    // We update world to support, that suppose that foot is flat on ground
    Eigen::Matrix3d imuMatrix = Eigen::AngleAxisd(imuYaw - imuYawOffset, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Affine3d newSupportToWorld = Eigen::Affine3d::Identity();
    newSupportToWorld.linear() =
        (imuMatrix * Eigen::AngleAxisd(orientationYaw("support_foot", "trunk"), Eigen::Vector3d::UnitZ()));
    newSupportToWorld.translation() = supportToWorld.translation();
    supportToWorld = newSupportToWorld;

    // We update worldToSupportPitchRoll, that take in account the pitch and roll from the IMU
    imuMatrix = imuMatrix * Eigen::AngleAxisd(imuPitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
                Eigen::AngleAxisd(imuRoll, Eigen::Vector3d::UnitX()).toRotationMatrix();

    Eigen::Affine3d newSupportToWorldPitchRoll;
    newSupportToWorldPitchRoll.linear() = (imuMatrix * orientation("support_foot", "trunk"));
    newSupportToWorldPitchRoll.translation() = supportToWorld.translation();
    supportToWorldPitchRoll = newSupportToWorldPitchRoll;
  }
  else
  {
    supportToWorldPitchRoll = supportToWorld;
  }
}

void HumanoidModel::setImu(bool present, double yaw, double pitch, double roll)
{
  hasImu = present;
  imuYaw = yaw;
  imuPitch = pitch;
  imuRoll = roll;

  updateImu();
}

void HumanoidModel::setImuYawOffset(double offset)
{
  imuYawOffset = offset;
}

double HumanoidModel::getYaw()
{
  return imuYaw - imuYawOffset;
}

Eigen::Affine3d HumanoidModel::flyingFootFlattenedToWorld(bool flatFoot)
{
  // Getting the flying foot in the world frame, supposing support foot is flat on the ground (no pitch or roll)
  auto flyingFootToWorld = frameToWorld("flying_foot", flatFoot);

  // Forcing the pitch and roll of flying foot to be zero, keeping only the yaw part in rotation
  makeParallelToFloor(flyingFootToWorld);

  // Forbidding the new frame not to be on the ground
  flyingFootToWorld.translation().z() = 0;

  return flyingFootToWorld;
}

Eigen::Affine3d HumanoidModel::frameToWorld(const std::string& frame, bool flatFoot)
{
  if (flatFoot)
  {
    return supportToWorld * transformation(frame, "support_foot");
  }
  else
  {
    return supportToWorldPitchRoll * transformation(frame, "support_foot");
  }
}

Eigen::Affine3d HumanoidModel::selfToWorld()
{
  return rhoban_utils::averageFrames(supportToWorld, flyingFootFlattenedToWorld(), 0.5);
}

bool HumanoidModel::cameraLookAt(double& panDOF, double& tiltDOF, const Eigen::Vector3d& posTarget)
{
  // Compute view vector in head yaw frame
  Eigen::Affine3d headBaseToWorld = frameToWorld("head_base");
  Eigen::Vector3d targetInHeadBase = headBaseToWorld.inverse() * posTarget;

  // The pan is simply the angle in the XY plane
  panDOF = atan2(targetInHeadBase.y(), targetInHeadBase.x());

  // We then consider the (head_base x axis, head_pitch) plane
  Eigen::Vector2d targetInHeadPitchPlane(sqrt(pow(targetInHeadBase.x(), 2) + pow(targetInHeadBase.y(), 2)),
                                         targetInHeadBase.z() - distHeadYawToPitchZ);

  double theta = M_PI / 2 - atan2(targetInHeadPitchPlane.y(), targetInHeadPitchPlane.x());

  // We just use beta = cos(opposed / hypothenus) to watch it with camera
  double ratio = distHeadPitchToCameraZ / targetInHeadPitchPlane.norm();
  if (ratio > 1 || ratio < -1)
  {
    return false;
  }
  double beta = acos(ratio);
  tiltDOF = theta - beta;

  return true;
}

Eigen::Affine3d HumanoidModel::getPositionFromFrame(const std::string& frame, const Eigen::Affine3d& frameToWorld,
                                                    bool forceOnFloor)
{
  Eigen::Affine3d supportToWorld;
  auto supportToFrame = transformation("support_foot", frame);
  auto supportToWorldPitchRoll = frameToWorld * supportToFrame;
  if (forceOnFloor)
  {
    supportToWorldPitchRoll.translation().z() = 0;
    return supportToWorldPitchRoll;
  }
  else
  {
    supportToWorld = supportToWorldPitchRoll;
    makeParallelToFloor(supportToWorld);
    return supportToWorld;
  }
}

void HumanoidModel::setPositionFromFrame(const std::string& frame, const Eigen::Affine3d& frameToWorld,
                                         bool forceOnFloor)
{
  auto supportToFrame = transformation("support_foot", frame);
  supportToWorldPitchRoll = frameToWorld * supportToFrame;
  if (forceOnFloor)
  {
    supportToWorldPitchRoll.translation().z() = 0;
  }

  supportToWorld = supportToWorldPitchRoll;
  makeParallelToFloor(supportToWorld);
}

void HumanoidModel::readFromHistories(rhoban_utils::HistoryCollection& histories, double timestamp, bool readSupport)
{
  // Updating DOFs from replay
  for (const std::string& name : getDofNames())
  {
    setDof(name, histories.number("read:" + name)->interpolate(timestamp));
  }

  double imuYaw = histories.angle("imu_gyro_yaw")->interpolate(timestamp);
  double imuPitch = histories.angle("imu_pitch")->interpolate(timestamp);
  double imuRoll = histories.angle("imu_roll")->interpolate(timestamp);
  setImu(true, imuYaw, imuPitch, imuRoll);

  if (readSupport)
  {
    // Reading the support matrices and support foot flag from the history
    // NOTE: Since supportToWorldPitchRoll is also read in that case, the update of the IMU will
    // be erased by this read
    // Updating robot position
    setSupportFoot(histories.boolean("supportIsLeft")->interpolate(timestamp) ? Left : Right);

    // Trunk is read here in order to make the interpolation working even on the foot swap
    setPositionFromFrame("trunk", histories.pose("trunk")->interpolate(timestamp));
  }
}

}  // namespace rhoban
