#include <algorithm>
#include "robot_model/humanoid_model.h"
#include "rhoban_utils/history/history.h"
#include <urdfreader/urdfreader.h>

namespace rhoban
{
HumanoidModel::HumanoidModel(std::string filename) : RobotModel(filename), legIK(nullptr)
{
  // Degrees of freedom
  dofs = { "head_yaw",        "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll",
           "left_elbow",      "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
           "left_hip_yaw",    "left_hip_roll",        "left_hip_pitch",      "left_knee",
           "left_ankle_roll", "left_ankle_pitch",     "right_hip_yaw",       "right_hip_roll",
           "right_hip_pitch", "right_knee",           "right_ankle_roll",    "right_ankle_pitch" };

  // Frames
  frames = { "trunk", "left_foot", "right_foot", "camera", "head_base" };

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
  distHeadYawToPitchZ = jointPosition("head_pitch", "trunk").z() - jointPosition("head_yaw", "trunk").z();
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
}

void HumanoidModel::resetWorldFrame()
{
  supportToWorld = Eigen::Affine3d::Identity();
  supportToWorld.translation().y() += supportFoot == Left ? distFootYOffset : -distFootYOffset;
  supportToWorldPitchRoll = supportToWorld;
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

void HumanoidModel::setSupportFoot(Side side, bool updateWorldPosition)
{
  if (side != supportFoot)
  {
    if (updateWorldPosition)
    {
      // Assigning new supportToWorld matrix
      supportToWorld = flyingFootFlattenedToWorld();
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
  if (hasImu)
  {
    // We update world to support, that suppose that foot is flat on ground
    Eigen::Matrix3d imuMatrix = Eigen::AngleAxisd(imuYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

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

Eigen::Affine3d HumanoidModel::flyingFootFlattenedToWorld()
{
  // Getting the flying foot in the world frame, supposing support foot is flat on the ground (no pitch or roll)
  auto flyingFootToWorld = frameToWorld("flying_foot", true);

  // Forcing the pitch and roll of flying foot to be zero, keeping only the yaw part in rotation
  double yaw = -atan2(flyingFootToWorld.rotation()(0, 1), flyingFootToWorld.rotation()(0, 0));
  flyingFootToWorld.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  // Forbidding the new frame not to be on the ground
  flyingFootToWorld.translation().z() = 0;
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
  Eigen::Vector3d baseCenter = headBaseToWorld.translation();
  Eigen::Matrix3d orientation = frameToWorld("trunk").inverse().rotation();
  Eigen::Vector3d viewVector = posTarget - baseCenter;
  Eigen::Vector3d viewVectorInBase = orientation * viewVector;

  // Compute yaw rotation around Z aligned with the target
  double yaw = atan2(viewVectorInBase.y(), viewVectorInBase.x());
  // Assign head yaw DOF
  panDOF = yaw;

  // Compute target in head_pitch frame fixed to head_yaw frame orientation
  Eigen::Vector3d targetInBase = headBaseToWorld.inverse() * posTarget;
  // Here, the head_yaw (no update) used is not aligned to the target point.
  // The missing yaw orientation is manually computed to not update the model
  double deltaYaw = yaw - getDof("head_yaw");
  targetInBase = Eigen::AngleAxisd(-deltaYaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * targetInBase;
  targetInBase.z() -= distHeadYawToPitchZ;

  // Conversion of target point to polar representation
  double R = targetInBase.norm();
  double gamma = atan2(targetInBase.z(), targetInBase.x());

  // Compute polar distance for camera from pitch joint
  double r = sqrt(pow(distHeadPitchToCameraZ, 2) + pow(distHeadPitchToCameraX, 2));
  // Compute angular correction to handle a non null
  // camera X translation offset
  double epsilon = atan(distHeadPitchToCameraX / distHeadPitchToCameraZ);
  double beta = epsilon;

  // Do the math. Geometric method is used (Alkashi, Trigo, Thales).
  // Use Maxima for system of 3 equations, 3 unknown solving
  double B = sin(beta);
  double cosAngle = r / R;
  if (R * R + r * r * B * B - r * r <= 0.0)
  {
    return false;
  }
  cosAngle = (B * sqrt(R * R + r * r * B * B - r * r) - r * B * B + r) / R;
  if (cosAngle >= 1.0 || cosAngle <= -1.0)
  {
    return false;
  }
  double alpha = M_PI / 2.0 - acos(cosAngle) - gamma;
  // Apply (inverse) angular correction to pitch
  alpha += -epsilon;

  // Assignement head pitch DOF
  tiltDOF = alpha;

  return true;
}

}  // namespace rhoban