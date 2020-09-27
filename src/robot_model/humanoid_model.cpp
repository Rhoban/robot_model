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

HumanoidModel::HumanoidModel(std::string filename) : RobotModel(filename), legIK(nullptr)
{
  // Degrees of freedom
  dofNames = { "head_yaw",        "head_pitch",           "left_shoulder_pitch", "left_shoulder_roll",
               "left_elbow",      "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
               "left_hip_yaw",    "left_hip_roll",        "left_hip_pitch",      "left_knee",
               "left_ankle_roll", "left_ankle_pitch",     "right_hip_yaw",       "right_hip_roll",
               "right_hip_pitch", "right_knee",           "right_ankle_roll",    "right_ankle_pitch" };

  legDofNames = { "left_hip_yaw",    "left_hip_roll",    "left_hip_pitch",   "left_knee",
                  "left_ankle_roll", "left_ankle_pitch", "right_hip_yaw",    "right_hip_roll",
                  "right_hip_pitch", "right_knee",       "right_ankle_roll", "right_ankle_pitch" };

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

Eigen::Matrix<double, 12, 12> HumanoidModel::computeJacobian(const std::string& frame,
                                                             const std::string& orientationFrame,
                                                             const std::string& flyingFrame)
{
  Eigen::Matrix<double, 12, 12> J;

  auto currentDofs = dofs;
  currentDofs[getJointId("left_knee")] = std::max(1e-3, dofs[getJointId("left_knee")]);
  currentDofs[getJointId("right_knee")] = std::max(1e-3, dofs[getJointId("right_knee")]);

  // Computing jacobian of flying
  RigidBodyDynamics::Math::Vector3d zeroPoint(0, 0, 0);
  RigidBodyDynamics::Math::MatrixNd G1 = Eigen::MatrixXd::Zero(6, currentDofs.size());
  RigidBodyDynamics::Math::MatrixNd G2 = Eigen::MatrixXd::Zero(6, currentDofs.size());
  RigidBodyDynamics::Math::MatrixNd G3 = Eigen::MatrixXd::Zero(6, currentDofs.size());
  RigidBodyDynamics::Math::Vector3d point = centerOfMass(frame);
  RigidBodyDynamics::Math::Vector3d point2 = position(flyingFrame, frame);
  RigidBodyDynamics::CalcPointJacobian6D(model, currentDofs, getBodyId(frame), point, G1, true);
  RigidBodyDynamics::CalcPointJacobian6D(model, currentDofs, getBodyId(flyingFrame), zeroPoint, G2, true);
  RigidBodyDynamics::CalcPointJacobian6D(model, currentDofs, getBodyId(frame), point2, G3, true);

  // std::cout << orientation("origin", orientationFrame) << std::endl;
  // std::cout << orientation("origin", frame) << std::endl;
  // std::cout << -orientation("origin", orientationFrame) * orientation("origin", frame) << std::endl;

  int index = 0;
  for (auto dof : legDofNames)
  {
    double mass;
    int jointIndex = getJointId(dof);
    RigidBodyDynamics::Math::Vector3d comRbdl;
    RigidBodyDynamics::Math::Vector3d comVel;
    RigidBodyDynamics::Math::VectorNd onlyOne = RigidBodyDynamics::Math::VectorNd::Zero(model.dof_count);
    onlyOne[jointIndex] = 1;

    RigidBodyDynamics::Utils::CalcCenterOfMass(model, currentDofs, onlyOne, mass, comRbdl, &comVel);

    // Rotation speed in frame
    J.block(0, index, 3, 1) = -orientation("origin", frame) * G1.block(0, jointIndex, 3, 1);
    // Speed of CoM
    // J.block(3, index, 3, 1) = comVel;
    J.block(3, index, 3, 1) =
        orientation("origin", frame) * comVel - orientation("origin", frame) * G1.block(3, jointIndex, 3, 1);

    // Rotation speed in frame
    J.block(6, index, 3, 1) = orientation("origin", frame) * G2.block(0, jointIndex, 3, 1) -
                              orientation("origin", frame) * G3.block(0, jointIndex, 3, 1);
    // Speed of CoM
    // J.block(3, index, 3, 1) = comVel;
    J.block(9, index, 3, 1) = orientation("origin", frame) * G2.block(3, jointIndex, 3, 1) -
                              orientation("origin", frame) * G3.block(3, jointIndex, 3, 1);

    index += 1;
  }

  return J;
}

inline Eigen::Vector3d matrixToAxis(const Eigen::Matrix3d& mat)
{
  // Skew is the anti-symetric matrix
  Eigen::Matrix3d skew = mat - mat.transpose();
  // Rotation axis extraction
  Eigen::Vector3d axis;
  axis(0) = skew(2, 1);
  axis(1) = skew(0, 2);
  axis(2) = skew(1, 0);
  // Compute rotation angle
  if (axis.norm() > 1e-6)
  {
    double theta = std::asin(axis.norm() / 2.0);
    return theta * axis.normalized();
  }
  else
  {
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }
}

Eigen::Matrix<double, 12, 1> HumanoidModel::getError(const std::string& frame, Eigen::Affine3d com,
                                                     const std::string& flyingFrame, Eigen::Affine3d flying,
                                                     const std::string& orientationFrame)
{
  Eigen::Matrix<double, 12, 1> error;

  // Orientation error
  auto R = this->orientation(orientationFrame, frame);
  Eigen::Affine3d orientation = transformation(frame, orientationFrame) * com;
  error.block(0, 0, 3, 1) = R * matrixToAxis(orientation.linear());
  error.block(3, 0, 3, 1) = com.translation() - centerOfMass(frame);

  // Center of mass error
  // Eigen::Vector3d comError = centerOfMass(frame) - com.translation();
  // error.block(3, 0, 3, 1) = R * comError;

  // Flying frame error part
  auto R2 = this->orientation(flyingFrame, frame);
  Eigen::Affine3d flyingError = transformation(frame, flyingFrame) * flying;
  error.block(6, 0, 3, 1) = R2 * matrixToAxis(flyingError.linear());
  error.block(9, 0, 3, 1) = R2 * flyingError.translation();

  return error;
}

}  // namespace rhoban