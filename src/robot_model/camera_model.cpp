#include "robot_model/camera_model.h"

#include <rhoban_utils/util.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

Eigen::Vector2d cv2Eigen(const cv::Point2f& p)
{
  return Eigen::Vector2d(p.x, p.y);
}

Eigen::Vector3d cv2Eigen(const cv::Point3f& p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

cv::Point2f eigen2CV(const Eigen::Vector2d& p)
{
  return cv::Point2f(p.x(), p.y());
}

cv::Point3f eigen2CV(const Eigen::Vector3d& p)
{
  return cv::Point3f((float)p.x(), (float)p.y(), (float)p.z());
}

namespace rhoban
{
CameraModel::CameraModel()
  : imgWidth(-1)
  , imgHeight(-1)
  , focalX(-1)
  , focalY(-1)
  , centerX(-1)
  , centerY(-1)
  , radialCoeffs(Eigen::Vector3d::Zero())
  , tangentialCoeffs(Eigen::Vector2d::Zero())
  , useIsPointInsideTheoreticalImage(true)
{
}

CameraModel::CameraModel(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const cv::Size& img_size)
{
  // TODO: check type
  focalX = camera_matrix.at<double>(0, 0);
  focalY = camera_matrix.at<double>(1, 1);
  centerX = camera_matrix.at<double>(0, 2);
  centerY = camera_matrix.at<double>(1, 2);
  if (distortion_coeffs.cols >= 4)
  {
    radialCoeffs(0) = distortion_coeffs.at<double>(0);
    radialCoeffs(1) = distortion_coeffs.at<double>(1);
    tangentialCoeffs(0) = distortion_coeffs.at<double>(2);
    tangentialCoeffs(1) = distortion_coeffs.at<double>(3);
  }
  if (distortion_coeffs.cols >= 5)
  {
    radialCoeffs(2) = distortion_coeffs.at<double>(4);
  }
  imgWidth = img_size.width;
  imgHeight = img_size.height;

  useIsPointInsideTheoreticalImage = true;
}

bool CameraModel::isValid() const
{
  return imgWidth > 0 && imgHeight > 0 && focalX > 0 && focalY > 0 && centerX > 0 && centerY > 0;
}

std::string CameraModel::getInvalidMsg() const
{
  std::string res = "because : ";
  if (imgWidth <= 0)
    res += "imgWidth <= 0 ";
  if (imgHeight <= 0)
    res += "imgHeight <= 0 ";
  if (focalX <= 0)
    res += "focalX <= 0 ";
  if (focalY <= 0)
    res += "focalY <= 0 ";
  if (centerX <= 0)
    res += "centerX <= 0 ";
  if (centerY <= 0)
    res += "centerY <= 0 ";
  return res;
}

cv::Size CameraModel::getImgSize() const
{
  return cv::Size(imgWidth, imgHeight);
}

int CameraModel::getImgWidth() const
{
  return imgWidth;
}

int CameraModel::getImgHeight() const
{
  return imgHeight;
}

double CameraModel::getImgDiag() const
{
  return std::sqrt(imgWidth * imgWidth + imgHeight * imgHeight);
}

Eigen::Vector2d CameraModel::getCenter() const
{
  return Eigen::Vector2d(centerX, centerY);
}

double CameraModel::getCenterX() const
{
  return centerX;
}

double CameraModel::getCenterY() const
{
  return centerY;
}

double CameraModel::getFocalX() const
{
  return focalX;
}

double CameraModel::getFocalY() const
{
  return focalY;
}

rhoban_utils::Angle CameraModel::getFOVX() const
{
  return rhoban_utils::Angle::fromXY(focalX, imgWidth / 2) * 2;
}

rhoban_utils::Angle CameraModel::getFOVY() const
{
  return rhoban_utils::Angle::fromXY(focalY, imgHeight / 2) * 2;
}

rhoban_utils::Angle CameraModel::getFOVDiag() const
{
  return rhoban_utils::Angle::fromXY(getFocalDist(), (imgWidth + imgHeight) / 2) * 2;
}

void CameraModel::setImgWidth(int width)
{
  imgWidth = width;
}

void CameraModel::setImgHeight(int height)
{
  imgHeight = height;
}

void CameraModel::setCenter(const Eigen::Vector2d& center)
{
  centerX = center.x();
  centerY = center.y();
}

void CameraModel::setFocal(const Eigen::Vector2d& focal)
{
  focalX = focal.x();
  focalY = focal.y();
}

void CameraModel::setDistortion(const Eigen::VectorXd& distortion)
{
  radialCoeffs.segment(0, 2) = distortion.segment(0, 2);
  radialCoeffs(2) = distortion(4);
  tangentialCoeffs = distortion.segment(2, 2);
}

bool CameraModel::containsPixel(const cv::Point2f& imgPos) const
{
  bool xOk = imgPos.x >= 0 && imgPos.x < imgWidth;
  bool yOk = imgPos.y >= 0 && imgPos.y < imgHeight;
  return xOk && yOk;
}

double CameraModel::getFocalDist() const
{
  return (focalX + focalY) / 2;
}

cv::Mat CameraModel::getCameraMatrix() const
{
  return (cv::Mat_<double>(3, 3) << focalX, 0, centerX, 0, focalY, centerY, 0, 0, 1);
}

cv::Mat CameraModel::getDistortionCoeffs() const
{
  return (cv::Mat_<double>(1, 5) << radialCoeffs(0), radialCoeffs(1), tangentialCoeffs(0), tangentialCoeffs(1),
          radialCoeffs(2));
}

Eigen::VectorXd CameraModel::getDistortionCoeffsAsEigen() const
{
  Eigen::VectorXd coeffs(5);
  coeffs.segment(0, 2) = radialCoeffs.segment(0, 2);
  coeffs.segment(2, 2) = tangentialCoeffs.segment(0, 2);
  coeffs(4) = radialCoeffs(2);
  return coeffs;
}

cv::Point2f CameraModel::toCorrectedImg(const cv::Point2f& imgPosUncorrected) const
{
  if (!isValid())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid config" + getInvalidMsg());
  }
  std::vector<cv::Point2f> uncorrected = { imgPosUncorrected };
  std::vector<cv::Point2f> corrected;
  cv::undistortPoints(uncorrected, corrected, getCameraMatrix(), getDistortionCoeffs());
  // When P is not provided, the position of the point is in normalized image,
  // therefore we have to unnormalize the coordinates
  return cv::Point2f(corrected[0].x * getFocalX() + getCenterX(), corrected[0].y * getFocalY() + getCenterY());
}

cv::Point2f CameraModel::toUncorrectedImg(const cv::Point2f& imgPosCorrected) const
{
  if (!isValid())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid config" + getInvalidMsg());
  }

  // Convert the pixel format to something usable by
  cv::Point3f normalized((imgPosCorrected.x - centerX) / focalX, (imgPosCorrected.y - centerY) / focalY, 1.0);

  if (!isPointValidForCorrection(normalized))
  {
    throw std::runtime_error(DEBUG_INFO + " point cannot be distorded, it is outside the valid area, which should "
                                          "means outside the image.");
  }

  if (useIsPointInsideTheoreticalImage and !isPointInsideTheoreticalImage(normalized))
  {
    throw std::runtime_error(DEBUG_INFO + " point (" + std::to_string(normalized.x) + ", " +
                             std::to_string(normalized.y) +
                             ") cannot be distorded, it is outside the theoretical image.");
  }

  std::vector<cv::Point2f> distorted;
  std::vector<cv::Point3f> undistorted = { normalized };
  cv::projectPoints(undistorted, cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), getCameraMatrix(),
                    getDistortionCoeffs(), distorted);
  return distorted[0];
}
bool CameraModel::isPointInsideTheoreticalImage(const cv::Point3f& pos) const
{
  cv::Point3f center(centerX, centerY, 1.0);

  cv::Point3f normalized_diff((0 - center.x) / focalX, (0 - center.y) / focalY, 1.0);
  if (cv::norm(normalized_diff) > cv::norm(pos))
  {
    return true;
  }

  normalized_diff.x = (imgWidth - center.x) / focalX;
  normalized_diff.y = (0 - center.y) / focalY;
  if (cv::norm(normalized_diff) > cv::norm(pos - center))
  {
    return true;
  }

  normalized_diff.x = (0 - center.x) / focalX;
  normalized_diff.y = (imgHeight - center.y) / focalY;
  if (cv::norm(normalized_diff) > cv::norm(pos))
  {
    return true;
  }

  normalized_diff.x = (imgWidth - center.x) / focalX;
  normalized_diff.y = (imgHeight - center.y) / focalY;
  if (cv::norm(normalized_diff) > cv::norm(pos))
  {
    return true;
  }

  return false;
}

bool CameraModel::isPointValidForCorrection(const cv::Point3f& pos) const
{
  if (pos.z != 1.0)
  {
    throw std::runtime_error(DEBUG_INFO + " pos.z should be equal to 1.0 instead of " + std::to_string(pos.z));
  }

  if (radialCoeffs(2) != 0 and tangentialCoeffs != Eigen::Vector2d::Zero())
  {
    return true;
  }
  // k3, p1 and p2 are equal to 0

  double x = static_cast<double>(pos.x);
  double y = static_cast<double>(pos.y);

  // we can suppose that x and y are positive
  if (x < 0)
  {
    x = -x;
  }
  if (y < 0)
  {
    y = -y;
  }

  // if the point is close to the center it is valid
  if (x < 0.01 * focalX / imgWidth and y < 0.01 * focalY / imgHeight)
  {
    return true;
  }
  else
  {
    double k1 = radialCoeffs(0);
    double k2 = radialCoeffs(1);

    if (k1 == 0 and k2 == 0)  // there is no distortion
    {
      return true;
    }

    if (k2 == 0 and k1 > 0)  // f' is always positive
    {
      return true;
    }

    double u = x + y;
    double p = x / u;
    double q = y / u;

    double a = 5 * k2 * std::pow(p * p + q * q, 2);
    double b = 3 * k1 * (p * p + q * q);
    double c = 1;

    if (k2 == 0 and k1 < 0)  // the roots of f' are +- 1/sqrt(3*(-k1)*(p^2+q^2))
    {
      return u * u < 1 / (-b);
    }

    // in the following k2 is non zero

    double disc = b * b - 4 * a * c;

    if (disc <= 0)  // the sign of g is constant, hence f is monotonic
    {
      return true;
    }
    else
    {
      double u1 = (-b - std::sqrt(disc)) / (2 * a);
      double u2 = (-b + std::sqrt(disc)) / (2 * a);

      if (u2 < u1)
      {
        double tmp = u1;
        u1 = u2;
        u2 = tmp;
      }

      // in the following u1 < u2

      if (u2 < 0)  // both are negative, hence the roots of f' are complexe
      {
        return true;
      }
      else if (u1 > 0)  // u1 is the smallest positivie root
      {
        return u * u < u1;
      }
      else  // u2 is the smallest positivie root
      {
        return u * u < u2;
      }
    }
  }
}

cv::Point2f CameraModel::getImgFromObject(const cv::Point3f& objectPosition, bool outputInCorrectedImg) const
{
  if (!isValid())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid config " + getInvalidMsg());
  }
  if (objectPosition.z <= 0.0)
  {
    throw std::runtime_error(DEBUG_INFO + " invalid object position: z=" + std::to_string(objectPosition.z));
  }

  double ratio = getFocalDist() / objectPosition.z;
  double px = ratio * objectPosition.x + centerX;
  double py = ratio * objectPosition.y + centerY;
  cv::Point2f posInCorrected(px, py);

  if (outputInCorrectedImg)
  {
    return posInCorrected;
  }
  return toUncorrectedImg(posInCorrected);
}

cv::Point3f CameraModel::getViewVectorFromImg(const cv::Point2f& imgPos, bool inputInCorrectedImg) const
{
  if (!isValid())
  {
    throw std::runtime_error(DEBUG_INFO + " invalid config" + getInvalidMsg());
  }
  cv::Point2f correctedPos = imgPos;
  if (!inputInCorrectedImg)
  {
    correctedPos = toCorrectedImg(imgPos);
  }
  double dX = (correctedPos.x - centerX);
  double dY = (correctedPos.y - centerY);
  double dZ = getFocalDist();
  double length = std::sqrt(dX * dX + dY * dY + dZ * dZ);
  return cv::Point3f(dX / length, dY / length, dZ / length);
}

// cv::Point3f CameraModel::getObjectPosFromImgAndPlan(const cv::Point2f & imgPos,
//                                                    cv::Point4f planEquation,
//                                                    bool inputInCorrectedImg) const
//{
//  throw std::logic_error(DEBUG_INFO + "not implemented");
//}

void computePixelBounding()
{
}

void CameraModel::fromJson(const Json::Value& v, const std::string& dir_name)
{
  (void)dir_name;
  rhoban_utils::tryRead(v, "img_width", &imgWidth);
  rhoban_utils::tryRead(v, "img_height", &imgHeight);
  rhoban_utils::tryRead(v, "focal_x", &focalX);
  rhoban_utils::tryRead(v, "focal_y", &focalY);
  rhoban_utils::tryRead(v, "center_x", &centerX);
  rhoban_utils::tryRead(v, "center_y", &centerY);
  rhoban_utils::tryReadEigen(v, "radial_coeffs", &radialCoeffs);
  rhoban_utils::tryReadEigen(v, "tangential_coeffs", &tangentialCoeffs);
}

Json::Value CameraModel::toJson() const
{
  Json::Value v;
  v["img_width"] = imgWidth;
  v["img_height"] = imgHeight;
  v["focal_x"] = focalX;
  v["focal_y"] = focalY;
  v["center_x"] = centerX;
  v["center_y"] = centerY;
  v["radial_coeffs"] = rhoban_utils::vector2Json(radialCoeffs);
  v["tangential_coeffs"] = rhoban_utils::vector2Json(tangentialCoeffs);
  return v;
}
std::string CameraModel::getClassName() const
{
  return "camera_model";
}

}  // namespace rhoban
