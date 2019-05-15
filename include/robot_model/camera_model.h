#pragma once

#include <rhoban_utils/angle.h>
#include <rhoban_utils/serialization/json_serializable.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

Eigen::Vector2d cv2Eigen(const cv::Point2f& p);
Eigen::Vector3d cv2Eigen(const cv::Point3f& p);
cv::Point2f eigen2CV(const Eigen::Vector2d& p);
cv::Point3f eigen2CV(const Eigen::Vector3d& p);

namespace rhoban
{
/// This model contains all the properties of the calibration of a camera:
/// Intrinsic parameters: center and focal of the camera
/// Distortion parameters: radial and tangential
///
/// It is based on the model presented in:
/// https://docs.opencv.org/3.4.1/dc/dbb/tutorial_py_calibration.html
///
/// Additionnal ressources can be found at:
/// - https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
///   -> Paragraph Detailed Description
///
/// Image points:
/// - 2D in the image referential
/// - Top-left is (0,0)
/// - Unit is pixel
///
/// Object points:
/// - Position of the object in the camera referential
/// - x and y matches the axes of the camera
/// - z points toward direction of the camera
class CameraModel : public rhoban_utils::JsonSerializable
{
public:
  /// Initial content does not allow to compute any transformation since it is
  /// intentionally not valid. Members have to be initialized before calling any
  /// other internal function
  CameraModel();

  /**
   * Build a camera model from opencv representation
   */
  CameraModel(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const cv::Size& img_size);

  /// Return true if the object has valid parameters
  bool isValid() const;
  std::string getInvalidMsg() const;

  int getImgWidth() const;
  int getImgHeight() const;
  double getImgDiag() const;
  Eigen::Vector2d getCenter() const;
  double getCenterX() const;
  double getCenterY() const;
  double getFocalX() const;
  double getFocalY() const;

  /// Return an approximate of the field of view width
  rhoban_utils::Angle getFOVX() const;

  /// Return an approximate of the field of view height
  rhoban_utils::Angle getFOVY() const;

  /// Return an approximate of the diagonal field of view
  rhoban_utils::Angle getFOVDiag() const;

  /// Set image width [px]
  void setImgWidth(int width);

  /// Set image height [px]
  void setImgHeight(int height);

  /// Set center of the focal: (x,y) [px]
  void setCenter(const Eigen::Vector2d& center);

  /// Set the focal length along both axis: (x,y) [px]
  void setFocal(const Eigen::Vector2d& focal);

  /// Set the distortion parameters: (k1,k2,p1,p2,k3) [px]
  void setDistortion(const Eigen::VectorXd& distortion_coeffs);

  /// Return true if pixel is inside image, false otherwise
  bool containsPixel(const cv::Point2f& imgPos) const;

  /// Average between x and y focals [px]
  double getFocalDist() const;

  /// 3 by 3 matrix
  cv::Mat getCameraMatrix() const;

  /// 5 by 1 matrix
  cv::Mat getDistortionCoeffs() const;
  Eigen::VectorXd getDistortionCoeffsAsEigen() const;

  /// From uncorrected image to corrected image
  cv::Point2f toCorrectedImg(const cv::Point2f& imgPosUncorrected) const;

  /// From corrected image to uncorrected image
  cv::Point2f toUncorrectedImg(const cv::Point2f& imgPosCorrected) const;

  /// Return the position of the object (in camera referential).
  ///
  /// If outputInCorrectedImg is enabled, then distortion is not computed, therefore the
  /// position return is in the correctedImage
  ///
  /// Throws a runtime_error if objectPosition.z <= 0 (behind plane)
  cv::Point2f getImgFromObject(const cv::Point3f& objectPosition, bool outputInCorrectedImg = false) const;

  /// Return the normalized view vector corresponding to a point in the image.
  ///
  /// If inputInCorrectedImg is enabled, then the imgPos is considered as already corrected
  cv::Point3f getViewVectorFromImg(const cv::Point2f& imgPos, bool inputInCorrectedImg = false) const;

  // No need to implement it from now
  //  /// Return the object position of the point at the intersection of the vision
  //  /// ray corresponding to imgPos and the plan defined by planEquation.
  //  ///
  //  /// If inputInCorrectedImg is enabled, then the distortion is not computed
  //  cv::Point3f getObjectPosFromImgAndPlan(const cv::Point2f & imgPos,
  //                                         cv::Point4f planEquation,
  //                                         bool inputInCorrectedImg = false) const;

  void fromJson(const Json::Value& json_value, const std::string& dir_name) override;
  Json::Value toJson() const override;
  std::string getClassName() const override;

private:
  /// Number of columns in an image [px]
  int imgWidth;
  /// Number of rows in image [px]
  int imgHeight;

  /// The focal distance along x-axis [px]
  ///
  /// Note: to obtain the focal in [mm] multiply it by sensorWidth [mm] / imgWidth [px]
  double focalX;
  /// The focal distance along y-axis [px]
  ///
  /// Note: to obtain the focal in [mm] multiply it by sensorHeight [mm] / imgHeight [px]
  double focalY;

  /// The center of the optical image along x-axis [px]
  double centerX;
  /// The center of the optical image along y-axis [px]
  double centerY;

  /// k_1, k_2, k_3
  Eigen::Vector3d radialCoeffs;

  /// p_1, p_2
  Eigen::Vector2d tangentialCoeffs;
};

}  // namespace Leph
