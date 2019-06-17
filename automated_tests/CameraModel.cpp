#include <robot_model/camera_model.h>

#include <gtest/gtest.h>

using namespace std;
using namespace rhoban;

string getAbsoluteTestFilePath()
{
  string filePath = __FILE__;
  string currentDirPath = filePath.substr(0, filePath.rfind("/"));
  return currentDirPath + "/cameraModelTest.json";
}

/// JsonLoader will be used for
TEST(jsonLoader, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  EXPECT_EQ(cameraModel.getImgWidth(), 800);
  EXPECT_EQ(cameraModel.getImgHeight(), 600);
  EXPECT_EQ(cameraModel.getCenterX(), 400);
  EXPECT_EQ(cameraModel.getCenterY(), 300);
  EXPECT_EQ(cameraModel.getFocalX(), 600);
  EXPECT_EQ(cameraModel.getFocalY(), 600);
}

TEST(getFOV, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  // Hand computed values
  rhoban_utils::Angle fovX = cameraModel.getFOVX();
  rhoban_utils::Angle fovY = cameraModel.getFOVY();
  EXPECT_NEAR(fovX.getSignedValue(), 67.38, 0.1);
  EXPECT_NEAR(fovY.getSignedValue(), 53.13, 0.1);
}

TEST(getCameraMatrix, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Mat cameraMatrix = cameraModel.getCameraMatrix();
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0, 0), cameraModel.getFocalX());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0, 1), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(0, 2), cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1, 0), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1, 1), cameraModel.getFocalY());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(1, 2), cameraModel.getCenterY());
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2, 0), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2, 1), 0.0);
  EXPECT_FLOAT_EQ(cameraMatrix.at<double>(2, 2), 1.0);
}

TEST(getDistortionCoeffs, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Mat distortionCoeffs = cameraModel.getDistortionCoeffs();
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0, 0), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0, 1), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0, 2), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0, 3), 0.0);
  EXPECT_FLOAT_EQ(distortionCoeffs.at<double>(0, 4), 0.0);
}

TEST(containsPixel, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f in1(0, 0);
  cv::Point2f in2(600, 500);
  cv::Point2f out1(-2, 50);
  cv::Point2f out2(2, -50);
  cv::Point2f out3(2, 602);
  cv::Point2f out4(802, 5);
  cv::Point2f out5(802, 602);
  cv::Point2f out6(-2, -20);

  EXPECT_TRUE(cameraModel.containsPixel(in1));
  EXPECT_TRUE(cameraModel.containsPixel(in2));
  EXPECT_FALSE(cameraModel.containsPixel(out1));
  EXPECT_FALSE(cameraModel.containsPixel(out2));
  EXPECT_FALSE(cameraModel.containsPixel(out3));
  EXPECT_FALSE(cameraModel.containsPixel(out4));
  EXPECT_FALSE(cameraModel.containsPixel(out5));
  EXPECT_FALSE(cameraModel.containsPixel(out6));
}

TEST(toCorrectedImg, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f uncorrectedPos, correctedPos;
  // Case 1: center, no deformation
  uncorrectedPos.x = cameraModel.getCenterX();
  uncorrectedPos.y = cameraModel.getCenterY();
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_FLOAT_EQ(uncorrectedPos.x, correctedPos.x);
  EXPECT_FLOAT_EQ(uncorrectedPos.y, correctedPos.y);
  // Case 2: border, no deformation
  uncorrectedPos = cv::Point2f(0, 0);
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_NEAR(uncorrectedPos.x, correctedPos.x, 0.001);
  EXPECT_NEAR(uncorrectedPos.y, correctedPos.y, 0.001);
  // Case 2: assymetric location, no deformation
  uncorrectedPos.x = cameraModel.getCenterX() * 1.5;
  uncorrectedPos.y = cameraModel.getCenterY() * 1.2;
  correctedPos = cameraModel.toCorrectedImg(uncorrectedPos);
  EXPECT_FLOAT_EQ(uncorrectedPos.x, correctedPos.x);
  EXPECT_FLOAT_EQ(uncorrectedPos.y, correctedPos.y);
}

TEST(getImgFromObject, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point3f objectPos;
  cv::Point2f imgPos;

  // 1. centered pixel should be at center of image
  objectPos = cv::Point3f(0, 0, 1);
  imgPos = cameraModel.getImgFromObject(objectPos);
  EXPECT_FLOAT_EQ(imgPos.x, cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(imgPos.y, cameraModel.getCenterY());
  // 2. zero in z throws runtime_error
  try
  {
    objectPos = cv::Point3f(0, 0, 0);
    imgPos = cameraModel.getImgFromObject(objectPos);
    EXPECT_TRUE(false);
  }
  catch (const std::runtime_error& exc)
  {
    EXPECT_TRUE(true);
  }
  // 3. negative value in z throws runtime_error
  try
  {
    objectPos = cv::Point3f(0, 0, -1);
    imgPos = cameraModel.getImgFromObject(objectPos);
    EXPECT_TRUE(false);
  }
  catch (const std::runtime_error& exc)
  {
    EXPECT_TRUE(true);
  }
  // 4. Offset along y-axis
  objectPos = cv::Point3f(0, cameraModel.getImgHeight() / 4., cameraModel.getFocalDist());
  imgPos = cameraModel.getImgFromObject(objectPos);
  EXPECT_FLOAT_EQ(imgPos.x, cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(imgPos.y, cameraModel.getCenterY() + cameraModel.getImgHeight() / 4.);
  // 5. Offset along y-axis same angle as 4. but further
  objectPos = cv::Point3f(0, cameraModel.getImgHeight() / 2., cameraModel.getFocalDist() * 2);
  imgPos = cameraModel.getImgFromObject(objectPos);
  EXPECT_FLOAT_EQ(imgPos.x, cameraModel.getCenterX());
  EXPECT_FLOAT_EQ(imgPos.y, cameraModel.getCenterY() + cameraModel.getImgHeight() / 4.);
}

TEST(getViewVectorFromImg, testSuccess)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  cv::Point2f imgPos;
  cv::Point3f viewVector;
  // Case 1: Center of img
  imgPos = cv::Point2f(cameraModel.getCenterX(), cameraModel.getCenterY());
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, 0.0);
  EXPECT_FLOAT_EQ(viewVector.y, 0.0);
  EXPECT_FLOAT_EQ(viewVector.z, 1.0);
  // Case 2: Only y value -> value corresponding to a 30 deg angle
  imgPos.x = cameraModel.getCenterX();
  imgPos.y = cameraModel.getCenterY();
  imgPos.y += cameraModel.getFocalY() * tan(M_PI / 6);
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, 0.0);
  EXPECT_FLOAT_EQ(viewVector.y, 0.5);
  EXPECT_FLOAT_EQ(viewVector.z, std::sqrt(3) / 2);
  // Case 3: Only x value
  imgPos.x = cameraModel.getCenterX() + cameraModel.getFocalX();
  imgPos.y = cameraModel.getCenterY();
  viewVector = cameraModel.getViewVectorFromImg(imgPos);
  EXPECT_FLOAT_EQ(viewVector.x, std::sqrt(2) / 2);
  EXPECT_FLOAT_EQ(viewVector.y, 0.0);
  EXPECT_FLOAT_EQ(viewVector.z, std::sqrt(2) / 2);
}

TEST(isPointValidForCorrection, easyCases)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  Eigen::VectorXd distortion;
  cv::Point3f point_outside = cv::Point3f(5.0, 5.0, 1.0);
  cv::Point3f point_inside = cv::Point3f(0.0001, 0.0001, 1.0);
  cv::Point3f point_x_y_zero = cv::Point3f(0, 0, 1.0);

  // p1 not zero
  distortion = Eigen::VectorXd::Zero(7);
  distortion(2) = 0.01;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_outside));

  // p2 not zero
  distortion = Eigen::VectorXd::Zero(7);
  distortion(3) = 0.01;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_outside));

  // k3 not zero
  distortion = Eigen::VectorXd::Zero(7);
  distortion(4) = 0.01;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_outside));

  // k2=0 and k1 > 0
  distortion = Eigen::VectorXd::Zero(7);
  distortion(1) = 0.05;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_outside));

  // k2=0 and k1 < 0
  distortion = Eigen::VectorXd::Zero(7);
  distortion(0) = -0.05;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));

  point_inside = cv::Point3f(2.0, 0, 1.0);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));

  point_outside = cv::Point3f(3.0, 0, 1.0);
  EXPECT_FALSE(cameraModel.isPointValidForCorrection(point_outside));

  point_inside = cv::Point3f(1.5, 1.5, 1.0);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));

  point_outside = cv::Point3f(2.5, 2.5, 1.0);
  EXPECT_FALSE(cameraModel.isPointValidForCorrection(point_outside));

  // k2 != 0
  distortion = Eigen::VectorXd::Zero(7);
  distortion(0) = -0.05;
  distortion(1) = 0.001;
  cameraModel.setDistortion(distortion);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_x_y_zero));

  point_inside = cv::Point3f(3.0, 0, 1.0);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  point_outside = cv::Point3f(4.0, 0, 1.0);
  EXPECT_FALSE(cameraModel.isPointValidForCorrection(point_outside));
  point_outside = cv::Point3f(5.0, 0, 1.0);
  EXPECT_FALSE(cameraModel.isPointValidForCorrection(point_outside));

  point_inside = cv::Point3f(1.5, 1.5, 1.0);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_inside));
  point_outside = cv::Point3f(2.0, 2.0, 1.0);
  EXPECT_TRUE(cameraModel.isPointValidForCorrection(point_outside));

  // z != 1.0 gives an error
  try
  {
    cv::Point3f point = cv::Point3f(1.0, 2.0, 2.0);
    cameraModel.isPointValidForCorrection(point);
    EXPECT_TRUE(false);
  }
  catch (const std::runtime_error& exc)
  {
    EXPECT_TRUE(true);
  }
}

TEST(toUncorrectedImg, outsideValidArea)
{
  CameraModel cameraModel;
  cameraModel.loadFile(getAbsoluteTestFilePath());

  Eigen::VectorXd distortion;
  distortion = Eigen::VectorXd::Zero(7);
  distortion(0) = -0.05;
  cameraModel.setDistortion(distortion);

  cv::Point2f point_outside = cv::Point2f(10000.0, 10000.0);
  try
  {
    cameraModel.toUncorrectedImg(point_outside);
    EXPECT_TRUE(false);
  }
  catch (const std::runtime_error& exc)
  {
    EXPECT_TRUE(true);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
