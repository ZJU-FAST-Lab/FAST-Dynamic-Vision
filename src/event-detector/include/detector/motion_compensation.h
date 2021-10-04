/**
 * @file motion_compensation.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-03-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef DETECTOR_COMP_H_
#define DETECTOR_COMP_H_

/* INCLUDES */
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "dvs_msgs/Event.h"
#include "dvs_msgs/EventArray.h"
#include "sensor_msgs/Imu.h"

/* GLOBAL DEFINES */
#define MAT_ROWS 480  // 240  //800
#define MAT_COLS 640  // 346  //1280

using std::vector;
using namespace std;
using namespace cv;
using namespace Eigen;

template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

/**
 * @brief convert a vector to skew matrix
 *
 * @tparam T
 * @param v
 * @return Mat3<typename T::Scalar>
 */
template <typename T>
Mat3<typename T::Scalar> vectorToSkewMat(const Eigen::MatrixBase<T> &v) {
  static_assert(T::ColsAtCompileTime == 1 && T::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  Mat3<typename T::Scalar> m;
  m << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;
  return m;
}

/**
 * @brief square
 *
 * @tparam T
 * @param x
 * @return T
 */
template <typename T>
T pow2(const T &x) {
  return x * x;
}

class MotComp {
 private:
  /* flags */
  enum ConvolutionType { CONVOLUTION_FULL, CONVOLUTION_VALID };
  string kIMUType_;

  /* parameters */
  Eigen::Matrix3f K;  // intrinstic matrix
  Eigen::Matrix3f K_inverse;

  /* data */
  vector<sensor_msgs::Imu> IMU_buffer_;
  vector<dvs_msgs::Event> events_buffer_;
  nav_msgs::Odometry odoms_buffer_;
  cv::Mat depth_img_;

  // std::vector<Isometry3d> trans_vector;

  /* image outputs */
  cv::Mat time_img_;      // mean-time graph
  cv::Mat event_counts_;  // counts for event in each pixel
  cv::Mat color_img_;

  /* utilities */
  Eigen::Matrix3f rotation_matrix_;
  // Eigen::Matrix3f rot_K;

  // Eigen::Vector3f
  Eigen::Vector3f omega_avg_;  // angular velocity (vector)
  float omega_ = 15;           // angular velocity (scalar)

  Eigen::Isometry3d trans_body = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d fc2world = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d cam2body = Eigen::Isometry3d::Identity();

  int event_size_ = 0;
  int imu_size_ = 0;

  float deltaT = 0.0f;
  float prevDeltaT = 0.0f;

  /* helper functions */
  void UpdateFC2world();
  void UpdateCam2body();

  /* inline functions */
  inline double ReadDepth(const cv::Mat &I, const int &x, const int &y);
  inline void ConvertToHomogeneous(Eigen::Vector3f *v);
  inline bool IsWithinTheBoundary(const Eigen::Vector3f &v);
  inline bool IsWithinTheBoundary(const int &x, const int &y);
  inline bool IsDepthInRange(const cv::Mat &depthImg, const int &x,
                             const int &y, int min, int max);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MotComp() {
    K << 5.3633325932983780e+02, 0, 3.2090009280822994e+02, 0,
        5.3631797700847164e+02, 2.3404853514480661e+02, 0, 0, 1;
    K_inverse = K.inverse();
  }

  ~MotComp() {}

  // void init();
  void main();
  void Clear();
  void AvgIMU_DJI();
  void AvgIMU_PX4();

  void notCompensate(cv::Mat *timeImg, cv::Mat *eventCount);
  void translationalCompensate(cv::Mat *timeImg, cv::Mat *eventCount);
  void rotationalCompensate(cv::Mat *timeImg, cv::Mat *eventCount);
  void RotTransCompensate(cv::Mat *timeImg, cv::Mat *eventCount);

  /* helper functions */
  void LoadIMUs(const sensor_msgs::ImuConstPtr &imu);
  void LoadEvents(const dvs_msgs::EventArray::ConstPtr &emsg);
  void LoadOdometry(const nav_msgs::Odometry::ConstPtr &odom);
  void LoadDepth(const cv::Mat &depth);
  void SetIMUType(const string &s);
  cv::Mat GetTimeImage(void) { return time_img_; }
  cv::Mat GetEventCount(void) { return event_counts_; }
  cv::Mat GetVisualization(void);

  /* Ptr */
  typedef std::unique_ptr<MotComp> Ptr;
};

#endif  // DETECTOR_COMP_H_
