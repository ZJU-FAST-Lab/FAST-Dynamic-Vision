/**
 * @file velocity_est.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief velocity estimation
 * @version 0.1
 * @date 2021-04-11
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef DETECTOR_VELOCITYESTIMATION_H_
#define DETECTOR_VELOCITYESTIMATION_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "memory.h"

class VelocityEst {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VelocityEst() { this->setup(); }
  ~VelocityEst() {}
  typedef std::unique_ptr<VelocityEst> Ptr;

  /* utilities */
  void setup();
  void run(const Eigen::Vector3f centerPosition);
  Eigen::Vector3f ptPosition;
  Eigen::Vector3f result_velocity_;
  Eigen::Vector3f result_position_;
  float dt = 0.01;

 private:
  Eigen::Matrix<float, 6, 1> _xhat;
  Eigen::Matrix<float, 6, 6> _A;
  Eigen::Matrix<float, 6, 6> _Q0;
  Eigen::Matrix<float, 6, 6> _P;
  Eigen::Matrix<float, 3, 3> _R0;
  Eigen::Matrix<float, 6, 3> _B;
  Eigen::Matrix<float, 3, 6> _C;
};

#endif  // DETECTOR_VELOCITYESTIMATION_H_
