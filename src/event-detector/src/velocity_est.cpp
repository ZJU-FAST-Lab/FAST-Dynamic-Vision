/**
 * @file velocity_est.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-04-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "detector/velocity_est.h"

/**
 * @brief
 *
 */
void VelocityEst::setup() {
  // TODO(@haojia): I am not sure whether parameters are correct
  dt = 0.01;
  _xhat.setZero();
  _A.setZero();
  _A.block(0, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<float, 3, 3>::Identity();
  _A.block(3, 3, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();
  _C.setZero();
  _C.block(0, 0, 3, 3) = Eigen::Matrix<float, 3, 3>::Identity();

  _P.setIdentity();
  _P = float(100) * _P;

  _Q0.setIdentity();
  _Q0.block(0, 0, 3, 3) =
      (dt / 20.f) *
      Eigen::Matrix<float, 3, 3>::Identity();  // Here roughly adjust
  _Q0.block(3, 3, 3, 3) = (dt / 20.f) * Eigen::Matrix<float, 3, 3>::Identity();

  _R0.setIdentity();
}

void VelocityEst::run(const Eigen::Vector3f centerPosition) {
  // TODO(@haojia): Noise configuration
  ptPosition = centerPosition;

  float sensor_noise = 0.01;
  float process_noise_p = 0.01;
  float process_noise_v = 0.01;

  // noise setup TODO:
  Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity();
  Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_p;
  Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_v;
  Eigen::Matrix<float, 3, 3> R = Eigen::Matrix<float, 3, 3>::Identity();
  R = _R0 * sensor_noise;

  /* measurement input */
  Eigen::Matrix<float, 3, 1> y;
  y << ptPosition;

  /* calculation */
  _xhat = _A * _xhat;
  _P = _A * _P * _A.transpose() + Q;
  Eigen::Matrix<float, 6, 3> _K =
      _P * _C.transpose() * (_C * _P * _C.transpose() + R);
  _xhat = _xhat + _K * (y - _C * _xhat);
  Eigen::Matrix<float, 6, 6> _S =
      Eigen::Matrix<float, 6, 6>::Identity() - _K * _C;
  _P = _S * _P * _S.transpose() + _K * R * _K.transpose();

  /* results */
  result_position_ = _xhat.block(0, 0, 3, 1);
  result_velocity_ = _xhat.block(3, 0, 3, 1);
}