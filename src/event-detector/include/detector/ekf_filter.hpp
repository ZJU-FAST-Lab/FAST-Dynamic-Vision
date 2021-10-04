/**
 * @file ekf_filter.hpp
 * @author Haojia Li
 * @brief EKF filter
 * @version 0.1
 * @date 2020-12-01
 *
 * @copyright Copyright (c) 2020
 *
 */
#include <Eigen/LU>
#include "Eigen/Core"
#include "ros/ros.h"

class EkfFilter  // 匀加速
{
 public:
  double x_, y_;
  double v_x_, v_y_;
  double a_x_, a_y_;
  ros::Time pre_time_;

  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 6> P;
  Eigen::Matrix<double, 6, 6> Q;
  Eigen::Matrix<double, 2, 2> R;

 public:
  EkfFilter() {
    Q << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 2, 0, 0, 0,
         0, 0, 0, 2, 0, 0,
         0, 0, 0, 0, 4, 0,
         0, 0, 0, 0, 0, 4;
    Q = Q * 100;  // 后面会乘以dt

    R << 0.5, 0, 0, 0.5;
    P = Eigen::MatrixXd::Identity(6, 6);
  }

  void GetPosition(double *x, double *y) {
    *x = x_;
    *y = y_;
  }

  /**
   * @brief predict EKF
   *
   * @param t
   */
  void predict(ros::Time t) {
    double dt = (t - pre_time_).toSec();
    x_ = x_ + v_x_ * dt + 0.5 * a_x_ * dt * dt;
    y_ = y_ + v_y_ * dt + 0.5 * a_y_ * dt * dt;
    v_x_ = v_x_ + a_x_ * dt;
    v_y_ = v_y_ + a_y_ * dt;

    F << 1, 0, dt, 0, 0.5 * dt * dt, 0,
         0, 1, 0, dt, 0, 0.5 * dt * dt,
         0, 0, 1, 0, dt, 0,
         0, 0, 0, 1, 0, dt,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    Eigen::Matrix<double, 6, 6> dtq = dt * Q;
    P = F * P * F.transpose() + dtq;
    pre_time_ = t;
  }

  /**
   * @brief update EKF
   * 
   * @param obs_x 
   * @param obs_y 
   */
  void update(double obs_x, double obs_y) {
    double xh = obs_x - x_;
    double yh = obs_y - y_;

    Eigen::Matrix<double, 2, 6> H;
    H << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
    Eigen::Matrix<double, 2, 2> S = H * P * H.transpose() + R;

    Eigen::Matrix<double, 6, 2> K = P * H.transpose() * S.inverse();
    Eigen::Matrix<double, 6, 1> Xk_1;
    Xk_1 << x_, y_, v_x_, v_y_, a_x_, a_y_;
    Eigen::Matrix<double, 6, 1> X;
    Eigen::Matrix<double, 2, 1> Y(xh, yh);
    X = Xk_1 + K * Y;
    P = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P;
    x_ = X(0);  // 直接用矩阵会不会好点
    y_ = X(1);
    v_x_ = X(2);
    v_y_ = X(3);
    a_x_ = X(4);
    a_y_ = X(5);
  }

  /**
   * @brief add a new observation
   * 
   * @param obs_x 
   * @param obs_y 
   */
  void add_new_obs(double obs_x, double obs_y) {
    add_new_obs(obs_x, obs_y, ros::Time::now());
  }

  /**
   * @brief add a new observation with time
   * 
   * @param obs_x 
   * @param obs_y 
   * @param obs_t 
   */
  void add_new_obs(double obs_x, double obs_y, ros::Time obs_t) {
    predict(obs_t);
    update(obs_x, obs_y);
  }

  /**
   * @brief reset this EKF
   * 
   * @param obs_x 
   * @param obs_y 
   * @param obs_t 
   */
  void reset_data(double obs_x, double obs_y, ros::Time obs_t) {
    P = Eigen::MatrixXd::Identity(6, 6);
    x_ = obs_x;
    y_ = obs_y;
    v_x_ = 0;
    v_y_ = 0;
    a_x_ = 0;
    a_y_ = 0;
    pre_time_ = obs_t;
  }
  ~EkfFilter() { ; }
};
