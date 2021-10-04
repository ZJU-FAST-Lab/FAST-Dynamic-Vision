/**
 * @file bullet_estimation.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-07-19
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef BULLET_ESTIMATION_H_
#define BULLET_ESTIMATION_H_

#include <sys/stat.h>
#include <traj_utils/PolyTraj.h>

#include "Camera.h"
#include "CameraPoseVisualization.h"
#include "Eigen/Core"
#include "Eigen/Geometry"  // Eigen 几何模块
#include "camera_models/Camera.h"
#include "camera_models/CameraFactory.h"
#include "ceres/ceres.h"
#include "cmath"
#include "cv_bridge/cv_bridge.h"
#include "deque"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "opencv2/opencv.hpp"
#include "ros/duration.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "string"
#include "vector"

namespace bullet_estimation {

const Eigen::Vector3d kGrav_(0, 0, -9.81);

/* residuals */
/**
 * @brief Event Residual
 * Project 3d trajectory into event camera frame
 *
 * residual was built upon projection errors compared to
 * observation in 2D camera plane
 *
 * We applied ceres solver to solve this non-linear optimization
 * problem
 */
struct EventResidual {
  double x_;
  double y_;
  double dt_;
  Eigen::Isometry3d T_;
  /**
   * @brief
   * @param obs_x observation coordinates x
   * @param obs_y observation coordinates y
   * @param T_w2c   Transform from world frame to camera frame
   */
  EventResidual(double obs_x, double obs_y, double dt, Eigen::Isometry3d T_w2c,
                camera_model::CameraPtr cptr)
      : dt_(dt), T_(T_w2c) {
    Eigen::Vector2d p_cam_2d(obs_x, obs_y);
    Eigen::Vector3d p_cam_3d;
    cptr->liftProjective(p_cam_2d, p_cam_3d);
    // x_ = obs_x;
    // y_ = obs_y;
    // Eigen::Vector3d p_world;
    // cam_->liftProjective(p_cam, p_world);
    x_ = p_cam_3d.x();
    y_ = p_cam_3d.y();
    // std::cout << " x " << x_ << " y " << y_ << std::endl;
  }

  template <typename T>
  bool operator()(const T *const traj, T *residual) const {
    T p[3];
    p[0] = traj[0] + traj[3] * T(dt_) + 0.5 * T(kGrav_[0]) * T(dt_) * T(dt_);
    p[1] = traj[1] + traj[4] * T(dt_) + 0.5 * T(kGrav_[1]) * T(dt_) * T(dt_);
    p[2] = traj[2] + traj[5] * T(dt_) + 0.5 * T(kGrav_[2]) * T(dt_) * T(dt_);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> point_(p);

    point_.applyOnTheLeft(T_.rotation().template cast<T>());
    point_ = point_ + T_.translation().template cast<T>();
    residual[0] = point_[0] / point_[2] - T(x_);
    residual[1] = point_[1] / point_[2] - T(y_);
    return true;
  }

  /** Auto differential cost function */
  static ceres::CostFunction *Create(double obs_x, double obs_y, double dt,
                                     Eigen::Isometry3d T_w2c,
                                     camera_model::CameraPtr cptr) {
    return (new ceres::AutoDiffCostFunction<EventResidual, 2, 6>(
        new EventResidual(obs_x, obs_y, dt, T_w2c, cptr)));
  }
};

/**
 * @brief depth residual
 * project 3d trajectory into 3d depth plane
 *
 * depth residual was built only on depth errors
 *
 * We applied ceres solver to solve this non-linear optimization
 * problem
 *
 */
struct DepthResidual {
  double obs_z_;
  double dt_;
  /** from world frame to camera frame */
  Eigen::Isometry3d T_;
  DepthResidual(double obs_z, double dt, Eigen::Isometry3d T_w2c)
      : obs_z_(obs_z), dt_(dt), T_(T_w2c) {}
  template <typename T>
  bool operator()(const T *const traj, T *residual) const {
    T p[3];
    p[0] = traj[0] + traj[3] * T(dt_) + 0.5 * T(kGrav_[0]) * T(dt_) * T(dt_);
    p[1] = traj[1] + traj[4] * T(dt_) + 0.5 * T(kGrav_[1]) * T(dt_) * T(dt_);
    p[2] = traj[2] + traj[5] * T(dt_) + 0.5 * T(kGrav_[2]) * T(dt_) * T(dt_);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> point_(p);

    point_.applyOnTheLeft(T_.rotation().template cast<T>());
    point_ = point_ + T_.translation().template cast<T>();
    // if(p[2] <= 1e-6) return false;
    residual[0] = (point_[2] - T(obs_z_));
    return true;
  }
  /** Auto differential cost function */
  static ceres::CostFunction *Create(double obs_z, double dt,
                                     Eigen::Isometry3d Tcam_w) {
    return (new ceres::AutoDiffCostFunction<DepthResidual, 1, 6>(
        new DepthResidual(obs_z, dt, Tcam_w)));
  }
};

class BulletEst {
 private:
  /* parameters */
  bool kIsVisualize;
  static int traj_id_;
  const double huber_delta_ = 1.0;
  // static const Eigen::Vector3d kGrav_;

  /* ROS Utilities */
  ros::NodeHandle nh_;
  ros::Subscriber event_obs_sub_, depth_obs_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher traj_pub_, cam_pose_pub_, vis_traj_pub_;
  ros::Publisher depth_obs_3d_pub_, event_obs_3d_pub_;

  /* camera modules */
  camera_model::CameraPtr cam_;
  CameraPoseVisualization camera_pose_visual_;
  std::string cam_param_yaml_path_;

  /**
   * @brief frame traslation
   * * cam frame: 'z' backwards, 'y' downwards
   * * body frame: 'X' forwards, 'z' upwards
   * * world frame: body frame when initializing
   */
  Eigen::Isometry3d T_c2b_;  ///< cam frame to body frame
  Eigen::Isometry3d T_c2w_;  ///< cam frame to world frame
  Eigen::Isometry3d T_w2c_;  ///< world frame to cam frame
  Eigen::Isometry3d T_b2w_;  // body frame to world frame

  /* trajectory estimation */
  ros::Time first_fire_time_;
  ros::Time prev_dectect_time_;
  double traj_param_[6];
  const double max_param_[6] = {20, 20, 20, 20, 20, 20};
  const double min_param_[6] = {-20, -20, -20, -20, -20, -20};

  /* optimization problem */
  ceres::Problem *traj_est_prob_;
  ceres::Solver::Options ceres_options_;

 public:
  BulletEst(ros::NodeHandle &nh)
      : nh_(nh),
        camera_pose_visual_(0, 1, 0, 1)
  // max_param_({20, 20, 20, 20, 20, 20}),
  // min_param_({-20, -20, -20, -20, -20, -20})
  {
    camera_pose_visual_.setScale(0.8);
    camera_pose_visual_.setLineWidth(0.05);
  };
  ~BulletEst(){};
  void Init();
  void EventObserveCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void DepthObserveCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
  void DroneOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void LoadCameraFile(std::string camera_model_file);
  void PublishBulletTraj();
  void VisualizeBulletTraj();
  inline bool IsFileExist(const std::string &name);
  inline bool IsNewObj(geometry_msgs::PointStamped now_point);
};  // namespace bullet_estimation

// const static Eigen::Vector3d BulletEst::kGrav_(0, 0, -0.981);

};  // namespace bullet_estimation

#endif  // BULLET_ESTIMATION_H_
