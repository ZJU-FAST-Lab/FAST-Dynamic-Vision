/**
 * @file bullet_traj.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-07-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef bullet_traj_H_
#define bullet_traj_H_

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cmath"
#include "deque"
#include "iostream"
#include "ros/duration.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "string"
#include <random>
#include <sys/stat.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/opencv.hpp"

#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>

#include "Camera.h"
#include "CameraFactory.h"
#include "CameraPoseVisualization.h"

// using namespace std;d

namespace bullet_traj {

class BulletCamera {

private:
  /* ROS Utilities */
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, bullet_sub_;
  /** observations on 2D image plane   */
  ros::Publisher depth_obs_pub_, event_obs_pub_;
  /** observatrions in 3D world frame */
  ros::Publisher depth_obs_3d_pub_, event_obs_3d_pub_;
  ros::Publisher cam_pose_pub_;
  ros::Timer event_pub_hz_, depth_pub_hz_;

  /* parameters */

  /* camera modules */
  std::string cam_param_yaml_path_;
  camera_model::CameraPtr cam_;
  CameraPoseVisualization cam_pose_visual_;
  int event_hz_, depth_hz_;
  double event_period_, depth_period_;
  /** observation covariance */
  double event_cov_, depth_cov_;

  /* world frames */
  /** two points for interpolation */
  Eigen::Vector3d w_point_prev_, w_point_now_; /* world points */
  ros::Time t_w_point_prev_, t_w_point_now_;   /* observe timestamps */

  // coordinates on event frame // TODO multi objs
  std::unique_ptr<std::normal_distribution<double>> x_dist_, y_dist_;
  // depth distribution
  std::unique_ptr<std::normal_distribution<double>> d_dist_;
  // generate random variables
  std::default_random_engine generator_;

  /**
   * @brief frame traslation
   * * cam frame: 'z' backwards, 'y' downwards
   * * body frame: 'X' forwards, 'z' upwards
   * * world frame: body frame when initializing
   */
  Eigen::Isometry3d T_c2b_; /* cam frame to body frame */
  Eigen::Isometry3d T_c2w_; /* cam frame to world frame */
  Eigen::Isometry3d T_w2c_; // world frame to came frame
  Eigen::Isometry3d T_b2w_; // body frame to world frame

public:
  BulletCamera(ros::NodeHandle &nh) : cam_pose_visual_(0, 1, 0, 1), nh_(nh){};
  ~BulletCamera(){};
  void init();
  void DroneOdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void BulletCallback(const geometry_msgs::PoseStamped::ConstPtr bullet_pose);
  void EventPubCallback(const ros::TimerEvent &t);
  void DepthPubCallback(const ros::TimerEvent &t);
  void LoadCameraFile(std::string camera_model_file);
  inline bool IsFileExist(const std::string &name);
  inline Eigen::Vector3d Interpolate(Eigen::Vector3d start,
                                     Eigen::Vector3d stop, double s);
};

} // namespace bullet_traj

#endif // bullet_traj_H_