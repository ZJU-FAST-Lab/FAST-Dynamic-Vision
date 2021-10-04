/**
 * @file bullet_traj.cpp
 * @author Haojia Li, Siyuan Wu (siyuanwu99@gmail.com)
 * @brief   camera model
 *          map the bullet trajectory to camera plane
 * @version 2.0
 * @date 2020-08-19
 * @note
 * 2021-07-16  refactor
 * 2020-8-23 Pressing the camera onto the aircraft
 *          (the camera moves with the aircraft) adds camera observation noise
 * 2020-8-24 add camera model
 * @copyright Copyright (c) 2021
 *
 */

#include <bullet_traj/bullet_traj.h>

namespace bullet_traj {

void BulletCamera::init() {
  /* params */
  event_hz_ = 40;
  depth_hz_ = 40;
  event_cov_ = 4;
  depth_cov_ = 0.1;

  /* node handle params */
  nh_.param("cam_param_yaml_path", cam_param_yaml_path_, cam_param_yaml_path_);
  nh_.param("cam_hz", event_hz_, event_hz_);
  nh_.param("came_depth_hz", depth_hz_, depth_hz_);
  nh_.param("cam_cov", event_cov_, event_cov_);
  nh_.param("cam_cov_depth", depth_cov_, depth_cov_);

  event_period_ = 1.0 / event_hz_;
  depth_period_ = 1.0 / depth_hz_;
  ROS_INFO("event_Hz %d, event_cov %lf", event_hz_, event_cov_);

  LoadCameraFile(cam_param_yaml_path_);

  /* reset observations */
  x_dist_.reset(new std::normal_distribution<double>(0, event_cov_));
  y_dist_.reset(new std::normal_distribution<double>(0, event_cov_));
  d_dist_.reset(new std::normal_distribution<double>(0, depth_cov_));

  /* rotation matrix, from cam to body */
  Eigen::Matrix3d R_c2b;
  R_c2b << 0, 0, -1, 1, 0, 0, 0, -1, 0;
  Eigen::Quaterniond q_c2b(R_c2b);  // quaternion
  T_c2b_ = Eigen::Isometry3d::Identity();
  T_c2b_.rotate(q_c2b);

  // subscribes, tcp nodelay
  bullet_sub_ = nh_.subscribe("/rm/visual/projectile_pose", 1,
                              &BulletCamera::BulletCallback, this,
                              ros::TransportHints().tcpNoDelay());
  odom_sub_ =
      nh_.subscribe("/dronesim_odom", 1, &BulletCamera::DroneOdomCallback, this,
                    ros::TransportHints().tcpNoDelay());

  /* publishes */
  event_obs_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet_point", 10);
  depth_obs_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/cam_depth_bullet_point", 10);
  /* visualize observation in world frame */
  event_obs_3d_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet/event_3d", 10);
  depth_obs_3d_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet/depth_3d", 10);
  cam_pose_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet/cam_pose", 10);
  /* Timers */
  ros::Timer event_pub_hz_ = nh_.createTimer(
      ros::Duration(event_period_), &BulletCamera::EventPubCallback, this);
  ros::Timer depth_pub_hz_ = /*!< problems here >*/
      nh_.createTimer(ros::Duration(depth_period_),
                      &BulletCamera::DepthPubCallback, this);
  event_pub_hz_.start();
  depth_pub_hz_.start();
  ros::spin();
}

/**
 * @brief
 * Transformation of the aircraft attitude into the camera coordinate system to
 * the world coordinate system
 * @param msg
 */
void BulletCamera::DroneOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  Eigen::Quaterniond q;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  q.w() = msg->pose.pose.orientation.w;

  // Convert to homogeneous transformation
  T_b2w_.setIdentity();
  T_b2w_.rotate(q);
  T_b2w_.pretranslate(Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z));

  T_c2w_ = T_b2w_ * T_c2b_;  // transition matrix camera to world
                             // （cam系在world下的位置与姿态）
  T_w2c_ = T_c2w_.inverse();

  /*  */
  geometry_msgs::PoseStamped cam_pose;  // camera pose
  cam_pose.header.stamp = ros::Time::now();
  cam_pose.header.frame_id = "world";
  cam_pose.pose.position.x = T_c2w_.translation().x();
  cam_pose.pose.position.y = T_c2w_.translation().y();
  cam_pose.pose.position.z = T_c2w_.translation().z();

  Eigen::Quaterniond q_w_cam(T_c2w_.rotation());
  cam_pose.pose.orientation.z = q_w_cam.z();
  cam_pose.pose.orientation.x = q_w_cam.x();
  cam_pose.pose.orientation.y = q_w_cam.y();
  cam_pose.pose.orientation.w = q_w_cam.w();
  cam_pose_visual_.reset();
  cam_pose_visual_.add_pose(T_c2w_.translation(), q_w_cam);
  cam_pose_visual_.publish_by(cam_pose_pub_, cam_pose.header);
}

/**
 * @brief Camera timer callbacks to achieve a fixed frequency of sending data.
 * The observations here are sent using the position of the most recent bullet,
 * projected onto the camera plane as the observed position, and the timestamp
 * is the timestamp from the bullet pose.
 * The camera pose is calculated using the most recent odometer data (odometer
 * frequency at 200hz, no linear interpolation, not much bias).
 *
 * @param t
 */
void BulletCamera::EventPubCallback(const ros::TimerEvent &t) {
  Eigen::Vector2d p_2d;  // point in camera 2d plane
  Eigen::Vector3d p_3d;  // point in camera 3d frame
  ros::Time t_p;

  if (t.current_real >= t_w_point_now_) {
    /* If the camera observation time is greater than the time of the most
     * recent bullet, take the position of the most recent bullet as the bullet
     * observation */

    if ((t.current_real - t_w_point_now_) > ros::Duration(2 * event_period_)) {
      // ROS_WARN("[Bullet] Event Observations: time error is too large");
      return;
    } else {
      p_3d = T_w2c_ * w_point_now_;
      t_p = t_w_point_now_;
    }

  } else if (t.current_real >= t_w_point_prev_) {
    /* Linear difference when the current camera observation is in the middle of
     * the two feedback bullet positions */

    t_p = t.current_real;
    double s = (t_p.toSec() - t_w_point_prev_.toSec()) /
               (t_w_point_now_.toSec() - t_w_point_prev_.toSec());
    Eigen::Vector3d p_new = Interpolate(w_point_prev_, w_point_now_, s);

    p_3d = T_w2c_ * p_new;

  } else {
    /* The current camera observation time is less than the last bullet
     * observation time */
    double dt = t.current_real.toSec() - t_w_point_prev_.toSec();
    ROS_WARN("CAM observe lag: %lf", (t_p.toSec() - t_w_point_prev_.toSec()));

    if (dt > 0.5) {
      return;
    }

    p_3d = T_w2c_ * w_point_prev_;
    t_p = t_w_point_prev_;
  }

  cam_->spaceToPlane(p_3d, p_2d);
  p_2d.x() = p_2d.x() + x_dist_->operator()(generator_);
  p_2d.y() = p_2d.y() + y_dist_->operator()(generator_);

  int x = round(p_2d.x());
  int y = round(p_2d.y());

  if (x < 0 || x > cam_->imageWidth() || y < 0 || y > cam_->imageHeight()) {
    ROS_WARN("This point is out of image plane");
    return;
  }

  /* publish event points */
  geometry_msgs::PointStamped p_2d_visual;
  p_2d_visual.header.stamp = t_p;
  p_2d_visual.header.frame_id = "cam";
  p_2d_visual.point.x = round(p_2d.x());
  p_2d_visual.point.y = round(p_2d.y());
  p_2d_visual.point.z = 0;
  event_obs_pub_.publish(p_2d_visual);

  geometry_msgs::PointStamped p_3d_visual;
  p_3d_visual.header.stamp = t_p;
  p_3d_visual.header.frame_id = "world";
  Eigen::Vector3d vp(p_3d.x() / p_3d.z(), p_3d.y() / p_3d.z(), 1);
  vp = T_c2w_ * vp;
  p_3d_visual.point.x = vp[0];
  p_3d_visual.point.y = vp[1];
  p_3d_visual.point.z = vp[2];
  event_obs_3d_pub_.publish(p_3d_visual);
}

/**
 * @brief Camera timer callback for fixed frequency sending data
 * The observations here are issued using the position of the most recent
 bullet, projected onto the camera plane as the observation position, and the
 timestamp is the timestamp in the bullet pose. The camera pose is calculated
 using the most recent odometer data to calculate the camera position (odometer
 frequency at 200hz, no linear interpolation done, not much error)

 * @param t
 */
void BulletCamera::DepthPubCallback(const ros::TimerEvent &t) {
  Eigen::Vector2d p_2d;  // point in camera 2d plane
  Eigen::Vector3d p_3d;  // point in camera 3d frame
  ros::Time t_p;

  if (t.current_real >= t_w_point_now_) {
    /* If the camera observation time is greater than the time of the most
     * recent bullet, take the position of the most recent bullet as the bullet
     * observation */

    if ((t.current_real - t_w_point_now_) > ros::Duration(2 * depth_period_)) {
      // ROS_WARN("[Bullet] Depth Observations: time error is too large");
      return;  //! For multiobj tracking, need to be revised
    } else {
      p_3d = T_w2c_ * w_point_now_;
      t_p = t_w_point_now_;
    }

  } else if (t.current_real >= t_w_point_prev_) {
    /* Linear difference when the current camera observation is in the middle of
     * the two feedback bullet positions */

    t_p = t.current_real;
    double s = (t_p.toSec() - t_w_point_prev_.toSec()) /
               (t_w_point_now_.toSec() - t_w_point_prev_.toSec());
    Eigen::Vector3d p_new = Interpolate(w_point_prev_, w_point_now_, s);

    p_3d = T_w2c_ * p_new;

  } else {
    /* The current camera observation time is less than the last bullet
     * observation time */
    double dt = t.current_real.toSec() - t_w_point_prev_.toSec();
    ROS_WARN("CAM observe lag: %lf", (t_p.toSec() - t_w_point_prev_.toSec()));

    if (dt > 0.5) {
      return;
    }

    p_3d = T_w2c_ * w_point_prev_;
    t_p = t_w_point_prev_;
  }

  cam_->spaceToPlane(p_3d, p_2d);

  p_3d.x() = p_3d.x() + d_dist_->operator()(generator_);
  p_3d.y() = p_3d.y() + d_dist_->operator()(generator_);
  p_3d.z() = p_3d.z() + d_dist_->operator()(generator_);

  int x = round(p_2d.x());
  int y = round(p_2d.y());

  if (x < 0 || x > cam_->imageWidth() || y < 0 || y > cam_->imageHeight()) {
    ROS_WARN("This point is out of image plane");
    return;
  }

  /* publish depth points */
  geometry_msgs::PointStamped p_c_visual;
  p_c_visual.header.stamp = t_p;
  p_c_visual.header.frame_id = "cam";
  p_c_visual.point.x = round(p_3d.x());
  p_c_visual.point.y = round(p_3d.y());
  p_c_visual.point.z = round(p_3d.z());
  depth_obs_pub_.publish(p_c_visual);

  geometry_msgs::PointStamped p_w_visual;  // visualization
  p_w_visual.header.stamp = t_p;
  p_w_visual.header.frame_id = "world";
  Eigen::Vector3d vp(p_3d.x(), p_3d.y(), p_3d.z());  // camera plane
  vp = T_c2w_ * vp;                                  // to world frame
  p_w_visual.point.x = vp[0];
  p_w_visual.point.y = vp[1];
  p_w_visual.point.z = vp[2];
  depth_obs_3d_pub_.publish(p_w_visual);
}

/**
 * @brief update observed bullet's position and timestamp
 *
 * @param bullet_pose bullet pose in world frame
 */
void BulletCamera::BulletCallback(
    const geometry_msgs::PoseStamped::ConstPtr bullet_pose) {
  w_point_prev_ = w_point_now_;
  t_w_point_prev_ = t_w_point_now_;

  w_point_now_.x() = bullet_pose->pose.position.x;
  w_point_now_.y() = bullet_pose->pose.position.y;
  w_point_now_.z() = bullet_pose->pose.position.z;
  t_w_point_now_ = bullet_pose->header.stamp;
}

/**
 * @brief Linear interpolation of point coordinates
 *
 * @param start Start point
 * @param stop Endpoint
 * @param s Interpolation factor 0<s<1
 * @return Vector3d
 */
inline Eigen::Vector3d BulletCamera::Interpolate(Eigen::Vector3d start,
                                                 Eigen::Vector3d stop,
                                                 double s) {
  Eigen::Vector3d f;
  f(0) = start.x() + (stop.x() - start.x()) * s;
  f(1) = start.y() + (stop.y() - start.y()) * s;
  f(2) = start.z() + (stop.z() - start.z()) * s;
  return f;
}

/**
 * @brief load camera info
 *
 * @param cam pointer to model configs
 * @param camera_model_file config model filepath
 */
void BulletCamera::LoadCameraFile(std::string camera_model_file) {
  if (IsFileExist(camera_model_file)) {
    cam_ = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(
        camera_model_file);
    std::cout << cam_->parametersToString() << std::endl;
  } else {
    ROS_ERROR("cam_param_yaml_path error! Please check the yaml path.");
  }
}

/**
 * @brief check if file exist
 *
 * @param name filepath
 * @return true exist
 * @return false
 */
inline bool BulletCamera::IsFileExist(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

}  // namespace bullet_traj
