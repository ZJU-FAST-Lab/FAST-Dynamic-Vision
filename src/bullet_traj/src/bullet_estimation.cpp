/**
 * @file bullet_estimation.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2021-07-19
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <bullet_traj/bullet_estimation.h>

using namespace bullet_estimation;

int BulletEst::traj_id_ = 0;

void BulletEst::Init() {
  /* node handle params */

  nh_.param("cam_param_yaml_path", cam_param_yaml_path_, cam_param_yaml_path_);
  nh_.param("visualization_on", kIsVisualize, kIsVisualize);
  kIsVisualize = true;
  LoadCameraFile(cam_param_yaml_path_);

  /* Camera rotations */

  /**
   * @brief rotation matrix
   * rotation from camera frame to body frame
   */
  Eigen::Matrix3d R_c2b;
  R_c2b << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Quaterniond q_c2b(R_c2b);  ///< quaternion
  // Eigen::Quaterniond q_c2b(0.5, -0.5, 0.5, -0.5);  ///< quaternion
  T_c2b_ = Eigen::Isometry3d::Identity();
  T_c2b_.rotate(q_c2b);

  /* config ceres problem */

  traj_est_prob_ = new ceres::Problem;
  traj_est_prob_->AddParameterBlock(traj_param_, 6);
  for (size_t i = 0; i < 6; i++) {
    traj_est_prob_->SetParameterLowerBound(traj_param_, i, min_param_[i]);
    traj_est_prob_->SetParameterUpperBound(traj_param_, i, max_param_[i]);
  }

  ceres_options_.linear_solver_type = ceres::DENSE_SCHUR;
  ceres_options_.minimizer_progress_to_stdout = false;
  ceres_options_.minimizer_type = ceres::TRUST_REGION;

  /* subscribes */

  odom_sub_ = nh_.subscribe("odom", 1, &BulletEst::DroneOdomCallback, this,
                            ros::TransportHints().tcpNoDelay());
  event_obs_sub_ =
      nh_.subscribe("/cam_bullet_point", 10, &BulletEst::EventObserveCallback,
                    this, ros::TransportHints().tcpNoDelay());
  depth_obs_sub_ = nh_.subscribe("/cam_depth_bullet_point", 10,
                                 &BulletEst::DepthObserveCallback, this,
                                 ros::TransportHints().tcpNoDelay());

  /* advertises */

  traj_pub_ = nh_.advertise<traj_utils::PolyTraj>("trajectory", 10);
  if (kIsVisualize) {
    /* visualize observation in world frame */
    event_obs_3d_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet/event", 10);
    depth_obs_3d_pub_ =
        nh_.advertise<geometry_msgs::PointStamped>("/cam_bullet/depth", 10);
    cam_pose_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/cam_bullet/cam_pose", 10);
    vis_traj_pub_ = nh_.advertise<nav_msgs::Path>("/vis_traj_path", 10);
  }

  // /* solve estimation problem */
  // if (traj_est_prob_->NumResidualBlocks() > 4) {
  //   ceres::Solver::Summary sum;
  //   ceres::Solve(ceres_options_, traj_est_prob_, &sum);
  // }
  // PublishBulletTraj();  // TODO: pub in a callback?
  // TODO multi-object publish
}

/**
 * @brief
 * * get 2D event observations,
 * * compute event residual
 * @param msg
 */
void BulletEst::EventObserveCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  static int num_times;

  ros::Time t_now = msg->header.stamp;

  Eigen::Vector3d p_c;
  Eigen::Vector2d p_pix(msg->point.x, msg->point.y);
  cam_->liftProjective(p_pix, p_c);
  Eigen::Vector3d p_w = T_c2w_ * p_c;

  if (kIsVisualize == true) {
    /* publish observation position in 3d world frame */
    geometry_msgs::PointStamped vis;
    vis.header.stamp = msg->header.stamp;
    vis.header.frame_id = "/world";
    vis.point.x = p_w(0);
    vis.point.y = p_w(1);
    vis.point.z = p_w(2);
    event_obs_3d_pub_.publish(vis);
  }

  num_times++;

  /**
   * @note optimization process
   * this optimization is based on event-based observations
   * Therefore, we use decided if a new object is recorded
   * only via events streams.
   */
  if (IsNewObj(*msg)) {
    first_fire_time_ = t_now;
    if (traj_est_prob_ != NULL) {
      delete traj_est_prob_;
    }
    traj_est_prob_ = new ceres::Problem;
    traj_est_prob_->AddParameterBlock(traj_param_, 6);
    for (size_t i = 0; i < 6; i++)  // set upper and lower bound
    {
      traj_est_prob_->SetParameterLowerBound(traj_param_, i, min_param_[i]);
      traj_est_prob_->SetParameterUpperBound(traj_param_, i, max_param_[i]);
      if (traj_param_[i] <= min_param_[i] || traj_param_[i] >= max_param_[i]) {
        traj_param_[i] = 0.1;
      }
    }
    num_times = 0;
  }

  /* event residuals */
  double dt = (t_now - first_fire_time_).toSec();
  ceres::CostFunction *cost_func =
      EventResidual::Create(p_pix[0], p_pix[1], dt, T_w2c_, cam_);
  traj_est_prob_->AddResidualBlock(
      cost_func, new ceres::HuberLoss(huber_delta_), traj_param_);

  if (traj_est_prob_->NumResidualBlocks() > 4) {
    ceres::Solver::Summary ceres_sum;
    ceres::Solve(ceres_options_, traj_est_prob_, &ceres_sum);
    std::cout << ceres_sum.BriefReport() << std::endl;

    std::cout << "[TRAJ_EST] Obs times: \t" << num_times
              << "\t param:" << std::endl;
    for (size_t i = 0; i < 6; i++) {
      std::cout << "  " << traj_param_[i];
    }
    std::cout << std::endl;

    PublishBulletTraj();
    VisualizeBulletTraj();
  }

  // TODO multi-object publish
}

/**
 * @brief depth observation callback
 *
 * optimize polynomial trajectory based on depth observations
 *
 * @param msg
 */
void BulletEst::DepthObserveCallback(
    const geometry_msgs::PointStamped::ConstPtr &msg) {
  static int depth_obs_times;
  ros::Time t_now = msg->header.stamp;
  depth_obs_times++;
  // prev_dectect_time_ = t_now;

  /* publish position of depth obs in world frame */
  if (kIsVisualize == true) {
    geometry_msgs::PointStamped p;
    Eigen::Vector3d p_cam(msg->point.x, msg->point.y, msg->point.z);
    Eigen::Vector3d p_world = T_c2w_ * p_cam;
    p.header = msg->header;
    p.header.frame_id = "/world";
    p.point.x = p_world(0) * 1e-3;  // millimeter -> meter
    p.point.y = p_world(1) * 1e-3;
    p.point.z = p_world(2) * 1e-3;
    depth_obs_3d_pub_.publish(p);
  }

  /* depth residuals */

  double dt = (t_now - first_fire_time_).toSec();
  /* need to convert millimeters -> meters */
  ceres::CostFunction *cost_func =
      DepthResidual::Create(msg->point.z * 1e-3, dt, T_w2c_);
  traj_est_prob_->AddResidualBlock(
      cost_func, new ceres::HuberLoss(huber_delta_), traj_param_);

  if (traj_est_prob_->NumResidualBlocks() > 4) {
    ceres::Solver::Summary ceres_sum;
    ceres::Solve(ceres_options_, traj_est_prob_, &ceres_sum);
  }
  PublishBulletTraj();
  VisualizeBulletTraj();
}

/**
 * @brief Callback function for drone odometry
 * Transformation of the aircraft attitude into the camera coordinate system to
 * the world coordinate system
 * @param msg
 */
void BulletEst::DroneOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  Eigen::Quaterniond q;
  q.x() = msg->pose.pose.orientation.x;
  q.y() = msg->pose.pose.orientation.y;
  q.z() = msg->pose.pose.orientation.z;
  q.w() = msg->pose.pose.orientation.w;

  T_b2w_.setIdentity();
  T_b2w_.rotate(q);
  T_b2w_.pretranslate(Eigen::Vector3d(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z));

  T_c2w_ = T_b2w_ * T_c2b_;  // transition matrix camera to world
  // T_c2w_（Homogeneous transform mapping cam frame to world frame）
  T_w2c_ = T_c2w_.inverse();

  if (kIsVisualize == true) {
    /* visulize camera pose */
    geometry_msgs::PoseStamped cam_pose;  // pose of camera
    cam_pose.header.stamp = ros::Time::now();
    cam_pose.header.frame_id = "/world";
    cam_pose.pose.position.x = T_c2w_.translation().x();
    cam_pose.pose.position.y = T_c2w_.translation().y();
    cam_pose.pose.position.z = T_c2w_.translation().z();

    Eigen::Quaterniond q_w_cam(T_c2w_.rotation());
    cam_pose.pose.orientation.z = q_w_cam.z();
    cam_pose.pose.orientation.x = q_w_cam.x();
    cam_pose.pose.orientation.y = q_w_cam.y();
    cam_pose.pose.orientation.w = q_w_cam.w();
    camera_pose_visual_.reset();
    camera_pose_visual_.add_pose(T_c2w_.translation(), q_w_cam);
    camera_pose_visual_.publish_by(cam_pose_pub_, cam_pose.header);
  }
}

/**
 * @brief load camera files
 *
 * @param cam pointer, points to camera model
 * @param camera_model_file path of config files
 */
void BulletEst::LoadCameraFile(std::string camera_model_file) {
  if (IsFileExist(camera_model_file)) {
    cam_ = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(
        camera_model_file);
    std::cout << cam_->parametersToString() << std::endl;
  } else {
    ROS_ERROR("cam_param_yaml_path error! Please check the yaml path.");
  }
}

/**
 * @brief publish trajectory via B spline
 *
 * @param traj_param_ trajectory parameters
 * encrypted by initial position and velocity
 * [ x_0, y_0, z_0, v_x0, v_y0, v_z0 ]
 *
 * @todo TODO: remove gravity assumption
 */
void BulletEst::PublishBulletTraj() {
  // debug  (only display x)
  ROS_DEBUG("[bspline] p: %f \tv: %f", traj_param_[2], traj_param_[5]);

  /* calculate projectile time T */
  double T, z0, vz0, a0;
  z0 = traj_param_[2];
  vz0 = traj_param_[5];
  a0 = kGrav_(2);
  if ((vz0 * vz0 - 2 * z0 * a0) < 0) {
    ROS_WARN("[bullet estimation:] Singular Result!");
    return;
  } else {
    T = (-vz0 - sqrt(vz0 * vz0 - 2 * z0 * a0)) / a0;
  }

  ROS_DEBUG("[bspline] T: %f", T);
  /* Poly traj_param_*/
  traj_utils::PolyTraj msg;
  msg.drone_id = 0;
  msg.traj_id = traj_id_;
  traj_id_++;
  msg.start_time = first_fire_time_;

  int order = 5;
  msg.order = order;  // only support 5
  // only 1 segment
  msg.coef_x.resize(6);
  msg.coef_y.resize(6);
  msg.coef_z.resize(6);
  msg.duration.resize(1);
  msg.duration[0] = T;
  for (int i = 0; i <= 5; i++) {
    switch (i) {
      case 0:
      case 1:
      case 2:
        msg.coef_x[i] = 0.0;
        msg.coef_y[i] = 0.0;
        msg.coef_z[i] = 0.0;
        break;
      case 3:
        msg.coef_x[i] = 0.5 * kGrav_(0);
        msg.coef_y[i] = 0.5 * kGrav_(1);
        msg.coef_z[i] = 0.5 * kGrav_(2);
        break;
      case 4:
        msg.coef_x[i] = traj_param_[3];
        msg.coef_y[i] = traj_param_[4];
        msg.coef_z[i] = traj_param_[5];
        break;
      case 5:
        msg.coef_x[i] = traj_param_[0];
        msg.coef_y[i] = traj_param_[1];
        msg.coef_z[i] = traj_param_[2];
        break;
      default:
        break;
    }
  }
  traj_pub_.publish(msg);
}

/**
 * @brief
 *
 */
void BulletEst::VisualizeBulletTraj() {
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  double p[3] = {0};
  double dt = -1;
  do {
    p[0] = traj_param_[0] + traj_param_[3] * dt + 0.5 * kGrav_[0] * dt * dt;
    p[1] = traj_param_[1] + traj_param_[4] * dt + 0.5 * kGrav_[1] * dt * dt;
    p[2] = traj_param_[2] + traj_param_[5] * dt + 0.5 * kGrav_[2] * dt * dt;
    dt += 0.01;
    geometry_msgs::PoseStamped trajp;
    trajp.header = msg.header;
    trajp.pose.orientation.w = 1;
    trajp.pose.orientation.x = 0;
    trajp.pose.orientation.y = 0;
    trajp.pose.orientation.z = 0;
    trajp.pose.position.x = p[0];
    trajp.pose.position.y = p[1];
    trajp.pose.position.z = p[2];
    msg.poses.emplace_back(trajp);
  } while (dt < 3);
  if (msg.poses.size() <= 2) return;
  vis_traj_pub_.publish(msg);
}

/**
 * @brief check if config file exist
 *
 * @param name path
 * @return true exist
 * @return false
 */
inline bool BulletEst::IsFileExist(const std::string &name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

/**
 * @brief check if this obseravation is a new object
 *
 * if time interval > 0.5s , then the point will be allocated
 * into a new bullet observation
 *
 * @param now_point
 * @return true This is a new object
 * @return false This is not a new object
 */
inline bool BulletEst::IsNewObj(geometry_msgs::PointStamped now_point) {
  static geometry_msgs::PointStamped prev_point;
  bool flag = false;
  /* if time interval > 0.5s */
  if ((now_point.header.stamp - prev_point.header.stamp) > ros::Duration(0.5)) {
    flag = true;
    ROS_WARN("[bullet] new_obj detected ");
  } else {
    double dx = now_point.point.x - prev_point.point.x;
    double dy = now_point.point.y - prev_point.point.y;
    double dis = fabs(dx) + fabs(dy);
    if (dis > 200) {
      flag = true;
      ROS_WARN("[bullet] new_obj detected ");
    }
  }
  prev_point = now_point;
  return flag;
}
