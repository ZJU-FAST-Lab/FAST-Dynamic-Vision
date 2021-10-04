/**
 * @file motion_compensation.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-03-29
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "detector/motion_compensation.h"

#include "ros/ros.h"

/**
 * @brief warp events and utilize motion compensation
 *
 */
void MotComp::main() {
  Clear();

  /* average imu data */
  if (kIMUType_ == "dji") {
    AvgIMU_DJI();
  } else if (kIMUType_ == "px4") {
    AvgIMU_PX4();
  } else {
    throw std::invalid_argument("IMU type has to be DJI or PX4");
  }

  UpdateCam2body();
  UpdateFC2world();
  IMU_buffer_.clear();  // clear the buffer after averaging

  // notCompensate(&time_img_, &event_counts_);
  rotationalCompensate(&time_img_, &event_counts_);
  // translationalCompensate(&time_img_, &event_counts_);
}

void MotComp::Clear() {
  omega_avg_.setZero();
  events_buffer_.clear();
  imu_size_ = IMU_buffer_.size();

  time_img_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);
  event_counts_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
}

void MotComp::LoadIMUs(const sensor_msgs::ImuConstPtr &imu) {
  IMU_buffer_.push_back(*imu);
}

void MotComp::LoadOdometry(const nav_msgs::Odometry::ConstPtr &odom) {
  odoms_buffer_ = *odom;
}

void MotComp::LoadEvents(const dvs_msgs::EventArray::ConstPtr &emsg) {
  events_buffer_.assign(emsg->events.begin(), emsg->events.end());
  event_size_ = events_buffer_.size();
}

void MotComp::SetIMUType(const string &s) { kIMUType_ = s; }

void MotComp::LoadDepth(const cv::Mat &depth) { depth_img_ = depth; }

cv::Mat MotComp::GetVisualization() {
  cv::Mat m, m_color;
  cv::normalize(time_img_, m, 0, 255, cv::NORM_MINMAX);
  m.convertTo(m, CV_8UC1);
  cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
  // cv::namedWindow("time image");
  // cv::imshow( "window", m_color);
  // cv::waitKey(0);
  return m_color;
}

/**
 * @brief average IMU data, and update the average angular velocity
 * This function is only for DJI imu
 * 
 * body frame is x forwards, z upwards
 * 
 * body frame:  x   y   z
 * imu frame :  x   -y  -z
 * 
 */
void MotComp::AvgIMU_DJI() {
  omega_avg_.setZero();
  imu_size_ = IMU_buffer_.size();
  if (imu_size_ <= 0) {
    omega_ = 0.0f;
  } else {
    for (int i = (imu_size_ >= 10) ? (imu_size_ - 10) : 0; i < imu_size_; i++) {
      omega_avg_[0] += IMU_buffer_[i].angular_velocity.x;
      omega_avg_[1] += -IMU_buffer_[i].angular_velocity.y;
      omega_avg_[2] += -IMU_buffer_[i].angular_velocity.z;
    }
    omega_avg_ = omega_avg_ / static_cast<float>(imu_size_);
    omega_ = omega_avg_.norm();
  }
}

/**
 * @brief average IMU data, and update the average angular velocity
 * This function is only for PX4 imu via mavros
 * 
 * IMU frame is same as body frame
 */
void MotComp::AvgIMU_PX4() {
  omega_avg_.setZero();
  imu_size_ = IMU_buffer_.size();
  if (imu_size_ <= 0) {
    omega_ = 0.0f;
  } else {
    for (int i = (imu_size_ >= 10) ? (imu_size_ - 10) : 0; i < imu_size_; i++) {
      omega_avg_[0] += IMU_buffer_[i].angular_velocity.x;
      omega_avg_[1] += IMU_buffer_[i].angular_velocity.y;
      omega_avg_[2] += IMU_buffer_[i].angular_velocity.z;
    }
    omega_avg_ = omega_avg_ / static_cast<float>(imu_size_);
    omega_ = omega_avg_.norm();
  }
}

/**
 * @brief update fc2world, fc2world is (@botao) The attitude of flight
 * controller corresponds to world frame.
 * @note if not using the frame of flight-controller as body frame, please add
 * external parameters
 * between flight-controller frame and body frame.
 */
void MotComp::UpdateFC2world() {  // TODO: dji & px4
  Eigen::Quaterniond q;
  if (imu_size_ > 1) {
    auto imu = IMU_buffer_[imu_size_ - 1];
    q.x() = imu.orientation.x;
    q.y() = imu.orientation.y;
    q.z() = imu.orientation.z;
    q.w() = imu.orientation.w;
  } else {
    // std::cout << "warning " << std::endl;
  }
  fc2world = Eigen::Isometry3d::Identity();
  fc2world.rotate(q);
}

/**
 * @brief
 * body frame: x -> front, y -> right, z -> up
 * cam frame: z -> front, x -> right, y -> down
 *
 */
void MotComp::UpdateCam2body() {
  cam2body = Eigen::Isometry3d::Identity();
  cam2body.rotate(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY()));
  cam2body.rotate(Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()));
}

/**
 * @brief Accumulate events without any compensation
 *
 * @param timeImg
 * @param eventCount
 */
void MotComp::notCompensate(cv::Mat *timeImg, cv::Mat *eventCount) {
  auto t0 = events_buffer_[0].ts;
  float prevDeltaT = 0.0f;

  for (int i = 0; i < event_size_; i++) {
    dvs_msgs::Event e = events_buffer_[i];
    float deltaT = (e.ts - t0).toSec();

    int ix = e.x;
    int iy = e.y;

    if (!IsWithinTheBoundary(ix, iy)) {
      continue;
    } else {
      int *c = eventCount->ptr<int>(iy, ix);
      float *q = timeImg->ptr<float>(iy, ix);
      *c += 1;
      float v = *q;
      *q += (deltaT - v) / (*c);
    }
  }
}

/**
 * @brief rotational motion compensation
 * The function is used to compensate the rotational ego-motion.
 * We use the integral of angular velocity to represent the change of attitude,
 * which can be represent as "rotation_vector = omega_avg_*deltaT". For
 * computational efficiency,
 * we discretely compute the attitude change "rotation_vector" every 1ms.
 * Then we move each event (x, y) to (x', y') according to its rotational matrix
 * "rotation_matrix_"
 * and record all timestamps of events that have been moved to the same
 * location.
 * Finally, we use the average timestamp of events that moved to the same
 * location as the value(mean-timestamp) of this pixel.
 * @param timeImg
 */
void MotComp::rotationalCompensate(cv::Mat *timeImg, cv::Mat *eventCount) {
  Eigen::Vector3f rotation_vector;
  Eigen::Matrix3f rot_K;
  Eigen::Vector3f eventVec;
  Eigen::Matrix3f rot_skew_mat;

  auto t0 = events_buffer_[0].ts;
  float prevDeltaT = 0.0f;

  for (int i = 0; i < event_size_; i++) {
    dvs_msgs::Event e = events_buffer_[i];
    float deltaT = (e.ts - t0).toSec();
    // eventVec.setZero();
    // rot_skew_mat.setZero();

    /* prepare rotation matrix */
    if (deltaT - prevDeltaT > 1e-3) {  // update rot_K every millisecond
      prevDeltaT = deltaT;
      rotation_vector = omega_avg_ * deltaT;
      rot_skew_mat = vectorToSkewMat(rotation_vector);
      rotation_matrix_ = rot_skew_mat.exp();  // vector space to Lee spin space
      rot_K = K * rotation_matrix_.transpose() * K_inverse;  // projection
    }

    /* prepare event vector */
    eventVec[0] = e.x;
    eventVec[1] = e.y;
    eventVec[2] = 1;
    eventVec = rot_K * eventVec;  // event warp
    ConvertToHomogeneous(&eventVec);
    // cout << "x " << eventVec[0] << " y " << eventVec[1] << " z " <<
    // eventVec[2] << endl;

    int ix = static_cast<int>(eventVec[0]);
    int iy = static_cast<int>(eventVec[1]);

    if (IsWithinTheBoundary(ix, iy)) {
      int *c = eventCount->ptr<int>(iy, ix);
      float *q = timeImg->ptr<float>(iy, ix);
      *c += 1;
      float v = *q;
      *q += (deltaT - v) / (*c);
      // eventCount->at<uchar>(iy, ix) += 1;
      // float q = time_img_.at<float>(iy, ix);
      // timeImg->at<float>(iy, ix) +=
      //     (deltaT - q) / eventCount->at<uchar>(iy, ix);
      // timeSumImg.at<float>(iy, ix) += deltaT;
    }
  }

  // cv::divide(timeSumImg, event_counts_, time_img_, 1.0f, CV_32FC1);

  ///////////// to debug ////////////////////////
  // double maxValue;
  // maxValue = *max_element(timeImg->begin<float>(), timeImg->end<float>());
  // std::cout << "time image max " << maxValue << std::endl;
  // maxValue = *max_element(eventCount->begin<uchar>(),
  // eventCount->end<uchar>()); std::cout << "event count max " << maxValue <<
  // std::endl; GetVisualization();
  //////////////////////////////////////////////
}

/**
 * @brief translational motion-compensation
 *
 * Finally, we re-project each pixel to event camera's frame and use the average
 * timestamp
 * of events with same coordinate as the value(mean-timestamp) of this pixel.
 * @param timeImg
 */
void MotComp::translationalCompensate(cv::Mat *timeImg, cv::Mat *eventCount) {
  Point MaxLoc;
  double maxTime;
  cv::Mat TMat = *timeImg;

  minMaxLoc(TMat, NULL, &maxTime, NULL, NULL);
  int maxTS = static_cast<int>(maxTime * 1000 + 1);

  ////////// debug /////////
  // std::cout << "Value at MaxLoc: " << TMat.at<float>(MaxLoc.y, MaxLoc.x)
  // << std::endl;
  //  cout << "maxTS:" << maxTS << std::endl;
  // might have format problems like float->double, use this line to check
  // it.
  //////////////////////////

  /* Get the max timestamp in timeImg and round up for discretely computing the
   * translational matrix.
   */
  std::vector<Isometry3d> trans_vector;
  trans_vector.resize(maxTS);

  for (int i = 0; i < maxTS; i++) {
    trans_body = Eigen::Isometry3d::Identity();
    trans_body.pretranslate(Eigen::Vector3d(
        odoms_buffer_.twist.twist.linear.x * static_cast<double>(i),
        odoms_buffer_.twist.twist.linear.y * static_cast<double>(i),
        odoms_buffer_.twist.twist.linear.z * static_cast<double>(i)));
    trans_vector[i] = cam2body.inverse() * fc2world.inverse() * trans_body *
                      fc2world * cam2body;
  }

  cv::Mat eventCount_new =
      cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
  cv::Mat TMat_new = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);

  for (int i = 0; i < MAT_COLS; i++) {
    for (int j = 0; j < MAT_ROWS; j++) {
      /* read timestamp from TMat */
      int counts = eventCount->at<uchar>(j, i);
      float deltaT = timeImg->at<float>(j, i);

      if (deltaT > 0.001 && deltaT < 1) {
        Eigen::Vector3f eventVec;
        eventVec[0] = i;
        eventVec[1] = j;
        eventVec[2] = 1;

        if (IsDepthInRange(depth_img_, i, j, 300, 10000)) {
          /* read depth from depth image */
          double z_depth = ReadDepth(depth_img_, i, j);
          /* update event vector */
          eventVec = K_inverse * eventVec * z_depth;
          Eigen::Vector4d depthInCam = {eventVec[0], eventVec[1], eventVec[2],
                                        1};

          int millisecond = static_cast<int>(1000.f * deltaT);
          depthInCam = trans_vector[millisecond] * depthInCam;

          eventVec[0] = static_cast<float>(depthInCam[0]);
          eventVec[1] = static_cast<float>(depthInCam[1]);
          eventVec[2] = static_cast<float>(depthInCam[2]);
          eventVec = static_cast<float>(1.0f / z_depth) * K * eventVec;
        }

        int iy = static_cast<int>(eventVec[1]);
        int ix = static_cast<int>(eventVec[0]);

        if (IsWithinTheBoundary(ix, iy)) {
          int *ec = eventCount_new.ptr<int>(iy, ix);
          float *tm = TMat_new.ptr<float>(iy, ix);
          *ec += 1;
          float v = *tm;
          *tm += (deltaT - v) / (*ec);
        }
      }
    }
  }

  *eventCount = eventCount_new;
  *timeImg = TMat_new;

  ///////////// to debug ////////////////////////
  // double maxValue;
  // maxValue = *max_element(timeImg->begin<float>(), timeImg->end<float>());
  // std::cout << "time image max " << maxValue << std::endl;
  // maxValue = *max_element(eventCount->begin<uchar>(),
  // eventCount->end<uchar>()); std::cout << "event count max " << maxValue <<
  // std::endl; GetVisualization();
  //////////////////////////////////////////////
}

/**
 * @brief rotational and translational compensation in a single function
 *
 * deprecated
 *
 * @param timeImg
 * @param eventCount
 */
void MotComp::RotTransCompensate(cv::Mat *timeImg, cv::Mat *eventCount) {
  cv::Mat T = *timeImg;
  cv::Mat C = *eventCount;
  cv::Mat timeSumImg = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);
  Eigen::Vector3f rotation_vector;
  Eigen::Matrix3f rot_K;
  Eigen::Vector3f eventVec;
  Eigen::Matrix3f rot_skew_mat;

  auto t0 = events_buffer_[0].ts;
  float prevDeltaT = 0.0f;

  std::vector<Isometry3d> trans_vector;
  for (int i = 0; i < 50; i++) {
    trans_body = Eigen::Isometry3d::Identity();
    trans_body.pretranslate(Eigen::Vector3d(
        -odoms_buffer_.twist.twist.linear.x * static_cast<double>(i),
        -odoms_buffer_.twist.twist.linear.y * static_cast<double>(i),
        -odoms_buffer_.twist.twist.linear.z * static_cast<double>(i)));
    trans_vector[i] = cam2body.inverse() * fc2world.inverse() * trans_body *
                      fc2world * cam2body;
  }

  for (int j = 0; j < event_size_; j++) {
    dvs_msgs::Event *e = &events_buffer_[j];
    float deltaT = (e->ts - t0).toSec();

    if (deltaT - prevDeltaT > 1e-3) {
      prevDeltaT = deltaT;
      rotation_vector = omega_avg_ * deltaT;

      rot_skew_mat = vectorToSkewMat(rotation_vector);
      rotation_matrix_ = rot_skew_mat.exp();  // vector space to Lee spin space

      rot_K = K * rotation_matrix_.transpose() * K_inverse;  // projection
    }

    /* prepare event vector */
    eventVec[0] = e->x;
    eventVec[1] = e->y;
    eventVec[2] = 1;
    eventVec = rot_K * eventVec;  // event warp

    ConvertToHomogeneous(&eventVec);

    int ix = static_cast<int>(eventVec[0]);
    int iy = static_cast<int>(eventVec[1]);

    if (IsWithinTheBoundary(ix, iy)) {
      if (IsDepthInRange(depth_img_, ix, iy, 500, 10000)) {
        double z_depth = ReadDepth(depth_img_, ix, iy);
        eventVec = K_inverse * eventVec * z_depth;

        Eigen::Vector4d depthInCam = {eventVec[0], eventVec[1], eventVec[2], 1};

        int millisecond = static_cast<int>(1000.f * deltaT);
        depthInCam = trans_vector[millisecond] * depthInCam;

        eventVec[0] = static_cast<float>(depthInCam[0]);
        eventVec[1] = static_cast<float>(depthInCam[1]);
        eventVec[2] = static_cast<float>(depthInCam[2]);
        eventVec = static_cast<float>(1.0f / z_depth) * K * eventVec;
      }
    }

    int ny = static_cast<int>(eventVec[1]);
    int nx = static_cast<int>(eventVec[0]);

    if (IsWithinTheBoundary(nx, ny)) {
      C.at<uchar>(ny, nx) += 1;
      timeSumImg.at<float>(ny, nx) += deltaT;
    }
  }

  cv::divide(timeSumImg, C, T, 1.0, CV_32FC1);
}

/**
 * @brief read depth value from the depth image
 *
 * @param I depth image
 * @param v event vector
 * @return double
 */
// deprecated: too slow
// inline double MotComp::ReadDepth(const cv::Mat& I, const int& x, const int&
// y) {
//   float depth = I.at<uint16_t>(x, y);
//   return sqrt(pow2(depth) / (1.0 + pow2(y - K(0, 2)) / pow2(K(0, 0)) +
//                              pow2(x - K(1, 2)) / pow2(K(1, 1))));
// }
inline double MotComp::ReadDepth(const cv::Mat &I, const int &x, const int &y) {
  float depth = I.at<uint16_t>(y, x);
  return sqrt((depth * depth) /
              (1.0 + (x - K(0, 2)) * (x - K(0, 2)) / (K(0, 0) * K(0, 0)) +
               (y - K(1, 2)) * (y - K(1, 2)) / (K(1, 1) * K(1, 1))));
}

/**
 * @brief convert a vector to homogeneous coordinates
 *
 * @param v
 */
inline void MotComp::ConvertToHomogeneous(Eigen::Vector3f *v) {
  (*v)[0] = (*v)[0] / (*v)[2];
  (*v)[1] = (*v)[1] / (*v)[2];
  (*v)[2] = 1;
}

/**
 * @brief check if this event in the bounds
 *
 * @param v
 * @return true: 0 < x < MAT_COLS && 0 < y < MAT_COLS
 * @return false
 */
inline bool MotComp::IsWithinTheBoundary(const Eigen::Vector3f &v) {
  return (static_cast<int>(v[0]) >= 0 && static_cast<int>(v[0]) < MAT_COLS &&
          static_cast<int>(v[1]) >= 0 && static_cast<int>(v[1]) < MAT_ROWS);
}
inline bool MotComp::IsWithinTheBoundary(const int &x, const int &y) {
  return (x >= 0 && x < MAT_COLS && y >= 0 && y < MAT_ROWS);
}

/**
 * @brief check if event's corresponding point in depth image at a valid
 * distance
 *
 * @param depthImg
 * @param x, y: event's corresponding point
 * @param min minimum depth value
 * @param max maximum depth value
 * @return true: this event in range
 * @return false
 */
inline bool MotComp::IsDepthInRange(const cv::Mat &depthImg, const int &x,
                                    const int &y, int min, int max) {
  return depthImg.at<uint16_t>(y, x) <= max &&
         depthImg.at<uint16_t>(y, x) >= min;
}