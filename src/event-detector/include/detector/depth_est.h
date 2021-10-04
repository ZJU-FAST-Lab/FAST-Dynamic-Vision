/**
 * @file depth_est.h
 * @author Haojia Li
 * @brief depth camera detection with event camera
 * @version 0.1
 * @date 2021-01-18
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef DETECTOR_DEPTH_H_
#define DETECTOR_DEPTH_H_

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/rgbd.hpp>
#include "geometry_msgs/PointStamped.h"
#include <memory>
// #include "picojson.h"
#include "ros/ros.h"

/* GLOBAL DEFINES */
#define MAT_ROWS 480  // 240  //800
#define MAT_COLS 640  // 346  //1280


using namespace cv;
using namespace std;
/**
 * @brief depth detcion class
 *  you need use SetEventDetectionRes to give the event camera dection result
 *
 */
class DepthEst {
 private:
  /* parameters */
  const int k_valid_frame_ = 10;  // event camera 检测结果有效的帧数
  const cv::Mat k_depth_camera_intrinsic_ =
      (cv::Mat_<float>(3, 3) << 385.7481384277344, 0.0, 319.36944580078125, 0.0,
       385.7481384277344, 238.4856414794922, 0.0, 0.0, 1.0);
  const cv::Mat k_event_camera_intrinsic_ =
      (cv::Mat_<float>(3, 3) << 5.3633325932983780e+02, 0,
       3.2090009280822994e+02, 0, 5.3631797700847164e+02,
       2.3404853514480661e+02, 0, 0, 1);
  const cv::Mat k_RT_event2depth_ = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0.015, 0,
                                     1, 0, -0.17, 0, 0, 1, 0.0779, 0, 0, 0, 1);
  const cv::Mat k_distort_coeff_ = (cv::Mat_<float>(5, 1) << 0, 0, 0, 0, 0);
  const cv::Size k_event_camera_plane_ = cv::Size(MAT_COLS, MAT_ROWS);

  /* flags */
  bool obj_depth_flag_;
  int valid_count_;
  int64 img_count_;
  bool is_obj_;

  /* depth images */
  cv::Rect roi_rect_;
  cv::Mat depth_gray_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);
  cv::Mat vis_depth_;  // visualization

  /* depth points */
  geometry_msgs::PointStamped depth_p_;

 public:
  /* flags */
  bool istart_;


  DepthEst() {
    valid_count_ = 0;
    is_obj_ = false;
    img_count_ = 0;
    istart_ = false;
    obj_depth_flag_ = false;
  }
  ~DepthEst() {}

  cv::Mat GetDepth() { return depth_gray_; }

  /* utilities */
  void main(const sensor_msgs::ImageConstPtr& msg);
  void SetEventDetectionRes(cv::Rect& roi_rect_);
  void SetEventDetectionRes(bool is_obj_);
  void CropDepthImage(const cv::Mat src, cv::Rect* dst_rect);
  int SegmentDepth(const cv::Mat& img);

  cv::Mat GetDepthVisualization(void) { return vis_depth_; }
  geometry_msgs::PointStamped GetDepthPoint(void) { return depth_p_; }

  /* pointers */
  typedef std::unique_ptr<DepthEst> Ptr;
};

#endif  // DETECTOR_DEPTH_H_
