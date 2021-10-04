/**
 * @file depth_est.cpp
 * @author Haojia Li
 * @brief depth camera detection with event camera
 * @version 0.1
 * @date 2021-01-18
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "detector/depth_est.h"
#include <opencv2/rgbd.hpp>

/**
 * @brief Set the event detecion result object
 *
 * @param roi_rect roi rectangle
 */
void DepthEst::SetEventDetectionRes(cv::Rect& roi_rect) {
  is_obj_ = true;
  roi_rect_ = roi_rect;
  valid_count_ = 0;
}

/**
 * @brief Set the event detecion result object
 *
 * @param is_obj   is valid object
 */
void DepthEst::SetEventDetectionRes(bool is_obj) {
  is_obj_ = is_obj;
  if (is_obj) {
    valid_count_ = 0;
  }
}

/**
 * @brief Depth camera callback
 *
 * @param msg
 */
void DepthEst::main(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  /* init depth image */
  cv::Mat depth_gray_u8(msg->height, msg->width, CV_8UC1);
  depth_gray_ = cv::Mat::zeros(cv::Size(msg->height, msg->width), CV_8UC1);

  /* register depth data to an external camera */
  cv::rgbd::registerDepth(k_depth_camera_intrinsic_, k_event_camera_intrinsic_,
                          k_distort_coeff_, k_RT_event2depth_, cv_ptr->image,
                          k_event_camera_plane_, depth_gray_, false);

  /* morphology operations */
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5),
                                             cv::Point(-1, -1));
  morphologyEx(depth_gray_, depth_gray_, CV_MOP_CLOSE, kernel,
               cv::Point(-1, -1), 1);
  depth_gray_.convertTo(depth_gray_u8, CV_8UC1, 1.0 / 256);

  /* if number of frames > k_valid_frame_, then the rest
  * detection will be regrad as invalid
  */
  valid_count_++;
  if (valid_count_ > k_valid_frame_) {
    is_obj_ = false;
  }

  if (is_obj_) {  
    cv::Rect r(roi_rect_);
    CropDepthImage(depth_gray_, &r);

    cv::Mat obj_img_u8 = depth_gray_u8(r);
    cv::Mat obj_img = depth_gray_(r);

    float u = r.x; 
    float v = r.y;  

    int loc = SegmentDepth(obj_img_u8);

    if (obj_depth_flag_) {
      cv::Mat mask_range;
      cv::inRange(obj_img_u8, loc - 1, loc + 1, mask_range);

      /* compute mean and std */
      cv::Scalar mean, std;
      cv::meanStdDev(obj_img, mean, std, mask_range);

      if ((std[0] < 100) && (std[0] > 0)) {
        auto m = cv::moments(mask_range, true);

        float roi_u = m.m10 / m.m00;
        float roi_v = m.m01 / m.m00;
        u += roi_u;
        v += roi_v;
        float u0 = k_event_camera_intrinsic_.at<float>(0, 2);
        float v0 = k_event_camera_intrinsic_.at<float>(1, 2);
        float fx = k_event_camera_intrinsic_.at<float>(0, 0);
        float fy = k_event_camera_intrinsic_.at<float>(1, 1);

        /* if detections on depth image are too closed to image edge, 
         * we will not fuse the coordinates from depth detection,
         * instead, we only fuse approximate depth data
         *
         */
        float du =
            (mask_range.cols - roi_u) > roi_u ? roi_u : mask_range.cols - roi_u;
        float dv =
            (mask_range.rows - roi_v) > roi_v ? roi_v : mask_range.rows - roi_v;

        // geometry_msgs::PointStamped depth_p;
        depth_p_.header.stamp = msg->header.stamp;
        depth_p_.header.frame_id = "/world";
        depth_p_.point.x = mean[0] * (u - u0) / fx;
        depth_p_.point.y = mean[0] * (v - v0) / fy;
        depth_p_.point.z = mean[0];  // millimeters

        /* WARNING: deprecated */
        /* chech the center of mass of the detection point,
         * if the detection point is too close to the edge,
         * we will ignore the center of mass,
         * and only use the depth to optimize.
        */
        // if ((du < 20) || (dv < 20)) {
        //   depth_p.header.frame_id = "/only_depth";
        // }
      }
    }

    /* visualization */
    cv::Mat vis_depth_(msg->height, msg->width, CV_8UC3);
    cv::applyColorMap(depth_gray_u8, vis_depth_, cv::COLORMAP_JET);
    cv::rectangle(vis_depth_, r, cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
  }
}

/**
 * @brief
 *
 * @param src source depth image
 * @param dst_rect pointer to destination rectangle
 */
void DepthEst::CropDepthImage(const cv::Mat src, cv::Rect* dst_rect) {
  dst_rect->x =
      (dst_rect->x - dst_rect->width / 2) < 0 ? 0 : (dst_rect->x - dst_rect->width / 2);
  dst_rect->y =
      (dst_rect->y - dst_rect->height / 2) < 0 ? 0 : (dst_rect->y - dst_rect->height / 2);
  dst_rect->height *= 2;
  dst_rect->width *= 2;
  if ((dst_rect->height + dst_rect->y) > src.rows)
    dst_rect->height = src.rows - dst_rect->y;
  if ((dst_rect->width + dst_rect->x) > src.cols) dst_rect->width = src.cols - dst_rect->x;
}

/**
 * @brief  segment depth belongs to objects on the histogram
 *
 * @param img depth image
 * @return int record
 */
int DepthEst::SegmentDepth(const cv::Mat& img) {
  cv::MatND hist_info;
  const int hist_size = 128;
  float hist_range[] = {1, 128};
  const float* hist_ranges[] = {hist_range};
  const int chs = 0;  // image channels

  /* compute histogram from depth image */
  cv::calcHist(&img, 1, &chs, cv::Mat(), hist_info, 1, &hist_size,
               &hist_ranges[0]);

  int record;
  obj_depth_flag_ = false;
  for (record = 0; record < hist_size; record++) {
    if (hist_info.at<int>(record) > 0.02 * img.rows * img.cols) {
      record++; 
      obj_depth_flag_ = true;
      break;
    }
  }
  return record;
}
