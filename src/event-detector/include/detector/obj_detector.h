/**
 * @file obj_detector.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief This file contains object detection and segmentation
 * @version 0.1
 * @date 2021-03-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#if !defined(DETECTOR_OBJDETECT_H_)
#define DETECTOR_OBJDETECT_H_

/* INCLUDES */
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#include "time.h"

/* GLOBAL DEFINES */
#define MAT_ROWS 480  // 240  //800
#define MAT_COLS 640  // 346  //1280

// using namespace cv;
// using namespace std;

class ObjDetector {
 private:
  /* flags */
  bool is_object_;

  /* parameters */
  const int k_rect_kernel_size_ = 3;  // repackage these three lines
  const float k_a_th_ = 0.2;          // using node.getParam
  const float k_b_th_ = -0.1;
  const float k_omega_ = 15;
  const double k_ratio_thres_ = 3.0;
  const float k_min_area_ =
      45;  // the minimum area size to be recognized as object

  /* images */
  cv::Mat event_counts_;     // CV_8UC1
  cv::Mat time_image_;       //  CV_32FC1
  cv::Mat processed_image_;  // init, CV_32FC1
  cv::Mat gray_image_;       // CV_8UC1
  cv::Mat iter_mat_;

  // cv::Rect ori_rect;
  cv::Rect last_rect_;

  /* utilities */

  /* helper functions */
  void GetRect(const cv::Mat &, cv::Rect *O);
  void Visualize(const cv::Mat &src, cv::Mat *dst);
  void InitRoi(const cv::Mat &src, const cv::Point &p, cv::Rect *dst);
  void GetRatio(const cv::Mat &src, double *ratio_x, double *ratio_y);
  void Mask(const cv::Mat &src, cv::Rect *roi);
  bool GaussianModelConvergence(const cv::Mat &msrc, const cv::Rect &rsrc,
                                cv::Mat *mdst, cv::Rect *rdst, int iterTimes);
  void MorphologyOperations(cv::Mat *dst);
  void DebugVisualize(const cv::Mat &src);

  /* inline functions */
  inline void IntTruncate(const int &ref, double *value);
  inline void GetAverage(const cv::Mat m, double *avg, int *num);
  inline void GetVariance(const cv::Mat m, double *var);
  inline bool IsTrue(const cv::Rect &src, const cv::Rect &src2,
                     const double &rx, const double &ry,
                     const double &thres);  // TODO: rename this

 public:
  ObjDetector() {
    processed_image_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
    gray_image_ = cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_8UC1);
  }
  ~ObjDetector() {}

  void Detect();
  void LoadImages(const cv::Mat &event_counts, const cv::Mat &time_image);
  cv::Rect GetDetectRsts(void) { return last_rect_; }
  bool ObjDetected(void) {return is_object_; }
  bool ObjMissed(void) {return !is_object_;}
  cv::Mat GetVisualization(void);

  typedef std::unique_ptr<ObjDetector> Ptr;
};

#endif  // DETECTOR_OBJDETECT_H_
