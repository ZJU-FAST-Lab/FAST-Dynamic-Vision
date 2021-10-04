/**
 * @file obj_detector.cpp
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 0.1
 * @date 2021-04-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "detector/obj_detector.h"

/**
 * @brief read data for detection object
 * @param event_counts
 * @param time_image
 */
void ObjDetector::LoadImages(const cv::Mat &event_counts,
                             const cv::Mat &time_image) {
  event_counts_ = event_counts;
  time_image_ = time_image;
}

void ObjDetector::Detect() {
  /* MorphologyOperations */
  MorphologyOperations(&gray_image_);

  /* find the maximum value and its location */
  cv::Point max_loc;  // center-of-mass
  cv::minMaxLoc(gray_image_, NULL, NULL, NULL, &max_loc);

  /* initialize ROI convergence */
  cv::Rect ori_rect;

  InitRoi(gray_image_, max_loc, &ori_rect);
  GetRect(gray_image_, &ori_rect);

  cv::blur(gray_image_, iter_mat_, cv::Size(5, 5));
  cv::threshold(iter_mat_, iter_mat_, 50, 255, cv::THRESH_TOZERO);

  cv::Rect temp_rect;
  bool is_roi = GaussianModelConvergence(iter_mat_, ori_rect, &processed_image_,
                                         &temp_rect, 2);

  if (is_roi) {
    /* if found the smallest ROI */
    double ratio_x = 0, ratio_y = 0;
    GetRatio(processed_image_, &ratio_x, &ratio_y);
    if (IsTrue(temp_rect, last_rect_, ratio_x, ratio_y, k_ratio_thres_)) {
      Mask(processed_image_, &temp_rect);
    }
    last_rect_ = temp_rect;
    is_object_ = true;

  } else {
    is_object_ = false;
  }

  /* corner case: check if roi large enough */
  if (last_rect_.area() < k_min_area_) {
    is_object_ = false;  // reject small roi
  } else if (is_object_) {
    cv::Rect min_obj = cv::boundingRect(gray_image_(last_rect_));
    last_rect_.x += min_obj.x;
    last_rect_.y += min_obj.y;
    last_rect_.width = min_obj.width;
    last_rect_.height = min_obj.height;
  }
}

/**
 * @brief image processing before bounding box convergence
 *
 * @param dst CV_8UC1 image
 */
void ObjDetector::MorphologyOperations(cv::Mat *dst) {
  cv::Mat normed_time_image =
      cv::Mat::zeros(cv::Size(MAT_COLS, MAT_ROWS), CV_32FC1);

  cv::Mat m;  // image matrix

  /* normalization */
  cv::normalize(time_image_, normed_time_image, 0, 1, cv::NORM_MINMAX);

  /* thresholding */
  float thres = cv::mean(normed_time_image, event_counts_)[0] +
                k_a_th_ * k_omega_ + k_b_th_;
  thres = cv::mean(normed_time_image, event_counts_)[0];
  cv::threshold(normed_time_image, m, thres, 1, cv::THRESH_TOZERO);

  /* Gaussian Blur */
  cv::blur(m, m, cv::Size(5, 5));
  cv::normalize(m, m, 0, 255, cv::NORM_MINMAX);

  /* Morphology */
  cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT, cv::Size(k_rect_kernel_size_, k_rect_kernel_size_),
      cv::Point(-1, -1));
  cv::morphologyEx(m, m, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

  /* 图像按位平方，增加对比度 */
  m = m.mul(m);
  cv::normalize(m, m, 0, 255, cv::NORM_MINMAX);
  m.convertTo(*dst, CV_8UC1);
}

/**
 * @brief Gaussian Model iteration to find out the optimal ROI
 *
 *
 * we use second order central moments of events clusters to represent the
 * variance of the cluster, then we compute the standard deviation for
 * ROI convergence
 *
 * @param src
 * @param dst output image
 * @param iterTimes
 * @return true
 * @return false
 */
bool ObjDetector::GaussianModelConvergence(const cv::Mat &msrc,
                                           const cv::Rect &rsrc, cv::Mat *mdst,
                                           cv::Rect *rdst, int iterTimes) {
  cv::Rect iter_rect = rsrc;

  cv::Mat matrix = msrc(iter_rect);  // matirx for iteration

  cv::Rect final_rect = iter_rect;

  for (int i = 0; i < iterTimes; i++) {
    cv::Moments m = cv::moments(matrix);
    cv::Point center_mass(static_cast<int>(m.m10 / m.m00),
                          static_cast<int>(m.m01 / m.m00));

    double width, height;
    int epsilon_x = static_cast<int>(sqrt(m.mu20 / m.m00));
    int epsilon_y = static_cast<int>(sqrt(m.mu02 / m.m00));
    width = 4 * epsilon_x;
    height = 4 * epsilon_y;

    if (width == 0 || height == 0) {
      final_rect.width = 0;
      final_rect.height = 0;
      *mdst = matrix;
      *rdst = final_rect;
      return false;
    }

    IntTruncate(matrix.cols, &width);
    IntTruncate(matrix.rows, &height);

    iter_rect.width = static_cast<int>(width);
    iter_rect.height = static_cast<int>(height);
    iter_rect.x = center_mass.x - iter_rect.width / 2;
    iter_rect.y = center_mass.y - iter_rect.height / 2;
    GetRect(matrix, &iter_rect);
    final_rect.x += iter_rect.x;
    final_rect.y += iter_rect.y;
    final_rect.width = iter_rect.width;
    final_rect.height = iter_rect.height;

    matrix = matrix(iter_rect);
  }

  double variance = 0.0f;
  GetVariance(matrix, &variance);

  if (variance < 1500) {
    final_rect.width = 0;
    final_rect.height = 0;
    *mdst = matrix;
    *rdst = final_rect;
    return false;
  } else {
    *mdst = matrix;
    *rdst = final_rect;
    return true;
  }
}

/**
 * @brief
 *
 * @param src
 * @param roi
 */
void ObjDetector::Mask(const cv::Mat &src, cv::Rect *roi) {
  cv::Mat img_bool, canny, labels, stats, centroids;
  src.convertTo(img_bool, CV_8UC1);

  cv::threshold(img_bool, img_bool, 0, 255, cv::THRESH_OTSU);

  int n_connected_components =
      cv::connectedComponentsWithStats(img_bool, labels, stats, centroids);

  /* init order map */
  std::map<float, int> mean_time_clusters;

  for (size_t i = 0; i < n_connected_components; i++) {
    if (stats.at<int>(i, cv::CC_STAT_AREA) < k_min_area_) {
      continue;  // skip too small areas
    } else {
      cv::Rect2f rect(stats.at<int>(i, cv::CC_STAT_LEFT),
                      stats.at<int>(i, cv::CC_STAT_TOP),
                      stats.at<int>(i, cv::CC_STAT_WIDTH),
                      stats.at<int>(i, cv::CC_STAT_HEIGHT));

      /* crop image */
      cv::Mat cropped_img = src(rect);

      /* get mean time of the ROI */
      float avg = cv::mean(cropped_img, cropped_img)[0];

      mean_time_clusters.insert(std::pair<float, int>(avg, i));
    }
  }

  if (!mean_time_clusters.empty()) {
    auto highest_mean_time = mean_time_clusters.end();
    highest_mean_time--;  // select cluster with the highest mean time
    int idx = highest_mean_time->second;

    /* update ROI */
    // cv::Rect2f r(stats.at<int>(idx, cv::CC_STAT_LEFT),
    //              stats.at<int>(idx, cv::CC_STAT_TOP),
    //              stats.at<int>(idx, cv::CC_STAT_WIDTH),
    //              stats.at<int>(idx, cv::CC_STAT_HEIGHT));
    roi->x += stats.at<float>(idx, cv::CC_STAT_LEFT);
    roi->y += stats.at<int>(idx, cv::CC_STAT_TOP);
    roi->width = stats.at<int>(idx, cv::CC_STAT_WIDTH);
    roi->height = stats.at<int>(idx, cv::CC_STAT_HEIGHT);
  }
}

/**
 * @brief   1/4 of image is set to be initial ROI of Gaussian Model Cluster
 *
 * @param src
 * @param p
 * @param dst
 */
void ObjDetector::InitRoi(const cv::Mat &src, const cv::Point &p,
                          cv::Rect *dst) {
  dst->width = src.cols / 2;
  dst->height = src.rows / 2;
  dst->x = p.x - src.cols / 4;
  dst->y = p.y - src.rows / 4;
}

/**
 * @brief
 *
 * @param src
 * @param ratio_x
 * @param ratio_y
 */
void ObjDetector::GetRatio(const cv::Mat &src, double *ratio_x,
                           double *ratio_y) {
  double cnt_x = 0, cnt_y = 0;
  for (int i = 0; i < src.rows; i++) {
    for (int j = 0; j < src.cols; j++) {
      if (src.at<uchar>(i, j)) {
        cnt_x++;
        break;
      }
    }
  }

  *ratio_x = cnt_x / src.rows;

  for (int j = 0; j < src.cols; j++) {
    for (int i = 0; i < src.rows; i++) {
      if (src.at<uchar>(i, j)) {
        cnt_y++;
        break;
      }
    }
  }

  *ratio_y = cnt_y / src.cols;
}

/**
 * @brief
 *
 * @param src source image
 * @param O rectangle
 * @return cv::Rect
 */
void ObjDetector::GetRect(const cv::Mat &src, cv::Rect *O) {
  if (O->x < 0) {
    O->width -= O->x;  // left edge moves right
    O->x = 0;
  }

  if (O->x + O->width > src.cols - 1) {
    O->width = src.cols - 1 - O->x;  // right edge truncates
  }

  if (O->y < 0) {
    O->height -= O->y;
    O->y = 0;
  }

  if (O->y + O->height > src.rows - 1) {
    O->height = src.rows - 1 - O->y;
  }
}

/**
 * @brief Type conversion and image visualization
 *    TODO: Node Handle required
 * @param src
 */
cv::Mat ObjDetector::GetVisualization() {
  cv::Mat m, m_color;
  // gray_image_.convertTo(m, CV_8UC1);
  cv::normalize(gray_image_, m, 0, 255, cv::NORM_MINMAX);
  cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
  if (is_object_) {  // draw bounding box
    cv::rectangle(m_color, last_rect_, cv::Scalar(0, 255, 0), 2, cv::LINE_8, 0);
  }
  return m_color;
}

/**
 * @brief truncate value w.r.t ref
 *
 * @param ref
 * @param value
 */
inline void ObjDetector::IntTruncate(const int &ref, double *value) {
  if (*value > ref) {
    *value = ref;
  }
}

/**
 * @brief compute average (in order to compute variance)
 *
 * @param m
 * @param avg
 * @param num
 */
inline void ObjDetector::GetAverage(const cv::Mat m, double *avg, int *num) {
  // int n = *num;
  double sum;
  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      if (m.at<uchar>(i, j)) {
        uchar v = m.at<uchar>(i, j);
        sum += v;
        (*num)++;
      }
    }
  }
  *avg = sum / *num;
}

/**
 * @brief compute variance
 *
 * @param m
 * @param var
 */
inline void ObjDetector::GetVariance(const cv::Mat m, double *var) {
  // double variance = *var;
  double average = 0.0f, square_sum = 0.0f;
  int n = 0;
  GetAverage(m, &average, &n);

  for (int i = 0; i < m.rows; i++) {
    for (int j = 0; j < m.cols; j++) {
      if (m.at<uchar>(i, j)) {
        uchar t = m.at<uchar>(i, j);
        square_sum += (t - average) * (t - average);
      }
    }
  }

  *var = square_sum / n;
}

inline bool ObjDetector::IsTrue(const cv::Rect &src, const cv::Rect &src2,
                                const double &rx, const double &ry,
                                const double &thres) {
  return rx < 0.25 || ry < 0.25 ||
         static_cast<double>(src.width * src.height) /
                 static_cast<double>(src2.width * src2.height) >
             thres;
}

/**
 * @brief Debug
 * @param src image to visualize
 */
void ObjDetector::DebugVisualize(const cv::Mat &src) {
  cv::Mat m, m_color;
  cv::normalize(src, m, 0, 255, cv::NORM_MINMAX);
  m.convertTo(m, CV_8UC1);
  cv::applyColorMap(m, m_color, cv::COLORMAP_JET);
  cv::namedWindow("visualize");
  cv::imshow("window", m_color);
  cv::waitKey(0);
}