#ifndef VISION_TUNE_UTILS_VISION_UTILS_HPP_
#define VISION_TUNE_UTILS_VISION_UTILS_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 

#include "vision_tune/utils/config_utils.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

namespace vision_tune::utils
{

  struct detection_result
  {
    bool detected = false;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float area = 0.0f;
    cv::Rect bbox;
    int class_id = -1;
    float score = 0.0f;
  };

  cv::Mat make_bird_view(
      const cv::Mat &frame,
      const std::vector<cv::Point2f> &src_points,
      const std::vector<cv::Point2f> &dst_points,
      int output_width,
      int output_height);

  cv::Mat make_hsv_mask(
      const cv::Mat &bgr_image,
      const hsv_range &range);

  detection_result find_largest_blob(const cv::Mat &binary_image);

  cv::Mat draw_detection_result(
      const cv::Mat &base_image,
      const detection_result &result);

} // namespace vision_tune::utils

namespace vision_tune::Cal_utils
{

  struct ObjectPos
  {
    double dist;
    double theta;
  };

  // 카메라 C
  // |\
  // | \
  // |  \  <- 카메라 광선
  // h   \  CP_
  // |    \
  // |     P'────────P  (공)
  // C'────
  // 선분 C_P_    PP_

  // 2D to 3D 변환: 삼각측량법 (거리 + 방향 추정)
  // tilt       : 카메라 틸트 각도 (degree)
  // h          : 카메라 높이 (mm)
  // focalLength: 카메라 초점거리 (px)
  // principalPt: 주점 (px)
  // pixelPt    : 공 이미지 좌표 (px)
  ObjectPos calcObjectDistance(double tilt, double h, const cv::Point2d &focalLength, const cv::Point2d &principalPt, const cv::Point2d &pixelPt)
  {
    ObjectPos pos;

    // 픽셀 to 정규화 좌표
    cv::Point2d normPt;
    normPt.x = (pixelPt.x - principalPt.x) / focalLength.x;
    normPt.y = (pixelPt.y - principalPt.y) / focalLength.y;
    double u = normPt.x;
    double v = normPt.y;

    // 삼각측량 계산
    double CC_ = h;                                                 // 카메라 높이 (mm)
    double C_P_ = CC_ * tan((M_PI / 2) + tilt * DEG2RAD - atan(v)); // 공이 멀어질 수록 발산

    // 카메라와 공 사이 비율 계산
    double CP_ = sqrt(CC_ * CC_ + C_P_ * C_P_);
    double Cp_ = sqrt(1 + v * v);
    double PP_ = u * CP_ / Cp_;

    pos.dist = sqrt(C_P_ * C_P_ + PP_ * PP_);
    pos.theta = -atan2(PP_, C_P_) * RAD2DEG;

    return pos;
  }

}

#endif