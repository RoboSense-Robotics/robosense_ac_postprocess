/************************************************************************
Copyright 2025 RoboSense Technology Co., Ltd

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
************************************************************************/
#ifndef COMMON_CAMERA_MODEL_H
#define COMMON_CAMERA_MODEL_H

#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <memory>

namespace robosense {
namespace postprocess {

class PinholeModel {
public:
  using Ptr = std::shared_ptr<PinholeModel>;

public:
  PinholeModel(const cv::Mat &transform_matrix, const cv::Mat &camera_matrix,
               const cv::Mat &distor_coe, const cv::Size &image_size);
  ~PinholeModel() = default;

  bool ProjectPts(const std::vector<Eigen::Vector3d> &pts_3d,
                  std::vector<cv::Point2i> &pts_2d, bool distort_flag);
  void LiftRay(const std::vector<cv::Point2i> &pts_2d,
               std::vector<Eigen::Vector3d> &pts_3d);
  void UndistortPts(const std::vector<cv::Point2i> &distorted_pts_2d,
                    std::vector<cv::Point2i> &undistorted_pts_2d);
  void UndistortImg(const cv::Mat &raw_img, cv::Mat &undistorted_img) {
    cv::remap(raw_img, undistorted_img, map_x_, map_y_, cv::INTER_LINEAR);
  }

  cv::Size GetSize() const { return image_size_; }
  cv::Mat GetTransformMatrix() const { return transform_matrix_; }
  cv::Mat GetIntriMatrix() const { return camera_matrix_; }
  cv::Mat GetDistort() const { return distor_coe_; }
  void UndistortImage(const cv::Mat &raw_img, cv::Mat &undistorted_img) {
    cv::remap(raw_img, undistorted_img, map_x_, map_y_, cv::INTER_LINEAR);
  }
  // basic_type::CameraID getCameraID() const { return camera_id_; }

private:
  void DistortPts(const std::vector<cv::Point2d> &pts_2d,
                  std::vector<cv::Point2d> &distorted_pts_2d);
  // void UndistortPts(const std::vector<Eigen::Vector3d> &distorted_pts_3d,
  //   std::vector<Eigen::Vector3d> &pts_3d)

private:
  cv::Mat map_x_;
  cv::Mat map_y_;
  cv::Size image_size_;

  // mono camera model
  cv::Mat camera_matrix_;
  cv::Mat distor_coe_;
  cv::Mat transform_matrix_;

  // cv version
  cv::Mat r_vec_;
  cv::Mat t_vec_;

  std::vector<double> norm_plane_range_;
};

} // namespace postprocess
} // namespace robosense

#endif // COMMON_CAMERA_MODEL_H