/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "postprocess/camera_model.h"
#include "opencv2/calib3d.hpp"

namespace robosense {
namespace postprocess {

PinholeModel::PinholeModel(const cv::Mat &transform_matrix,
                           const cv::Mat &camera_matrix,
                           const cv::Mat &distor_coe,
                           const cv::Size &image_size) {
  transform_matrix_ = transform_matrix;
  camera_matrix_ = camera_matrix;
  distor_coe_ = distor_coe;
  image_size_ = image_size;

  t_vec_ = cv::Mat::zeros(3, 1, CV_64F);
  t_vec_.at<double>(0) = transform_matrix_.at<double>(0, 3);
  t_vec_.at<double>(1) = transform_matrix_.at<double>(1, 3);
  t_vec_.at<double>(2) = transform_matrix_.at<double>(2, 3);

  cv::Mat tmp_mat = cv::Mat::zeros(3, 3, CV_64F);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      tmp_mat.at<double>(row, col) = transform_matrix_.at<double>(row, col);
    }
  }
  cv::Rodrigues(tmp_mat, r_vec_);

  if (distor_coe.cols == 4) {
    cv::fisheye::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(),
                                         camera_matrix_, image_size_, CV_16SC2,
                                         map_x_, map_y_);
  } else {
    cv::initUndistortRectifyMap(camera_matrix_, distor_coe_, cv::Mat(),
                                camera_matrix_, image_size_, CV_16SC2, map_x_,
                                map_y_);
  }

  std::vector<cv::Point2i> in_pts = {
      cv::Point2i(0, 0), cv::Point2i(image_size_.width, image_size_.height)};
  std::vector<Eigen::Vector3d> out_pts;
  this->LiftRay(in_pts, out_pts);
  norm_plane_range_ = {out_pts[0].x(), out_pts[1].x(), out_pts[0].y(),
                       out_pts[1].y()};
}

bool PinholeModel::ProjectPts(const std::vector<Eigen::Vector3d> &pts_3d,
                              std::vector<cv::Point2i> &pts_2d,
                              bool distort_flag) {
  const auto &k = camera_matrix_;
  const auto &rt = transform_matrix_;
  pts_2d.resize(pts_3d.size());

  std::vector<cv::Point2d> tmp_pts_2d(pts_3d.size());
  size_t count = 0;
  for (size_t i = 0; i < pts_3d.size(); ++i) {
    const auto &pt = pts_3d[i];
    Eigen::Vector3d pt_cam;
    pt_cam.x() = rt.at<double>(0, 0) * pt.x() + rt.at<double>(0, 1) * pt.y() +
                 rt.at<double>(0, 2) * pt.z() + rt.at<double>(0, 3);
    pt_cam.y() = rt.at<double>(1, 0) * pt.x() + rt.at<double>(1, 1) * pt.y() +
                 rt.at<double>(1, 2) * pt.z() + rt.at<double>(1, 3);
    pt_cam.z() = rt.at<double>(2, 0) * pt.x() + rt.at<double>(2, 1) * pt.y() +
                 rt.at<double>(2, 2) * pt.z() + rt.at<double>(2, 3);

    if (pt_cam.z() <= 0) {
      count++;
      continue;
    }

    tmp_pts_2d[i].x = pt_cam.x() / pt_cam.z();
    tmp_pts_2d[i].y = pt_cam.y() / pt_cam.z();

    tmp_pts_2d[i].x = std::max(tmp_pts_2d[i].x, norm_plane_range_[0]);
    tmp_pts_2d[i].x = std::min(tmp_pts_2d[i].x, norm_plane_range_[1]);
    tmp_pts_2d[i].y = std::max(tmp_pts_2d[i].y, norm_plane_range_[0]);
    tmp_pts_2d[i].y = std::min(tmp_pts_2d[i].y, norm_plane_range_[1]);
  }

  if (count == tmp_pts_2d.size()) {
    return false;
  }

  std::vector<cv::Point2d> distorted_pts2d;
  if (distort_flag) {
    DistortPts(tmp_pts_2d, distorted_pts2d);
  } else {
    distorted_pts2d = tmp_pts_2d;
  }

  for (size_t i = 0; i < distorted_pts2d.size(); ++i) {
    pts_2d[i].x = static_cast<int>(distorted_pts2d[i].x * k.at<double>(0, 0) +
                                   k.at<double>(0, 2));
    pts_2d[i].y = static_cast<int>(distorted_pts2d[i].y * k.at<double>(1, 1) +
                                   k.at<double>(1, 2));
  }

  return true;
}

void PinholeModel::LiftRay(const std::vector<cv::Point2i> &pts_2d,
                           std::vector<Eigen::Vector3d> &pts_3d) {
  const auto &k = camera_matrix_;
  const auto &rt = transform_matrix_;
  const auto &d = distor_coe_;
  pts_3d.resize(pts_2d.size());

  std::vector<cv::Point2d> in_pts_2d(pts_2d.size()), out_pts_2d(pts_2d.size());
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    in_pts_2d[i].x = static_cast<double>(pts_2d[i].x);
    in_pts_2d[i].y = static_cast<double>(pts_2d[i].y);
  }

  if (d.cols == 4) {
    cv::fisheye::undistortPoints(in_pts_2d, out_pts_2d, k, d);
  } else {
    cv::undistortPoints(in_pts_2d, out_pts_2d, k, d);
  }

  for (size_t i = 0; i < pts_2d.size(); ++i) {
    pts_3d[i].x() = out_pts_2d[i].x;
    pts_3d[i].y() = out_pts_2d[i].y;
    pts_3d[i].z() = 1.;
  }
}

void PinholeModel::UndistortPts(
    const std::vector<cv::Point2i> &distorted_pts_2d,
    std::vector<cv::Point2i> &undistorted_pts_2d) {
  const auto &k = camera_matrix_;
  const auto &rt = transform_matrix_;
  const auto &d = distor_coe_;
  undistorted_pts_2d.resize(distorted_pts_2d.size());

  std::vector<cv::Point2d> in_pts_2d(distorted_pts_2d.size()),
      out_pts_2d(distorted_pts_2d.size());
  for (size_t i = 0; i < in_pts_2d.size(); ++i) {
    in_pts_2d[i].x = static_cast<double>(distorted_pts_2d[i].x);
    in_pts_2d[i].y = static_cast<double>(distorted_pts_2d[i].y);
  }

  if (d.cols == 4) {
    cv::fisheye::undistortPoints(in_pts_2d, out_pts_2d, k, d, k);
  } else {
    cv::undistortPoints(in_pts_2d, out_pts_2d, k, d, k);
  }

  for (size_t i = 0; i < out_pts_2d.size(); ++i) {
    undistorted_pts_2d[i].x = static_cast<int>(out_pts_2d[i].x);
    undistorted_pts_2d[i].y = static_cast<int>(out_pts_2d[i].y);
  }
}

void PinholeModel::DistortPts(const std::vector<cv::Point2d> &pts_2d,
                              std::vector<cv::Point2d> &distorted_pts_2d) {
  distorted_pts_2d.resize(pts_2d.size());

  const auto &d = distor_coe_;
  for (size_t i = 0; i < pts_2d.size(); ++i) {
    if (d.cols == 4) {
      float r =
          std::sqrt(pts_2d[i].x * pts_2d[i].x + pts_2d[i].y * pts_2d[i].y);
      float th = std::atan2(r, 1.0);
      float th_d = th + d.at<double>(0) * std::pow(th, 3) +
                   d.at<double>(1) * std::pow(th, 5) +
                   d.at<double>(2) * std::pow(th, 7) +
                   d.at<double>(3) * std::pow(th, 9);
      distorted_pts_2d[i].x = (th_d / (r + 0.000001)) * pts_2d[i].x;
      distorted_pts_2d[i].y = (th_d / (r + 0.000001)) * pts_2d[i].y;
    } else {
      double r2 = pts_2d[i].x * pts_2d[i].x + pts_2d[i].y * pts_2d[i].y;
      double coeff = 1 + d.at<double>(0, 0) * r2 +
                     d.at<double>(0, 1) * r2 * r2 +
                     d.at<double>(0, 4) * r2 * r2 * r2;
      if (d.cols >= 8) {
        coeff *=
            1 / (1 + d.at<double>(0, 5) * r2 + d.at<double>(0, 6) * r2 * r2 +
                 d.at<double>(0, 7) * r2 * r2 * r2);
      }

      distorted_pts_2d[i].x = pts_2d[i].x * coeff;
      distorted_pts_2d[i].y = pts_2d[i].y * coeff;

      distorted_pts_2d[i].x +=
          (2 * d.at<double>(0, 2) * pts_2d[i].x * pts_2d[i].y +
           d.at<double>(0, 3) * (r2 + 2 * pts_2d[i].x * pts_2d[i].x));
      distorted_pts_2d[i].y +=
          (2 * d.at<double>(0, 3) * pts_2d[i].x * pts_2d[i].y +
           d.at<double>(0, 2) * (r2 + 2 * pts_2d[i].y * pts_2d[i].y));
    }
  }
}

} // namespace postprocess
} // namespace robosense
