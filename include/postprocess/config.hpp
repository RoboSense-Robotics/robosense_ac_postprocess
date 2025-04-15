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
#ifndef POSTPROCESS_CONFIG_H_
#define POSTPROCESS_CONFIG_H_

#include <string>
#include <vector>
#include <memory>

#include "ros_dep.h"
#include "postprocess/yaml_reader.hpp"
#include "postprocess/range_image.hpp"
namespace robosense {
namespace postprocess {

struct PostprocessOutputMsg {
  using Ptr = std::shared_ptr<PostprocessOutputMsg>;
  PostprocessOutputMsg() {
    motion_points_ptr = PointCloud2MsgPtr(new PointCloud2Msg());
    rgb_points_ptr = PointCloud2MsgPtr(new PointCloud2Msg());
    points_proj_img_ptr = ImageMsgPtr(new ImageMsg());
  }
  PointCloud2MsgPtr motion_points_ptr;
  PointCloud2MsgPtr rgb_points_ptr;
  ImageMsgPtr points_proj_img_ptr;
};

struct TransformXYZQuat {
  TransformXYZQuat() = default;
  double qx;
  double qy;
  double qz;
  double qw;
  double x;
  double y;
  double z;
};

struct Intrinsics {
  Intrinsics() = default;
  std::vector<double> distortion_coeffs; // Distortion coefficients
  std::vector<double> camera_matrix;     // Intrinsic matrix values (fx, 0, cx; 0, fy, cy; 0, 0, 1)
  std::string camera_model;              // Type of camera model

  // Constructor to initialize intrinsics with values
  Intrinsics(std::vector<double> d_values, std::vector<double> k_values, std::string model)
    : distortion_coeffs(d_values), camera_matrix(k_values), camera_model(model) {}
};

struct MotionConfig {
  MotionConfig() = default;
  bool motion_correct;                  // Whether to perform motion correction
  bool using_imu_linear_acceleration;   // Whether to use IMU linear acceleration data for motion compensation
  bool using_odom_linear_velocity;      // Whether to use ODOM data as linear velocity compensation, if true, it will override the displacement calculated from IMU linear acceleration
  bool frame_tail;                      // Whether to compensate points to the end time of the point cloud frame
  bool use_range_img;
  double thres;
  std::size_t num_drift;                // The number of IMU data required to calculate drift
  std::string projection_root;          // Path to save data (commented out)
  std::string sub_image_topic;
  std::string sub_lidar_topic;
  std::string sub_imu_topic;

  std::string ori_points_topic;
  std::string motion_points_topic;
  std::string motion_rgb_points_topic;
  std::string ori_rgb_points_topic;
  std::string ori_deocc_rgb_points_topic;
  std::string ori_img_topic;
  std::string motion_proj_img_topic;
  std::string motion_range_img_topic;
  std::string motion_cam_range_img_topic;
};

struct CameraConfig {
  CameraConfig() = default;
  Intrinsics  intrinsics;
  CameraConfig(Intrinsics intr) : intrinsics(intr) {}
};

struct LidarConfig {
  LidarConfig() = default;
  TransformXYZQuat T_Lidar2Cam;    // Transformation from LIDAR to Camera
  TransformXYZQuat T_Lidar2IMU;    // Transformation from LIDAR to IMU
  RangeImageParams range_image_info;
};

struct NodeConfig {
  NodeConfig() = default;
  MotionConfig motion_config;
  CameraConfig camera_config;
  LidarConfig  lidar_config;
};

NodeConfig ParseNodeConfig(const YAML::Node& cfg);

} // namespace postprocess
} // namespace robosense
#endif  // POSTPROCESS_CONFIG_H_
