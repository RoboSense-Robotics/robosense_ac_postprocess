/******************************************************************************
 * Copyright 2024 RoboSense All rights reserved.
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
#ifndef POSTPROCESS_CONFIG_H_
#define POSTPROCESS_CONFIG_H_

#include <string>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "postprocess/yaml_reader.hpp"
#include "postprocess/range_image.hpp"
namespace robosense {
namespace postprocess {

struct PostprocessOutputMsg {
  using Ptr = std::shared_ptr<PostprocessOutputMsg>;
  PostprocessOutputMsg() {
    motion_points_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
    rgb_points_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
    points_proj_img_ptr = std::make_shared<sensor_msgs::msg::Image>();
  }
  sensor_msgs::msg::PointCloud2::SharedPtr motion_points_ptr;
  sensor_msgs::msg::PointCloud2::SharedPtr rgb_points_ptr;
  sensor_msgs::msg::Image::SharedPtr points_proj_img_ptr;
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
