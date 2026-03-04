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
#ifndef POSTPROCESS_NODE_H
#define POSTPROCESS_NODE_H
#include <thread>
#include "postprocess/config.hpp"
#include "ros_dep.h"
#include "postprocess/yaml_reader.hpp"
#include "postprocess/postprocess_impl.hpp"

namespace robosense {
namespace postprocess {
class PostprocessNode {
public:
  PostprocessNode(const YAML::Node& cfg_node);
#if defined(USE_ROS2)
  rclcpp::Node::SharedPtr GetRos2Node() {
    return ros2_node_ptr_;
  }
#endif

  ~PostprocessNode() {
    Stop();
  }

  void Init(const YAML::Node& cfg_node);
  void Start();
  void Stop();

  inline void SubPointCloudCallback(const PointCloud2MsgPtr msg_ptr);
  inline void SubImageCallback(const ImageMsgPtr msg_ptr);
  inline void SubCalibInfoCallback(const CalibMsgPtr msg_ptr);
  inline void SubCompressedImageCallback(const CompressedImageMsgPtr msg_ptr);
  inline void SubStereoImageCallback(const ImageMsgConstPtr msg_ptr, const ImageMsgConstPtr second_msg_ptr);
  inline void SubStereoCompressedImageCallback(const CompressedImageMsgConstPtr msg_ptr, const CompressedImageMsgConstPtr second_msg_ptr);
#if defined(USE_ROS2)
  inline void SubZCPointCloudCallbackAC1(const robosense_msgs::msg::RsPointCloud8M::Ptr zc_msg);
  inline void SubZCCameraCallbackAC1(const robosense_msgs::msg::RsImage8M::Ptr msg_ptr);
  inline void SubZCPointCloudCallbackAC2(const robosense_msgs::msg::RsPointCloud4M::Ptr zc_msg);
  inline void SubZCCameraCallbackAC2(const robosense_msgs::msg::RsImage4M::Ptr msg_ptr);
#endif
  inline void SubIMUCallback(const ImuMsgPtr msg_ptr);
  void PubMsg(PostprocessOutputMsg::Ptr out_msg_ptr);

private:
  const std::string Name() { return "PostprocessNode"; }
  void Core();
#if defined(USE_ROS1)
  ros::NodeHandle nh_;
  ros::Subscriber pts_sub_, pts_mc_sub_, cam_sub_, cam_compressed_sub_, calib_sub_;
  ros::Publisher pub_motion_points_, pub_motion_rgb_points_, pub_motion_stereo_rgb_points_, pub_edge_points_, pub_motion_proj_img_, pub_ori_img_;
#elif defined(USE_ROS2)
  bool zero_copy_;
  rclcpp::Node::SharedPtr ros2_node_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pts_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RsPointCloud8M>::SharedPtr ac1_pts_sub_zc_;
  rclcpp::Subscription<robosense_msgs::msg::RsPointCloud4M>::SharedPtr ac2_pts_sub_zc_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pts_mc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr cam_compressed_sub_;
  rclcpp::Subscription<robosense_msgs::msg::RsImage8M>::SharedPtr ac1_cam_sub_zc_;
  rclcpp::Subscription<robosense_msgs::msg::RsImage4M>::SharedPtr ac2_cam_sub_zc_;
  rclcpp::Subscription<robosense_msgs::msg::RsACDeviceCalib>::SharedPtr calib_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_edge_points_;  // 边缘颜色突变点
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_motion_points_;  // 运动矫正后点云
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_motion_rgb_points_; // 运动校正后点云 + 对应投影RGB
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_motion_proj_img_; // 运动矫正后投影图像
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_ori_img_; // 原始图像

#endif

  StereoImageSub left_cam_sub_;
  StereoImageSub right_cam_sub_;
  std::shared_ptr<Sync> sync_;

  CompressedStereoImageSub left_compressed_cam_sub_;
  CompressedStereoImageSub right_compressed_cam_sub_;
  std::shared_ptr<CompressedSync> compressed_sync_;

  NodeConfig node_cfg_;
  bool run_flag_ = false;
  bool motion_correct_ = false;
  std::atomic<bool> rec_depth_frame_{false};
  std::atomic<bool> rec_imu_{false};
  std::atomic<bool> rec_image_frame_{false};
  std::unique_ptr<std::thread> thread_ptr_ = nullptr;

  PostprocessImpl::Ptr impl_ptr;
 };

 }  // namespace postprocess
 }  // namespace robosense

#endif  // POSTPROCESS_NODE_H
