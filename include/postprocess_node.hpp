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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "postprocess/yaml_reader.hpp"
#include "postprocess/config.hpp"
#include "postprocess/postprocess_impl.hpp"

namespace robosense {
namespace postprocess {

class PostprocessNode : public rclcpp::Node {
public:
  PostprocessNode(const YAML::Node& cfg_node) : rclcpp::Node("postprocess_node") {
    Init(cfg_node);
    Start();
  }
  ~PostprocessNode() {
    Stop();
  }

  void Init(const YAML::Node& cfg_node) {
    std::cout << Name() << ": get cfg is " << std::endl << cfg_node;
    node_cfg_ = ParseNodeConfig(cfg_node);
    impl_ptr = std::make_shared<PostprocessImpl>();
    impl_ptr->Init(node_cfg_);

    const MotionConfig& motion_cfg  = node_cfg_.motion_config;
    // sub
    const std::string pts_topic = motion_cfg.sub_lidar_topic;
    auto pts_callback = std::bind(&PostprocessNode::SubPointCloudCallback, this, std::placeholders::_1);
    pts_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pts_topic, 100, pts_callback);

    const std::string imu_topic = motion_cfg.sub_imu_topic;
    auto imu_callback = std::bind(&PostprocessNode::SubIMUCallback, this, std::placeholders::_1);
    pts_mc_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 100, imu_callback);

    const std::string cam_topic = motion_cfg.sub_image_topic;
    auto cam_callback = std::bind(&PostprocessNode::SubCameraCallback, this, std::placeholders::_1);
    cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(cam_topic, 300, cam_callback);

    // pub
    {
      pub_motion_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(motion_cfg.motion_points_topic, 10); // 运动矫正后点云
      pub_motion_rgb_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(motion_cfg.motion_rgb_points_topic, 10);  // 运动校正后点云 + 对应投影RGB
      pub_motion_proj_img_ = this->create_publisher<sensor_msgs::msg::Image>(motion_cfg.motion_proj_img_topic, 30); // 运动矫正后投影图像
    }

    std::cout << "------------------------------------------------------" << std::endl;
    std::cout << "PROJ_IMG_TOPIC                 : " + motion_cfg.motion_proj_img_topic << std::endl;
    std::cout << "RANGE_IMG_TOPIC                : " + motion_cfg.motion_range_img_topic << std::endl;
    std::cout << "CAM_RANGE_IMG_TOPIC            : " + motion_cfg.motion_cam_range_img_topic << std::endl;
    std::cout << "------------------------------------------------------" << std::endl;

    std::cout << Name() << ": init succeed!" << std::endl;
  }
  void Start() {
    run_flag_ = true;
    if (thread_ptr_ == nullptr) {
      thread_ptr_ = std::make_unique<std::thread>(&PostprocessNode::Core, this);
    }
  }

  void Stop() {
    if (thread_ptr_ != nullptr) {
      run_flag_ = false;
      std::cout << Name() << ": stoped!" << std::endl;
      if (thread_ptr_->joinable()) {
        thread_ptr_->join();
      }
      thread_ptr_.reset(nullptr);
    }
  }

  inline void SubPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) {
    rec_depth_frame_.store(true);
    impl_ptr->AddData(msg_ptr);
  }
  inline void SubCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg_ptr) {
    rec_image_frame_.store(true);
    impl_ptr->AddData(msg_ptr);
  }
  inline void SubIMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg_ptr) {
    rec_imu_.store(true);
    impl_ptr->AddData(msg_ptr);
  }
  void PubMsg(PostprocessOutputMsg::Ptr out_msg_ptr) {
    if (out_msg_ptr->motion_points_ptr != nullptr) {
      pub_motion_points_->publish(*(out_msg_ptr->motion_points_ptr));
    }
    if (out_msg_ptr->points_proj_img_ptr != nullptr) {
      pub_motion_proj_img_->publish(*(out_msg_ptr->points_proj_img_ptr));
    }
    if (out_msg_ptr->rgb_points_ptr!= nullptr) {
      pub_motion_rgb_points_->publish(*(out_msg_ptr->rgb_points_ptr));
    }
  }

private:
  const std::string Name() { return "PostprocessNode"; }
  void Core() {
    while (run_flag_) {
      if (rec_imu_.load() && rec_depth_frame_.load() && rec_image_frame_.load()) {
        PostprocessOutputMsg::Ptr out_msg_ptr(new PostprocessOutputMsg);
        auto start = std::chrono::high_resolution_clock::now();
        impl_ptr->Process(out_msg_ptr);
        PubMsg(out_msg_ptr);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        rec_imu_.store(false);
        rec_depth_frame_.store(false);
        rec_image_frame_.store(false);
        std::cout << "Succ Process, Cost time: " << duration.count() << " ms" << std::endl;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pts_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr pts_mc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_motion_points_;  // 运动矫正后点云
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_motion_rgb_points_; // 运动校正后点云 + 对应投影RGB
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_motion_proj_img_; // 运动矫正后投影图像

  NodeConfig node_cfg_;
  bool run_flag_ = false;
  std::atomic<bool> rec_depth_frame_{false};
  std::atomic<bool> rec_imu_{false};
  std::atomic<bool> rec_image_frame_{false};
  std::unique_ptr<std::thread> thread_ptr_ = nullptr;

  PostprocessImpl::Ptr impl_ptr;
 };

 }  // namespace postprocess
 }  // namespace robosense

#endif  // POSTPROCESS_NODE_H
