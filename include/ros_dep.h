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
#ifndef ROS_DEP_H
#define ROS_DEP_H

#ifdef USE_ROS1 // 在ros1环境下的依赖
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <robosense_msgs/RsACDeviceCalib.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>

using ImuMsgPtr = sensor_msgs::Imu::Ptr;
using PointCloud2MsgConstPtr = sensor_msgs::PointCloud2::ConstPtr;
using PointCloud2Msg = sensor_msgs::PointCloud2;
using PointCloud2MsgPtr = sensor_msgs::PointCloud2::Ptr;
using ImageMsgConstPtr = sensor_msgs::Image::ConstPtr;
using ImageMsg = sensor_msgs::Image;
using ImageMsgPtr = sensor_msgs::Image::Ptr;
using StereoImageSub  = message_filters::Subscriber<sensor_msgs::Image>;
using SyncPolicy      = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
using Sync            = message_filters::Synchronizer<SyncPolicy>;

using CompressedImageMsgConstPtr = sensor_msgs::CompressedImage::ConstPtr;
using CompressedImageMsgPtr = sensor_msgs::CompressedImage::Ptr;
using CompressedImageMsg = sensor_msgs::CompressedImage;
using CompressedStereoImageSub  = message_filters::Subscriber<sensor_msgs::CompressedImage>;
using CompressedSyncPolicy      = message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>;
using CompressedSync            = message_filters::Synchronizer<CompressedSyncPolicy>;

using PointCloud2Msgs = sensor_msgs::PointCloud2;
using CalibMsg = robosense_msgs::RsACDeviceCalib;
using CalibMsgPtr = robosense_msgs::RsACDeviceCalib::ConstPtr;

inline double HeaderToSec(const std_msgs::Header& header) {
  return header.stamp.toSec();
}
inline ros::Time SecToHeaderStamp(const double sec) {
  return ros::Time(sec);
}

inline double HeaderToNanoSec(const std_msgs::Header& header) {
  return header.stamp.toNSec();
}

template<typename... Args>
void RosInfoPrint(const std::string& format, const Args&... args) {
  ROS_INFO(format.c_str(), args...);
}

template<typename... Args>
void RosWarnPrint(const std::string& format, const Args&... args) {
  ROS_WARN(format.c_str(), args...);
}

template<typename... Args>
void RosErrorPrint(const std::string& format, const Args&... args) {
  ROS_ERROR(format.c_str(), args...);
}

inline ros::Time GetTimeNow() { return ros::Time::now(); }

inline double GetTimeNowInSecond() { return ros::Time::now().toSec(); }

inline bool IsRosOK() { return ros::ok(); }

#elif defined(USE_ROS2) // 在ros2环境下的依赖
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <robosense_msgs/msg/rs_ac_device_calib.hpp>
#include <robosense_msgs/msg/rs_image1_m.hpp>
#include <robosense_msgs/msg/rs_image2_m.hpp>
#include <robosense_msgs/msg/rs_image4_m.hpp>
#include <robosense_msgs/msg/rs_image6_m.hpp>
#include <robosense_msgs/msg/rs_image8_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud1_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud2_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud4_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud6_m.hpp>
#include <robosense_msgs/msg/rs_point_cloud8_m.hpp>

using ImuMsgPtr = sensor_msgs::msg::Imu::SharedPtr;
using PointCloud2MsgPtr = sensor_msgs::msg::PointCloud2::SharedPtr;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using ImageMsgConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;
using ImageMsgPtr = sensor_msgs::msg::Image::SharedPtr;
using ImageMsg = sensor_msgs::msg::Image;
using RosRate = rclcpp::Rate;

using CompressedImageMsgConstPtr = sensor_msgs::msg::CompressedImage::ConstSharedPtr;
using CompressedImageMsgPtr = sensor_msgs::msg::CompressedImage::SharedPtr;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

using StereoImageSub  = message_filters::Subscriber<sensor_msgs::msg::Image>;
using SyncPolicy      = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
using Sync            = message_filters::Synchronizer<SyncPolicy>;

using CompressedStereoImageSub  = message_filters::Subscriber<sensor_msgs::msg::CompressedImage>;
using CompressedSyncPolicy      = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>;
using CompressedSync            = message_filters::Synchronizer<CompressedSyncPolicy>;

using CalibMsg = robosense_msgs::msg::RsACDeviceCalib;
using CalibMsgPtr = robosense_msgs::msg::RsACDeviceCalib::SharedPtr;

inline double HeaderToSec(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp).seconds();
}
inline rclcpp::Time SecToHeaderStamp(const double sec) {
  return rclcpp::Time(sec);
}
inline double HeaderToNanoSec(const std_msgs::msg::Header& header) {
  return rclcpp::Time(header.stamp).nanoseconds();
}

template<typename... Args>
void RosInfoPrint(const std::string& format, const Args&... args) {
  RCLCPP_INFO(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

template<typename... Args>
void RosWarnPrint(const std::string& format, const Args&... args) {
  RCLCPP_WARN(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

template<typename... Args>
void RosErrorPrint(const std::string& format, const Args&... args) {
  RCLCPP_ERROR(rclcpp::get_logger("global_logger"), format.c_str(), args...);
}

inline rclcpp::Time GetTimeNow() { return rclcpp::Clock().now(); }

inline double GetTimeNowInSecond() { return rclcpp::Clock().now().seconds(); }

inline bool IsRosOK() { return rclcpp::ok(); }

//AC1 8M
//AC2 4M
// ============================================
// 主体函数：处理通用的 PointCloud 转换逻辑
// ============================================
template<typename PointCloudMsgT>
inline PointCloud2MsgPtr ConvertZCPointsImpl(const PointCloudMsgT* msg) {
  PointCloud2MsgPtr msg_ptr(new PointCloud2Msg());
  
  // header
  std_msgs::msg::Header header;
  std::string frame_id;
  for(char c : msg->header.frame_id){
      if(c == '\0') break;
      frame_id.push_back(c);
  }
  header.frame_id = frame_id;
  header.stamp = rclcpp::Time(msg->header.stamp);
  msg_ptr->header = header;
  
  // other
  msg_ptr->is_bigendian = msg->is_bigendian;
  msg_ptr->point_step = msg->point_step;
  msg_ptr->row_step = msg->row_step;
  msg_ptr->height = msg->height;
  msg_ptr->width = msg->width;
  msg_ptr->is_dense = msg->is_dense;
  
  // fields
  msg_ptr->fields.clear();  // 修复：先清空，避免 resize + push_back 导致重复
  msg_ptr->fields.reserve(msg->fields.size());
  for (const auto& field : msg->fields) {
    sensor_msgs::msg::PointField point_field;
    std::string name;
    for (char c : field.name) {
      if (c == '\0') break;
      name.push_back(c);
    }
    point_field.name = name;
    point_field.offset = field.offset;
    point_field.datatype = field.datatype;
    point_field.count = field.count;
    msg_ptr->fields.push_back(point_field);
  }
  
  // data
  msg_ptr->data.resize(msg->row_step);
  // copy data
  memcpy(msg_ptr->data.data(), msg->data.data(), msg->row_step);
  
  return msg_ptr;
}

// ============================================
// 4M 格式调用函数
// ============================================
inline PointCloud2MsgPtr ConvertZCPointsAC2(const robosense_msgs::msg::RsPointCloud4M::Ptr msg) {
  return ConvertZCPointsImpl(msg.get());
}

// ============================================
// 8M 格式调用函数
// ============================================
inline PointCloud2MsgPtr ConvertZCPointsAC1(const robosense_msgs::msg::RsPointCloud8M::Ptr msg) {
  return ConvertZCPointsImpl(msg.get());
}

// ============================================
// 主体函数：处理通用的 Image 转换逻辑
// ============================================
template<typename ImageMsgT>
inline ImageMsgPtr ConvertZCImageImpl(const ImageMsgT* msg) {
  ImageMsgPtr msg_ptr(new ImageMsg());
  
  // header
  std_msgs::msg::Header header;
  std::string frame_id;
  for(char c : msg->header.frame_id){
      if(c == '\0') break;
      frame_id.push_back(c);
  }
  header.frame_id = frame_id;
  header.stamp = rclcpp::Time(msg->header.stamp);
  msg_ptr->header = header;
  
  // other
  msg_ptr->height = msg->height;
  msg_ptr->width = msg->width;
  msg_ptr->is_bigendian = msg->is_bigendian;
  msg_ptr->step = msg->step;
  
  // data
  msg_ptr->data.resize(msg->step * msg->height);
  memcpy(msg_ptr->data.data(), msg->data.data(), msg->step * msg->height);
  
  return msg_ptr;
}

// ============================================
// 4M 格式调用函数
// ============================================
inline ImageMsgPtr ConvertZCImageAC2(const robosense_msgs::msg::RsImage4M::Ptr msg) {
  return ConvertZCImageImpl(msg.get());
}

// ============================================
// 8M 格式调用函数
// ============================================
inline ImageMsgPtr ConvertZCImageAC1(const robosense_msgs::msg::RsImage8M::Ptr msg) {
  return ConvertZCImageImpl(msg.get());
}


// inline PointCloud2MsgPtr ConvertZCPoints(const robosense_msgs::msg::RsPointCloud8M::Ptr msg) {
//   PointCloud2MsgPtr msg_ptr(new PointCloud2Msg());
//   // header
//   std_msgs::msg::Header header;
//   std::string frame_id;
//   for(char c : msg->header.frame_id){
//       if(c == '\0') break;
//       frame_id.push_back(c);
//   }
//   header.frame_id = frame_id;
//   header.stamp = rclcpp::Time(msg->header.stamp);
//   msg_ptr->header = header;
//   // other
//   msg_ptr->is_bigendian = msg->is_bigendian;
//   msg_ptr->point_step = msg->point_step;
//   msg_ptr->row_step = msg->row_step;
//   msg_ptr->height = msg->height;
//   msg_ptr->width = msg->width;
//   msg_ptr->is_dense = msg->is_dense;
//   // fields
//   msg_ptr->fields.resize(msg->fields.size());
//   for (const auto& field : msg->fields) {
//     sensor_msgs::msg::PointField point_field;
//     std::string name;
//     for (char c : field.name) {
//       if (c == '\0') break;
//       name.push_back(c);
//     }
//     point_field.name = name;
//     point_field.offset = field.offset;
//     point_field.datatype = field.datatype;
//     point_field.count = field.count;
//     msg_ptr->fields.push_back(point_field);
//   }
//   // data
//   msg_ptr->data.resize(msg->row_step);
//   // copy data
//   memcpy(msg_ptr->data.data(), msg->data.data(), msg->row_step);
//   return msg_ptr;
// }

//AC1 8M
//AC2 4M
// inline ImageMsgPtr ConvertZCImage(const robosense_msgs::msg::RsImage8M::Ptr msg) {
//   ImageMsgPtr msg_ptr(new ImageMsg());
//   // header
//   std_msgs::msg::Header header;
//   std::string frame_id;
//   for(char c : msg->header.frame_id){
//       if(c == '\0') break;
//       frame_id.push_back(c);
//   }
//   header.frame_id = frame_id;
//   header.stamp = rclcpp::Time(msg->header.stamp);
//   msg_ptr->header = header;
//   // other
//   msg_ptr->height = msg->height;
//   msg_ptr->width = msg->width;
//   msg_ptr->is_bigendian = msg->is_bigendian;
//   msg_ptr->step = msg->step;
//   // data
//   msg_ptr->data.resize(msg->step * msg->height);
//   memcpy(msg_ptr->data.data(), msg->data.data(), msg->step * msg->height);
//   return msg_ptr;
// }

#endif
#endif  // ROS_DEP_H
