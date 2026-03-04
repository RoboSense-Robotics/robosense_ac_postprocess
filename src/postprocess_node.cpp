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
#include "postprocess_node.hpp"

namespace robosense {
namespace postprocess {

PostprocessNode::PostprocessNode(const YAML::Node& cfg_node) {
  Init(cfg_node);
  Start();
}

void PostprocessNode::Start() {
  run_flag_ = true;
  if (thread_ptr_ == nullptr) {
    thread_ptr_ = std::make_unique<std::thread>(&PostprocessNode::Core, this);
  }
}

void PostprocessNode::Stop() {
  if (thread_ptr_ != nullptr) {
    run_flag_ = false;
    std::cout << Name() << ": stoped!" << std::endl;
    if (thread_ptr_->joinable()) {
      thread_ptr_->join();
    }
    thread_ptr_.reset(nullptr);
  }
}


void PostprocessNode::Init(const YAML::Node& cfg_node) {
  std::cout << Name() << ": get cfg is " << std::endl << cfg_node;
  node_cfg_ = ParseNodeConfig(cfg_node);
  impl_ptr = std::make_shared<PostprocessImpl>();
  impl_ptr->Init(node_cfg_);

  const MotionConfig& motion_cfg  = node_cfg_.motion_config;
  const std::string pts_topic = motion_cfg.sub_lidar_topic;
  const std::string imu_topic = motion_cfg.sub_imu_topic;
  const std::string cam_topic = motion_cfg.sub_image_topic;
  const std::string second_cam_topic = motion_cfg.sub_second_image_topic;
  const std::string calib_topic = motion_cfg.sub_calib_topic;
  const std::string ori_image_topic = motion_cfg.pub_ori_image_topic;

  motion_correct_ = motion_cfg.motion_correct;

#if defined(USE_ROS1)
  pts_sub_ = nh_.subscribe(pts_topic, 100, &PostprocessNode::SubPointCloudCallback, this);
  pts_mc_sub_ = nh_.subscribe(imu_topic, 100, &PostprocessNode::SubIMUCallback, this);

  if(motion_cfg.use_compressed_image)
  {
    if(motion_cfg.use_second_image)
    {
      left_compressed_cam_sub_.subscribe(nh_, cam_topic, 1);
      right_compressed_cam_sub_.subscribe(nh_, second_cam_topic, 1);
      compressed_sync_ = std::make_shared<CompressedSync>(CompressedSyncPolicy(10), left_compressed_cam_sub_, right_compressed_cam_sub_);
      compressed_sync_->registerCallback(boost::bind(&PostprocessNode::SubStereoCompressedImageCallback, this, _1, _2));
    }
    else
    {
      cam_compressed_sub_ = nh_.subscribe(cam_topic, 100, &PostprocessNode::SubCompressedImageCallback, this);
    }
  }
  else
  {
    if(motion_cfg.use_second_image)
    {
      left_cam_sub_.subscribe(nh_, cam_topic, 1);
      right_cam_sub_.subscribe(nh_, second_cam_topic, 1);
      sync_ = std::make_shared<Sync>(SyncPolicy(10), left_cam_sub_, right_cam_sub_);
      sync_->registerCallback(boost::bind(&PostprocessNode::SubStereoImageCallback, this, _1, _2));
    }
    else
    {
      cam_sub_ = nh_.subscribe(cam_topic, 100, &PostprocessNode::SubImageCallback, this);
    }
  }

  calib_sub_ = nh_.subscribe(calib_topic, 100, &PostprocessNode::SubCalibInfoCallback, this);

  pub_edge_points_ = nh_.advertise<sensor_msgs::PointCloud2>(motion_cfg.edge_points_topic, 10);
  pub_motion_points_ = nh_.advertise<sensor_msgs::PointCloud2>(motion_cfg.motion_points_topic, 10);
  pub_motion_rgb_points_ = nh_.advertise<sensor_msgs::PointCloud2>(motion_cfg.motion_rgb_points_topic, 10);
  pub_motion_stereo_rgb_points_ = nh_.advertise<sensor_msgs::PointCloud2>(motion_cfg.motion_stereo_rgb_points_topic, 10);
  pub_motion_proj_img_ = nh_.advertise<sensor_msgs::Image>(motion_cfg.motion_proj_img_topic, 30);
  pub_ori_img_ = nh_.advertise<sensor_msgs::Image>(ori_image_topic, 30);
#elif defined(USE_ROS2)
  const char* val = std::getenv("RMW_FASTRTPS_USE_QOS_FROM_XML");
  if (val != nullptr && std::string(val) == "1") {
    zero_copy_ = true;
  } else {
    zero_copy_ = false;
  }
  ros2_node_ptr_ = rclcpp::Node::make_shared("postprocess_node");
  if (zero_copy_) {
    if(motion_cfg.ac_mode==1)
    {
      auto pts_callback = std::bind(&PostprocessNode::SubZCPointCloudCallbackAC1, this, std::placeholders::_1);
      ac1_pts_sub_zc_ = ros2_node_ptr_->create_subscription<robosense_msgs::msg::RsPointCloud8M>(pts_topic, 100, pts_callback);
      auto cam_callback = std::bind(&PostprocessNode::SubZCCameraCallbackAC1, this, std::placeholders::_1);
      ac1_cam_sub_zc_ = ros2_node_ptr_->create_subscription<robosense_msgs::msg::RsImage8M>(cam_topic, 300, cam_callback);
    }
    else
    {
      auto pts_callback = std::bind(&PostprocessNode::SubZCPointCloudCallbackAC2, this, std::placeholders::_1);
      ac2_pts_sub_zc_ = ros2_node_ptr_->create_subscription<robosense_msgs::msg::RsPointCloud4M>(pts_topic, 100, pts_callback);
      auto cam_callback = std::bind(&PostprocessNode::SubZCCameraCallbackAC2, this, std::placeholders::_1);
      ac2_cam_sub_zc_ = ros2_node_ptr_->create_subscription<robosense_msgs::msg::RsImage4M>(cam_topic, 300, cam_callback);
    }
  } else {
    auto pts_callback = std::bind(&PostprocessNode::SubPointCloudCallback, this, std::placeholders::_1);
    pts_sub_ = ros2_node_ptr_->create_subscription<sensor_msgs::msg::PointCloud2>(pts_topic, 100, pts_callback);
    if(motion_cfg.use_compressed_image)
    {
      if(motion_cfg.use_second_image)
      {
        left_compressed_cam_sub_.subscribe(ros2_node_ptr_.get(), cam_topic);
        right_compressed_cam_sub_.subscribe(ros2_node_ptr_.get(), second_cam_topic);
        compressed_sync_ = std::make_shared<CompressedSync>(CompressedSyncPolicy(10), left_compressed_cam_sub_, right_compressed_cam_sub_);
        compressed_sync_->registerCallback(std::bind(&PostprocessNode::SubStereoCompressedImageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
          cam_compressed_sub_ = ros2_node_ptr_->create_subscription<sensor_msgs::msg::CompressedImage>(
             cam_topic, 300, [this](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
                 this->SubCompressedImageCallback(msg);
             });
      }
    }
    else
    {
      if(motion_cfg.use_second_image)
      {
        left_cam_sub_.subscribe(ros2_node_ptr_.get(), cam_topic);
        right_cam_sub_.subscribe(ros2_node_ptr_.get(), second_cam_topic);
        sync_ = std::make_shared<Sync>(SyncPolicy(10), left_cam_sub_, right_cam_sub_);
        sync_->registerCallback(std::bind(&PostprocessNode::SubStereoImageCallback, this, std::placeholders::_1, std::placeholders::_2));
      }
      else
      {
        auto cam_callback = std::bind(&PostprocessNode::SubImageCallback, this, std::placeholders::_1);
        cam_sub_ = ros2_node_ptr_->create_subscription<sensor_msgs::msg::Image>(cam_topic, 300, cam_callback);
      }
    }
  }
  auto imu_callback = std::bind(&PostprocessNode::SubIMUCallback, this, std::placeholders::_1);
  pts_mc_sub_ = ros2_node_ptr_->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 100, imu_callback);
  auto calib_callback = std::bind(&PostprocessNode::SubCalibInfoCallback, this, std::placeholders::_1);
  calib_sub_ = ros2_node_ptr_->create_subscription<robosense_msgs::msg::RsACDeviceCalib>(calib_topic, 100, calib_callback);

  pub_edge_points_ = ros2_node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(motion_cfg.edge_points_topic, 10);
  pub_motion_points_ = ros2_node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(motion_cfg.motion_points_topic, 10); // 运动矫正后点云
  pub_motion_rgb_points_ = ros2_node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(motion_cfg.motion_rgb_points_topic, 10);  // 运动校正后点云 + 对应投影RGB
  pub_motion_proj_img_ = ros2_node_ptr_->create_publisher<sensor_msgs::msg::Image>(motion_cfg.motion_proj_img_topic, 30); // 运动矫正后投影图像
  pub_ori_img_ = ros2_node_ptr_->create_publisher<sensor_msgs::msg::Image>(motion_cfg.pub_ori_image_topic, 30);

#endif
  std::cout << "------------------------------------------------------" << std::endl;
  std::cout << "PROJ_IMG_TOPIC                 : " + motion_cfg.motion_proj_img_topic << std::endl;
  std::cout << "RANGE_IMG_TOPIC                : " + motion_cfg.motion_range_img_topic << std::endl;
  std::cout << "CAM_RANGE_IMG_TOPIC            : " + motion_cfg.motion_cam_range_img_topic << std::endl;
  std::cout << "------------------------------------------------------" << std::endl;

  std::cout << Name() << ": init succeed!" << std::endl;
}

void PostprocessNode::SubPointCloudCallback(const PointCloud2MsgPtr msg_ptr) {
  rec_depth_frame_.store(true);
  impl_ptr->AddData(msg_ptr);
}
void PostprocessNode::SubImageCallback(const ImageMsgPtr msg_ptr) {
  rec_image_frame_.store(true);
  impl_ptr->AddData(msg_ptr);
}

void PostprocessNode::SubStereoImageCallback(const ImageMsgConstPtr msg_ptr, const ImageMsgConstPtr second_msg_ptr){
  rec_image_frame_.store(true);
#if defined(USE_ROS1)
  ImageMsgPtr non_const_msg_ptr = boost::const_pointer_cast<ImageMsg>(msg_ptr);
  ImageMsgPtr non_const_second_msg_ptr = boost::const_pointer_cast<ImageMsg>(second_msg_ptr);
  impl_ptr->AddData(non_const_msg_ptr, non_const_second_msg_ptr);
#elif defined(USE_ROS2)
  ImageMsgPtr non_const_msg_ptr = std::make_shared<ImageMsg>(*msg_ptr);
  ImageMsgPtr non_const_second_msg_ptr = std::make_shared<ImageMsg>(*second_msg_ptr);
  impl_ptr->AddData(non_const_msg_ptr, non_const_second_msg_ptr);
#endif
}

void PostprocessNode::SubCompressedImageCallback(const CompressedImageMsgPtr msg_ptr)
{
  try
  {
    rec_image_frame_.store(true);
    // 1. cv_bridge 直接解码
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");  // 需要彩色就 bgr8，灰度就 mono8
    // 2. 重新打包
    ImageMsgPtr out;
#if defined(USE_ROS1)
    out.reset(new sensor_msgs::Image);
#elif defined(USE_ROS2)
    out = std::make_shared<sensor_msgs::msg::Image>();
#endif
    cv_ptr->toImageMsg(*out);                 // 拷贝到 out
    std::cout << "Compressed to Image" << std::endl;
    impl_ptr->AddData(out);
  }
  catch (const cv_bridge::Exception & e)
  {
    // 解码失败（格式不支持、数据损坏等）
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }
}

void PostprocessNode::SubStereoCompressedImageCallback(const CompressedImageMsgConstPtr msg_ptr, const CompressedImageMsgConstPtr second_msg_ptr){
  try
  {
    rec_image_frame_.store(true);
    // 1. cv_bridge 直接解码
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_ptr, "bgr8");  // 需要彩色就 bgr8，灰度就 mono8
    cv_bridge::CvImagePtr second_cv_ptr = cv_bridge::toCvCopy(second_msg_ptr, "bgr8");  // 需要彩色就 bgr8，灰度就 mono8
    // 2. 重新打包
    ImageMsgPtr out;
    ImageMsgPtr second_out;
#if defined(USE_ROS1)
    out.reset(new sensor_msgs::Image);
    second_out.reset(new sensor_msgs::Image);
#elif defined(USE_ROS2)
    out = std::make_shared<sensor_msgs::msg::Image>();
    second_out = std::make_shared<sensor_msgs::msg::Image>();
#endif
    cv_ptr->toImageMsg(*out);                 // 拷贝到 out
    second_cv_ptr->toImageMsg(*second_out);   // 拷贝到 second_out
    std::cout << "Compressed to Image" << std::endl;
    impl_ptr->AddData(out, second_out);
  }
  catch (const cv_bridge::Exception & e)
  {
    // 解码失败（格式不支持、数据损坏等）
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }
}

void PostprocessNode::SubCalibInfoCallback(const CalibMsgPtr msg_ptr)
{
  impl_ptr->AddData(msg_ptr);
}

#if defined(USE_ROS2)
void PostprocessNode::SubZCPointCloudCallbackAC1(const robosense_msgs::msg::RsPointCloud8M::Ptr zc_msg_ptr) {
  rec_depth_frame_.store(true);
  PointCloud2MsgPtr msg_ptr = ConvertZCPointsAC1(zc_msg_ptr);
  impl_ptr->AddData(msg_ptr);
}
void PostprocessNode::SubZCCameraCallbackAC1(const robosense_msgs::msg::RsImage8M::Ptr zc_msg_ptr) {
  rec_image_frame_.store(true);
  ImageMsgPtr msg_ptr = ConvertZCImageAC1(zc_msg_ptr);
  impl_ptr->AddData(msg_ptr);
}
void PostprocessNode::SubZCPointCloudCallbackAC2(const robosense_msgs::msg::RsPointCloud4M::Ptr zc_msg_ptr) {
  rec_depth_frame_.store(true);
  PointCloud2MsgPtr msg_ptr = ConvertZCPointsAC2(zc_msg_ptr);
  impl_ptr->AddData(msg_ptr);
}
void PostprocessNode::SubZCCameraCallbackAC2(const robosense_msgs::msg::RsImage4M::Ptr zc_msg_ptr) {
  rec_image_frame_.store(true);
  ImageMsgPtr msg_ptr = ConvertZCImageAC2(zc_msg_ptr);
  impl_ptr->AddData(msg_ptr);
}
#endif


void PostprocessNode::SubIMUCallback(const ImuMsgPtr msg_ptr) {
  rec_imu_.store(true);
  impl_ptr->AddData(msg_ptr);
}
void PostprocessNode::PubMsg(PostprocessOutputMsg::Ptr out_msg_ptr) {
#if defined(USE_ROS1)
  if (out_msg_ptr->motion_points_ptr != nullptr) {
    pub_motion_points_.publish(*(out_msg_ptr->motion_points_ptr));
  }
  if (out_msg_ptr->edge_points_ptr != nullptr) {
    pub_edge_points_.publish(*(out_msg_ptr->edge_points_ptr));
  }
  if (out_msg_ptr->points_proj_img_ptr != nullptr) {
    pub_motion_proj_img_.publish(*(out_msg_ptr->points_proj_img_ptr));
  }
  if (out_msg_ptr->ori_img_ptr != nullptr) {
    pub_ori_img_.publish(*(out_msg_ptr->ori_img_ptr));
  }
  if (out_msg_ptr->rgb_points_ptr!= nullptr) {
    pub_motion_rgb_points_.publish(*(out_msg_ptr->rgb_points_ptr));
  }
#elif defined(USE_ROS2)
  if (out_msg_ptr->edge_points_ptr != nullptr) {
    pub_edge_points_->publish(*(out_msg_ptr->edge_points_ptr));
  }
  if (out_msg_ptr->motion_points_ptr != nullptr) {
    pub_motion_points_->publish(*(out_msg_ptr->motion_points_ptr));
  }
  if (out_msg_ptr->points_proj_img_ptr != nullptr) {
    pub_motion_proj_img_->publish(*(out_msg_ptr->points_proj_img_ptr));
  }
  if (out_msg_ptr->ori_img_ptr != nullptr) {
    pub_ori_img_->publish(*(out_msg_ptr->ori_img_ptr));
  }
  if (out_msg_ptr->rgb_points_ptr!= nullptr) {
    pub_motion_rgb_points_->publish(*(out_msg_ptr->rgb_points_ptr));
  }
#endif
}

void PostprocessNode::Core() {
  while (run_flag_) {
    if ((!motion_correct_ || rec_imu_.load()) && rec_depth_frame_.load() && rec_image_frame_.load())
    {
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
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace postprocess
}  // namespace robosense

std::string ParseConfigOption(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      if (i + 1 < argc) {
        return argv[i + 1];
      } else {
        std::cerr << "--config need a config file" << std::endl;
        return "";
      }
    }
  }
  return "config/usr_config.yaml";
}

int main(int argc, char** argv) {
  const std::string cfg_path = ParseConfigOption(argc, argv);
  YAML::Node cfg = YAML::LoadFile(cfg_path)["postprocess_node"];
#if defined(USE_ROS1)
  ros::init(argc, argv, "postprocess_node");
#elif defined(USE_ROS2)
  rclcpp::init(argc, argv);
#endif
  auto node = std::make_shared<robosense::postprocess::PostprocessNode>(cfg);
#if defined(USE_ROS1)
  ros::spin();
#elif defined(USE_ROS2)
  rclcpp::spin(node->GetRos2Node());
  rclcpp::shutdown();
#endif
  return 0;
}
