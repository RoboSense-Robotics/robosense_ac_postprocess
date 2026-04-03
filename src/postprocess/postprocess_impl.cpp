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
#include "postprocess/postprocess_impl.hpp"
#include "postprocess/imu_process.hpp"
#include "postprocess/camera_model.h"
#include <thread>
#include <cv_bridge/cv_bridge.h>
namespace robosense {
namespace postprocess {
void PostprocessImpl::Init(const NodeConfig motion_cfg) {
  motion_cfg_ = motion_cfg;
  InitCalib();
  imu_module_ = std::make_shared<ImuProcess>(num_drift_, using_imu_linear_acceleration_, 9.81);
  imu_image_module_ = std::make_shared<ImuProcess>(num_drift_, using_imu_linear_acceleration_, 9.81);
  CreateColormapLUT(colormap_lut_, min_z_, max_z_);

  last_cloud_stamp_ = 0;
  last_img_stamp_ = 0;
  std::cout << Name() << ": init succeed!";
}

void PostprocessImpl::InitCalib() {
  const LidarConfig& lidar_cfg   = motion_cfg_.lidar_config;
  const CameraConfig& cam_cfg    = motion_cfg_.camera_config;
  const MotionConfig& motion_cfg = motion_cfg_.motion_config;

  Tf_lidar_2_imu_ = lidar_cfg.tf_Lidar2IMU;
  Tf_imu_2_lidar_ = lidar_cfg.tf_IMU2Lidar;
  Tf_imu_2_cam_ = lidar_cfg.tf_Lidar2Cam * lidar_cfg.tf_IMU2Lidar;

  Tf_camR_2_cam_ = cam_cfg.tf_CamR2Cam;

  stereo_baseline_ = cam_cfg.tf_CamR2Cam(0, 3);

  std::cout << "-- Tf_lidar_2_imu_  --- " << std::endl;
  std::cout << Tf_lidar_2_imu_ << std::endl;

  std::cout << "-- Tf_imu_2_lidar_  --- " << std::endl;
  std::cout << Tf_imu_2_lidar_ << std::endl;

  std::cout << "-- Tf_imu_2_cam_  --- " << std::endl;
  std::cout << Tf_imu_2_cam_ << std::endl;

  std::cout << "-- Tf_camR_2_cam_  --- " << std::endl;
  std::cout << Tf_camR_2_cam_ << std::endl;

  std::cout << "-- stereo_baseline_  --- " << std::endl;
  std::cout << stereo_baseline_ << std::endl;

  // 畸变系数
  std::vector<double> distortion_coeffs = cam_cfg.intrinsics.distortion_coeffs;
  distortion_coeffs_ = cv::Mat(distortion_coeffs).clone();
  // 内参
  std::vector<double> intrinsic_matrix = cam_cfg.intrinsics.camera_matrix;
  camera_intrisic_ = cv::Mat(3, 3, CV_64F, intrinsic_matrix.data()).clone();
  // 运动矫正参数
  projection_root_               = motion_cfg.projection_root;
  motion_correct_                = motion_cfg.motion_correct;
  using_imu_linear_acceleration_ = motion_cfg.using_imu_linear_acceleration;
  using_odom_linear_velocity_    = motion_cfg.using_odom_linear_velocity;
  frame_tail_                    = motion_cfg.frame_tail;
  num_drift_                     = motion_cfg.num_drift;
  thres_                         = motion_cfg.thres;
  use_range_img_                 = motion_cfg.use_range_img;
  use_compressed_img_            = motion_cfg.use_compressed_image;
  use_online_calib_info_         = motion_cfg.use_online_calib_topic;
  use_second_image_              = motion_cfg.use_second_image;
  use_edge_filter_               = motion_cfg.use_edge_filter;

  downsample_row_                = motion_cfg.downsample_row;
  downsample_col_                = motion_cfg.downsample_col;

  inflate_depth_param_           = motion_cfg.inflate_depth_param;
  if (use_range_img_) {
    use_ori_img_ = false;
  } else {
    use_ori_img_ = true;
  }

  rgb_index_ = {0, 1, 2};
  if (use_compressed_img_)
  {
    rgb_index_ = {2, 1, 0};
  }

  get_online_calib_info_ = false;
  if(!use_online_calib_info_)
  {
    get_online_calib_info_ = true;
  }

  range_image_ = std::make_shared<RangeImage>(lidar_cfg.range_image_info);
  cam_range_image_ = std::make_shared<RangeImage>(lidar_cfg.range_image_info);
  rgb_post_process_ = std::make_shared<RgbPostprocess>();
  stereo_img_projector_ = std::make_shared<StereoImageProjector>();

  if(use_second_image_)
  {
    // 畸变系数
    std::vector<double> left_distortion_coeffs = cam_cfg.left_intrinsics.distortion_coeffs;
    std::vector<double> left_intrinsic_matrix = cam_cfg.left_intrinsics.camera_matrix;
    std::vector<double> right_distortion_coeffs = cam_cfg.right_intrinsics.distortion_coeffs;
    std::vector<double> right_intrinsic_matrix = cam_cfg.right_intrinsics.camera_matrix;

    left_distortion_coeffs_ = cv::Mat(left_distortion_coeffs).clone();
    left_camera_intrisic_ = cv::Mat(3, 3, CV_64F, left_intrinsic_matrix.data()).clone();
    right_distortion_coeffs_ = cv::Mat(right_distortion_coeffs).clone();
    right_camera_intrisic_ = cv::Mat(3, 3, CV_64F, right_intrinsic_matrix.data()).clone();

    stereo_img_projector_->cameraParamInitialization(left_distortion_coeffs_, left_camera_intrisic_,
                                                     right_distortion_coeffs_, right_camera_intrisic_,
                                                     stereo_baseline_);
  }

  double verti_fov = lidar_cfg.range_image_info.pitch_max - lidar_cfg.range_image_info.pitch_min;
  double hori_fov = lidar_cfg.range_image_info.yaw_max - lidar_cfg.range_image_info.yaw_min;
  rgb_post_process_->Init(lidar_cfg.range_image_info.res, hori_fov, verti_fov,
                          lidar_cfg.range_image_info.detection_range, lidar_cfg.range_image_info.range_change_thresh
                          , lidar_cfg.range_image_info.color_change_thresh, lidar_cfg.range_image_info.color_min_thresh);

  std::cout << "------------------------------------------------------\n";
  std::cout << "MOTION_CORRECT                 : " + std::to_string(motion_correct_) << std::endl;
  std::cout << "USE_RANGE_IMG                  : " + std::to_string(use_range_img_) << std::endl;
  std::cout << "CAMERA_LIDAR_TIME_DURA_THRES   : " + std::to_string(thres_) << std::endl;
  std::cout << "USING_IMU_LINEAR_ACCELERATION  : " + std::to_string(using_imu_linear_acceleration_)  << std::endl;
  std::cout << "USING_ODOM_LINEAR_VELOCITY     : " + std::to_string(using_odom_linear_velocity_)  << std::endl;
  std::cout << "FRAME_TAIL                     : " + std::to_string(frame_tail_)  << std::endl;
  std::cout << "PROJ_IMG_TOPIC                 : " + projection_root_  << std::endl;
  std::cout << "------------------------------------------------------\n";
  std::cout << Name() << ": init calib done!";
}

void PostprocessImpl::AddData(const ImuMsgPtr& msg_ptr) {
  if (msg_ptr != nullptr) {
    if(!motion_correct_)
    {
      return;
    }
    // RERROR << name() << ": get odom msg! " << msg_ptr->header.frame_id << ": " << msg_ptr->header.timestamp;
    auto imu_data = FromMsg(msg_ptr);
    imu_module_->addImuData(imu_data);
    imu_image_module_->addImuData(imu_data);
  } else {
    std::cout << Name() << ": get null imu msg!\n";
  }
}

void PostprocessImpl::AddData(const PointCloud2MsgPtr& msg_ptr) {
  if (!get_online_calib_info_)
  {
    std::cout << Name() << ": use online calib mode , cloud waiting for calib msg!\n" ;
    return;
  }
  if (msg_ptr != nullptr) {
    point_cloud_queue_.push(msg_ptr);
    std::cout << "point_cloud_queue_ size "<< point_cloud_queue_.size() << "\n";
  } else {
    std::cout << Name() << ": get null point cloud msg!\n" ;
  }
}

void PostprocessImpl::AddData(const ImageMsgPtr& msg_ptr) {
  if (!get_online_calib_info_)
  {
    std::cout << Name() << ": use online calib mode , image waiting for calib msg!\n" ;
    return;
  }

  if (msg_ptr != nullptr) {
    std::unique_lock<std::mutex> lck(cam_mt_);
    cam_queue_.push_back(msg_ptr);
    if (cam_queue_.size() > 30) {
      cam_queue_.pop_front();
    }
    // std::cout << "cam_queue_ size "<< cam_queue_.size() << "\n";
  } else {
    std::cout << Name() << ": get null image msg!\n";
  }
}

void PostprocessImpl::AddData(const ImageMsgPtr& msg_ptr, const ImageMsgPtr& second_msg_ptr) {
  if (!get_online_calib_info_)
  {
    std::cout << Name() << ": use online calib mode , image waiting for calib msg!\n" ;
    return;
  }

  if (msg_ptr != nullptr || second_msg_ptr != nullptr) {
    std::unique_lock<std::mutex> lck(cam_mt_);
    cam_queue_.push_back(msg_ptr);
    second_cam_queue_.push_back(second_msg_ptr);
    if (cam_queue_.size() > 30) {
      cam_queue_.pop_front();
      second_cam_queue_.pop_front();
    }
    std::cout << "cam_queue_ size "<< cam_queue_.size() <<" second_cam_queue_ " <<second_cam_queue_.size()<< "\n";
  } else {
    std::cout << Name() << ": get null image msg!\n";
  }
}

void PostprocessImpl::AddData(const CalibMsgPtr& msg_ptr) {
    std::cout << Name() << ": CalibMsgPtr \n" ;
    std::cout << Name() << ": use_online_calib_info_ " <<use_online_calib_info_ << " get_online_calib_info_ "<<get_online_calib_info_<<std::endl;

  if (!use_online_calib_info_ || get_online_calib_info_)
  {
    return;
  }

  get_online_calib_info_ = true;

  std::vector<double> distortion_coeffs = {
      msg_ptr->camdistcoeff1,
      msg_ptr->camdistcoeff2,
      msg_ptr->camdistcoeff3,
      msg_ptr->camdistcoeff4,
      msg_ptr->camdistcoeff5,
      msg_ptr->camdistcoeff6,
      msg_ptr->camdistcoeff7,
      msg_ptr->camdistcoeff8,
  };
  distortion_coeffs_ = cv::Mat(distortion_coeffs).clone();
  std::vector<double> intrinsic_matrix = {
    msg_ptr->camfx, 0, msg_ptr->camcx,
    0, msg_ptr->camfy, msg_ptr->camcy,
    0, 0, 1
  };
  camera_intrisic_ = cv::Mat(3, 3, CV_64F, intrinsic_matrix.data()).clone();

  left_distortion_coeffs_ = distortion_coeffs_;
  left_camera_intrisic_ = camera_intrisic_;

  distortion_coeffs = {
      msg_ptr->camrdistcoeff1,
      msg_ptr->camrdistcoeff2,
      msg_ptr->camrdistcoeff3,
      msg_ptr->camrdistcoeff4,
      msg_ptr->camrdistcoeff5,
      msg_ptr->camrdistcoeff6,
      msg_ptr->camrdistcoeff7,
      msg_ptr->camrdistcoeff8,
  };
  right_distortion_coeffs_ = cv::Mat(distortion_coeffs).clone();
  intrinsic_matrix = {
    msg_ptr->camrfx, 0, msg_ptr->camrcx,
    0, msg_ptr->camrfy, msg_ptr->camrcy,
    0, 0, 1
  };
  right_camera_intrisic_ = cv::Mat(3, 3, CV_64F, intrinsic_matrix.data()).clone();

  Eigen::Matrix4d tf_Lidar2Cam = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_Lidar2IMU = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_IMU2Lidar = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d tf_Cam2Lidar = Eigen::Matrix4d::Identity();

  Eigen::Quaterniond q_imu2lidar(msg_ptr->imutolidarqw,
                                 msg_ptr->imutolidarqx,
                                 msg_ptr->imutolidarqy,
                                 msg_ptr->imutolidarqz);
  tf_IMU2Lidar.block<3, 3>(0, 0) = q_imu2lidar.toRotationMatrix();
  tf_IMU2Lidar.block<3, 1>(0, 3) = Eigen::Vector3d(
    msg_ptr->imutolidartx, msg_ptr->imutolidarty, msg_ptr->imutolidartz);
  tf_Lidar2IMU = tf_IMU2Lidar.inverse();

  Eigen::Quaterniond q_cam2lidar(msg_ptr->camtolidarqw,
                                 msg_ptr->camtolidarqx,
                                 msg_ptr->camtolidarqy,
                                 msg_ptr->camtolidarqz);
  tf_Cam2Lidar.block<3, 3>(0, 0) = q_cam2lidar.toRotationMatrix();
  tf_Cam2Lidar.block<3, 1>(0, 3) = Eigen::Vector3d(
    msg_ptr->camtolidartx+inflate_depth_param_, msg_ptr->camtolidarty, msg_ptr->camtolidartz);
  tf_Lidar2Cam = tf_Cam2Lidar.inverse();

  Tf_imu_2_lidar_ = tf_IMU2Lidar;
  Tf_lidar_2_imu_ = tf_Lidar2IMU;
  Tf_imu_2_cam_ = tf_Lidar2Cam * tf_IMU2Lidar;

  Eigen::Quaterniond q_camR2cam(msg_ptr->camrtocamqw,
                                msg_ptr->camrtocamqx,
                                msg_ptr->camrtocamqy,
                                msg_ptr->camrtocamqz);

  Tf_camR_2_cam_.block<3, 3>(0, 0) = q_camR2cam.toRotationMatrix();
  Tf_camR_2_cam_.block<3, 1>(0, 3) = Eigen::Vector3d(
    msg_ptr->camrtocamtx, msg_ptr->camrtocamty, msg_ptr->camrtocamtz);

  Tf_camR_2_lidar_ = tf_Cam2Lidar * Tf_camR_2_cam_;
  Tf_lidar_2_camR_ = Tf_camR_2_lidar_.inverse();

  stereo_baseline_ = msg_ptr->camrtocamtx;
  if(use_second_image_)
  {
    stereo_img_projector_->cameraParamInitialization(left_distortion_coeffs_, left_camera_intrisic_,
                                                     right_distortion_coeffs_, right_camera_intrisic_,
                                                     stereo_baseline_);
  }
  std::cout << Name() << ": update online calib info done  ! " <<std::endl;
  std::cout<<"-- camera_intrisic_ "<<std::endl;
  std::cout<<camera_intrisic_<<std::endl;
  std::cout<<"-- distortion_coeffs "<<std::endl;
  std::cout<< distortion_coeffs_ <<std::endl;
  std::cout<< "Tf_imu_2_cam_" <<std::endl;
  std::cout<< Tf_imu_2_cam_ <<std::endl;
  std::cout<< "Tf_imu_2_lidar_" <<std::endl;
  std::cout<< Tf_imu_2_lidar_ <<std::endl;
  std::cout<< "Tf_lidar_2_imu_" <<std::endl;
  std::cout<< Tf_lidar_2_imu_ <<std::endl;
  std::cout<<" inflate_depth_param_ "<<std::endl;
  std::cout<<inflate_depth_param_<<std::endl;
  std::cout << "-----------------" << std::endl;
}

void PostprocessImpl::Process(const PostprocessOutputMsg::Ptr& msg_ptr) {
  out_msg_ptr_ = msg_ptr;
  std::cout << "---------------process-------------" << std::endl;
  if (!ProcessPointCloud() || !ProcessCompressedImage())
  {
    std::cout << "Process failed\n";
  }
}
bool PostprocessImpl::ProcessPointCloud() {
  auto start = std::chrono::high_resolution_clock::now();
  auto msg = point_cloud_queue_.pop();
  if (msg == nullptr) {
    return false;
  }
  pcl::PointCloud<PointXYZIRT>::Ptr ori_cloud(new pcl::PointCloud<PointXYZIRT>());
  pcl::fromROSMsg(*msg, *ori_cloud);
  pcl::PointCloud<PointXYZIRT>::Ptr cm_cloud(new pcl::PointCloud<PointXYZIRT>());

  std::cout << "---------- ori_cloud " << ori_cloud->size() << std::endl;

  int point_index = 0;
  double head_stamp = ori_cloud->points.front().timestamp;
  double tail_stamp = ori_cloud->points.back().timestamp;
  double points_stamp = frame_tail_ ? tail_stamp : head_stamp;
  std::cout << "ProcessPointCloud: msg head stamp: " << std::to_string(head_stamp)
        << " tail stamp: " << std::to_string(tail_stamp)
        << " using stamp: " << std::to_string(points_stamp) << "\n";

  if(!motion_correct_)
  {
    for (size_t i = 0, num_pts = ori_cloud->points.size(); i < num_pts; ++i) {
    const auto& current_point = ori_cloud->points.at(point_index);
    PointXYZIRT cm_pt;
    cm_pt.x = current_point.x;
    cm_pt.y = current_point.y;
    cm_pt.z = current_point.z;
    cm_pt.intensity = current_point.intensity;
    cm_pt.ring = point_index;
    cm_pt.timestamp = points_stamp;
    cm_cloud->push_back(cm_pt);
    point_index++;
    }
    auto cm_cloud_msg = out_msg_ptr_->motion_points_ptr;
    pcl::toROSMsg(*cm_cloud, *cm_cloud_msg);
    cm_cloud_msg->header.frame_id = "rslidar";
    cm_cloud_msg->header.stamp = SecToHeaderStamp(points_stamp);
    mc_point_cloud_vec_.add(cm_cloud);
    point_cloud_free_vec_.add(ori_cloud);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "!motion_correct_ Succ Process PointCloud, Cost time: " + std::to_string(duration.count()) + " ms" 
    <<" cloud size "<<point_cloud_free_vec_.size() <<" mc cloud size "<<mc_point_cloud_vec_.size()<< "\n";
    return true;
  }

  MCStatus status = MCStatus::SKIPT;
  std::size_t count = 0, count_thres = 5;
  while (count++ < count_thres && !imu_module_->setHeadStamp(points_stamp, status, frame_tail_)) {
    std::cout << count << " setHeadStamp failed status: " << int(status) << "\n";
    if (MCStatus::WAITING == status || MCStatus::SKIPT == status) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }
    break;
  }

  if (MCStatus::SKIPT == status || MCStatus::WAITING == status) {
    std::cout << "Montion Correct Warning, SetHeadStamp Failed find Frames, Skip Montion Correct!\n";
    return false;
  }
  std::cout << "SetHeadStamp success find Frames, pts timestamp: "
      + std::to_string(points_stamp)
      + " imu head: " + std::to_string(imu_module_->imu_window_.front().imu_data.timestamp)
      + " imu tail: " + std::to_string(imu_module_->imu_window_.back().imu_data.timestamp)
      + " imu size: " + std::to_string(imu_module_->imu_window_.size()) << "\n";

  size_t search_idx = 0;
  Eigen::Matrix4d T_i1_i2 = Eigen::Matrix4d::Identity();
  for (size_t i = 0, num_pts = ori_cloud->points.size(); i < num_pts; ++i) {
    const auto& current_point = ori_cloud->points.at(point_index);
    double cur_stamp = current_point.timestamp;
    if (cur_stamp > tail_stamp) {
      std::cout << "points stamp error than head_stamp or tail_stamp:  "
      << std::to_string(head_stamp) << " " << std::to_string(cur_stamp) << " " << std::to_string(tail_stamp) << "\n";
    }
    if (!imu_module_->calHeadImuRot(cur_stamp, T_i1_i2, search_idx, frame_tail_)) {
      std::cout << "Montion Correct Warning, cal calHeadIMURot failed " + std::to_string(head_stamp) + " " +
                  std::to_string(cur_stamp) + " " + std::to_string(tail_stamp) << "\n";
      point_index++;
      continue;
    }
    if (frame_tail_) {
      Eigen::Matrix<double, 4, 4> _T_i1_i2_inv = T_i1_i2.eval();
      T_i1_i2 = _T_i1_i2_inv.inverse();
    }
    Eigen::Vector4d pt2(current_point.x, current_point.y, current_point.z, 1.0);

    Eigen::Matrix4d T_l1_l2 = Tf_imu_2_lidar_ * T_i1_i2 * Tf_lidar_2_imu_;
    Eigen::Vector4d trans_pt1 = T_l1_l2 * pt2;
    PointXYZIRT cm_pt;
    cm_pt.x = trans_pt1[0];
    cm_pt.y = trans_pt1[1];
    cm_pt.z = trans_pt1[2];
    cm_pt.intensity = current_point.intensity;
    cm_pt.ring = point_index;
    cm_pt.timestamp = points_stamp;
    cm_cloud->push_back(cm_pt);
    point_index++;
  }
  auto cm_cloud_msg = out_msg_ptr_->motion_points_ptr;
  pcl::toROSMsg(*cm_cloud, *cm_cloud_msg);
  cm_cloud_msg->header.frame_id = "rslidar";
  cm_cloud_msg->header.stamp = SecToHeaderStamp(points_stamp);
  mc_point_cloud_vec_.add(cm_cloud);
  point_cloud_free_vec_.add(ori_cloud);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration = end - start;
  std::cout << "motion_correct_ Succ Process PointCloud, Cost time: " + std::to_string(duration.count()) + " ms" << "\n";
  return true;
}

inline void PostprocessImpl::labelImage(cv::Mat& img, const std::string& label) {
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.2;
  int thickness = 2;
  cv::Point textOrg(30, 120);
  cv::putText(img, label, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness, 8);
}

ImageMsgPtr PostprocessImpl::findNearestCam(double point_stamp, ImageMsgPtr &second_image_msg_ptr) {
  std::unique_lock<std::mutex> lck(cam_mt_);
  ImageMsgPtr res_msg = nullptr;
  second_image_msg_ptr = nullptr;
  double min_dur = std::numeric_limits<double>::max();

  for (size_t i = 0; i < cam_queue_.size(); i++) {
    auto msg = cam_queue_[i];
    // auto cur_stamp = msg->capture_time.tv_sec + msg->capture_time.tv_usec / 1e6;
    double cur_stamp = HeaderToSec(msg->header);
    if(cur_stamp < last_img_stamp_)
    {
      continue;
    }
    auto cam_point_dura = std::abs(cur_stamp - point_stamp);
    if (cam_point_dura < thres_ && cam_point_dura < min_dur) {
      res_msg = msg;
      min_dur = cam_point_dura;
      if(second_cam_queue_.size() == cam_queue_.size())
      {
        second_image_msg_ptr = second_cam_queue_[i];
      }
    }
  }

  if (res_msg == nullptr) {
    // auto tail_stamp = cam_queue_.back()->capture_time.tv_sec + cam_queue_.back()->capture_time.tv_usec / 1e6;
    if(cam_queue_.size()<1)
    {
      std::cout<<"!!!! cam_queue_ "<<cam_queue_.size()<<std::endl;
    }
    else
    {
      double tail_stamp = HeaderToSec(cam_queue_.back()->header);
      std::cout << "Failed to find cam, tail cam is " << std::to_string(tail_stamp)
             << " " << std::to_string(point_stamp - tail_stamp);
    }
  }
  else
  {
    last_img_stamp_ = HeaderToSec(res_msg->header);
  }
  return res_msg;
}

bool PostprocessImpl::ProcessCompressedImage() {
  if(point_cloud_free_vec_.size()<1||mc_point_cloud_vec_.size()<1)
  {
    return false;
  }
  std::size_t img_count = 0;
  auto start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<PointXYZIRT>::Ptr ori_pts, cm_pts;
  ori_pts = point_cloud_free_vec_.back();
  cm_pts = mc_point_cloud_vec_.back();
  auto pts_stamp = cm_pts->points.front().timestamp;
  ImageMsgPtr second_image_msg_ptr = nullptr;
  if(pts_stamp < last_cloud_stamp_)
  {
    return false;
  }
  auto msg = findNearestCam(pts_stamp, second_image_msg_ptr);
  if (msg == nullptr) {
    std::cout << "cam msg is null\n";
    return false;
  }

  last_cloud_stamp_ = pts_stamp;

  cv::Mat img(cv::Size(msg->width, msg->height), CV_8UC3, msg->data.data());
  cv::Mat second_img;

  if(second_image_msg_ptr!=nullptr)
  {
    second_img = cv::Mat(second_image_msg_ptr->height, second_image_msg_ptr->width, CV_8UC3, second_image_msg_ptr->data.data());
  }

  double cam_stamp = HeaderToSec(msg->header);
  Eigen::Matrix4d T_cam_cl = getTCamCl(cam_stamp, pts_stamp);
  Eigen::Matrix4d T_camR_cl = Tf_camR_2_cam_.inverse() * T_cam_cl;

  // Project the point cloud to image
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr stereo_rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
  cv::Mat img_proj = img.clone();
  cv::Mat second_img_proj = second_img.clone();
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZI>);
  if (use_ori_img_) {
    if(second_image_msg_ptr!=nullptr)
    {
      // rgb_pts = projectStereoImgToCloud(img, second_img, img_proj, second_img_proj, cm_pts, T_cam_cl, T_camR_cl);

      rgb_pts = stereo_img_projector_->projectStereoImgToCloud(img, second_img, img_proj, second_img_proj, cm_pts, T_cam_cl, T_camR_cl);

      edge_points = rgb_post_process_->processCloud(rgb_pts, use_edge_filter_);
    }
    else
    {
      rgb_pts = projectImgToCloud(img, img_proj, cm_pts, T_cam_cl);
      edge_points = rgb_post_process_->processCloud(rgb_pts, use_edge_filter_);
    }
  } else if (use_range_img_) {
    rgb_pts = projectRangeImgToCloud(img, img_proj, cm_pts, T_cam_cl);
  }

  pcl::toROSMsg(*rgb_pts, *(out_msg_ptr_->rgb_points_ptr));
  pcl::toROSMsg(*edge_points, *(out_msg_ptr_->edge_points_ptr));

  out_msg_ptr_->rgb_points_ptr->header.frame_id = "rslidar";
  out_msg_ptr_->rgb_points_ptr->header.stamp = SecToHeaderStamp(pts_stamp);
  out_msg_ptr_->edge_points_ptr->header.frame_id = "rslidar";
  out_msg_ptr_->edge_points_ptr->header.stamp = SecToHeaderStamp(pts_stamp);

  // if(!use_compressed_img_)
  // {
  //   cv::cvtColor(img_proj, img_proj, cv::COLOR_RGB2BGR);
  //   cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
  // }

  auto img_msg_header = msg->header;
  img_msg_header.stamp = SecToHeaderStamp(pts_stamp);
  out_msg_ptr_->points_proj_img_ptr = cv_bridge::CvImage(img_msg_header, "rgb8", img_proj).toImageMsg();
  out_msg_ptr_->ori_img_ptr = cv_bridge::CvImage(img_msg_header, "rgb8", img).toImageMsg();
  // robosense::ToImageMsg(img_proj);
  if (!projection_root_.empty()) {
    std::string file_name =
      projection_root_ + std::to_string(cam_stamp) + ".png";
    RINFO << "Save Image: " + file_name;
    cv::imwrite(file_name, img_proj);
  }

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration = end - start;
  std::cout << "Succ Process Camera, Cost time: " << duration.count() << " ms\n";
  return true;
}

Eigen::Matrix4d PostprocessImpl::getTCamCl(double cam_stamp, double pts_stamp)
{
  std::cout << "Process image msg " << std::to_string(cam_stamp) << " with point stamp "
            << std::to_string(pts_stamp)
            << " diff is: " << std::to_string(pts_stamp - cam_stamp) << std::endl;

  // Get the transformation matrix from camera to lidar
  Eigen::Matrix4d T_ic_il = Eigen::Matrix4d::Identity(), T_il_ic = Eigen::Matrix4d::Identity();
  std::size_t imu_search_idx = 0;
  if (motion_correct_) {
    if (cam_stamp < pts_stamp) {
      if (!imu_image_module_->calImuRot(cam_stamp, pts_stamp, T_ic_il, imu_search_idx)) {
        std::cout << "Image proj CalIMURot Error, cam_stamp: " + std::to_string(cam_stamp) + " pts_stamp: " + std::to_string(pts_stamp) << std::endl;
      }
      T_il_ic = T_ic_il.inverse();
    } else if (cam_stamp > pts_stamp) {
      if (!imu_image_module_->calImuRot(pts_stamp, cam_stamp, T_il_ic, imu_search_idx)) {
        std::cout << "Image proj CalIMURot Error, cam_stamp: " + std::to_string(cam_stamp) + " pts_stamp: " + std::to_string(pts_stamp) << std::endl;
      }
      T_ic_il = T_il_ic.inverse();
    }

    if (imu_search_idx > 20) {
      imu_image_module_->clearData(imu_search_idx - 20);
      imu_search_idx = imu_search_idx - 20;
    }
  }

  // Get the angle and linear velocity
  Eigen::Matrix3d rotation_matrix = T_ic_il.block<3, 3>(0, 0);
  Eigen::AngleAxisd angle_axis(rotation_matrix);
  double angle = angle_axis.angle() * (180.0 / M_PI);
  double angle_velocity = angle / std::abs(pts_stamp - cam_stamp);
  Eigen::Vector3d translation = T_ic_il.block<3, 1>(0, 3);
  double linear_velocity = translation.norm() / std::abs(pts_stamp - cam_stamp);
  Eigen::Matrix4d T_cam_cl = Tf_imu_2_cam_ * T_ic_il * Tf_lidar_2_imu_;

  return T_cam_cl;
}

float calPointDepth(cv::Point3f &p)
{
  return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PostprocessImpl::projectStereoImgToCloud(
    cv::Mat &left_img, cv::Mat &right_img,
    cv::Mat &left_img_proj, cv::Mat &right_img_proj,
    pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
    Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl) {

    pcl::PointCloud<PointXYZIRT>::Ptr ori_pts = point_cloud_free_vec_.back();

    // parameters and thresh
    const int DOWNSAMPLE_BLOCK_ROW_SIZE = downsample_row_;
    const int DOWNSAMPLE_BLOCK_COL_SIZE = downsample_col_;
    const float fx = (left_camera_intrisic_.at<double>(0, 0) + right_camera_intrisic_.at<double>(0, 0)) / 2; 
    const float fy = (left_camera_intrisic_.at<double>(1, 1) + right_camera_intrisic_.at<double>(1, 1)) / 2;
    const float MIN_DISPARITY = 0.0f;           // 最小有效视差（像素）
    const float MAX_DISPARITY = stereo_baseline_ * ((fx + fy) / 2) / 0.1;         // 最大有效视差（像素）= baseline * focal_length / 0.1m
    const float COLOR_DIFF_THRESHOLD = 40.0f;   // 颜色差异阈值
    const float COLOR_SIMILARITY_THRESHOLD = 0.7f; // 颜色相似度阈值
    const float MAX_DISPARITY_CHANGE = 10.0f;   // 相邻点最大视差变化
    const float COLOR_CONSISTENCY_THRESHOLD = 0.8f; // 融合点颜色一致性阈值
    std::cout << "MAX_DISPARITY: " << MAX_DISPARITY << std::endl;

    // 2. 初始化深度缓冲区和索引缓冲区（基于降采样后的网格）
    int downsampled_cols = (left_img.cols + DOWNSAMPLE_BLOCK_COL_SIZE - 1) / DOWNSAMPLE_BLOCK_COL_SIZE;
    int downsampled_rows = (left_img.rows + DOWNSAMPLE_BLOCK_ROW_SIZE - 1) / DOWNSAMPLE_BLOCK_ROW_SIZE;
    
    cv::Mat left_depthBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32FC1, cv::Scalar(FLT_MAX));
    cv::Mat right_depthBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32FC1, cv::Scalar(FLT_MAX));
    
    cv::Mat left_indexBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32SC1, cv::Scalar(-1));
    cv::Mat right_indexBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32SC1, cv::Scalar(-1));
    
    // 置信度缓冲区
    cv::Mat left_confidenceBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32FC1, cv::Scalar(0.0f));
    cv::Mat right_confidenceBuffer = cv::Mat(downsampled_rows, downsampled_cols, CV_32FC1, cv::Scalar(0.0f));
    
    std::vector<cv::Point3f> left_transform_points;
    std::vector<cv::Point3f> right_transform_points;
    std::vector<double> left_points_depth;
    std::vector<double> right_points_depth;
    std::vector<cv::Point2f> left_image_points;
    std::vector<cv::Point2f> right_image_points;
    
    // 3. 计算变换、深度和投影
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        Eigen::Vector4d pt(cm_pts->points[i].x, cm_pts->points[i].y, cm_pts->points[i].z, 1.0);
        
        // 左目变换
        Eigen::Vector4d left_pt_trans = T_cam_cl * pt;
        left_transform_points.push_back(cv::Point3f(
            left_pt_trans[0], left_pt_trans[1], left_pt_trans[2]));
        left_points_depth.push_back(std::sqrt(
            left_pt_trans[0]*left_pt_trans[0] + 
            left_pt_trans[1]*left_pt_trans[1] + 
            left_pt_trans[2]*left_pt_trans[2]));
        
        // 右目变换
        Eigen::Vector4d right_pt_trans = T_camR_cl * pt;
        right_transform_points.push_back(cv::Point3f(
            right_pt_trans[0], right_pt_trans[1], right_pt_trans[2]));
        right_points_depth.push_back(std::sqrt(
            right_pt_trans[0]*right_pt_trans[0] + 
            right_pt_trans[1]*right_pt_trans[1] + 
            right_pt_trans[2]*right_pt_trans[2]));
    }
    
    // 4. 投影到图像平面
    cv::projectPoints(left_transform_points, 
                      cv::Mat::zeros(3, 1, CV_64F), 
                      cv::Mat::zeros(3, 1, CV_64F),
                      left_camera_intrisic_, 
                      left_distortion_coeffs_, 
                      left_image_points);
    
    cv::projectPoints(right_transform_points, 
                      cv::Mat::zeros(3, 1, CV_64F), 
                      cv::Mat::zeros(3, 1, CV_64F),
                      right_camera_intrisic_, 
                      right_distortion_coeffs_, 
                      right_image_points);
    
    std::cout << "Total points: " << cm_pts->points.size() << std::endl;
    
    // 5. 第一遍：处理左目投影，建立左目深度缓冲区（带降采样）
    int left_visible_count = 0;
    for (size_t i = 0; i < left_image_points.size(); ++i) {
        int u = std::round(left_image_points[i].x);
        int v = std::round(left_image_points[i].y);
        
        // 检查是否在左目图像范围内
        if (u < 0 || u >= left_img.cols || v < 0 || v >= left_img.rows) {
            continue;
        }
        
        float left_depth = left_points_depth[i];
        
        // 降采样：计算块的网格坐标
        int grid_u = u / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v = v / DOWNSAMPLE_BLOCK_ROW_SIZE;
        
        if (grid_u >= downsampled_cols || grid_v >= downsampled_rows) {
            continue;
        }
        
        // 深度缓冲区检查：只保留深度更小的点（更近的点）
        if (left_depth < left_depthBuffer.at<float>(grid_v, grid_u)) {
            left_depthBuffer.at<float>(grid_v, grid_u) = left_depth;
            left_indexBuffer.at<int>(grid_v, grid_u) = i;
            
            // 计算置信度（深度越小，置信度越高）
            left_confidenceBuffer.at<float>(grid_v, grid_u) = 1.0f / (1.0f + left_depth);
            left_visible_count++;
        }
    }
    std::cout << "Left visible points (after downsampling): " << left_visible_count << std::endl;
    
    // 6. 第二遍：处理右目投影，建立右目深度缓冲区（带降采样）
    int right_visible_count = 0;
    for (size_t i = 0; i < right_image_points.size(); ++i) {
        int u = std::round(right_image_points[i].x);
        int v = std::round(right_image_points[i].y);
        
        // 检查是否在右目图像范围内
        if (u < 0 || u >= right_img.cols || v < 0 || v >= right_img.rows) {
            continue;
        }
        
        float right_depth = right_points_depth[i];
        
        // 降采样：计算块的网格坐标
        int grid_u = u / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v = v / DOWNSAMPLE_BLOCK_ROW_SIZE;
        
        if (grid_u >= downsampled_cols || grid_v >= downsampled_rows) {
            continue;
        }
        
        // 深度缓冲区检查：只保留深度更小的点（更近的点）
        if (right_depth < right_depthBuffer.at<float>(grid_v, grid_u)) {
            right_depthBuffer.at<float>(grid_v, grid_u) = right_depth;
            right_indexBuffer.at<int>(grid_v, grid_u) = i;
            
            // 计算置信度（深度越小，置信度越高）
            right_confidenceBuffer.at<float>(grid_v, grid_u) = 1.0f / (1.0f + right_depth);
            right_visible_count++;
        }
    }
    std::cout << "Right visible points (after downsampling): " << right_visible_count << std::endl;
    
    // 7. 计算每个点的视差和颜色一致性信息
    std::vector<float> disparity_values(cm_pts->points.size(), 0.0f);
    std::vector<float> color_consistency_scores(cm_pts->points.size(), 0.0f);
    std::vector<bool> color_consistent_flags(cm_pts->points.size(), false);
    std::vector<bool> left_color_is_dark_flags(cm_pts->points.size(), true);
    
    int stereo_visible_count = 0;
    int color_consistent_count = 0;
    
    // 计算每个点的视差和颜色一致性
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        int u_left = std::round(left_image_points[i].x);
        int v_left = std::round(left_image_points[i].y);
        int u_right = std::round(right_image_points[i].x);
        int v_right = std::round(right_image_points[i].y);
        
        // 检查是否都在图像范围内
        bool left_in = (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows);
        bool right_in = (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows);
        
        if (!left_in || !right_in) {
            continue;
        }
        
        // 计算网格坐标
        int grid_u_left = u_left / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v_left = v_left / DOWNSAMPLE_BLOCK_ROW_SIZE;
        int grid_u_right = u_right / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v_right = v_right / DOWNSAMPLE_BLOCK_ROW_SIZE;
        
        // 检查是否是各自网格上深度最小的点
        bool left_valid = (left_indexBuffer.at<int>(grid_v_left, grid_u_left) == static_cast<int>(i));
        bool right_valid = (right_indexBuffer.at<int>(grid_v_right, grid_u_right) == static_cast<int>(i));
        
        if (!left_valid || !right_valid) {
            continue;
        }
        
        stereo_visible_count++;
        
        // 计算视差
        float disparity = std::abs(u_left - u_right);
        
        // 视差有效性检查
        if (disparity < MIN_DISPARITY || disparity > MAX_DISPARITY) {
            continue;
        }
        
        disparity_values[i] = disparity;
        
        // 获取左右目颜色
        cv::Vec3b left_color = left_img.at<cv::Vec3b>(v_left, u_left);
        cv::Vec3b right_color = right_img.at<cv::Vec3b>(v_right, u_right);
        
        float left_brightness = 0.299f * left_color[0] + 0.587f * left_color[1] + 0.114f * left_color[2];
        float right_brightness = 0.299f * right_color[0] + 0.587f * right_color[1] + 0.114f * right_color[2];

        if(left_brightness-right_brightness > 5)
        {
          left_color_is_dark_flags[i] = false;
        }

        // 计算颜色差异（自适应阈值，视差越大阈值越宽松）
        float adaptive_color_threshold = COLOR_DIFF_THRESHOLD * (1.0f + disparity / 100.0f);
        
        float color_diff = std::sqrt(
            std::pow(static_cast<int>(left_color[0]) - static_cast<int>(right_color[0]), 2) +
            std::pow(static_cast<int>(left_color[1]) - static_cast<int>(right_color[1]), 2) +
            std::pow(static_cast<int>(left_color[2]) - static_cast<int>(right_color[2]), 2)
        );
        
        // 计算颜色相似度（余弦相似度）
        float dot_product = left_color[0]*right_color[0] + left_color[1]*right_color[1] + left_color[2]*right_color[2];
        float norm_left = std::sqrt(left_color[0]*left_color[0] + left_color[1]*left_color[1] + left_color[2]*left_color[2]);
        float norm_right = std::sqrt(right_color[0]*right_color[0] + right_color[1]*right_color[1] + right_color[2]*right_color[2]);
        
        float color_similarity = 0.0f;
        if (norm_left > 0 && norm_right > 0) {
            color_similarity = dot_product / (norm_left * norm_right);
        }
        
        // 计算颜色一致性分数
        float adjusted_similarity_threshold = COLOR_SIMILARITY_THRESHOLD - (disparity / 300.0f);
        adjusted_similarity_threshold = std::max(adjusted_similarity_threshold, 0.4f);
        
        bool color_consistent = (color_diff < adaptive_color_threshold && 
                                color_similarity > adjusted_similarity_threshold);
        
        if (color_consistent) {
            color_consistent_flags[i] = true;
            color_consistency_scores[i] = color_similarity * (1.0f + disparity / 200.0f);
            color_consistent_count++;
        }
    }
    
    std::cout << "Stereo visible points: " << stereo_visible_count << std::endl;
    std::cout << "Color consistent points: " << color_consistent_count << std::endl;
    
    // 8. 视差统计
    float avg_disparity = 0.0f;
    float max_disparity = 0.0f;
    float min_disparity = FLT_MAX;
    int disparity_count = 0;
    
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        if (color_consistent_flags[i] && disparity_values[i] > 0) {
            float disparity = disparity_values[i];
            avg_disparity += disparity;
            max_disparity = std::max(max_disparity, disparity);
            min_disparity = std::min(min_disparity, disparity);
            disparity_count++;
        }
    }
    
    if (disparity_count > 0) {
        avg_disparity /= disparity_count;
        std::cout << "Disparity stats - Avg: " << avg_disparity 
                  << ", Min: " << min_disparity 
                  << ", Max: " << max_disparity << std::endl;
    }
    
    // 9. 确定每个点最终使用的相机（取并集逻辑 + 视差优化）
    std::vector<bool> use_left_camera(cm_pts->points.size(), false);
    std::vector<bool> use_right_camera(cm_pts->points.size(), false);
    std::vector<bool> use_both_camera(cm_pts->points.size(), false);
    std::vector<cv::Point2f> final_image_points(cm_pts->points.size());
    std::vector<float> final_depth_values(cm_pts->points.size(), 0.0f);
    
    int left_only_count = 0;
    int right_only_count = 0;
    int disparity_left_only_count = 0;
    int disparity_right_only_count = 0;
    int both_count = 0;
    int neither_count = 0;
    
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        int u_left = std::round(left_image_points[i].x);
        int v_left = std::round(left_image_points[i].y);
        int u_right = std::round(right_image_points[i].x);
        int v_right = std::round(right_image_points[i].y);
        
        bool left_in_image = (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows);
        bool right_in_image = (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows);
        
        int grid_u_left = left_in_image ? (u_left / DOWNSAMPLE_BLOCK_COL_SIZE) : -1;
        int grid_v_left = left_in_image ? (v_left / DOWNSAMPLE_BLOCK_ROW_SIZE) : -1;
        int grid_u_right = right_in_image ? (u_right / DOWNSAMPLE_BLOCK_COL_SIZE) : -1;
        int grid_v_right = right_in_image ? (v_right / DOWNSAMPLE_BLOCK_ROW_SIZE) : -1;
        
        bool left_is_closest = false;
        bool right_is_closest = false;
        float left_confidence = 0.0f;
        float right_confidence = 0.0f;
        
        if (left_in_image && grid_u_left >= 0 && grid_u_left < downsampled_cols && 
            grid_v_left >= 0 && grid_v_left < downsampled_rows) {
            left_is_closest = (left_indexBuffer.at<int>(grid_v_left, grid_u_left) == static_cast<int>(i));
            if (left_is_closest) {
                left_confidence = left_confidenceBuffer.at<float>(grid_v_left, grid_u_left);
            }
        }
        
        if (right_in_image && grid_u_right >= 0 && grid_u_right < downsampled_cols && 
            grid_v_right >= 0 && grid_v_right < downsampled_rows) {
            right_is_closest = (right_indexBuffer.at<int>(grid_v_right, grid_u_right) == static_cast<int>(i));
            if (right_is_closest) {
                right_confidence = right_confidenceBuffer.at<float>(grid_v_right, grid_u_right);
            }
        }
        
        // 视差和颜色一致性检查
        float disparity = disparity_values[i];
        bool valid_disparity = (disparity >= MIN_DISPARITY && disparity <= MAX_DISPARITY);
        bool color_consistent = color_consistent_flags[i];
        
        // 主逻辑：取并集 + 视差优化
        bool use_left = false;
        bool use_right = false;
        bool use_both = false;
        
        if (!left_in_image && right_in_image && right_is_closest) {
            // 左目看不到，取右目（如果右目是最深的）
            if (valid_disparity) {
                use_right = true;
                final_image_points[i] = right_image_points[i];
                final_depth_values[i] = right_points_depth[i];
                right_only_count++;
            }
        }
        else if (!right_in_image && left_in_image && left_is_closest) {
            // 右目看不到，取左目（如果左目是最深的）
            if (valid_disparity) {
                use_left = true;
                final_image_points[i] = left_image_points[i];
                final_depth_values[i] = left_points_depth[i];
                left_only_count++;
            }
        }
        else if (left_in_image && right_in_image) {
            // 左右目都可见
            if (left_is_closest && !right_is_closest) {
                // 左目最近，右目不是最近 -> 用左目
                if (valid_disparity) {
                    use_left = true;
                    final_image_points[i] = left_image_points[i];
                    final_depth_values[i] = left_points_depth[i];
                    left_only_count++;
                }
            }
            else if (!left_is_closest && right_is_closest) {
                // 右目最近，左目不是最近 -> 用右目
                if (valid_disparity) {
                    use_right = true;
                    final_image_points[i] = right_image_points[i];
                    final_depth_values[i] = right_points_depth[i];
                    right_only_count++;
                }
            }
            else if (left_is_closest && right_is_closest) {
                // 左右目都是最近的
                if (valid_disparity) {
                    if (color_consistent && color_consistency_scores[i] > COLOR_CONSISTENCY_THRESHOLD) {
                        // 颜色一致性高，进行加权融合
                        use_both = true;
                        final_image_points[i] = left_image_points[i]; // 使用左目坐标
                        final_depth_values[i] = (left_points_depth[i] + right_points_depth[i]) / 2.0f;
                        both_count++;
                    } else {
                        // 颜色一致性不高，删除
                    }
                }
            }
            else {
                // 都不是最近的（被遮挡）
                neither_count++;
                continue;
            }
        }
        else {
            // 至少有一目不在图像范围内，且另一目也不是最近的
            neither_count++;
            continue;
        }

        // 如果最终没有选择任何相机，但点通过了所有检查，则使用置信度更高的相机
        if (!use_left && !use_right && !use_both && valid_disparity) {
            if (left_in_image && left_is_closest) {
                use_left = true;
                final_image_points[i] = left_image_points[i];
                final_depth_values[i] = left_points_depth[i];
                disparity_left_only_count++;
            } else if (right_in_image && right_is_closest) {
                use_right = true;
                final_image_points[i] = right_image_points[i];
                final_depth_values[i] = right_points_depth[i];
                disparity_right_only_count++;
            }
        }
        
        use_left_camera[i] = use_left;
        use_right_camera[i] = use_right;
        use_both_camera[i] = use_both;
    }
    
    std::cout << "Left only: " << left_only_count 
              << ", Right only: " << right_only_count
              << ", disparity_left_only_count: " << disparity_left_only_count 
              << ", disparity_right_only_count: " << disparity_right_only_count
              << ", Both (fusion): " << both_count
              << ", Neither: " << neither_count << std::endl;
    
    // 10. 创建彩色点云（使用优化后的颜色融合策略）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {

        pcl::PointXYZRGB pt;
        const auto& cm_pt = ori_pts->points[i];
        pt.x = cm_pt.x;
        pt.y = cm_pt.y;
        pt.z = cm_pt.z;

        if (!use_left_camera[i] && !use_right_camera[i] && !use_both_camera[i]) {
            // pt.r = 0;
            // pt.g = 0;
            // pt.b = 0;
            // rgb_pts->push_back(pt);
            continue;
        }

        int u = std::round(final_image_points[i].x);
        int v = std::round(final_image_points[i].y);
        
        cv::Vec3b pixel_value(0, 0, 0);
        
        if (use_left_camera[i]) {
            // 使用左目颜色
            if (u >= 0 && u < left_img.cols && v >= 0 && v < left_img.rows) {
                pixel_value = left_img.at<cv::Vec3b>(v, u);
            }
        }
        else if (use_right_camera[i]) {
            // 使用右目颜色
            if (u >= 0 && u < right_img.cols && v >= 0 && v < right_img.rows) {
                pixel_value = right_img.at<cv::Vec3b>(v, u);
            }
        }
        else if (use_both_camera[i]) {
            // 加权融合：结合左右目颜色、深度、视差和一致性
            int u_left = std::round(left_image_points[i].x);
            int v_left = std::round(left_image_points[i].y);
            int u_right = std::round(right_image_points[i].x);
            int v_right = std::round(right_image_points[i].y);
            
            cv::Vec3b left_color(0, 0, 0);
            cv::Vec3b right_color(0, 0, 0);
            
            bool left_valid = false;
            bool right_valid = false;
            
            if (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows) {
                left_color = left_img.at<cv::Vec3b>(v_left, u_left);
                left_valid = true;
            }
            
            if (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows) {
                right_color = right_img.at<cv::Vec3b>(v_right, u_right);
                right_valid = true;
            }
            
            if (left_valid && right_valid) {
                float left_depth = left_points_depth[i];
                float right_depth = right_points_depth[i];
                float disparity = disparity_values[i];
                float consistency = color_consistency_scores[i];
                
                // 综合权重计算（深度 + 视差 + 一致性）
                float depth_weight_left = 1.0f / (1.0f + left_depth);
                float depth_weight_right = 1.0f / (1.0f + right_depth);
                
                // 视差影响：视差越大，左目权重越大（假设左目是主相机）
                float disparity_weight = 0.5f + (disparity / (disparity + 50.0f)) * 0.5f;
                
                // 一致性权重
                float consistency_weight = consistency;
                
                // 综合权重
                float left_weight = depth_weight_left * disparity_weight * consistency_weight;
                float right_weight = depth_weight_right * (1.0f - disparity_weight) * consistency_weight;
                
                // 归一化
                float total_weight = left_weight + right_weight;
                if (total_weight > 0) {
                    left_weight /= total_weight;
                    right_weight /= total_weight;
                    
                    pixel_value[0] = static_cast<uchar>(left_color[0] * left_weight + right_color[0] * right_weight);
                    pixel_value[1] = static_cast<uchar>(left_color[1] * left_weight + right_color[1] * right_weight);
                    pixel_value[2] = static_cast<uchar>(left_color[2] * left_weight + right_color[2] * right_weight);
                } else {
                    // 如果权重都为0，取平均值
                    pixel_value[0] = (left_color[0] + right_color[0]) / 2;
                    pixel_value[1] = (left_color[1] + right_color[1]) / 2;
                    pixel_value[2] = (left_color[2] + right_color[2]) / 2;
                }
            } else if (left_valid) {
                pixel_value = left_color;
            } else if (right_valid) {
                pixel_value = right_color;
            }
        }
        
        // 赋值颜色
        pt.r = pixel_value[0];
        pt.g = pixel_value[1];
        pt.b = pixel_value[2];
        
        rgb_pts->push_back(pt);
    }
    
    std::cout << "Final point cloud size: " << rgb_pts->size() 
              << " (Retention rate: " << (rgb_pts->size() * 100.0 / cm_pts->points.size()) << "%)" << std::endl;
    
    // 11. 可视化：显示多种信息
    if (!left_img_proj.empty() && !right_img_proj.empty()) {
        cv::Mat left_display = left_img.clone();
        cv::Mat right_display = right_img.clone();
        cv::Mat confidence_display = cv::Mat::zeros(left_img.size(), CV_8UC3);
        
        for (size_t i = 0; i < cm_pts->points.size(); ++i) {
            if (use_left_camera[i] || use_right_camera[i] || use_both_camera[i]) {
                int u_left = std::round(left_image_points[i].x);
                int v_left = std::round(left_image_points[i].y);
                int u_right = std::round(right_image_points[i].x);
                int v_right = std::round(right_image_points[i].y);
                
                // 根据点类型选择颜色
                cv::Scalar point_color;
                if (use_left_camera[i]) {
                    point_color = cv::Scalar(0, 0, 255); // 红色：左目
                } else if (use_right_camera[i]) {
                    point_color = cv::Scalar(0, 255, 0); // 绿色：右目
                } else if (use_both_camera[i]) {
                    point_color = cv::Scalar(255, 0, 0); // 蓝色：融合
                }
                
                // 在左图像上绘制
                if (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows) {
                    cv::circle(left_display, cv::Point(u_left, v_left), 2, point_color, -1);
                    
                    // 置信度显示
                    if (use_both_camera[i]) {
                        float confidence = color_consistency_scores[i];
                        int intensity = static_cast<int>(confidence * 255);
                        cv::circle(confidence_display, cv::Point(u_left, v_left), 2, 
                                  cv::Scalar(intensity, intensity, intensity), -1);
                    }
                }
                
                // 在右图像上绘制
                if (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows) {
                    cv::circle(right_display, cv::Point(u_right, v_right), 2, point_color, -1);
                }
            }
        }
        
        // cv::imshow("Left Image with Projected Points", left_display);
        // cv::imshow("Right Image with Projected Points", right_display);
        // cv::imshow("Confidence Map", confidence_display);
        // cv::waitKey(1);

        cv::Mat depthBuffer = cv::Mat::zeros(left_img_proj.size(), CV_32FC1);
        for (size_t i = 0; i < left_image_points.size(); ++i) {
          int u = left_image_points[i].x;
          int v = left_image_points[i].y;
          if (u<0 || u >= left_img_proj.cols || v<0 || v >= left_img_proj.rows) {
            continue;
          }
          float depth = left_points_depth[i];

          if (depthBuffer.at<float>(v, u) > depth)
          {
            left_image_points[i].x = -1;
            left_image_points[i].y = -1;
            continue;
          }
          depthBuffer.at<float>(v, u) = depth;
          int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
          cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
          cv::Scalar point_color(color[0], color[1], color[2]);
          cv::circle(left_img_proj, left_image_points[i], 3, point_color, -1);
        }
    }
    
    return rgb_pts;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PostprocessImpl::projectImgToCloud(cv::Mat &img, cv::Mat &img_proj,
                                                                          pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                                          Eigen::Matrix4d &T_cam_cl)
{
    pcl::PointCloud<PointXYZIRT>::Ptr ori_pts;
    ori_pts = point_cloud_free_vec_.back();

    std::vector<cv::Point2f> image_points;
    DrawProjImage(img_proj, cm_pts, T_cam_cl, image_points);
    // 构造输出的点云消息
    std::cout << "image_points size: " << image_points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d pub_trans = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < image_points.size(); ++i) {
      pcl::PointXYZRGB pt;
      // const auto& cm_pt = cm_pts->points[i]; // ? 为啥不是cm_pts->points[i];
      const auto& cm_pt = ori_pts->points[i]; // ? 为啥不是cm_pts->points[i];
      Eigen::Vector4d pt_eigen(cm_pt.x, cm_pt.y, cm_pt.z, 1.0);
      Eigen::Vector4d pt_trans = pub_trans * pt_eigen;

      pt.x = pt_trans[0];
      pt.y = pt_trans[1];
      pt.z = pt_trans[2];

      int u = image_points[i].x;
      int v = image_points[i].y;
      cv::Vec3b pixel_value(0, 0, 0);
      if (u >= 0 && u< img.cols && v >= 0 && v < img.rows) {
        pixel_value = img.at<cv::Vec3b>(v, u);
        pt.r = pixel_value[rgb_index_[0]];
        pt.g = pixel_value[rgb_index_[1]];
        pt.b = pixel_value[rgb_index_[2]];
        rgb_pts->push_back(pt);
      }

    }
    // std::cout << "rgb_pts size: " << rgb_pts->size() << std::endl;
    return rgb_pts;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PostprocessImpl::projectRangeImgToCloud(cv::Mat &img, cv::Mat &img_proj,
                                                                               pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                                               Eigen::Matrix4d &T_cam_cl)
{
   // range image
    std::vector<Eigen::Vector3d> eigen_pts(cm_pts->size());
    int num = 0;
    for (const auto &pt : cm_pts->points) {
      if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
      eigen_pts[num].x() = pt.x;
      eigen_pts[num].y() = pt.y;
      eigen_pts[num].z() = pt.z;
      num++;
    }
    eigen_pts.resize(num);
    range_image_->loadCloud(eigen_pts, true);
    // cv::Mat range_img = range_image_->getCvMat();
    // sensor_msgs::msg::Image::SharedPtr range_msg = robosense::ToImageMsg(range_img);
    // pub_motion_range_img_->publish(*range_msg);

    // camera range image
    Eigen::Matrix4d cam_rot;
    cam_rot << 0, 0, 1, 0,
               -1, 0, 0, 0,
               0, -1, 0, 0,
               0, 0, 0, 1;
    Eigen::Matrix4d T_caml_cl = cam_rot * T_cam_cl;
    Eigen::Matrix3d rot = T_caml_cl.topLeftCorner(3, 3);
    Eigen::Vector3d t = T_caml_cl.block<3, 1>(0, 3);
    for (auto &pt : eigen_pts) {
      pt = rot * pt + t;
    }
    std::vector<uint8_t> valid_index = cam_range_image_->loadCloud(eigen_pts, true);
    // cv::Mat cam_range_img = cam_range_image_->getCvMat();
    // sensor_msgs::msg::Image::SharedPtr cam_range_msg = robosense::ToImageMsg(cam_range_img);
    // pub_motion_cam_range_img_->publish(*cam_range_msg);

    // project
    std::vector<cv::Point2f> image_points_deocc;
    DrawProjImage(img_proj, eigen_pts, valid_index, image_points_deocc);

    // color
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts_deocc = rgb_pts;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts_deocc(new pcl::PointCloud<pcl::PointXYZRGB>());
    rgb_pts_deocc->resize(image_points_deocc.size());
    Eigen::Matrix4d T_cl_caml = T_caml_cl.inverse();
    rot = T_cl_caml.topLeftCorner(3, 3);
    t = T_cl_caml.block<3, 1>(0, 3);
    for (size_t i = 0; i < image_points_deocc.size(); ++i)
    {
      cv::Vec3b pixel_value(0.0, 0.0, 0.0);
      int u = image_points_deocc[i].x;
      int v = image_points_deocc[i].y;
      if (u >= 0 && u < img.cols && v >= 0 && v < img.rows && valid_index[i])
      {
        pixel_value = img.at<cv::Vec3b>(v, u);
      }

      Eigen::Vector3d pt = rot * eigen_pts[i] + t;
      rgb_pts_deocc->points[i].x = pt.x();
      rgb_pts_deocc->points[i].y = pt.y();
      rgb_pts_deocc->points[i].z = pt.z();
      rgb_pts_deocc->points[i].r = pixel_value[rgb_index_[0]];
      rgb_pts_deocc->points[i].g = pixel_value[rgb_index_[1]];
      rgb_pts_deocc->points[i].b = pixel_value[rgb_index_[2]];
    }
    return rgb_pts_deocc;
}

void PostprocessImpl::CreateColormapLUT(cv::Mat& colormap_lut, float min_z, float max_z) {
  int lut_size = 256;  // You can adjust the LUT size for more granularity
  cv::Mat colormap_lut_gray = cv::Mat(lut_size, 1, CV_8UC1);
  colormap_lut = cv::Mat(lut_size, 1, CV_8UC3);

  // Fill the LUT with values from 0 to 255
  for (int i = 0; i < lut_size; ++i)
  {
    colormap_lut_gray.at<uchar>(i, 0) = static_cast<uchar>(i);
  }

  // Apply the colormap to the LUT
  cv::applyColorMap(colormap_lut_gray, colormap_lut, cv::COLORMAP_JET);
}

int PostprocessImpl::GetColormapIndex(float depth, float min_z, float max_z, int lut_size) {
  depth = depth > max_z ? max_z : (depth<min_z ? min_z : depth);
  float normalized_depth = (depth - min_z) / (max_z - min_z);
  return static_cast<int>(normalized_depth * (lut_size - 1));
}

void PostprocessImpl::DrawProjImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                                    const Eigen::Matrix4d& _transform, std::vector<cv::Point2f>& _image_points) {

    cv::Mat depthBuffer = cv::Mat::zeros(_img.size(), CV_32FC1);
    cv::Mat depthChangeBuffer = cv::Mat::zeros(_img.size(), CV_32FC1);
    int count = 0;
#if defined(_WIN32) && false
    cv::Mat transform(4, 4, CV_64FC1);
    for(size_t i = 0; i < 4; i++){
      for(size_t j = 0; j < 4; j++){
        transform.at<double>(i, j) = _transform(i, j);
      }
    }
    PinholeModel::Ptr pinhole_model_ptr_(new PinholeModel(transform, camera_intrisic_, distortion_coeffs_, _img.size()));

    std::vector<Eigen::Vector3d> pts_3d;
    pts_3d.resize(_cm_pts->points.size());
    for(size_t i = 0; i < _cm_pts->points.size(); ++i){
      const auto& point = _cm_pts->points[i];
      pts_3d[i] = Eigen::Vector3d(point.x, point.y, point.z);
    }

    std::vector<cv::Point2i> pts_2d;
    bool isSuccessed = pinhole_model_ptr_->ProjectPts(pts_3d, pts_2d, true);
    if(isSuccessed == false){
      return;
    }
    _image_points.resize(pts_2d.size());
    for(size_t i = 0; i < pts_2d.size(); ++i){
      _image_points[i] = cv::Point2f(pts_2d[i].x, pts_2d[i].y);
    }
    for (size_t i = 0; i < _image_points.size(); ++i) {
      int u = _image_points[i].x;
      int v = _image_points[i].y;
      if (u<0 || u >= _img.cols || v<0 || v >= _img.rows) {
        continue;
      }
      float depth = calPointDepth(pts_3d[i]);

      if (depthBuffer.at<float>(v, u) > depth)
      {
        _image_points[i].x = -1;
        _image_points[i].y = -1;
        count++;
        continue;
      }
      depthBuffer.at<float>(v, u) = depth;
      int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
      cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
      cv::Scalar point_color(color[0], color[1], color[2]);
      cv::circle(_img, _image_points[i], 3, point_color, -1);
    }
#else
  std::vector<cv::Point3f> transform_points;
  for (size_t i = 0; i< _cm_pts->points.size(); ++i) {
    Eigen::Vector4d pt(_cm_pts->points[i].x, _cm_pts->points[i].y, _cm_pts->points[i].z, 1.0);
    Eigen::Vector4d pt_trans = _transform * pt;
    transform_points.push_back(cv::Point3f(pt_trans[0], pt_trans[1], pt_trans[2]));
  }

  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F),
                      camera_intrisic_, distortion_coeffs_, _image_points);

  for (size_t i = 0; i < _image_points.size(); ++i) {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u<0 || u >= _img.cols || v<0 || v >= _img.rows) {
      continue;
    }
    float depth = calPointDepth(transform_points[i]);

    if (depthBuffer.at<float>(v, u) > depth)
    {
      _image_points[i].x = -1;
      _image_points[i].y = -1;
      depthChangeBuffer.at<float>(v, u) = depth;
      count++;
      continue;
    }
    depthBuffer.at<float>(v, u) = depth;
    int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
    cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
    cv::Scalar point_color(color[0], color[1], color[2]);
    cv::circle(_img, _image_points[i], 3, point_color, -1);
  }
  // cv::imshow("depthBuffer", depthBuffer);
  // cv::waitKey(10);
  // cv::imshow("depthChangeBuffer", depthChangeBuffer);
  // cv::waitKey(10);
  std::cout<<"------------get  depth   count  "<<count<<std::endl;
#endif
}

void PostprocessImpl::DrawProjStereoImage(cv::Mat& _left_img, cv::Mat& _right_img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                                          const Eigen::Matrix4d& _left_transform, const Eigen::Matrix4d& _right_transform, std::vector<cv::Point2f>& _image_points)
{
}

void PostprocessImpl::DrawProjImage(cv::Mat& _img, const std::vector<Eigen::Vector3d>& cam_pts,
  const std::vector<uint8_t> &valid_index, std::vector<cv::Point2f>& _image_points) {

  cv::Mat depthBuffer = cv::Mat::zeros(_img.size(), CV_32FC1);

  std::vector<cv::Point3f> transform_points(cam_pts.size());
  for (size_t i = 0; i < cam_pts.size(); ++i) {
    transform_points[i].x = -cam_pts[i].y();
    transform_points[i].y = -cam_pts[i].z();
    transform_points[i].z = cam_pts[i].x();
  }

  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_intrisic_,
                      distortion_coeffs_, _image_points);

  int count = 0;
  for (size_t i = 0; i < _image_points.size(); ++i) {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u < 0 || u >= _img.cols || v < 0 || v >= _img.rows || !valid_index[i]) {
      continue;
    }

    float depth = transform_points[i].z;
    if (depthBuffer.at<float>(v, u) > depth)
    {
      _image_points[i].x = -1;
      _image_points[i].y = -1;
      count++;
      continue;
    }
    depth = depth > max_z_ ? max_z_ : depth;
    depth = std::max(depth, min_z_);
    float normalized_depth = (depth - min_z_) / (max_z_ - min_z_);
    int colormap_value = static_cast<int>(normalized_depth * 255);
    cv::Mat colormap_mat(1, 1, CV_8UC1, cv::Scalar(colormap_value));

    int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
    cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
    cv::Scalar point_color(color[0], color[1], color[2]);  // BGR format
    cv::circle(_img, _image_points[i], 3, point_color, -1);
  }
}

}  // namespace postprocess
}  // namespace robosense