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
  std::cout << Name() << ": init succeed!";
}

void PostprocessImpl::InitCalib() {
  const LidarConfig& lidar_cfg   = motion_cfg_.lidar_config;
  const CameraConfig& cam_cfg    = motion_cfg_.camera_config;
  const MotionConfig& motion_cfg = motion_cfg_.motion_config;
  // lidar2imu
  const TransformXYZQuat& lidar2imu_cfg = lidar_cfg.T_Lidar2IMU;
  Eigen::Quaterniond q_lidar2imu(lidar2imu_cfg.qw, lidar2imu_cfg.qx, lidar2imu_cfg.qy, lidar2imu_cfg.qz);
  RDEBUG << "lidar2imu: w " << q_lidar2imu.w() << " x "  << q_lidar2imu.x() << " y "  << q_lidar2imu.y() << " z "  << q_lidar2imu.z();
  T_imu_lidar_.block<3, 3>(0, 0) = q_lidar2imu.toRotationMatrix();
  T_imu_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar2imu_cfg.x, lidar2imu_cfg.y, lidar2imu_cfg.z);
  T_lidar_imu_ = T_imu_lidar_.inverse();
  // lidar2cam
  const TransformXYZQuat& lidar2cam_cfg = lidar_cfg.T_Lidar2Cam;
  Eigen::Quaterniond q_lidar2cam(lidar2cam_cfg.qw, lidar2cam_cfg.qx, lidar2cam_cfg.qy, lidar2cam_cfg.qz);
  RDEBUG << "lidar2cam: w " << q_lidar2cam.w() << " x "  << q_lidar2cam.x() << " y "  << q_lidar2cam.y() << " z "  << q_lidar2cam.z();
  T_cam_lidar_.block<3, 3>(0, 0) = q_lidar2cam.toRotationMatrix();
  T_cam_lidar_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar2cam_cfg.x, lidar2cam_cfg.y, lidar2cam_cfg.z);
  T_cam_imu_ = T_cam_lidar_ * T_imu_lidar_.inverse();
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
  if (use_range_img_) {
    use_ori_img_ = false;
  } else {
    use_ori_img_ = true;
  }

  range_image_ = std::make_shared<RangeImage>(lidar_cfg.range_image_info);
  cam_range_image_ = std::make_shared<RangeImage>(lidar_cfg.range_image_info);

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
    // RERROR << name() << ": get odom msg! " << msg_ptr->header.frame_id << ": " << msg_ptr->header.timestamp;
    auto imu_data = FromMsg(msg_ptr);
    imu_module_->addImuData(imu_data);
    imu_image_module_->addImuData(imu_data);
  } else {
    std::cout << Name() << ": get null imu msg!\n";
  }
}

void PostprocessImpl::AddData(const PointCloud2MsgPtr& msg_ptr) {
  if (msg_ptr != nullptr) {
    point_cloud_queue_.push(msg_ptr);
    std::cout << "point_cloud_queue_ size "<< point_cloud_queue_.size() << "\n";
  } else {
    std::cout << Name() << ": get null point cloud msg!\n" ;
  }
}

void PostprocessImpl::AddData(const ImageMsgPtr& msg_ptr) {
  if (msg_ptr != nullptr) {
    std::unique_lock<std::mutex> lck(cam_mt_);
    cam_queue_.push_back(msg_ptr);
    if (cam_queue_.size() > 30) {
      cam_queue_.pop_front();
    }
    std::cout << "cam_queue_ size "<< cam_queue_.size() << "\n";
  } else {
    std::cout << Name() << ": get null image msg!\n";
  }
}
void PostprocessImpl::Process(const PostprocessOutputMsg::Ptr& msg_ptr) {
  out_msg_ptr_ = msg_ptr;
  if (!ProcessPointCloud() || !ProcessCompressedImage()) {
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

  int point_index = 0;
  double head_stamp = ori_cloud->points.front().timestamp;
  double tail_stamp = ori_cloud->points.back().timestamp;
  double points_stamp = frame_tail_ ? tail_stamp : head_stamp;
  std::cout << "ProcessPointCloud: msg head stamp: " << std::to_string(head_stamp)
        << " tail stamp: " << std::to_string(tail_stamp)
        << " using stamp: " << std::to_string(points_stamp) << "\n";

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

    Eigen::Matrix4d T_l1_l2 = T_lidar_imu_ * T_i1_i2 * T_imu_lidar_;
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
  std::cout << "Succ Process PointCloud, Cost time: " + std::to_string(duration.count()) + " ms" << "\n";
  return true;
}

inline void PostprocessImpl::labelImage(cv::Mat& img, const std::string& label) {
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.2;
  int thickness = 2;
  cv::Point textOrg(30, 120);
  cv::putText(img, label, textOrg, fontFace, fontScale, cv::Scalar(0, 0, 255), thickness, 8);
}

ImageMsgPtr PostprocessImpl::findNearestCam(double point_stamp) {
  std::unique_lock<std::mutex> lck(cam_mt_);
  ImageMsgPtr res_msg = nullptr;
  double min_dur = std::numeric_limits<double>::max();
  for (size_t i = 0; i < cam_queue_.size(); i++) {
    auto msg = cam_queue_[i];
    // auto cur_stamp = msg->capture_time.tv_sec + msg->capture_time.tv_usec / 1e6;
    double cur_stamp = HeaderToSec(msg->header);
    auto cam_point_dura = std::abs(cur_stamp - point_stamp);
    if (cam_point_dura < thres_ && cam_point_dura < min_dur) {
      res_msg = msg;
      min_dur = cam_point_dura;
    }
  }
  if (res_msg == nullptr) {
    // auto tail_stamp = cam_queue_.back()->capture_time.tv_sec + cam_queue_.back()->capture_time.tv_usec / 1e6;
    double tail_stamp = HeaderToSec(cam_queue_.back()->header);
    std::cout << "Failed to find cam, tail cam is " << std::to_string(tail_stamp)
           << " " << std::to_string(point_stamp - tail_stamp);
  }
  return res_msg;
}

bool PostprocessImpl::ProcessCompressedImage() {
  std::size_t img_count = 0;
  auto start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<PointXYZIRT>::Ptr ori_pts, cm_pts;
  ori_pts = point_cloud_free_vec_.back();
  cm_pts = mc_point_cloud_vec_.back();
  auto pts_stamp = cm_pts->points.front().timestamp;
  auto msg = findNearestCam(pts_stamp);
  if (msg == nullptr) {
    std::cout << "cam msg is null\n";
    return false;
  }
  double cam_stamp = HeaderToSec(msg->header);
  std::cout << "Process image msg " << std::to_string(cam_stamp) << " with point stamp "
            << std::to_string(pts_stamp)
            << " diff is: " << std::to_string(pts_stamp - cam_stamp) << std::endl;

  cv::Mat img(cv::Size(msg->width, msg->height), CV_8UC3, msg->data.data());

  // 4. Get the transformation matrix from camera to lidar
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
  // 5. Get the angle and linear velocity
  Eigen::Matrix3d rotation_matrix = T_ic_il.block<3, 3>(0, 0);
  Eigen::AngleAxisd angle_axis(rotation_matrix);
  double angle = angle_axis.angle() * (180.0 / M_PI);
  double angle_velocity = angle / std::abs(pts_stamp - cam_stamp);
  Eigen::Vector3d translation = T_ic_il.block<3, 1>(0, 3);
  double linear_velocity = translation.norm() / std::abs(pts_stamp - cam_stamp);
  Eigen::Matrix4d T_cam_cl = T_cam_imu_ * T_ic_il * T_imu_lidar_;

  // 6. Project the point cloud to image
  if (use_ori_img_) {
    std::vector<cv::Point2f> image_points;
    cv::Mat img_proj = img.clone();
    DrawProjImage(img_proj, cm_pts, T_cam_cl, image_points);
    // 构造输出的点云消息
    std::cout << "image_points size: " << image_points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    Eigen::Matrix4d pub_trans = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < image_points.size(); ++i) {
      pcl::PointXYZRGB pt;
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
      }
      pt.r = pixel_value[0];
      pt.g = pixel_value[1];
      pt.b = pixel_value[2];
      rgb_pts->push_back(pt);
    }
    // std::cout << "rgb_pts size: " << rgb_pts->size() << std::endl;
    pcl::toROSMsg(*rgb_pts, *(out_msg_ptr_->rgb_points_ptr));
    out_msg_ptr_->rgb_points_ptr->header.frame_id = "rslidar";
    out_msg_ptr_->rgb_points_ptr->header.stamp = SecToHeaderStamp(pts_stamp);
    cv::cvtColor(img_proj, img_proj, cv::COLOR_RGB2BGR);
    auto img_msg_header = msg->header;
    img_msg_header.stamp = SecToHeaderStamp(pts_stamp);
    out_msg_ptr_->points_proj_img_ptr = cv_bridge::CvImage(img_msg_header, "bgr8", img_proj).toImageMsg();
    // robosense::ToImageMsg(img_proj);
    if (!projection_root_.empty()) {
      std::string file_name =
        projection_root_ + std::to_string(cam_stamp) + ".png";
      RINFO << "Save Image: " + file_name;
      cv::imwrite(file_name, img_proj);
    }
  } else if (use_range_img_) {
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
    cv::Mat img_proj_deocc = img.clone();
    DrawProjImage(img_proj_deocc, eigen_pts, valid_index, image_points_deocc);

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
      rgb_pts_deocc->points[i].r = pixel_value[0];
      rgb_pts_deocc->points[i].g = pixel_value[1];
      rgb_pts_deocc->points[i].b = pixel_value[2];
    }

    pcl::toROSMsg(*rgb_pts_deocc, *(out_msg_ptr_->rgb_points_ptr));
    out_msg_ptr_->rgb_points_ptr->header.frame_id = "rslidar";
    out_msg_ptr_->rgb_points_ptr->header.stamp = SecToHeaderStamp(pts_stamp);
    cv::cvtColor(img_proj_deocc, img_proj_deocc, cv::COLOR_RGB2BGR);
    auto img_msg_header = msg->header;
    img_msg_header.stamp = SecToHeaderStamp(pts_stamp);
    out_msg_ptr_->points_proj_img_ptr = cv_bridge::CvImage(img_msg_header, "bgr8", img_proj_deocc).toImageMsg();
  }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    std::cout << "Succ Process Camera, Cost time: " << duration.count() << " ms\n";
    return true;
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
      float depth = pts_3d[i].z();
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
    float depth = transform_points[i].z;
    int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
    cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
    cv::Scalar point_color(color[0], color[1], color[2]);
    cv::circle(_img, _image_points[i], 3, point_color, -1);
  }
#endif
}

void PostprocessImpl::DrawProjImage(cv::Mat& _img, const std::vector<Eigen::Vector3d>& cam_pts,
  const std::vector<uint8_t> &valid_index, std::vector<cv::Point2f>& _image_points) {
  std::vector<cv::Point3f> transform_points(cam_pts.size());
  for (size_t i = 0; i < cam_pts.size(); ++i) {
    transform_points[i].x = -cam_pts[i].y();
    transform_points[i].y = -cam_pts[i].z();
    transform_points[i].z = cam_pts[i].x();
  }

  cv::projectPoints(transform_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), camera_intrisic_,
                      distortion_coeffs_, _image_points);

  for (size_t i = 0; i < _image_points.size(); ++i) {
    int u = _image_points[i].x;
    int v = _image_points[i].y;
    if (u < 0 || u >= _img.cols || v < 0 || v >= _img.rows || !valid_index[i]) {
      continue;
    }

    float depth = transform_points[i].z;
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