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
#ifndef POSTPROCESS_POSTPROCESS_IMPL_H_
#define POSTPROCESS_POSTPROCESS_IMPL_H_
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include "postprocess/queue.hpp"
#include "postprocess/config.hpp"
#include "postprocess/imu_process.hpp"

struct EIGEN_ALIGN16 PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
      std::uint16_t, ring, ring)(double, timestamp, timestamp))

namespace robosense {
namespace postprocess {
class PostprocessImpl {
public:
  using Ptr = std::shared_ptr<PostprocessImpl>;

  PostprocessImpl() = default;

  ~PostprocessImpl() {}

  void Init(const NodeConfig motion_cfg);

  void InitCalib();

  void AddData(const ImuMsgPtr& msg_ptr);

  void AddData(const PointCloud2MsgPtr& msg_ptr);

  void AddData(const ImageMsgPtr& msg_ptr);

  void Process(const PostprocessOutputMsg::Ptr& msg_ptr);

  bool ProcessPointCloud();

  bool ProcessCompressedImage();

  void CreateColormapLUT(cv::Mat& colormap_lut, float min_z, float max_z);

  int GetColormapIndex(float depth, float min_z, float max_z, int lut_size);

  void DrawProjImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts, const Eigen::Matrix4d& _transform,
                     std::vector<cv::Point2f>& _image_points);

  void DrawProjImage(cv::Mat& _img, const std::vector<Eigen::Vector3d>& cam_pts,
                      const std::vector<uint8_t> &valid_index, std::vector<cv::Point2f>& _image_points);
 private:
  const std::string Name() const { return "PostprocessImpl"; }
  inline void labelImage(cv::Mat& img, const std::string& label);
  ImageMsgPtr findNearestCam(double point_stamp);
  NodeConfig motion_cfg_;
  // for color mapping
  float min_z_ = 0;
  float max_z_ = 15.0;
  int lut_size_ = 256;
  cv::Mat colormap_lut_;

  std::shared_ptr<ImuProcess> imu_module_;
  std::shared_ptr<ImuProcess> imu_image_module_;
  Eigen::Matrix4d T_imu_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity(); // Lidar to base

  cv::Mat distortion_coeffs_;
  cv::Mat camera_intrisic_;
  std::string projection_root_;
  std::string fusion_pcd_root_;
  bool motion_correct_;

  std::shared_ptr<RangeImage> range_image_;
  std::shared_ptr<RangeImage> cam_range_image_;

  bool use_ori_img_;
  bool use_range_img_;
  bool using_imu_linear_acceleration_;
  bool using_odom_linear_velocity_;
  bool frame_tail_;
  std::size_t num_drift_;
  double thres_;

  PostprocessOutputMsg::Ptr out_msg_ptr_;

  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> point_cloud_free_vec_;
  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> mc_point_cloud_vec_;

  SyncQueue<PointCloud2MsgPtr> point_cloud_queue_{1};
  std::mutex cam_mt_;
  std::deque<ImageMsgPtr> cam_queue_;
};

}  // namespace postprocess
}  // namespace robosense

#endif  // POSTPROCESS_POSTPROCESS_IMPL_H_
