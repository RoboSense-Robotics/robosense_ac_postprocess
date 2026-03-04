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

#include "postprocess/point_type.hpp"
#include "postprocess/queue.hpp"
#include "postprocess/config.hpp"
#include "postprocess/imu_process.hpp"
#include "postprocess/rgb_postprocess.hpp"
#include "postprocess/stereo_image_projector.hpp"

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

  void AddData(const ImageMsgPtr& msg_ptr, const ImageMsgPtr& second_msg_ptr);

  void AddData(const CalibMsgPtr& msg_ptr);

  void Process(const PostprocessOutputMsg::Ptr& msg_ptr);

  bool ProcessPointCloud();

  bool ProcessCompressedImage();

  Eigen::Matrix4d getTCamCl(double cam_stamp, double pts_stamp);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectStereoImgToCloud(cv::Mat &left_img, cv::Mat &right_img, 
                                                                 cv::Mat &left_img_proj, cv::Mat &right_img_proj,
                                                                 pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                                 Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectImgToCloud(cv::Mat &img, cv::Mat &img_proj,
                                                           pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                           Eigen::Matrix4d &T_cam_cl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectRangeImgToCloud(cv::Mat &img, cv::Mat &img_proj,
                                                                pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                                Eigen::Matrix4d &T_cam_cl);

  void CreateColormapLUT(cv::Mat& colormap_lut, float min_z, float max_z);

  int GetColormapIndex(float depth, float min_z, float max_z, int lut_size);

  void DrawProjStereoImage(cv::Mat& _left_img, cv::Mat& _right_img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts,
                           const Eigen::Matrix4d& _left_transform, const Eigen::Matrix4d& _right_transform, std::vector<cv::Point2f>& _image_points);

  void DrawProjImage(cv::Mat& _img, const pcl::PointCloud<PointXYZIRT>::Ptr& _cm_pts, const Eigen::Matrix4d& _transform,
                     std::vector<cv::Point2f>& _image_points);

  void DrawProjImage(cv::Mat& _img, const std::vector<Eigen::Vector3d>& cam_pts,
                      const std::vector<uint8_t> &valid_index, std::vector<cv::Point2f>& _image_points);
 private:
  const std::string Name() const { return "PostprocessImpl"; }
  inline void labelImage(cv::Mat& img, const std::string& label);
  ImageMsgPtr findNearestCam(double point_stamp, ImageMsgPtr &second_image_msg_ptr);
  NodeConfig motion_cfg_;
  // for color mapping
  float min_z_ = 0;
  float max_z_ = 15.0;
  int lut_size_ = 256;
  cv::Mat colormap_lut_;

  double last_cloud_stamp_;
  double last_img_stamp_;

  std::shared_ptr<ImuProcess> imu_module_;
  std::shared_ptr<ImuProcess> imu_image_module_;

  Eigen::Matrix4d Tf_camR_2_cam_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_camR_2_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_lidar_2_imu_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_imu_2_lidar_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_lidar_2_cam_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_lidar_2_camR_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d Tf_imu_2_cam_ = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_base_lidar_ = Eigen::Matrix4d::Identity(); // Lidar to base

  cv::Mat distortion_coeffs_;
  cv::Mat camera_intrisic_;

  cv::Mat left_distortion_coeffs_;
  cv::Mat left_camera_intrisic_;
  cv::Mat right_distortion_coeffs_;
  cv::Mat right_camera_intrisic_;
  std::string projection_root_;
  std::string fusion_pcd_root_;
  bool motion_correct_;

  std::shared_ptr<RangeImage> range_image_;
  std::shared_ptr<RangeImage> cam_range_image_;

  std::shared_ptr<RgbPostprocess> rgb_post_process_;
  std::shared_ptr<StereoImageProjector> stereo_img_projector_;

  std::vector<int> rgb_index_;

  bool use_ori_img_;
  bool use_compressed_img_;
  bool use_range_img_;
  bool use_second_image_;
  int downsample_row_;
  int downsample_col_;
  bool use_online_calib_info_;
  bool get_online_calib_info_;
  bool using_imu_linear_acceleration_;
  bool using_odom_linear_velocity_;
  bool use_edge_filter_;
  bool frame_tail_;
  std::size_t num_drift_;
  double thres_;
  double inflate_depth_param_;
  double stereo_baseline_;

  PostprocessOutputMsg::Ptr out_msg_ptr_;

  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> point_cloud_free_vec_;
  SyncVector<pcl::PointCloud<PointXYZIRT>::Ptr> mc_point_cloud_vec_;

  SyncQueue<PointCloud2MsgPtr> point_cloud_queue_{1};
  std::mutex cam_mt_;
  std::deque<ImageMsgPtr> cam_queue_;
  std::deque<ImageMsgPtr> second_cam_queue_;
};

}  // namespace postprocess
}  // namespace robosense

#endif  // POSTPROCESS_POSTPROCESS_IMPL_H_
