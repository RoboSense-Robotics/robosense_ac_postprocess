/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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

  void AddData(const sensor_msgs::msg::Imu::SharedPtr& msg_ptr);

  void AddData(const sensor_msgs::msg::PointCloud2::SharedPtr& msg_ptr);

  void AddData(const sensor_msgs::msg::Image::SharedPtr& msg_ptr);

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
  std::shared_ptr<sensor_msgs::msg::Image> findNearestCam(double point_stamp);
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

  SyncQueue<std::shared_ptr<sensor_msgs::msg::PointCloud2>> point_cloud_queue_{1};
  std::mutex cam_mt_;
  std::deque<std::shared_ptr<sensor_msgs::msg::Image>> cam_queue_;
};

}  // namespace postprocess
}  // namespace robosense

#endif  // POSTPROCESS_POSTPROCESS_IMPL_H_
