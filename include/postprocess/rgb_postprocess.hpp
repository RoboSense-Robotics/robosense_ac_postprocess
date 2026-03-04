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
#ifndef RGB_POSTPROCESS_HPP
#define RGB_POSTPROCESS_HPP
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <thread>
#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <array>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace robosense {
namespace postprocess {

class RgbPostprocess {
public:
  RgbPostprocess();
  ~RgbPostprocess() {
  }

  void Init(const double angle_res, const double hori_fov,  const double verti_fov,
            const double detection_range, const double range_change_thresh, 
            const double color_change_thresh, const double color_min_thresh);

  void cloudToImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
  void updateCannyThresh();

  void detectColorEdges(const cv::Mat &color_image, cv::Mat &gradient_mag);
  std::vector<std::pair<int, double>> neighbourPointsDepthColorCount(const int cur_col, const int cur_row,
                                                                     int &depth_close_color_close_cnt, int &depth_jump_color_close_cnt,
                                                                     int &depth_close_color_jump_cnt, int &null_cnt);

  pcl::PointCloud<pcl::PointXYZI>::Ptr processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const bool use_edge_filter = false);

private:

  double half_cloud_horizontal_fov_;
  double half_cloud_vertical_fov_;

  double half_hori_angle_res_;
  double half_verti_angle_res_;

  double cloud_horizontal_fov_;
  double cloud_vertical_fov_;
  double cloud_hori_angle_res_;
  double cloud_verti_angle_res_;
  double detection_range_thresh_;
  double range_change_thresh_;
  double range_close_thresh_;
  double color_change_thresh_;
  double color_close_thresh_;
  double color_min_thresh_;
  double color_max_thresh_;

  int update_edge_thresh_count_;

  int img_yaw_cells_;
  int img_pitch_cells_;

  float canny_low_;
  float canny_high_;

  float detection_range_thresh_2_;

  int min_row_;
  int max_row_;
  int min_col_;
  int max_col_;
  int row_neighbour_;
  int col_neighbour_;
  int neighbour_num_ = 24;
  // 邻域偏移表（行偏移先行，方便预取）
  int dx_[24] = {
    -2, -1, 0, 1, 2,  // 上一行 (dy = -2)
    -2, -1, 0, 1, 2,  // 上一行 (dy = -1)
    -2, -1,    1, 2,   // 中间行 (dy = 0)
    -2, -1, 0, 1, 2   // 下一行 (dy = 1)
    -2, -1, 0, 1, 2,  // 上一行 (dy = 2)
  };
  int dy_[24] = {
    -2, -2, -2, -2, -2,
    -1, -1, -1, -1, -1,
     0,  0,      0,  0,
     1, 1, 1, 1, 1,
     2, 2, 2, 2, 2,
  };

  float cur_depth_;
  cv::Vec3b cur_color_;
  float depth_close_low_;
  float depth_close_high_;
  float depth_jump_low_;
  float depth_jump_high_;

  std::map<std::string, std::vector<int>> same_cell_points_map_; // 记录落入相同栅格的点的信息

  // 尺寸一次算好，生命周期随节点
  cv::Mat_<float> range32_;             // 32F 深度图
  cv::Mat_<float> idx32_;               // 32F 索引图
  cv::Mat color_image_;         // 颜色图像
  cv::Mat gradient_magnitude_;  // 颜色梯度图像
  cv::Mat range8u_;             // 8u 深度图
  cv::Mat open_;                // 形态学开运算
  cv::Mat gray_;                // 灰度图
  cv::Mat edge_;                //  边缘检测
  cv::Mat open_kernel_;                 // 形态学算子
  // 弧度→像素 比例预倒数，避免除法
  float ru_, rv_;                       // 1.0 / cloud_hori_angle_res_  ...
  float half_u_, half_v_;               // 半 FoV 偏移

  // 预计算的映射系数
  float yaw_scale_;
  float pitch_scale_;
  float yaw_offset_;
  float pitch_offset_;
};

}  // namespace postprocess
}  // namespace robosense

#endif  // RGB_POSTPROCESS_HPP
