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
#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace robosense {
namespace postprocess {
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

struct RangeImageParams
{
  RangeImageParams() = default;
  double res;
  double pitch_min;
  double pitch_max;
  double yaw_min;
  double yaw_max;
};

// 查表法计算asin
class AsinLookupTable
{
 public:
  AsinLookupTable(int table_size = 10000)
  {
    res_ = 2.0 / (table_size - 1);
    res_inv_ = (table_size - 1) / 2.0;
    table_.resize(table_size);
    for (int i = 0; i < table_size; ++i)
    {
      double sin_value = -1.0 + i * res_;  // 生成[-1, 1]区间的sin值
      table_[i] = std::asin(sin_value);    // 计算asin(x)
    }
  }

  inline double lookup(double sin_value)
  {
    int index = static_cast<int>((sin_value + 1.0) * res_inv_);
    return table_[index];
  }

 private:
  double res_;                 // sin分辨率
  double res_inv_;             // sin分辨率倒数
  std::vector<double> table_;  // 存储asin值的查找表
};

class RangeImage
{
 public:
  template <typename Type, int Order = Eigen::RowMajor>
  using Image = Eigen::Array<Type, Eigen::Dynamic, Eigen::Dynamic, Order>;

  RangeImage(const RangeImageParams &range_image_info, double scale = 1.0);
  ~RangeImage() = default;

  /* 接口函数 */
  std::vector<uint8_t> loadCloud(std::vector<Eigen::Vector3d> &points, bool fillHole = true);
  cv::Mat getCvMat();

  /* 读取像素 */
  inline int getIndex(int x, int y) { return index_img_(x, y); }
  inline double getDepth(int x, int y) { return depth_img_(x, y); }

  /* 存入像素 */
  inline void setIndex(int x, int y, int i) { index_img_(x, y) = i; }
  inline void setDepth(int x, int y, double d) { depth_img_(x, y) = d; }

  /* 获取行列 */
  inline int getRow(double p) { return int((pitch_max_ - p) * res_inv_); }
  inline int getCol(double y) { return int((yaw_max_ - y) * res_inv_); }

  /* 获取角度 */
  inline double getPitch(int x) { return pitch_max_ - (x + 0.5) * res_; }
  inline double getYaw(int y) { return yaw_max_ - (y + 0.5) * res_; }

  /* 投影点云 */
  inline double getPitch(const Eigen::Vector3d &pt) { return lookupAsin(pt.z() / pt.norm()); }
  inline double getYaw(const Eigen::Vector3d &pt) { return lookupAsin(pt.y() / pt.head(2).norm()); }
  inline double getAbsYaw(const Eigen::Vector3d &pt) { return std::atan2(pt.y(), pt.x()); }

  /* 判断范围 */
  inline bool inPitchRoi(double p) { return p > pitch_min_ && p < pitch_max_; }
  inline bool inPitchRoi(int p) { return p >= 0 && p < pitch_size_; }
  inline bool inYawRoi(double y) { return y > yaw_min_ && y < yaw_max_; }
  inline bool inYawRoi(int y) { return y >= 0 && y < yaw_size_; }
  inline bool inRoi(double p, double y) { return inPitchRoi(p) && inYawRoi(y); }
  inline bool inRoi(int p, int y) { return inPitchRoi(p) && inYawRoi(y); }

  /* arcsin 查表 */
  AsinLookupTable asin_table_;  // asin 快速查表
  inline double lookupAsin(double sin) { return asin_table_.lookup(sin); }

  /* range image */
  double res_, res_inv_;
  double pitch_min_, yaw_min_;
  double pitch_max_, yaw_max_;
  int pitch_size_, yaw_size_;
  Image<int> index_img_;     // 点云索引图
  Image<double> depth_img_;  // 深度图

  /* colormap */
  cv::Mat color_img_;
  cv::Mat colormap_;
  int colormap_size_ = 256;
  double min_depth_ = 0.0;
  double max_depth_ = 5.0;
  inline int getColormapIndex(double depth)
  {
    depth = depth > max_depth_ ? max_depth_ : (depth < min_depth_ ? min_depth_ : depth);
    double depth_norm = (depth - min_depth_) / (max_depth_ - min_depth_);
    return static_cast<int>(depth_norm * (colormap_size_ - 1));
  }

  /* 三角函数按行查表 */
  std::vector<double> row2sin_, row2cos_, row2tan_;
  std::vector<double> col2sin_, col2cos_, col2tan_;
  inline double getRowSin(int row) { return row2sin_[row]; }
  inline double getRowCos(int row) { return row2cos_[row]; }
  inline double getRowTan(int row) { return row2tan_[row]; }
  inline double getColSin(int col) { return col2sin_[col]; }
  inline double getColCos(int col) { return col2cos_[col]; }
  inline double getColTan(int col) { return col2tan_[col]; }
};
}  // namespace postprocess
}  // namespace robosense
