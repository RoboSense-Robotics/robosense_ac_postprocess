#include "postprocess/range_image.hpp"
namespace robosense {
namespace postprocess {
RangeImage::RangeImage(const RangeImageParams &range_image_info, double scale)
{
  std::cout << "====== Range Image Info =======" << std::endl;
  std::cout << "  resolution: " << range_image_info.res * scale << "°" << std::endl;
  std::cout << "  pitch: [" << range_image_info.pitch_min << "°, " << range_image_info.pitch_max << "°]" << std::endl;
  std::cout << "  yaw: [" << range_image_info.yaw_min << "°, " << range_image_info.yaw_max << "°]" << std::endl;

  pitch_min_ = deg2rad(range_image_info.pitch_min);
  pitch_max_ = deg2rad(range_image_info.pitch_max);
  yaw_min_ = deg2rad(range_image_info.yaw_min);
  yaw_max_ = deg2rad(range_image_info.yaw_max);
  res_ = deg2rad(range_image_info.res * scale);
  res_inv_ = 1.0 / res_;

  pitch_size_ = (int)(std::ceil((pitch_max_ - pitch_min_) / res_));
  yaw_size_ = (int)(std::ceil((yaw_max_ - yaw_min_) / res_));
  index_img_.resize(pitch_size_, yaw_size_);
  depth_img_.resize(pitch_size_, yaw_size_);
  index_img_.setConstant(-1);
  depth_img_.setConstant(10000);
  std::cout << "  size: " << pitch_size_ << " * " << yaw_size_ << std::endl;

  // colormap
  color_img_ = cv::Mat::zeros(pitch_size_, yaw_size_, CV_8UC3);
  colormap_ = cv::Mat(colormap_size_, 1, CV_8UC1);
  for (int i = 0; i < colormap_size_; i++)
  {
    colormap_.at<uchar>(i, 0) = static_cast<uchar>(i);
  }
  cv::applyColorMap(colormap_, colormap_, cv::COLORMAP_JET);

  // 三角函数按行查表
  row2sin_.resize(pitch_size_);
  row2cos_.resize(pitch_size_);
  row2tan_.resize(pitch_size_);
  for (int row = 0; row < pitch_size_; ++row) {
    double pitch = getPitch(row);
    row2sin_[row] = std::sin(pitch);
    row2cos_[row] = std::cos(pitch);
    row2tan_[row] = std::tan(pitch);
  }
  col2sin_.resize(yaw_size_);
  col2cos_.resize(yaw_size_);
  col2tan_.resize(yaw_size_);
  for (int col = 0; col < yaw_size_; ++col) {
    double yaw = getYaw(col);
    col2sin_[col] = std::sin(yaw);
    col2cos_[col] = std::cos(yaw);
    col2tan_[col] = std::tan(yaw);
  }
}

std::vector<uint8_t> RangeImage::loadCloud(std::vector<Eigen::Vector3d> &points, bool fillHole)
{
  std::vector<uint8_t> valid_index(points.size(), 0);
  index_img_.setConstant(-1);
  depth_img_.setConstant(10000);
  for (size_t i = 0; i < points.size(); i++)
  {
    // 1. 投影点云
    const auto &pt = points[i];
    double pitch = getPitch(pt);
    double yaw = getYaw(pt);
    if (!inRoi(pitch, yaw)) continue;
    // 2. 更新 range image
    int row = getRow(pitch);
    int col = getCol(yaw);
    double depth = pt.head(2).norm();
    double depth_diff = depth - getDepth(row, col);
    if (getIndex(row, col) != -1)
    {
      if (depth_diff > 0) {
        valid_index[i] = depth_diff < 0.05;
        continue;
      }
      if (depth_diff < -0.05) {
        valid_index[getIndex(row, col)] = 0;
      }
    }
    setIndex(row, col, i);
    setDepth(row, col, depth);
    valid_index[i] = 1;
  }

  if (fillHole)
  {
    for (int row = 5; row < pitch_size_ - 5; row++)
    {
      for (int col = 5; col < yaw_size_ - 5; col++)
      {
        double depth = getDepth(row, col);
        double up_depth = std::min(getDepth(row - 2, col), getDepth(row - 1, col));
        double down_depth = std::min(getDepth(row + 2, col), getDepth(row + 1, col));
        double left_depth = std::min(getDepth(row, col - 2), getDepth(row, col - 1));
        double right_depth = std::min(getDepth(row, col + 2), getDepth(row, col + 1));
        bool col_hole = depth > std::max(up_depth, down_depth) * 1.05;// || (std::abs(up_depth - down_depth) < 0.1 && depth > std::max(up_depth, down_depth) * 1.03);
        bool row_hole = depth > std::max(left_depth, right_depth) * 1.05;// || (std::abs(left_depth - right_depth) < 0.1 && depth > std::max(left_depth, right_depth) * 1.03);
        // bool col_hole = depth - up_depth > 0.2 && depth - down_depth > 0.2;
        // bool row_hole = depth - left_depth > 0.2 && depth - right_depth > 0.2;
        if (!row_hole && !col_hole)
        {
          continue;
        }
        double new_depth = row_hole ? std::max(left_depth, right_depth) : std::max(up_depth, down_depth);
        // double new_depth = row_hole ? ((left_depth + right_depth) / 2.) : ((up_depth + down_depth) / 2.);
        double x = new_depth * getColCos(col);
        double y = new_depth * getColSin(col);
        double z = new_depth * getRowTan(row);
        points.emplace_back(Eigen::Vector3d{x, y, z});
        if (getIndex(row, col) != -1) {
          valid_index[getIndex(row, col)] = 0;
        }
        setDepth(row, col, new_depth);
        setIndex(row, col, points.size() - 1);
        valid_index.emplace_back(2);
      }
    }
    for (int row = 5; row < pitch_size_ - 5; row++)
    {
      for (int col = 5; col < yaw_size_ - 5; col++)
      {
        double depth = getDepth(row, col);
        double up_depth = std::min(getDepth(row - 2, col), getDepth(row - 1, col));
        double down_depth = std::min(getDepth(row + 2, col), getDepth(row + 1, col));
        double left_depth = std::min(getDepth(row, col - 2), getDepth(row, col - 1));
        double right_depth = std::min(getDepth(row, col + 2), getDepth(row, col + 1));
        bool col_hole = depth > std::max(up_depth, down_depth) * 1.05;// || (std::abs(up_depth - down_depth) < 0.1 && depth > std::max(up_depth, down_depth) * 1.03);
        bool row_hole = depth > std::max(left_depth, right_depth) * 1.05;// || (std::abs(left_depth - right_depth) < 0.1 && depth > std::max(left_depth, right_depth) * 1.03);
        // bool col_hole = depth - up_depth > 0.2 && depth - down_depth > 0.2;
        // bool row_hole = depth - left_depth > 0.2 && depth - right_depth > 0.2;
        if (!row_hole && !col_hole)
        {
          continue;
        }
        double new_depth = row_hole ? std::max(left_depth, right_depth) : std::max(up_depth, down_depth);
        // double new_depth = row_hole ? ((left_depth + right_depth) / 2.) : ((up_depth + down_depth) / 2.);
        double x = new_depth * getColCos(col);
        double y = new_depth * getColSin(col);
        double z = new_depth * getRowTan(row);
        points.emplace_back(Eigen::Vector3d{x, y, z});
        if (getIndex(row, col) != -1) {
          valid_index[getIndex(row, col)] = 0;
        }
        setDepth(row, col, new_depth);
        setIndex(row, col, points.size() - 1);
        valid_index.emplace_back(2);
      }
    }
  }

  return valid_index;
}

cv::Mat RangeImage::getCvMat()
{
  for (int row = 0; row < pitch_size_; row++)
  {
    for (int col = 0; col < yaw_size_; col++)
    {
      if (getIndex(row, col) == -1)
      {
        color_img_.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
        continue;
      }
      int colormap_index = getColormapIndex(getDepth(row, col));
      color_img_.at<cv::Vec3b>(row, col) = colormap_.at<cv::Vec3b>(colormap_index, 0);
    }
  }

  return color_img_;
}
}  // namespace postprocess
}  // namespace robosense