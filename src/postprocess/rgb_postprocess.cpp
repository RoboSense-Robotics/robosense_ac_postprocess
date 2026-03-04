#include "postprocess/rgb_postprocess.hpp"

namespace robosense {
namespace postprocess {

RgbPostprocess::RgbPostprocess()
{
}

void RgbPostprocess::Init(const double angle_res, const double hori_fov,  const double verti_fov,
                          const double detection_range, const double range_change_thresh, 
                          const double color_change_thresh, const double color_min_thresh)
{
    cloud_horizontal_fov_ = hori_fov;
    cloud_vertical_fov_ = verti_fov;
    cloud_hori_angle_res_ = angle_res;
    cloud_verti_angle_res_ = angle_res;
    detection_range_thresh_ = detection_range;
    range_change_thresh_ = range_change_thresh;
    color_change_thresh_ = color_change_thresh;
    color_min_thresh_ = color_min_thresh;
    color_max_thresh_ = color_min_thresh*4 > 240?240:color_min_thresh*4;
    range_close_thresh_ = range_change_thresh_*0.5;
    color_close_thresh_ = color_change_thresh_*0.5;

    img_yaw_cells_ = static_cast<int>(cloud_horizontal_fov_ / cloud_hori_angle_res_);
    img_pitch_cells_ = static_cast<int>(cloud_vertical_fov_ / cloud_verti_angle_res_);

    half_hori_angle_res_ = cloud_hori_angle_res_ * 0.5;
    half_verti_angle_res_ = cloud_verti_angle_res_ * 0.5;

    half_cloud_horizontal_fov_ = cloud_horizontal_fov_*0.5;
    half_cloud_vertical_fov_ = cloud_vertical_fov_*0.5;

    canny_low_ = 40;
    canny_high_ = 120;
    detection_range_thresh_2_ = detection_range_thresh_*detection_range_thresh_;

    range32_.create(img_pitch_cells_, img_yaw_cells_);
    idx32_.create(img_pitch_cells_, img_yaw_cells_);
    range8u_.create(img_pitch_cells_, img_yaw_cells_, CV_8UC1);
    color_image_.create(img_pitch_cells_, img_yaw_cells_, CV_8UC3);
    open_.create(img_pitch_cells_, img_yaw_cells_, CV_8UC1);
    gray_.create(img_pitch_cells_, img_yaw_cells_, CV_8UC1);
    edge_.create(img_pitch_cells_, img_yaw_cells_, CV_8UC1);
    open_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    ru_ = 1.0f / cloud_hori_angle_res_;
    rv_ = 1.0f / cloud_verti_angle_res_;
    half_u_ = half_cloud_horizontal_fov_ + half_hori_angle_res_;
    half_v_ = half_cloud_vertical_fov_ + half_verti_angle_res_;
    row_neighbour_ = 2;
    col_neighbour_ = 2;
    min_row_ = row_neighbour_;
    max_row_ = edge_.rows - row_neighbour_;
    min_col_ = col_neighbour_;
    max_col_ = edge_.cols - col_neighbour_;
}

void RgbPostprocess::cloudToImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    // 0. 清零（比重新 create 快 3×）
    range8u_ = 255;
    range32_ = 255.f;
    idx32_ = -1.f;

    color_image_.setTo(cv::Scalar(0, 0, 0));

    same_cell_points_map_.clear();

    const int rows = range32_.rows;
    const int cols = range32_.cols;

    // 预计算每行指针（避免循环中重复调用 ptr）
    std::vector<float*> range_rows(rows);
    std::vector<float*> idx_rows(rows);
    std::vector<cv::Vec3b*> color_rows(rows);
    for (int r = 0; r < rows; ++r) {
        range_rows[r] = range32_.ptr<float>(r);
        idx_rows[r] = idx32_.ptr<float>(r);
        color_rows[r] = color_image_.ptr<cv::Vec3b>(r);
    }

    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& p = cloud->points[i];
        // 手动检查有限性（避免 pcl::isFinite 的函数调用开销）
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        // 距离平方提前比较，省一次 sqrt
        float rng2 = p.x * p.x + p.y * p.y + p.z * p.z;
        if (rng2 > detection_range_thresh_2_) continue;
        float rng = std::sqrt(rng2);
        // 弧度 → 像素（用乘法代替除法）
        float yaw   = (std::atan2(p.y, p.x) * 57.2957795131f + half_u_) * ru_;
        float pitch = (std::asin(p.z / rng) * 57.2957795131f + half_v_) * rv_;
        int u = static_cast<int>(yaw + 0.5f);
        int v = static_cast<int>(pitch + 0.5f);
        // 边界检查
        if (u < 0 || u >= cols || v < 0 || v >= rows) continue;
        // 直接写入（如果需要最近点，可添加 if (rng < range_rows[v][u])）
        if(range_rows[v][u] < 255)
        {
            std::string cur_u_v = std::to_string(u) + "_" + std::to_string(v);
            auto it = same_cell_points_map_.find(cur_u_v);
            // 第一次出现重复点的栅格，要把当前点也记录到栅格中
            if (it == same_cell_points_map_.end())
            {
                same_cell_points_map_[cur_u_v].push_back(idx_rows[v][u]);
            }
            // 将当前点也记录
            same_cell_points_map_[cur_u_v].push_back(i);
        }
        range_rows[v][u] = rng;
        idx_rows[v][u] = static_cast<float>(i);
        // 提取并存储RGB颜色
        uint32_t rgb = *reinterpret_cast<const uint32_t*>(&p.rgb);
        color_rows[v][u] = cv::Vec3b(
            (rgb >> 16) & 0xFF,    // R
            (rgb >> 8) & 0xFF,     // G
            (rgb) & 0xFF           // B
        );
    }

    // 2. 8U 量化（一次并行 convertTo）
    double scale = 255.0 / detection_range_thresh_;
    range32_.convertTo(range8u_, CV_8U, scale);
}

void RgbPostprocess::updateCannyThresh()
{
    // 确保 8UC1（几乎总是成立，可省分支）
    open_.convertTo(gray_, CV_8UC1);
    // 全黑/全白保护
    // const auto non_zero = cv::countNonZero(gray_);
    // if (non_zero == 0 || non_zero == gray_.total()) {
    //     canny_high_ = 80;
    // } else 
    // {
        cv::Mat _;   // dummy
        canny_high_ = cv::threshold(gray_, _, 0, 255, cv::THRESH_OTSU);
    // }
    canny_low_ = canny_high_ * 0.4;
}

// 把 imgs 按 row×col 拼成一张大图
cv::Mat tileImages(const std::vector<cv::Mat>& imgs,
                   int row, int col)
{
    CV_Assert(imgs.size() == row * col);
    cv::Size cell = imgs[0].size();
    cv::Mat big(cell.height * row, cell.width * col, imgs[0].type());

    for (int r = 0; r < row; ++r)
    {
        for (int c = 0; c < col; ++c)
        {
            cv::Mat roi = big(cv::Rect(c * cell.width,
                                       r * cell.height,
                                       cell.width,
                                       cell.height));
            imgs[r * col + c].copyTo(roi);
        }
    }
    return big;
}

void RgbPostprocess::detectColorEdges(const cv::Mat& color_image, cv::Mat& gradient_mag) {
    cv::Mat gray, grad_x, grad_y;
    // 转换为灰度图计算梯度
    cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
    // Sobel算子计算梯度
    cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
    // 计算梯度幅值
    cv::magnitude(grad_x, grad_y, gradient_mag);
    // 可选：非极大值抑制
    // applyNonMaximumSuppression(grad_x, grad_y, gradient_mag);
}

std::vector<std::pair<int, double>> RgbPostprocess::neighbourPointsDepthColorCount(const int cur_col, const int cur_row,
                                                int &depth_close_color_close_cnt, int &depth_jump_color_close_cnt,
                                                int &depth_close_color_jump_cnt, int &null_cnt)
{
    depth_jump_low_ = cur_depth_ - range_change_thresh_;
    depth_jump_high_ = cur_depth_ + range_change_thresh_;
    depth_close_low_ = cur_depth_ - range_close_thresh_;
    depth_close_high_ = cur_depth_ + range_close_thresh_;

    std::vector<std::pair<int, double>> neighbor_to_remove;
    for (int k = 0; k < neighbour_num_; ++k)
    {
        int u = cur_col + dx_[k];
        int v = cur_row + dy_[k];
        float neighbour_depth = *(range32_.ptr<float>(v) + u); // 当前边缘行邻域点深度
        if (neighbour_depth == 255.f)
        {
          null_cnt++;       // 无效深度
          continue;
        }
        cv::Vec3b& neighbor_color = color_image_.at<cv::Vec3b>(v, u);
        if (neighbor_color == cv::Vec3b(0, 0, 0)) {
            null_cnt++;       // 无效颜色
            continue;
        }
        int n_idx = static_cast<int>(*(idx32_.ptr<float>(v) + u));
        if (n_idx < 0)
        {
            continue;
        }
        // 计算颜色差异
        float color_diff = std::sqrt(
            std::pow(cur_color_[0] - neighbor_color[0], 2) +
            std::pow(cur_color_[1] - neighbor_color[1], 2) +
            std::pow(cur_color_[2] - neighbor_color[2], 2)
        );
        if (neighbour_depth >= depth_jump_low_ && neighbour_depth <= depth_jump_high_)
        {
          if(color_diff <= color_change_thresh_)
          {
              depth_close_color_close_cnt++;
              if(color_diff <= color_close_thresh_ && neighbour_depth >= depth_close_low_ && neighbour_depth <= depth_close_high_)
              {
                std::pair<int, double> new_p;
                new_p.first = n_idx;
                new_p.second = 200;
                neighbor_to_remove.push_back(new_p);
              }
          }
          else
          {
              depth_close_color_jump_cnt++;
          }
        }
        else{
            if(color_diff <= color_change_thresh_)
            {
                depth_jump_color_close_cnt++;
            }
        }
    }
    return neighbor_to_remove;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RgbPostprocess::processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const bool use_edge_filter)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZI>);

  if(!use_edge_filter)
  {
    return edge_points;
  }
  std::cout<<"-----------------------------------get cloud "<<std::endl;
  cloudToImage(cloud);

  cv::morphologyEx(range8u_, open_, cv::MORPH_OPEN, open_kernel_, cv::Point(-1,-1), 1);

  updateCannyThresh();

  cv::Canny(open_, edge_, canny_low_, canny_high_);

  // 预分配，避免 vector 反复 grow
  edge_points->reserve(edge_.rows * edge_.cols / 10);

  for (int y = min_row_; y < max_row_; ++y) 
  {
      const uchar* eptr  = edge_.ptr<uchar>(y);   // 边缘图行指针
      const float* idxPtr = idx32_.ptr<float>(y); // 索引图行指针
      const float* rng_row = range32_.ptr<float>(y);   // 深度图行指针
      const cv::Vec3b* color_row = color_image_.ptr<cv::Vec3b>(y); // 颜色图行指针
      for (int x = min_col_; x < max_col_; ++x) 
      {
          if (eptr[x] == 0) continue;                  // 非边缘
          int idx = static_cast<int>(idxPtr[x]);
          if (idx < 0) continue;
          cur_depth_ = rng_row[x];            // 当前边缘点深度
          if (cur_depth_ == 255.0f) continue; // 无效深度
          cur_color_ = color_row[x];                     // 获取当前点颜色
          double cur_gray = 0.299*cur_color_[0] + 0.587*cur_color_[1] + 0.114*cur_color_[2];
          if(cur_gray < color_min_thresh_)
          {
            continue;
          }
        //   if (cur_color_ == cv::Vec3b(0, 0, 0))
        //       continue; // 检查是否是黑色像素（无效点）

          int depth_close_color_close_cnt = 0;// 深度颜色都连续的点
          int depth_jump_color_close_cnt = 0;// 深度跳变但颜色接近的点
          int depth_close_color_jump_cnt = 0;// 深度连续但颜色跳变的点
          int null_cnt = 0;
          // 存储需要删除的邻域点索引
          std::vector<std::pair<int, double>> neighbor_to_remove = neighbourPointsDepthColorCount(x, y, depth_close_color_close_cnt,
                                                                               depth_jump_color_close_cnt, depth_close_color_jump_cnt, null_cnt);
          int depth_close_cnt = depth_close_color_close_cnt + depth_close_color_jump_cnt;

          // 保护小目标
          if (depth_close_cnt < 5)
          {
              continue;
          }

          std::pair<int, double> cur_pair;
          cur_pair.first = idx;
          cur_pair.second = 0;
          //深度跳变但是颜色连续点数很少，但是有
          //深度连续颜色连续点数很多
          //深度连续颜色跳变不存在
          if (depth_jump_color_close_cnt>0&&depth_jump_color_close_cnt<3 || depth_close_color_close_cnt > 10|| depth_close_color_jump_cnt==0)
          {
              neighbor_to_remove.clear();
          }
          else
          {
              // 深度跳变颜色连续点数大于3个，深度连续颜色连续点数少于6个，存在深度连续且颜色跳变的点
              cur_pair.second = 255;
              neighbor_to_remove.push_back(cur_pair);
          }
          if(cur_gray > color_max_thresh_ &&(neighbor_to_remove.size()>1||(depth_close_color_close_cnt<15 && depth_close_color_jump_cnt >0 && null_cnt>6)))
          {
            // 很亮的边缘的少数点
              cur_pair.second = 200;
              neighbor_to_remove.push_back(cur_pair);
          }
          if (null_cnt > 15)
          {
            cur_pair.second = 50; // 边缘孤立噪点
            neighbor_to_remove.push_back(cur_pair);
          }

          for(auto pair:neighbor_to_remove)
          {
              int index = pair.first;
              auto &ori_p = cloud->points[index];
              pcl::PointXYZI new_p;
              new_p.x = ori_p.x;
              new_p.y = ori_p.y;
              new_p.z = ori_p.z;
              new_p.intensity = pair.second;
              edge_points->points.emplace_back(new_p);
              ori_p.x = ori_p.y = ori_p.z = std::numeric_limits<float>::quiet_NaN();
            //   ori_p.r = ori_p.g = ori_p.b = 0;
          }
          // 如果当前点不需要删除，那么占据相同栅格的，深度连续的点也不需要删除
        //   if(neighbor_to_remove.size()<1)
        //   {
        //     continue;
        //   }
          std::string cur_u_v = std::to_string(x) + "_" + std::to_string(y);
          auto it = same_cell_points_map_.find(cur_u_v);
          if (it != same_cell_points_map_.end())
          {
              for (auto index : same_cell_points_map_[cur_u_v])
              {
                auto &ori_p = cloud->points[index];
                pcl::PointXYZI new_p;
                new_p.x = ori_p.x;
                new_p.y = ori_p.y;
                new_p.z = ori_p.z;
                new_p.intensity = 100; // 边缘膨胀点
                edge_points->points.emplace_back(new_p);
                // ori_p.r = ori_p.g = ori_p.b = 0;
                ori_p.x = ori_p.y = ori_p.z = std::numeric_limits<float>::quiet_NaN();
              }
          }
        }
  }

//   std::vector<cv::Mat> v = {range8u_, open_, edge_};
//   cv::Mat tiled = tileImages(v, 1, 3);
//   cv::imshow("img", tiled);
//   cv::waitKey(10);

//   cv::imshow("color_image", color_image_);
//   cv::waitKey(10);
  std::cout<<"-------------------------edge_points "<<edge_points->size()<<std::endl;

  return edge_points;
}
}
}