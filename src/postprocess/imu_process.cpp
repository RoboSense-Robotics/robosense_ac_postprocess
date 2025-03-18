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
#include <rclcpp/rclcpp.hpp>
#include "postprocess/imu_process.hpp"

namespace robosense {
namespace postprocess {
ImuProcess::ImuProcess(std::size_t _num)
  : num_window_(_num),
    use_linear_acceleration_(false),
    imu_frame_rate_(200),
    lidar_frame_rate_(10) {
  drift_ = ImuData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, 9.81);
  this->print();
}

ImuProcess::ImuProcess(std::size_t _num, bool _use_linear_acceleration, double _gravity)
  : num_window_(_num),
    use_linear_acceleration_(_use_linear_acceleration),
    imu_frame_rate_(200),
    lidar_frame_rate_(10) {
  drift_ = ImuData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, _gravity);
  this->print();
}

ImuProcess::ImuProcess(std::size_t _num, bool _use_linear_acceleration, double _gravity, double _imu_frame_rate, double _lidar_frame_rate)
  : num_window_(_num),
    use_linear_acceleration_(_use_linear_acceleration),
    imu_frame_rate_(_imu_frame_rate),
    lidar_frame_rate_(_lidar_frame_rate) {
  drift_ = ImuData();
  gravity_vector_ = Eigen::Vector3d(0.0, 0.0, _gravity);
  this->print();
}

void ImuProcess::print() {
  std::cout << "------------------------------------------------------" << std::endl;
  std::cout << "IMU Module Config" << std::endl;
  std::cout << "Use Linear Acceleration: " + std::to_string(use_linear_acceleration_) << std::endl;
  std::cout << "Number Frame Drift: " + std::to_string(num_window_) << std::endl;
  std::cout << "Init Global Gravity: " + std::to_string(gravity_vector_[0]) + " " + std::to_string(gravity_vector_[1]) + " " + std::to_string(gravity_vector_[2]) << std::endl;
  std::cout << "------------------------------------------------------" << std::endl;
}

ImuData& ImuProcess::back() {
  std::lock_guard<std::mutex> lg(mtx_);
  return imu_datas_.back().imu_data;
}

bool ImuProcess::isDrifted() {
  std::lock_guard<std::mutex> lg(drift_mtx_);
  return is_drifted_;
}

void ImuProcess::pop_front() {
  std::lock_guard<std::mutex> lg(mtx_);
  imu_datas_.pop_front();
}

void ImuProcess::pop_back() {
  std::lock_guard<std::mutex> lg(mtx_);
  imu_datas_.pop_back();
}

std::size_t ImuProcess::size() {
  std::lock_guard<std::mutex> lg(mtx_);
  return imu_datas_.size();
}

bool ImuProcess::addImuData(const ImuData& _data) {
  std::lock_guard<std::mutex> lg(mtx_);
  InteImuData integration_imu_data(_data);
  double delta_time = imu_datas_.empty() ? 0.005 : _data.timestamp - imu_datas_.back().imu_data.timestamp;
  // if (delta_stamp < 0.004 || delta_stamp > 0.006)
  // {
  //   LOG_WARNING << "Time step of IMU warning: " << std::to_string(delta_stamp) << std::endl;
  // }

  if (use_linear_acceleration_ && is_drifted_)
  {
    // 如果为空则lhs假设值和当前帧相同
    const InteImuData lhs = imu_datas_.empty() ? integration_imu_data : imu_datas_.back();
    const Eigen::Matrix4d T_inter = getPose(lhs.imu_data, _data);
    const Eigen::Matrix3d R_current = lhs.R * T_inter.block<3, 3>(0, 0); // R_global_local
    integration_imu_data.R = R_current;
    const Eigen::Vector3d linear_acceleration_local = Eigen::Vector3d(_data.linear_acceleration_x, _data.linear_acceleration_y, _data.linear_acceleration_z);
    Eigen::Vector3d linear_velocity_local = lhs.v + delta_time * (linear_acceleration_local - R_current.transpose() * gravity_vector_);
    integration_imu_data.v = linear_velocity_local;
  }

  imu_datas_.emplace_back(integration_imu_data);
  if (!is_drifted_ && imu_datas_.size() > num_window_)
  {
    std::lock_guard<std::mutex> lg_shift(drift_mtx_);

    for (auto it = imu_datas_.begin(); it != imu_datas_.begin() + num_window_; ++it)
    {
      drift_ = drift_ + it->imu_data;
    }
    drift_ = drift_ / static_cast<double>(num_window_);
    is_drifted_ = true;

    std::ostringstream oss;
    oss << "Montion Correct IMU drifted finished!"
              << " angle_vx: "<< std::to_string(drift_.angular_velocity_x) << " angle_vy: "<< std::to_string(drift_.angular_velocity_y) << " angle_vz: "<< std::to_string(drift_.angular_velocity_z)
              << " linear_ax: "<< std::to_string(drift_.linear_acceleration_x) << " linear_ay: "<< std::to_string(drift_.linear_acceleration_y) << " linear_az: "<< std::to_string(drift_.linear_acceleration_z);
    RINFO << oss.str();
    if (use_linear_acceleration_)
    {
      Eigen::Vector3d gravity_vector_local(drift_.linear_acceleration_x, drift_.linear_acceleration_y, drift_.linear_acceleration_z);
      double gravity = gravity_vector_local.norm();
      gravity_vector_ = Eigen::Vector3d(0.0, 0.0, gravity);

      // calculate positon from gravity
      Eigen::Matrix3d R_global_local = calRotationFromTwoVector(gravity_vector_local, gravity_vector_);

      Eigen::Vector3d rpy = eulerAnglesZYX(Eigen::Quaterniond(R_global_local)) * (180.0 / M_PI);
      Eigen::Vector3d gravity_vector_local_fixed = R_global_local.transpose() * gravity_vector_;
      RINFO << "Local Gravity         : " + std::to_string(gravity_vector_local[0]) + " " + std::to_string(gravity_vector_local[1]) + " " + std::to_string(gravity_vector_local[2]);
      RINFO << "Global Gravity        : " + std::to_string(gravity_vector_[0]) + " " + std::to_string(gravity_vector_[1]) + " " + std::to_string(gravity_vector_[2]);
      RINFO << "Global Gravity Fixed  : " + std::to_string(gravity_vector_local_fixed[0]) + " " + std::to_string(gravity_vector_local_fixed[1]) + " " + std::to_string(gravity_vector_local_fixed[2]);
      RINFO << "Angle (rpy)           : " + std::to_string(rpy[0]) + " " + std::to_string(rpy[1]) + " " + std::to_string(rpy[2]);
      for (auto it = imu_datas_.begin(); it != imu_datas_.end(); ++it)
      {
        it->R = R_global_local;
      }
    }
  }

  return true;
}

bool ImuProcess::findFrames(double _stamp, InteImuData& _lhs, InteImuData& _rhs, std::size_t& _search_idx)
{
  if (_stamp < imu_window_.front().imu_data.timestamp)
  {
    // RS_WARNING << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames stamp: " << _stamp << " less than front " << imu_window_.front().imu_data.timestamp << std::endl;
    return false;
  }

  if (_stamp > imu_window_.back().imu_data.timestamp)
  {
    // RS_WARNING << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames stamp: " << _stamp << " more than back " << imu_window_.back().imu_data.timestamp << std::endl;
    return false;
  }

  if (_search_idx >= imu_window_.size())
  {
    _search_idx = imu_window_.size() - 1;
    // RCLCPP_WARN(rclcpp::get_logger(), "Montion Correct Warning, findFrames search_idx outof range");
    return false;
  }

  std::size_t search_idx = _search_idx;
  while (search_idx < imu_window_.size() && _stamp > imu_window_.at(search_idx).imu_data.timestamp)
  {
    search_idx++;
  }

  while (search_idx > 1 && _stamp <= imu_window_.at(search_idx-1).imu_data.timestamp)
  {
    search_idx--;
  }

  if(search_idx > 1 && _stamp > imu_window_.at(search_idx-1).imu_data.timestamp
          && search_idx < imu_window_.size() && _stamp <= imu_window_.at(search_idx).imu_data.timestamp)
  {
    _lhs = imu_window_.at(search_idx-1);
    _rhs = imu_window_.at(search_idx);
    _search_idx = search_idx;
  }
  else
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, findFrames error"
      << " _search_idx: " << std::to_string(_search_idx)
      << " size: " << std::to_string(imu_window_.size())
      << " stamp: " << std::to_string(_stamp)
      << " low stamp" << std::to_string(imu_window_.at(search_idx-1).imu_data.timestamp)
      << " high stamp" << std::to_string(imu_window_.at(search_idx).imu_data.timestamp);
    // RWARN << oss.str();
    // RCLCPP_WARN(rclcpp::get_logger(), oss.str());
    return false;
  }

  return true;
}

bool ImuProcess::calImuRot(double _st_stamp, double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _lhs_idx)
{
  std::lock_guard<std::mutex> lg(mtx_);
  if (!is_drifted_ || imu_datas_.empty())
  {
    return false;
  }

  if (_st_stamp < imu_datas_.front().imu_data.timestamp || _end_stamp > imu_datas_.back().imu_data.timestamp)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9) << "Montion Correct Warning calIMURot Wrong Stamp! "
      << imu_datas_.front().imu_data.timestamp << " "
      << _st_stamp << " "
      << imu_datas_.back().imu_data.timestamp << " "
      << _end_stamp << "\n";
    // std::cout << oss.str();
    return false;
  }

  // std::size_t lhs_idx = _lhs_idx;
  std::size_t search_rhs_thres = imu_datas_.size() - 1;

  Eigen::Matrix4d transfrom = Eigen::Matrix4d::Identity();

  for (std::size_t i = 0; i < search_rhs_thres; i++)
  {
    auto lhs = imu_datas_[i];
    auto rhs = imu_datas_[i+1];
    if (rhs.imu_data.timestamp < _st_stamp)
    {
      continue;
    }

    if (lhs.imu_data.timestamp > _st_stamp && rhs.imu_data.timestamp < _end_stamp)
    {
      transfrom *= getPose(lhs, rhs);
    }
    else if (lhs.imu_data.timestamp < _st_stamp && rhs.imu_data.timestamp >= _st_stamp)
    {
      transfrom *= getInterpPose(_st_stamp, lhs, rhs, true);
    }
    else if (lhs.imu_data.timestamp < _end_stamp && rhs.imu_data.timestamp >= _end_stamp)
    {
      transfrom *= getInterpPose(_end_stamp, lhs, rhs);
      _lhs_idx = i + 1;
      break;
    }
    else if (lhs.imu_data.timestamp > _end_stamp && rhs.imu_data.timestamp > _end_stamp)
    {
      _lhs_idx = i;
      break;
    }
  }

  _intergration_transform = transfrom;

  return true;
}

Eigen::Matrix4d ImuProcess::getPose(const InteImuData& _lhs, const InteImuData& _rhs)
{
  Eigen::Matrix4d delta_trans = Eigen::Matrix4d::Identity();
  ImuData avg_imu_data = (_lhs.imu_data + _rhs.imu_data) / 2;
  Eigen::Vector3d mid_rotvec;

  {
    std::lock_guard<std::mutex> lg_shift(drift_mtx_);
    mid_rotvec = Eigen::Vector3d (
      avg_imu_data.angular_velocity_x - drift_.angular_velocity_x,
      avg_imu_data.angular_velocity_y - drift_.angular_velocity_y,
      avg_imu_data.angular_velocity_z - drift_.angular_velocity_z
    );
  }

  double dt = _rhs.imu_data.timestamp - _lhs.imu_data.timestamp;
  // if (dt < 0.004 || dt > 0.006)
  // {
  //   RS_WARNING << "Time step of IMU warning: " << std::to_string(dt) << std::endl;
  // }
  mid_rotvec *= dt;
  Eigen::AngleAxisd rotation_vector(mid_rotvec.norm(), mid_rotvec.normalized());

  delta_trans.block<3, 3>(0, 0) = rotation_vector.toRotationMatrix();

  if (use_linear_acceleration_)
  {
    // translation
    const Eigen::Vector3d ag = Eigen::Vector3d(avg_imu_data.linear_acceleration_x, avg_imu_data.linear_acceleration_y, avg_imu_data.linear_acceleration_z);
    const Eigen::Vector3d a = ag - _lhs.R.transpose() * gravity_vector_;
    const Eigen::Vector3d v1 = _lhs.v + dt * a;
    Eigen::Vector3d translation = dt * (_lhs.v + v1) / 2.0;

#ifdef VIZ_DEBUG
    const Eigen::Vector3d gravity_local_vector = _lhs.R.transpose() * gravity_vector_;
    RINFO << "acceleration: " << a[0] << ", " << a[1] << ", " << a[2] << std::endl;
    RINFO << "gravity_local_vector: " << gravity_local_vector[0] << ", " << gravity_local_vector[1] << ", " << gravity_local_vector[2] << std::endl;
    RINFO << "v0: " << _lhs.v[0] << ", " << _lhs.v[1] << ", " << _lhs.v[2] << std::endl;
    RINFO << "v1: " << v1[0] << ", " << v1[1] << ", " << v1[2] << std::endl;
    RINFO << "translation: " << translation[0] << ", " << translation[1] << ", " << translation[2] << std::endl;
#endif
    delta_trans.block<3, 1>(0, 3) = translation;
  }
  return delta_trans;
}

Eigen::Matrix4d ImuProcess::getInterpPose(double _stamp, const InteImuData& _lhs, const InteImuData& _rhs, bool reverse)
{
  Eigen::Matrix4d hole_trans = getPose(_lhs, _rhs);
  Eigen::Matrix3d hole_rotation_matrix = hole_trans.block<3,3>(0,0);

  Eigen::Quaterniond q1(Eigen::Matrix3d::Identity());
  Eigen::Quaterniond q2(hole_rotation_matrix);
  double scale = abs(_stamp - _lhs.imu_data.timestamp) / abs(_rhs.imu_data.timestamp - _lhs.imu_data.timestamp);
  Eigen::Quaterniond q_interp = q1.slerp(scale, q2);
  Eigen::Matrix4d trans_interp = Eigen::Matrix4d::Identity();
  trans_interp.block<3, 3>(0, 0) = q_interp.toRotationMatrix();

  if (use_linear_acceleration_)
  {
    Eigen::Vector3d interp_translation = hole_trans.block<3, 1>(0,3);
    trans_interp.block<3, 1>(0,3) = scale * interp_translation;
  }

  if (reverse)
  {
    return trans_interp.inverse() * hole_trans;
  }
  return trans_interp;
}

bool ImuProcess::calHeadImuRot(double _cur_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx, const bool _tail)
{
  if (!is_drifted_)
  {
    return false;
  }

  InteImuData lhs, rhs;
  if (!findFrames(_cur_stamp, lhs, rhs, _search_idx))
  {
    // RWARN << "Montion Correct Warning, calHeadIMURot find Frames Failed";;
    return false;
  }

  Eigen::Matrix4d head_trans, tail_trans;
  if (_tail)
  {
    // mid trans
    tail_trans = tmp_pose_[-1];
    if (tmp_pose_.find(static_cast<int>(_search_idx)) != tmp_pose_.end())
    {
      tail_trans = tmp_pose_[static_cast<int>(_search_idx)];
    }
    else
    {
      std::size_t st_loop = head_rhs_idx_-1, end_loop = _search_idx;
      for (std::size_t idx = st_loop; idx > end_loop && idx > 0; --idx)
      {
        tail_trans = getPose(imu_window_.at(idx-1), imu_window_.at(idx)) * tail_trans;
      }
      tmp_pose_[static_cast<int>(_search_idx)] = tail_trans;
    }
    head_trans = getInterpPose(_cur_stamp, lhs, rhs, true);
  }
  else
  {
    // mid trans
    head_trans = tmp_pose_[-1];
    if (tmp_pose_.find(static_cast<int>(_search_idx-1)) != tmp_pose_.end())
    {
      head_trans = tmp_pose_[static_cast<int>(_search_idx-1)];
    }
    else
    {
      std::size_t st_loop = head_rhs_idx_, end_loop = _search_idx > 1 ? _search_idx : 1;

      for (std::size_t idx = st_loop; idx < (end_loop-1) && idx < imu_window_.size(); idx++)
      {
        head_trans *= getPose(imu_window_.at(idx), imu_window_.at(idx+1));
      }
      tmp_pose_[static_cast<int>(_search_idx-1)] = head_trans;
    }

    /* quaternons spherical interpolation for the tail transform */
    tail_trans = getInterpPose(_cur_stamp, lhs, rhs, false);
  }

#ifdef VIZ_DEBUG
  Eigen::Vector3d ypr_head = eulerAnglesZYX(Eigen::Quaterniond(head_trans.block<3,3>(0,0)));
  Eigen::Vector3d ypr_tail = eulerAnglesZYX(Eigen::Quaterniond(tail_trans.block<3,3>(0,0)));

  std::cout << "ypr_head: " << ypr_head[2]  * (180.0 / M_PI) << ", " << ypr_head[1] * (180.0 / M_PI) << ", " << ypr_head[0]  * (180.0 / M_PI)<< std::endl;
  std::cout << "ypr_tail: " << ypr_tail[2]  * (180.0 / M_PI) << ", " << ypr_tail[1] * (180.0 / M_PI) << ", " << ypr_tail[0]  * (180.0 / M_PI)<< std::endl;
#endif

  _intergration_transform = head_trans * tail_trans;
  return true;
}

bool ImuProcess::setHeadStamp(double _stamp, MCStatus& _status, const double _tail)
{
  if (!is_drifted_)
  {
    _status = MCStatus::WAITING;
    return false;
  }

  head_stamp_ = _stamp;
  tmp_pose_.clear();
  imu_window_.clear();

  InteImuData lhs, rhs;
  std::size_t rhs_idx = 1;

  double num_imu_per_lidar_fram = imu_frame_rate_ / lidar_frame_rate_;
  std::size_t min_num_window = static_cast<std::size_t>(std::ceil(1.2 * num_imu_per_lidar_fram));
  std::size_t left_thres = static_cast<std::size_t>(std::ceil(0.5 * num_imu_per_lidar_fram));
  std::size_t right_thres = static_cast<std::size_t>(std::ceil(1.5 * num_imu_per_lidar_fram));

  if (_tail)
  {
    double tmp = left_thres;
    left_thres = right_thres;
    right_thres = tmp;
  }
  {
    std::lock_guard<std::mutex> lg(mtx_);
    bool find_hs = false;

    for (auto it = imu_datas_.begin() + 1; it != imu_datas_.end(); ++it)
    {
      lhs = *(it - 1);
      rhs = *it;
      // RDEBUG << std::to_string(lhs.imu_data.timestamp) << " " << std::to_string(_stamp) << " " << std::to_string(rhs.imu_data.timestamp) << " " << std::endl;
      if (_stamp > lhs.imu_data.timestamp && _stamp <= rhs.imu_data.timestamp)
      {
        find_hs = true;
        // RDEBUG << "find hs: " << std::to_string(lhs.imu_data.timestamp) << " "  << std::to_string(_stamp) << " " << std::to_string(rhs.imu_data.timestamp) << std::endl;
        break;
      }
      rhs_idx++;
    }
    if (!find_hs)
    {
      // RWARN << "Montion Correct Warning, setHeadStamp Find Frames Failed, Skip Frame!"
      //  << " list of imu stamp is " << std::to_string(imu_datas_.front().imu_data.timestamp) << " " << std::to_string(imu_datas_.back().imu_data.timestamp);
      _status = MCStatus::SKIPT;
      return false;
    }

    auto start_it = (rhs_idx >= left_thres) ? (imu_datas_.begin() + rhs_idx - left_thres) : imu_datas_.begin();
    std::size_t left_boundry = (rhs_idx >= left_thres) ? rhs_idx - left_thres : 0;

    auto end_it = (rhs_idx + right_thres < imu_datas_.size()) ? (imu_datas_.begin() + rhs_idx + right_thres) : imu_datas_.end();
    std::size_t right_boundry = (rhs_idx + right_thres < imu_datas_.size()) ? rhs_idx + right_thres : imu_datas_.size();

    auto it = start_it;
    for (std::size_t idx = left_boundry; idx < right_boundry && it < end_it; ++it, ++idx)
    {
      imu_window_.emplace_back(*it);
      if (it->imu_data.timestamp == rhs.imu_data.timestamp)
      {
        head_rhs_idx_ = imu_window_.size() - 1;
      }
    }

    if (imu_window_.size() < min_num_window || imu_window_.front().imu_data.timestamp > _stamp || imu_window_.back().imu_data.timestamp < _stamp)
    {
      std::ostringstream oss;
      if (!imu_window_.empty())
      {
        oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, SetHeadStamp Failed find Frames!"
            << " window size: " << std::to_string(imu_window_.size())
            << " rhs_idx: " << rhs_idx << " " << left_boundry << " " << right_boundry
            << " head stamp:  " << std::to_string(imu_window_.front().imu_data.timestamp)
            << " stamp: " << std::to_string(_stamp)
            << " tail stamp:  " << std::to_string(imu_window_.back().imu_data.timestamp)
            << " data_frames: " << std::to_string(imu_datas_.size()) << " " << imu_datas_.back().imu_data.timestamp;
      }
      else
      {
        oss << std::fixed << std::setprecision(9) << "Montion Correct Warning, SetHeadStamp Failed find Frames!"
            << " window size: " << std::to_string(imu_window_.size())
            << " rhs_idx: " << rhs_idx << " " << left_boundry << " " << right_boundry
            << " stamp: " << std::to_string(_stamp)
            << " data_frames: " << std::to_string(imu_datas_.size()) << " " << std::to_string(imu_datas_.front().imu_data.timestamp) << " " << std::to_string(imu_datas_.back().imu_data.timestamp);
      }
      // RWARN << oss.str();
      _status = MCStatus::WAITING;
      return false;
    }
  }

  // 移除数据， 避免数据积累
  std::size_t left_boundry_idx = std::max(static_cast<int>(rhs_idx) - static_cast<int>(left_thres) - 1, 0);
  clearData(left_boundry_idx);

  if (_tail)
  {
    Eigen::Matrix4d tail_trans = getInterpPose(_stamp, lhs, rhs, false);
    tmp_pose_[-1] = tail_trans;
    Eigen::Matrix4d mid_trans = tail_trans;
    for (std::size_t rhs_idx = head_rhs_idx_ - 1; rhs_idx > 1 && rhs_idx < imu_window_.size(); --rhs_idx)
    {
      Eigen::Matrix4d delta = getPose(imu_window_[rhs_idx-1], imu_window_[rhs_idx]);
      mid_trans = delta * mid_trans;
      tmp_pose_[rhs_idx-1] = mid_trans; // deep copy
    }
  }
  else
  {
    Eigen::Matrix4d head_trans = getInterpPose(_stamp, lhs, rhs, true);
    tmp_pose_[-1] = head_trans;
    Eigen::Matrix4d mid_trans = head_trans;
    for (std::size_t rhs_idx = head_rhs_idx_ + 1; rhs_idx > 1 && rhs_idx < imu_window_.size(); ++rhs_idx)
    {
      Eigen::Matrix4d delta = getPose(imu_window_[rhs_idx-1], imu_window_[rhs_idx]);
      mid_trans *= delta;
      tmp_pose_[rhs_idx] = mid_trans; // deep copy
    }
  }
  // RDEBUG << "setHeadStamp success, size:  " << imu_window_.size() << " "
  //          << std::to_string(imu_window_.front().imu_data.timestamp) << " "
  //          << std::to_string(imu_window_.back().imu_data.timestamp) << " " << std::endl;
  _status = MCStatus::SUCCESS;
  return true;
}

bool ImuProcess::clearData(const std::size_t left_boundry_idx)
{
  std::lock_guard<std::mutex> lg(mtx_);
  std::size_t idx = 0;
  while (!imu_datas_.empty() && idx++ < left_boundry_idx)
  {
    imu_datas_.pop_front();
  }
  return true;
}

// ImuData FromMsg(const std::shared_ptr<robosense::common::MotionFrame>& imu_msg) {
//   ImuData data;

//   data.timestamp = imu_msg->capture_time.tv_sec + imu_msg->capture_time.tv_usec / 1000000.0;

//   // 从消息中提取角速度
//   data.angular_velocity_x = imu_msg->gyro.x;
//   data.angular_velocity_y = imu_msg->gyro.y;
//   data.angular_velocity_z = imu_msg->gyro.z;

//   // 从消息中提取线性加速度
//   data.linear_acceleration_x = imu_msg->accel.x;
//   data.linear_acceleration_y = imu_msg->accel.y;
//   data.linear_acceleration_z = imu_msg->accel.z;

//   return data;
// }

// ImuData FromMsg(const common::Odom::Ptr& imu_msg)
// {
//   ImuData data;

//   data.timestamp = imu_msg->header.timestamp / 1e9;

//   // 从消息中提取角速度
//   data.angular_velocity_x = imu_msg->odom_data.angle_vel_roll;
//   data.angular_velocity_y = imu_msg->odom_data.angle_vel_pitch;
//   data.angular_velocity_z = imu_msg->odom_data.angle_vel_yaw;

//   // 从消息中提取线性加速度
//   data.linear_acceleration_x = imu_msg->odom_data.acceleration_x;
//   data.linear_acceleration_y = imu_msg->odom_data.acceleration_y;
//   data.linear_acceleration_z = imu_msg->odom_data.acceleration_z;

//   return data;
// }

ImuData FromMsg(const sensor_msgs::msg::Imu::SharedPtr& imu_msg_ptr) {
  ImuData data;

  // 从 ROS 消息中提取时间戳
  data.timestamp = rclcpp::Time(imu_msg_ptr->header.stamp).seconds();

  // 从 ROS 消息中提取角速度
  data.angular_velocity_x = imu_msg_ptr->angular_velocity.x;
  data.angular_velocity_y = imu_msg_ptr->angular_velocity.y;
  data.angular_velocity_z = imu_msg_ptr->angular_velocity.z;

  // 从 ROS 消息中提取线性加速度
  data.linear_acceleration_x = imu_msg_ptr->linear_acceleration.x;
  data.linear_acceleration_y = imu_msg_ptr->linear_acceleration.y;
  data.linear_acceleration_z = imu_msg_ptr->linear_acceleration.z;

  return data;
}
}  // namespace postprocess
}  // namespace robosense