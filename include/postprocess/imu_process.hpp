/******************************************************************************
 * Copyright 2024 RoboSense All rights reserved.
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
#ifndef POSTPROCESS_Imu_PROCESS_H_
#define POSTPROCESS_Imu_PROCESS_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>
#include <deque>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <memory>
#include <iostream>

#include "sensor_msgs/msg/imu.hpp"
#include "postprocess/log.hpp"

namespace robosense {
namespace postprocess {

enum class MCStatus {
  SUCCESS = 0,
  SKIPT = 1,
  WAITING = 2
};

/**
 * @brief calclute rotation from vector_a and vector_b.
 *
 * @param _va vector3 a.
 * @param _vb vector3 b.
 * @return R_b_a from _va coordinate to _vb coordinate.
 */
template <typename T>
inline Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 3> calRotationFromTwoVector(const T& _va, const T& _vb) {
  typedef typename std::remove_reference<T>::type::Scalar Scalar;

  Eigen::Vector3d van = _va.normalized();
  Eigen::Vector3d vbn = _vb.normalized();

  Scalar theta_rad = std::acos(van.dot(vbn));
  Eigen::Matrix<Scalar, 3, 1> rotation_axis = van.cross(vbn).normalized();
  Eigen::AngleAxisd angle_axis(theta_rad, rotation_axis);
  Eigen::Matrix<Scalar, 3, 3> rotation_matrix = angle_axis.toRotationMatrix();

  return rotation_matrix;
}

template <typename Scalar>
inline Eigen::Matrix<Scalar, 4, 4> eulerXYZToTransMatrix(const Scalar& x, const Scalar& y, const Scalar& z, const Scalar& roll, const Scalar& pitch, const Scalar& yaw) {
  Eigen::Transform<Scalar, 3, Eigen::Affine> transform = Eigen::Translation<Scalar, 3>(x, y, z) *
                                                         Eigen::AngleAxis<Scalar>(yaw, Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
                                                         Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
                                                         Eigen::AngleAxis<Scalar>(roll, Eigen::Matrix<Scalar, 3, 1>::UnitX());
  return transform.matrix();
}

template <typename T>
inline Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 1> eulerAnglesZYX(const T& q_in) {
  typedef typename std::remove_reference<T>::type::Scalar Scalar;

  Eigen::Matrix<Scalar, 4, 1> q = q_in.normalized().coeffs();

  Scalar s = -2 * (q(0) * q(2) - q(3) * q(1));
  if (s > 1)
    s = 1;
  return (Eigen::Matrix<Scalar, 3, 1>()
              << atan2f(2 * (q(0) * q(1) + q(3) * q(2)), q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2)),
          asin(s), atan2(2 * (q(1) * q(2) + q(3) * q(0)), q(3) * q(3) - q(0) * q(0) - q(1) * q(1) + q(2) * q(2)))
      .finished();
}

struct ImuData {
  bool state;
  double timestamp;  // Time in seconds
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;
  ImuData()
      : state{false},
        timestamp(0),
        orientation_x(0.0),
        orientation_y(0.0),
        orientation_z(0.0),
        orientation_w(1.0),
        angular_velocity_x(0.0),
        angular_velocity_y(0.0),
        angular_velocity_z(0.0),
        linear_acceleration_x(0.0),
        linear_acceleration_y(0.0),
        linear_acceleration_z(0.0) {}

  // Parameterized constructor to initialize all members
  ImuData(bool valid, double ts, float ori_x, float ori_y, float ori_z, float ori_w,
          float ang_vel_x, float ang_vel_y, float ang_vel_z,
          float lin_acc_x, float lin_acc_y, float lin_acc_z)
      : state{valid},
        timestamp(ts),
        orientation_x(ori_x),
        orientation_y(ori_y),
        orientation_z(ori_z),
        orientation_w(ori_w),
        angular_velocity_x(ang_vel_x),
        angular_velocity_y(ang_vel_y),
        angular_velocity_z(ang_vel_z),
        linear_acceleration_x(lin_acc_x),
        linear_acceleration_y(lin_acc_y),
        linear_acceleration_z(lin_acc_z) {}
  ImuData(const ImuData& other) {
    if (this != &other) {
      state = other.state;
      timestamp = other.timestamp;
      orientation_x = other.orientation_x;
      orientation_y = other.orientation_y;
      orientation_z = other.orientation_z;
      orientation_w = other.orientation_w;
      angular_velocity_x = other.angular_velocity_x;
      angular_velocity_y = other.angular_velocity_y;
      angular_velocity_z = other.angular_velocity_z;
      linear_acceleration_x = other.linear_acceleration_x;
      linear_acceleration_y = other.linear_acceleration_y;
      linear_acceleration_z = other.linear_acceleration_z;
    }
  }
  ImuData& operator=(const ImuData& other) {
    if (this != &other) {
      state = other.state;
      timestamp = other.timestamp;
      orientation_x = other.orientation_x;
      orientation_y = other.orientation_y;
      orientation_z = other.orientation_z;
      orientation_w = other.orientation_w;
      angular_velocity_x = other.angular_velocity_x;
      angular_velocity_y = other.angular_velocity_y;
      angular_velocity_z = other.angular_velocity_z;
      linear_acceleration_x = other.linear_acceleration_x;
      linear_acceleration_y = other.linear_acceleration_y;
      linear_acceleration_z = other.linear_acceleration_z;
    }
    return *this;
  }

  ImuData operator+(const ImuData& other) const {
      ImuData result;
      result.timestamp = std::max(timestamp, other.timestamp);
      result.angular_velocity_x = angular_velocity_x + other.angular_velocity_x;
      result.angular_velocity_y = angular_velocity_y + other.angular_velocity_y;
      result.angular_velocity_z = angular_velocity_z + other.angular_velocity_z;
      result.linear_acceleration_x = linear_acceleration_x + other.linear_acceleration_x;
      result.linear_acceleration_y = linear_acceleration_y + other.linear_acceleration_y;
      result.linear_acceleration_z = linear_acceleration_z + other.linear_acceleration_z;
      return result;
  }

  ImuData operator/(float scalar) const {
      ImuData result;
      result.angular_velocity_x = angular_velocity_x / scalar;
      result.angular_velocity_y = angular_velocity_y / scalar;
      result.angular_velocity_z = angular_velocity_z / scalar;
      result.linear_acceleration_x = linear_acceleration_x / scalar;
      result.linear_acceleration_y = linear_acceleration_y / scalar;
      result.linear_acceleration_z = linear_acceleration_z / scalar;
      return result;
  }
};

struct InteImuData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuData imu_data;
  Eigen::Vector3d p;  // position
  Eigen::Vector3d v;  // v_local
  Eigen::Matrix3d R;  // R_global_local

  InteImuData()
  {
    imu_data = ImuData();
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    R = Eigen::Matrix3d::Identity();
  }

  InteImuData(const ImuData& _data)
    : imu_data(_data)
  {
    p = Eigen::Vector3d::Zero();
    v = Eigen::Vector3d::Zero();
    R = Eigen::Matrix3d::Identity();
  }

  InteImuData(const ImuData& _data, const Eigen::Vector3d& _v, const Eigen::Matrix3d& _R)
    : imu_data(_data),
      v(_v),
      R(_R)
  {
    p = Eigen::Vector3d::Zero();
  }

  InteImuData(const ImuData& _data, const Eigen::Vector3d& _p, const Eigen::Vector3d& _v, const Eigen::Matrix3d& _R)
    : imu_data(_data),
      p(_p),
      v(_v),
      R(_R)
  {  }

  // copy
  InteImuData(const InteImuData& other)
    : imu_data(other.imu_data),
      p(other.p),
      v(other.v),
      R(other.R)
  {  }

  // =
  InteImuData& operator=(const InteImuData& other) {
    if (this != &other) {
      imu_data = other.imu_data;
      p = other.p;
      v = other.v;
      R = other.R;
    }
    return *this;
  }
};

class ImuProcess {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuProcess(std::size_t _num);
  ImuProcess(std::size_t _num, bool _use_linear_acceleration, double _gravity);
  ImuProcess(std::size_t _num, bool _use_linear_acceleration, double _gravity, double _Imu_frame_rate, double _lidar_frame_rate);

  bool addImuData(const ImuData& _data);
  bool calImuRot(double _st_stamp, double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx);
  bool calHeadImuRot(
    double _end_stamp, Eigen::Matrix4d& _intergration_transform, std::size_t& _search_idx, const bool _tail = false);
  bool findFrames(double _stamp, InteImuData& _lhs, InteImuData& _rhs, std::size_t& _search_idx);
  std::vector<std::size_t> getCurrentWindowCopy();

  bool setHeadStamp(double _stamp, MCStatus& _status, const double _tail = false);
  bool clearData(const std::size_t _left_boundry_idx);
  bool isDrifted();
  ImuData& back();
  std::size_t size();
  void pop_front();
  void pop_back();

  void print();
  std::vector<InteImuData> imu_window_;
private:
  Eigen::Matrix4d getPose(const InteImuData& _lhs, const InteImuData& _rhs);
  Eigen::Matrix4d getInterpPose(double _stamp, const InteImuData& _lhs, const InteImuData& _rhs, bool reverse=false);

  std::deque<InteImuData> imu_datas_;

  Eigen::Vector3d gravity_vector_;  // global

  std::size_t num_window_ = 50;

  std::mutex mtx_;
  std::mutex drift_mtx_;
  ImuData drift_;
  bool is_drifted_ = false;
  bool use_linear_acceleration_ = false;

  double mointion_ratio_ = 0.5;
  double mointion_thres_ = 0.002;

  double static_ratio_ = 0.6;
  double static_thres_ = 0.001;

  double head_stamp_;
  std::size_t head_rhs_idx_ = 0.0;

  // for angle correction
  std::unordered_map<int, Eigen::Matrix4d> tmp_pose_;

  double imu_frame_rate_;
  double lidar_frame_rate_;

};

// ImuData FromMsg(const common::Odom::Ptr& imu_msg);

// ImuData FromMsg(const std::shared_ptr<robosense::common::MotionFrame>& imu_msg);
ImuData FromMsg(const sensor_msgs::msg::Imu::SharedPtr& imu_msg_ptr);

}
}
#endif  // HYPER_VISION_POSTPROCESS_Imu_MODULE_H_
