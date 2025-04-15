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
#include "postprocess/config.hpp"
#include <unistd.h>  // Linux/macOS
#ifdef _WIN32
#include <direct.h> // Windows（需包含此头文件）
#endif
namespace robosense {
namespace postprocess {

NodeConfig ParseNodeConfig(const YAML::Node& cfg) {
  char buffer[256];
  if (getcwd(buffer, sizeof(buffer)) != nullptr) {
      std::cout << "current work dir is: " << buffer << std::endl;
  }
  NodeConfig res_cfg;
  std::string calib_file = cfg["calib_file"].as<std::string>();
#if defined(USE_ROS1)
    calib_file = std::string(PROJECT_PATH) + "/" + calib_file;
#elif defined(USE_ROS2)
    calib_file = std::string(buffer) + "/" + calib_file;
#endif
  std::cout << "calib_file: " << calib_file << std::endl;
  YAML::Node calib_node = YAML::LoadFile(calib_file);
  const YAML::Node& lidar_cfg = calib_node["Sensor"]["Lidar"];
  const YAML::Node& lidar2imu_cfg = lidar_cfg["extrinsic"];

  res_cfg.lidar_config.T_Lidar2IMU.qw = lidar2imu_cfg["quaternion"]["w"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.qx = lidar2imu_cfg["quaternion"]["x"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.qy = lidar2imu_cfg["quaternion"]["y"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.qz = lidar2imu_cfg["quaternion"]["z"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.x = lidar2imu_cfg["translation"]["x"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.y = lidar2imu_cfg["translation"]["y"].as<float>();
  res_cfg.lidar_config.T_Lidar2IMU.z = lidar2imu_cfg["translation"]["z"].as<float>();
  // std::cout << "lidar2imu_cfg:\n" <<lidar2imu_cfg;
  // camera
  const YAML::Node& camera_cfg = calib_node["Sensor"]["Camera"];
  const YAML::Node& cam2imu_cfg = camera_cfg["extrinsic"];
  TransformXYZQuat cam2imu;
  cam2imu.qx = cam2imu_cfg["quaternion"]["x"].as<float>();
  cam2imu.qy = cam2imu_cfg["quaternion"]["y"].as<float>();
  cam2imu.qz = cam2imu_cfg["quaternion"]["z"].as<float>();
  cam2imu.qw = cam2imu_cfg["quaternion"]["w"].as<float>();
  cam2imu.x = cam2imu_cfg["translation"]["x"].as<float>();
  cam2imu.y = cam2imu_cfg["translation"]["y"].as<float>();
  cam2imu.z = cam2imu_cfg["translation"]["z"].as<float>();
  // std::cout << "cam2imu_cfg:\n" << cam2imu_cfg;

  Eigen::Matrix4d T_liadr2imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_cam2imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d T_lidar2cam = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q_lidar2imu(res_cfg.lidar_config.T_Lidar2IMU.qw,
                                res_cfg.lidar_config.T_Lidar2IMU.qx,
                                res_cfg.lidar_config.T_Lidar2IMU.qy,
                                res_cfg.lidar_config.T_Lidar2IMU.qz);
  T_liadr2imu.block<3, 3>(0, 0) = q_lidar2imu.toRotationMatrix();
  T_liadr2imu.block<3, 1>(0, 3) = Eigen::Vector3d(
                                                res_cfg.lidar_config.T_Lidar2IMU.x,
                                                res_cfg.lidar_config.T_Lidar2IMU.y,
                                                res_cfg.lidar_config.T_Lidar2IMU.z);
  Eigen::Quaterniond q_cam2imu(cam2imu.qw, cam2imu.qx, cam2imu.qy, cam2imu.qz);
  T_cam2imu.block<3, 3>(0, 0) = q_cam2imu.toRotationMatrix();
  T_cam2imu.block<3, 1>(0, 3) = Eigen::Vector3d(cam2imu.x, cam2imu.y, cam2imu.z);
  T_lidar2cam = T_cam2imu.inverse() * T_liadr2imu;
  Eigen::Quaterniond quant(T_lidar2cam.block<3, 3>(0, 0));
  Eigen::Vector3d trans(T_lidar2cam.block<3, 1>(0, 3));
  res_cfg.lidar_config.T_Lidar2Cam.qw = quant.w();
  res_cfg.lidar_config.T_Lidar2Cam.qx = quant.x();
  res_cfg.lidar_config.T_Lidar2Cam.qy = quant.y();
  res_cfg.lidar_config.T_Lidar2Cam.qz = quant.z();
  res_cfg.lidar_config.T_Lidar2Cam.x  = trans.x();
  res_cfg.lidar_config.T_Lidar2Cam.y  = trans.y();
  res_cfg.lidar_config.T_Lidar2Cam.z  = trans.z();

  {
    std::vector<double> distortion_coeffs;
    for (size_t i = 0; i < camera_cfg["intrinsic"]["dist_coeff"].size(); ++i) {
      distortion_coeffs.push_back(camera_cfg["intrinsic"]["dist_coeff"][i].as<double>());
    }
    res_cfg.camera_config.intrinsics.distortion_coeffs = distortion_coeffs;

    std::vector<double> intrinsic_matrix;
    if (camera_cfg["intrinsic"] && camera_cfg["intrinsic"]["int_matrix"])
    {
      for (size_t i = 0; i < camera_cfg["intrinsic"]["int_matrix"].size(); ++i)
      {
        intrinsic_matrix.push_back(camera_cfg["intrinsic"]["int_matrix"][i].as<double>());
      }
    }
    res_cfg.camera_config.intrinsics.camera_matrix = intrinsic_matrix;
  }
  std::cout << "camera_config.intrinsics.distortion_coeffs:\n";
  for (auto& i : res_cfg.camera_config.intrinsics.distortion_coeffs) {
    std::cout << i << " ";
  }
  std::cout << "\ncamera_config.intrinsics.camera_matrix:\n";
  for (auto& i : res_cfg.camera_config.intrinsics.camera_matrix) {
    std::cout << i << " ";
  }
  std::cout << std::endl;
  {
    yamlRead<bool>(cfg, "MOTION_CORRECT",                res_cfg.motion_config.motion_correct,                 true);
    yamlRead<bool>(cfg, "USING_IMU_LINEAR_ACCELERATION", res_cfg.motion_config.using_imu_linear_acceleration, false);
    yamlRead<bool>(cfg, "USING_ODOM_LINEAR_VELOCITY",    res_cfg.motion_config.using_odom_linear_velocity,    false);
    yamlRead<bool>(cfg, "FRAME_TAIL",                    res_cfg.motion_config.frame_tail,                    false);
    yamlRead<bool>(cfg, "USE_RANGE_IMG",                 res_cfg.motion_config.use_range_img,                 false);
    yamlRead<double>(cfg, "CAMERA_LIDAR_TIME_DURA_THRES",                     res_cfg.motion_config.thres,     0.07);
    yamlRead<std::size_t>(cfg, "NUM_DRIFT",              res_cfg.motion_config.num_drift,                        50);

    yamlRead<std::string>(cfg, "PROJECTION_ROOT",        res_cfg.motion_config.projection_root,               "");
    yamlRead<std::string>(cfg, "sub_image_topic",       res_cfg.motion_config.sub_image_topic,              "/rs_camera/rgb");
    yamlRead<std::string>(cfg, "sub_lidar_topic",       res_cfg.motion_config.sub_lidar_topic,              "/rs_lidar/points");
    yamlRead<std::string>(cfg, "sub_imu_topic",       res_cfg.motion_config.sub_imu_topic,              "/rs_imu");

    const YAML::Node& range_image_cfg = cfg["RANGE_IMAGE"];
    yamlRead<double>(range_image_cfg, "resolution", res_cfg.lidar_config.range_image_info.res, 0.625);
    yamlRead<double>(range_image_cfg, "pitch_min", res_cfg.lidar_config.range_image_info.pitch_min, -60.0);
    yamlRead<double>(range_image_cfg, "pitch_max", res_cfg.lidar_config.range_image_info.pitch_max, 60.0);
    yamlRead<double>(range_image_cfg, "yaw_min", res_cfg.lidar_config.range_image_info.yaw_min, -30.0);
    yamlRead<double>(range_image_cfg, "yaw_max", res_cfg.lidar_config.range_image_info.yaw_max, 30.0);

    yamlRead<std::string>(cfg, "ORI_POINTS_TOPIC",       res_cfg.motion_config.ori_points_topic,              "/rslidar_points_origin");
    yamlRead<std::string>(cfg, "CM_POINTS_TOPIC",        res_cfg.motion_config.motion_points_topic,           "/rslidar_points_motion");
    yamlRead<std::string>(cfg, "CM_RGB_POINTS_TOPIC",    res_cfg.motion_config.motion_rgb_points_topic,       "/rslidar_points_motion_rgb");

    yamlRead<std::string>(cfg, "ORI_DEOCC_RGB_POINTS_TOPIC", res_cfg.motion_config.ori_deocc_rgb_points_topic, "/rslidar_points_origin_deocc_rgb");
    yamlRead<std::string>(cfg, "ORI_IMG_TOPIC",          res_cfg.motion_config.ori_img_topic,                 "/camera/image_raw/proj_ori");
    yamlRead<std::string>(cfg, "PROJ_IMG_TOPIC",         res_cfg.motion_config.motion_proj_img_topic,         "/camera/image_raw/proj_motion");
    // yamlRead<std::string>(motion_cfg, "PROJ_DEOCC_IMG_TOPIC",   res_cfg.motion_config.motion_deocc_proj_img_topic,   "/camera/image_raw/proj_motion_deocc");
    yamlRead<std::string>(cfg, "RANGE_IMG_TOPIC",        res_cfg.motion_config.motion_range_img_topic,        "/camera/image_raw/range_motion");
    yamlRead<std::string>(cfg, "CAM_RANGE_IMG_TOPIC",    res_cfg.motion_config.motion_cam_range_img_topic,    "/camera/image_raw/cam_range_motion");
  }
  return res_cfg;
}

}  // namespace postprocess
}  // namespace robosense