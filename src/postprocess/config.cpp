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


void loadOfflineCaliInfo(const std::string &calib_file, NodeConfig &res_cfg, const bool use_stereo)
{
  YAML::Node calib_node = YAML::LoadFile(calib_file);

  Eigen::Matrix4d tf_Lidar2Cam;
  Eigen::Matrix4d tf_Lidar2IMU;
  Eigen::Matrix4d tf_IMU2Lidar;
  Eigen::Matrix4d tf_Cam2Lidar;

  const YAML::Node& imu_cfg = calib_node["Sensor"]["IMU"];
  const YAML::Node& imu2lidar_cfg = imu_cfg["extrinsic"];
  const YAML::Node& cam_cfg = calib_node["Sensor"]["Camera"];
  const YAML::Node& cam2lidar_cfg = cam_cfg["extrinsic"];

  res_cfg.lidar_config.T_IMU2Lidar.qw = imu2lidar_cfg["quaternion"]["w"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.qx = imu2lidar_cfg["quaternion"]["x"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.qy = imu2lidar_cfg["quaternion"]["y"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.qz = imu2lidar_cfg["quaternion"]["z"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.x = imu2lidar_cfg["translation"]["x"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.y = imu2lidar_cfg["translation"]["y"].as<float>();
  res_cfg.lidar_config.T_IMU2Lidar.z = imu2lidar_cfg["translation"]["z"].as<float>();

  res_cfg.lidar_config.T_Cam2Lidar.qw = cam2lidar_cfg["quaternion"]["w"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.qx = cam2lidar_cfg["quaternion"]["x"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.qy = cam2lidar_cfg["quaternion"]["y"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.qz = cam2lidar_cfg["quaternion"]["z"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.x = cam2lidar_cfg["translation"]["x"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.y = cam2lidar_cfg["translation"]["y"].as<float>();
  res_cfg.lidar_config.T_Cam2Lidar.z = cam2lidar_cfg["translation"]["z"].as<float>();

  res_cfg.lidar_config.tf_IMU2Lidar = Eigen::Matrix4d::Identity();
  res_cfg.lidar_config.tf_Cam2Lidar = Eigen::Matrix4d::Identity();

  Eigen::Quaterniond q_imu2lidar(res_cfg.lidar_config.T_IMU2Lidar.qw,
                                 res_cfg.lidar_config.T_IMU2Lidar.qx,
                                 res_cfg.lidar_config.T_IMU2Lidar.qy,
                                 res_cfg.lidar_config.T_IMU2Lidar.qz);
  res_cfg.lidar_config.tf_IMU2Lidar.block<3, 3>(0, 0) = q_imu2lidar.toRotationMatrix();
  res_cfg.lidar_config.tf_IMU2Lidar.block<3, 1>(0, 3) = Eigen::Vector3d(res_cfg.lidar_config.T_IMU2Lidar.x,
                                                                        res_cfg.lidar_config.T_IMU2Lidar.y,
                                                                        res_cfg.lidar_config.T_IMU2Lidar.z);
  res_cfg.lidar_config.tf_Lidar2IMU = res_cfg.lidar_config.tf_IMU2Lidar.inverse();

  Eigen::Quaterniond q_cam2lidar(res_cfg.lidar_config.T_Cam2Lidar.qw,
                                 res_cfg.lidar_config.T_Cam2Lidar.qx,
                                 res_cfg.lidar_config.T_Cam2Lidar.qy,
                                 res_cfg.lidar_config.T_Cam2Lidar.qz);
  res_cfg.lidar_config.tf_Cam2Lidar.block<3, 3>(0, 0) = q_cam2lidar.toRotationMatrix();
  res_cfg.lidar_config.tf_Cam2Lidar.block<3, 1>(0, 3) = Eigen::Vector3d(res_cfg.lidar_config.T_Cam2Lidar.x,
                                                                        res_cfg.lidar_config.T_Cam2Lidar.y,
                                                                        res_cfg.lidar_config.T_Cam2Lidar.z);
  res_cfg.lidar_config.tf_Lidar2Cam = res_cfg.lidar_config.tf_Cam2Lidar.inverse();

  loadCamInfo(cam_cfg, res_cfg.camera_config.intrinsics);

  if(!use_stereo)
  {
    return;
  }

  res_cfg.camera_config.left_intrinsics = res_cfg.camera_config.intrinsics;

  const YAML::Node &cam_right_cfg = calib_node["Sensor"]["Camera_R"];

  loadCamInfo(cam_right_cfg, res_cfg.camera_config.right_intrinsics);

  Eigen::Quaterniond q_camR2cam(cam_right_cfg["extrinsic"]["quaternion"]["w"].as<double>(),
                                cam_right_cfg["extrinsic"]["quaternion"]["x"].as<double>(),
                                cam_right_cfg["extrinsic"]["quaternion"]["y"].as<double>(),
                                cam_right_cfg["extrinsic"]["quaternion"]["z"].as<double>());

  res_cfg.camera_config.tf_CamR2Cam.block<3, 3>(0, 0) = q_camR2cam.toRotationMatrix();
  res_cfg.camera_config.tf_CamR2Cam.block<3, 1>(0, 3) = Eigen::Vector3d(
    cam_right_cfg["extrinsic"]["translation"]["x"].as<double>(),
    cam_right_cfg["extrinsic"]["translation"]["y"].as<double>(),
    cam_right_cfg["extrinsic"]["translation"]["z"].as<double>());

  res_cfg.lidar_config.tf_CamR2Lidar = res_cfg.lidar_config.tf_Cam2Lidar * res_cfg.camera_config.tf_CamR2Cam;
  res_cfg.lidar_config.tf_Lidar2CamR = res_cfg.lidar_config.tf_CamR2Lidar.inverse();
}

void loadCamInfo(const YAML::Node& cam_cfg, Intrinsics & intrinsics)
{
    std::vector<double> distortion_coeffs;
    for (size_t i = 0; i < cam_cfg["intrinsic"]["dist_coeff"].size(); ++i) {
      distortion_coeffs.push_back(cam_cfg["intrinsic"]["dist_coeff"][i].as<double>());
    }
    intrinsics.distortion_coeffs = distortion_coeffs;

    std::vector<double> intrinsic_matrix;
    if (cam_cfg["intrinsic"] && cam_cfg["intrinsic"]["int_matrix"])
    {
      for (size_t i = 0; i < cam_cfg["intrinsic"]["int_matrix"].size(); ++i)
      {
        intrinsic_matrix.push_back(cam_cfg["intrinsic"]["int_matrix"][i].as<double>());
      }
    }
    intrinsics.camera_matrix = intrinsic_matrix;

    std::cout << "intrinsics.distortion_coeffs:\n";
    for (auto& i : intrinsics.distortion_coeffs) {
      std::cout << i << " ";
    }
    std::cout << "\nintrinsics.camera_matrix:\n";
    for (auto& i : intrinsics.camera_matrix) {
      std::cout << i << " ";
    }

    std::cout << std::endl;
}

NodeConfig ParseNodeConfig(const YAML::Node& cfg) {
  char buffer[256];
  if (getcwd(buffer, sizeof(buffer)) != nullptr) {
      std::cout << std::endl;
      std::cout << "current work dir is: " << buffer << std::endl;
  }

  std::string calib_file = cfg["calib_file"].as<std::string>();
#if defined(USE_ROS1)
    calib_file = std::string(PROJECT_PATH) + "/" + calib_file;
#elif defined(USE_ROS2)
    std::string share_dir = ament_index_cpp::get_package_share_directory("robosense_ac_postprocess");
    calib_file = share_dir + "/" + calib_file;
#endif
  std::cout << "calib_file: " << calib_file << std::endl;

  NodeConfig res_cfg;

  {
    yamlRead<bool>(cfg, "MOTION_CORRECT",                 res_cfg.motion_config.motion_correct,                 true);
    yamlRead<bool>(cfg, "USING_IMU_LINEAR_ACCELERATION",  res_cfg.motion_config.using_imu_linear_acceleration,  false);
    yamlRead<bool>(cfg, "USING_ODOM_LINEAR_VELOCITY",     res_cfg.motion_config.using_odom_linear_velocity,     false);
    yamlRead<bool>(cfg, "FRAME_TAIL",                     res_cfg.motion_config.frame_tail,                     false);
    yamlRead<bool>(cfg, "USE_RANGE_IMG",                  res_cfg.motion_config.use_range_img,                  false);
    yamlRead<bool>(cfg, "USE_COMPRESSED_IMAGE",           res_cfg.motion_config.use_compressed_image,           false);
    yamlRead<bool>(cfg, "USE_ONLINE_CALIB_TOPIC",         res_cfg.motion_config.use_online_calib_topic,         false);
    yamlRead<bool>(cfg, "USE_SECOND_IMAGE",               res_cfg.motion_config.use_second_image,               false);
    yamlRead<bool>(cfg, "EDGE_FILTER",                    res_cfg.motion_config.use_edge_filter,                false);

    yamlRead<int>(cfg, "ac_mode",                         res_cfg.motion_config.ac_mode,                 1);
    yamlRead<int>(cfg, "DOWNSAMPLE_ROW",                  res_cfg.motion_config.downsample_row,                 6);
    yamlRead<int>(cfg, "DOWNSAMPLE_COL",                  res_cfg.motion_config.downsample_col,                 6);
    yamlRead<double>(cfg, "CAMERA_LIDAR_TIME_DURA_THRES", res_cfg.motion_config.thres,                          0.07);
    yamlRead<double>(cfg, "INFLATE_DEPTH_PARAM",          res_cfg.motion_config.inflate_depth_param,            0.00);
    yamlRead<std::size_t>(cfg, "NUM_DRIFT",               res_cfg.motion_config.num_drift,                        50);

    yamlRead<std::string>(cfg, "PROJECTION_ROOT",        res_cfg.motion_config.projection_root,               "");
    yamlRead<std::string>(cfg, "sub_calib_topic",        res_cfg.motion_config.sub_calib_topic,               "/device_calib_info");
    yamlRead<std::string>(cfg, "sub_image_topic",        res_cfg.motion_config.sub_image_topic,               "/rs_camera/rgb");
    yamlRead<std::string>(cfg, "sub_second_image_topic", res_cfg.motion_config.sub_second_image_topic,        "/rs_camera/rgb");
    yamlRead<std::string>(cfg, "sub_lidar_topic",        res_cfg.motion_config.sub_lidar_topic,               "/rs_lidar/points");
    yamlRead<std::string>(cfg, "sub_imu_topic",          res_cfg.motion_config.sub_imu_topic,                 "/rs_imu");
    yamlRead<std::string>(cfg, "pub_ori_image_topic",    res_cfg.motion_config.pub_ori_image_topic,           "/camera/ori_image");

    const YAML::Node& range_image_cfg = cfg["RANGE_IMAGE"];
    yamlRead<double>(range_image_cfg, "resolution", res_cfg.lidar_config.range_image_info.res, 0.625);
    yamlRead<double>(range_image_cfg, "pitch_min", res_cfg.lidar_config.range_image_info.pitch_min, -60.0);
    yamlRead<double>(range_image_cfg, "pitch_max", res_cfg.lidar_config.range_image_info.pitch_max, 60.0);
    yamlRead<double>(range_image_cfg, "yaw_min", res_cfg.lidar_config.range_image_info.yaw_min, -30.0);
    yamlRead<double>(range_image_cfg, "yaw_max", res_cfg.lidar_config.range_image_info.yaw_max, 30.0);
    yamlRead<double>(range_image_cfg, "detection_range", res_cfg.lidar_config.range_image_info.detection_range, 10.0);
    yamlRead<double>(range_image_cfg, "range_change_thresh", res_cfg.lidar_config.range_image_info.range_change_thresh, 0.5);
    yamlRead<double>(range_image_cfg, "color_change_thresh", res_cfg.lidar_config.range_image_info.color_change_thresh, 50);
    yamlRead<double>(range_image_cfg, "color_min_thresh", res_cfg.lidar_config.range_image_info.color_min_thresh, 200);

    yamlRead<std::string>(cfg, "ORI_POINTS_TOPIC",       res_cfg.motion_config.ori_points_topic,              "/rslidar_points_origin");
    yamlRead<std::string>(cfg, "CM_EDGE_POINTS_TOPIC",        res_cfg.motion_config.edge_points_topic,        "/rslidar_points_edge");
    yamlRead<std::string>(cfg, "CM_POINTS_TOPIC",        res_cfg.motion_config.motion_points_topic,           "/rslidar_points_motion");
    yamlRead<std::string>(cfg, "CM_RGB_POINTS_TOPIC",    res_cfg.motion_config.motion_rgb_points_topic,       "/rslidar_points_motion_rgb");
    yamlRead<std::string>(cfg, "CM_STEREO_RGB_POINTS_TOPIC",    res_cfg.motion_config.motion_stereo_rgb_points_topic, "/rslidar_points_motion_stereo_rgb");

    yamlRead<std::string>(cfg, "ORI_DEOCC_RGB_POINTS_TOPIC", res_cfg.motion_config.ori_deocc_rgb_points_topic, "/rslidar_points_origin_deocc_rgb");
    yamlRead<std::string>(cfg, "ORI_IMG_TOPIC",          res_cfg.motion_config.ori_img_topic,                 "/camera/image_raw/proj_ori");
    yamlRead<std::string>(cfg, "PROJ_IMG_TOPIC",         res_cfg.motion_config.motion_proj_img_topic,         "/camera/image_raw/proj_motion");
    // yamlRead<std::string>(motion_cfg, "PROJ_DEOCC_IMG_TOPIC",   res_cfg.motion_config.motion_deocc_proj_img_topic,   "/camera/image_raw/proj_motion_deocc");
    yamlRead<std::string>(cfg, "RANGE_IMG_TOPIC",        res_cfg.motion_config.motion_range_img_topic,        "/camera/image_raw/range_motion");
    yamlRead<std::string>(cfg, "CAM_RANGE_IMG_TOPIC",    res_cfg.motion_config.motion_cam_range_img_topic,    "/camera/image_raw/cam_range_motion");
  }

  loadOfflineCaliInfo(calib_file, res_cfg, res_cfg.motion_config.use_second_image);

  return res_cfg;
}

}  // namespace postprocess
}  // namespace robosense