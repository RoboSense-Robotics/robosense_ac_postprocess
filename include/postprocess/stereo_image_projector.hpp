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
#ifndef STEREO_IMAGE_PROJECTOR_HPP
#define STEREO_IMAGE_PROJECTOR_HPP
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
#include "postprocess/point_type.hpp"

namespace robosense {
namespace postprocess {

struct ImageFeatureInfo
{
    cv::Mat image_depthBuffer;
    cv::Mat image_indexBuffer;
    cv::Mat image_confidenceBuffer;
    std::vector<cv::Point3f> transform_points;
    std::vector<double> points_depth;
    std::vector<cv::Point2f> image_points;
    int image_cols;
    int image_rows;
};

class StereoImageProjector
{
private:
    void dataReset(const int image_cols, const int image_rows, const int points_size);
    void transformAndProjectToStereoPlanes(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                           Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl);
    void calculateImageFeatureInfo(ImageFeatureInfo &img_feature);
    void calculateDisparityAndColorConsistent(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts, cv::Mat &left_img, cv::Mat &right_img);
    void selectFinalImageColor(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFinalColorPoints(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts, cv::Mat &left_img, cv::Mat &right_img);
    void imgVisualization(cv::Mat &left_img, cv::Mat &right_img,
                          cv::Mat &left_img_proj, cv::Mat &right_img_proj,
                          pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts);

    int GetColormapIndex(float depth, float min_z, float max_z, int lut_size);
    void CreateColormapLUT(cv::Mat &colormap_lut, float min_z, float max_z);
    // for color mapping
    float min_z_ = 0;
    float max_z_ = 15.0;
    int lut_size_ = 256;
    cv::Mat colormap_lut_;

    // camera info
    cv::Mat left_distortion_coeffs_;
    cv::Mat left_camera_intrinsic_;
    cv::Mat right_distortion_coeffs_;
    cv::Mat right_camera_intrinsic_;

    int DOWNSAMPLE_BLOCK_ROW_SIZE;
    int DOWNSAMPLE_BLOCK_COL_SIZE;
    float MIN_DISPARITY;
    float MAX_DISPARITY;
    float MAX_DISPARITY_CHANGE;
    float COLOR_DIFF_THRESHOLD;
    float COLOR_SIMILARITY_THRESHOLD;
    float COLOR_CONSISTENCY_THRESHOLD;

    int downsampled_cols_;
    int downsampled_rows_;

    int image_cols_;
    int image_rows_;

    ImageFeatureInfo left_img_feature_;
    ImageFeatureInfo right_img_feature_;

    std::vector<float> disparity_values_;
    std::vector<float> color_consistency_scores_;
    std::vector<bool> color_consistent_flags_;

    std::vector<bool> use_left_camera_;
    std::vector<bool> use_right_camera_;
    std::vector<bool> use_both_camera_;
    std::vector<float> final_depth_values_;
    std::vector<cv::Point2f> final_image_points_;
public:
    StereoImageProjector(/* args */);
    ~StereoImageProjector();

    void cameraParamInitialization(const cv::Mat &left_distortion_coeffs,  const cv::Mat &left_camera_intrinsic,
                                   const cv::Mat &right_distortion_coeffs, const cv::Mat &right_camera_intrinsic,
                                   const double &stereo_baseline);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectStereoImgToCloud(
        cv::Mat &left_img, cv::Mat &right_img,
        cv::Mat &left_img_proj, cv::Mat &right_img_proj,
        pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
        Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl);
};

}  // namespace postprocess
}  // namespace robosense

#endif  // STEREO_IMAGE_PROJECTOR_HPP
