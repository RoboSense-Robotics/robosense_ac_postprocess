#include "postprocess/stereo_image_projector.hpp"

namespace robosense {
namespace postprocess {

StereoImageProjector::StereoImageProjector()
{
  CreateColormapLUT(colormap_lut_, min_z_, max_z_);
}

StereoImageProjector::~StereoImageProjector()
{
}

void StereoImageProjector::cameraParamInitialization(const cv::Mat &left_distortion_coeffs,  const cv::Mat &left_camera_intrinsic,
                                                     const cv::Mat &right_distortion_coeffs, const cv::Mat &right_camera_intrinsic,
                                                     const double &stereo_baseline)
{
    left_distortion_coeffs_ = left_distortion_coeffs;
    left_camera_intrinsic_ = left_camera_intrinsic;
    right_distortion_coeffs_ = right_distortion_coeffs;
    right_camera_intrinsic_ = right_camera_intrinsic;

    float fx = (left_camera_intrinsic_.at<double>(0, 0) + right_camera_intrinsic_.at<double>(0, 0)) / 2;
    float fy = (left_camera_intrinsic_.at<double>(1, 1) + right_camera_intrinsic_.at<double>(1, 1)) / 2;

    // Downsample
    DOWNSAMPLE_BLOCK_ROW_SIZE = 6;
    DOWNSAMPLE_BLOCK_COL_SIZE = 6;
    // Disparity thresholds
    MIN_DISPARITY = 0.0f;
    // MAX（pixel）= baseline * focal_length / 0.1m
    MAX_DISPARITY = stereo_baseline * ((fx + fy) / 2) / 0.1;
    MAX_DISPARITY_CHANGE = 10.0f;
    // Color consistency thresholds
    COLOR_DIFF_THRESHOLD = 40.0f;
    COLOR_SIMILARITY_THRESHOLD = 0.7f;
    COLOR_CONSISTENCY_THRESHOLD = 0.8f;
}

void StereoImageProjector::dataReset(const int image_cols, const int image_rows, const int points_size)
{
    downsampled_cols_ = (image_cols + DOWNSAMPLE_BLOCK_COL_SIZE - 1) / DOWNSAMPLE_BLOCK_COL_SIZE;
    downsampled_rows_ = (image_rows + DOWNSAMPLE_BLOCK_ROW_SIZE - 1) / DOWNSAMPLE_BLOCK_ROW_SIZE;

    left_img_feature_.image_depthBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32FC1, cv::Scalar(FLT_MAX));
    left_img_feature_.image_indexBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32SC1, cv::Scalar(-1));
    left_img_feature_.image_confidenceBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32FC1, cv::Scalar(0.0f));
    right_img_feature_.image_depthBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32FC1, cv::Scalar(FLT_MAX));
    right_img_feature_.image_indexBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32SC1, cv::Scalar(-1));
    right_img_feature_.image_confidenceBuffer = cv::Mat(downsampled_rows_, downsampled_cols_, CV_32FC1, cv::Scalar(0.0f));

    left_img_feature_.transform_points.clear();
    left_img_feature_.image_points.clear();
    left_img_feature_.points_depth.clear();
    right_img_feature_.transform_points.clear();
    right_img_feature_.image_points.clear();
    right_img_feature_.points_depth.clear();


    image_cols_ = image_cols;
    image_rows_ = image_rows;
    left_img_feature_.image_cols = image_cols_;
    left_img_feature_.image_rows = image_rows_;
    right_img_feature_.image_cols = image_cols_;
    right_img_feature_.image_rows = image_rows_;

    disparity_values_.resize(points_size, 0.0f);
    color_consistency_scores_.resize(points_size, 0.0f);
    color_consistent_flags_.resize(points_size, false);

    use_left_camera_.resize(points_size, false);
    use_right_camera_.resize(points_size, false);
    use_both_camera_.resize(points_size, false);
    final_depth_values_.resize(points_size, 0.0f);
    final_image_points_.resize(points_size);
}


void StereoImageProjector::transformAndProjectToStereoPlanes(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
                                                             Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl)
{
    // transform lidar_points to camera , depth calculation
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        Eigen::Vector4d pt(cm_pts->points[i].x, cm_pts->points[i].y, cm_pts->points[i].z, 1.0);

        // left
        Eigen::Vector4d left_pt_trans = T_cam_cl * pt;
        left_img_feature_.transform_points.push_back(cv::Point3f(
            left_pt_trans[0], left_pt_trans[1], left_pt_trans[2]));
        left_img_feature_.points_depth.push_back(std::sqrt(
            left_pt_trans[0]*left_pt_trans[0] +
            left_pt_trans[1]*left_pt_trans[1] +
            left_pt_trans[2]*left_pt_trans[2]));

        // right
        Eigen::Vector4d right_pt_trans = T_camR_cl * pt;
        right_img_feature_.transform_points.push_back(cv::Point3f(
            right_pt_trans[0], right_pt_trans[1], right_pt_trans[2]));
        right_img_feature_.points_depth.push_back(std::sqrt(
            right_pt_trans[0]*right_pt_trans[0] +
            right_pt_trans[1]*right_pt_trans[1] +
            right_pt_trans[2]*right_pt_trans[2]));
    }

    // projection
    cv::projectPoints(left_img_feature_.transform_points,
                      cv::Mat::zeros(3, 1, CV_64F),
                      cv::Mat::zeros(3, 1, CV_64F),
                      left_camera_intrinsic_,
                      left_distortion_coeffs_,
                      left_img_feature_.image_points);

    cv::projectPoints(right_img_feature_.transform_points,
                      cv::Mat::zeros(3, 1, CV_64F),
                      cv::Mat::zeros(3, 1, CV_64F),
                      right_camera_intrinsic_,
                      right_distortion_coeffs_,
                      right_img_feature_.image_points);

    std::cout << "Total points: " << cm_pts->points.size() << std::endl;
}

void StereoImageProjector::calculateImageFeatureInfo(ImageFeatureInfo & img_feature)
{
    // establish a depth buffer (with downsampling)
    int visible_count = 0;
    for (size_t i = 0; i < img_feature.image_points.size(); ++i) {
        int u = std::round(img_feature.image_points[i].x);
        int v = std::round(img_feature.image_points[i].y);

        if (u < 0 || u >= img_feature.image_cols || v < 0 || v >= img_feature.image_rows) {
            continue;
        }

        float cur_depth = img_feature.points_depth[i];

        // downsampling
        int grid_u = u / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v = v / DOWNSAMPLE_BLOCK_ROW_SIZE;

        if (grid_u >= downsampled_cols_ || grid_v >= downsampled_rows_) {
            continue;
        }

        // keep closest points
        if (cur_depth < img_feature.image_depthBuffer.at<float>(grid_v, grid_u)) {
            img_feature.image_depthBuffer.at<float>(grid_v, grid_u) = cur_depth;
            img_feature.image_indexBuffer.at<int>(grid_v, grid_u) = i;
            // depth smaller confidence larger
            img_feature.image_confidenceBuffer.at<float>(grid_v, grid_u) = 1.0f / (1.0f + cur_depth);
            visible_count++;
        }
    }
    std::cout << "visible points (after downsampling): " << visible_count << std::endl;
}

void StereoImageProjector::calculateDisparityAndColorConsistent(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts, cv::Mat &left_img, cv::Mat &right_img)
{
    int stereo_visible_count = 0;
    int color_consistent_count = 0;
    float avg_disparity = 0.0f;
    float max_disparity = 0.0f;
    float min_disparity = FLT_MAX;
    int disparity_count = 0;

    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        int u_left = std::round(left_img_feature_.image_points[i].x);
        int v_left = std::round(left_img_feature_.image_points[i].y);
        int u_right = std::round(right_img_feature_.image_points[i].x);
        int v_right = std::round(right_img_feature_.image_points[i].y);

        bool left_in_image = (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows);
        bool right_in_image = (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows);

        if (!left_in_image || !right_in_image) {
            continue;
        }

        int grid_u_left = u_left / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v_left = v_left / DOWNSAMPLE_BLOCK_ROW_SIZE;
        int grid_u_right = u_right / DOWNSAMPLE_BLOCK_COL_SIZE;
        int grid_v_right = v_right / DOWNSAMPLE_BLOCK_ROW_SIZE;

        // check is closest point
        bool left_valid = (left_img_feature_.image_indexBuffer.at<int>(grid_v_left, grid_u_left) == static_cast<int>(i));
        bool right_valid = (right_img_feature_.image_indexBuffer.at<int>(grid_v_right, grid_u_right) == static_cast<int>(i));
        if (!left_valid || !right_valid) {
            continue;
        }

        stereo_visible_count++;
        float disparity = std::abs(u_left - u_right);

        if (disparity < MIN_DISPARITY || disparity > MAX_DISPARITY) {
            continue;
        }
        disparity_values_[i] = disparity;

        // get color
        cv::Vec3b left_color = left_img.at<cv::Vec3b>(v_left, u_left);
        cv::Vec3b right_color = right_img.at<cv::Vec3b>(v_right, u_right);

        float left_brightness = 0.299f * left_color[0] + 0.587f * left_color[1] + 0.114f * left_color[2];
        float right_brightness = 0.299f * right_color[0] + 0.587f * right_color[1] + 0.114f * right_color[2];

        // disparity larger threshold bigger
        float adaptive_color_threshold = COLOR_DIFF_THRESHOLD * (1.0f + disparity / 100.0f);

        float color_diff = std::sqrt(
            std::pow(static_cast<int>(left_color[0]) - static_cast<int>(right_color[0]), 2) +
            std::pow(static_cast<int>(left_color[1]) - static_cast<int>(right_color[1]), 2) +
            std::pow(static_cast<int>(left_color[2]) - static_cast<int>(right_color[2]), 2)
        );

        // calculate color_similarity（cosine)
        float dot_product = left_color[0]*right_color[0] + left_color[1]*right_color[1] + left_color[2]*right_color[2];
        float norm_left = std::sqrt(left_color[0]*left_color[0] + left_color[1]*left_color[1] + left_color[2]*left_color[2]);
        float norm_right = std::sqrt(right_color[0]*right_color[0] + right_color[1]*right_color[1] + right_color[2]*right_color[2]);
        float color_similarity = 0.0f;
        if (norm_left > 0 && norm_right > 0) {
            color_similarity = dot_product / (norm_left * norm_right);
        }
        // calculate color_consistent
        float adjusted_similarity_threshold = COLOR_SIMILARITY_THRESHOLD - (disparity / 300.0f);
        adjusted_similarity_threshold = std::max(adjusted_similarity_threshold, 0.4f);
        bool color_consistent = (color_diff < adaptive_color_threshold &&
                                color_similarity > adjusted_similarity_threshold);
        if (color_consistent) {
            color_consistent_flags_[i] = true;
            color_consistency_scores_[i] = color_similarity * (1.0f + disparity / 200.0f);
            color_consistent_count++;
            // Accumulate disparity statistics
            avg_disparity += disparity;
            max_disparity = std::max(max_disparity, disparity);
            min_disparity = std::min(min_disparity, disparity);
            disparity_count++;
        }
    }

    std::cout << "Stereo visible points: " << stereo_visible_count << std::endl;
    std::cout << "Color consistent points: " << color_consistent_count << std::endl;
    if (disparity_count > 0) {
        avg_disparity /= disparity_count;
        std::cout << "Disparity stats - Avg: " << avg_disparity 
                  << ", Min: " << min_disparity 
                  << ", Max: " << max_disparity << std::endl;
    }
}

void StereoImageProjector::selectFinalImageColor(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts)
{
    int left_only_count = 0;
    int right_only_count = 0;
    int disparity_left_only_count = 0;
    int disparity_right_only_count = 0;
    int both_count = 0;
    int neither_count = 0;

    for (size_t i = 0; i < cm_pts->points.size(); ++i) {
        int u_left = std::round(left_img_feature_.image_points[i].x);
        int v_left = std::round(left_img_feature_.image_points[i].y);
        int u_right = std::round(right_img_feature_.image_points[i].x);
        int v_right = std::round(right_img_feature_.image_points[i].y);

        bool left_in_image = (u_left >= 0 && u_left < left_img_feature_.image_cols && v_left >= 0 && v_left < left_img_feature_.image_rows);
        bool right_in_image = (u_right >= 0 && u_right < right_img_feature_.image_cols && v_right >= 0 && v_right < right_img_feature_.image_rows);

        int grid_u_left = left_in_image ? (u_left / DOWNSAMPLE_BLOCK_COL_SIZE) : -1;
        int grid_v_left = left_in_image ? (v_left / DOWNSAMPLE_BLOCK_ROW_SIZE) : -1;
        int grid_u_right = right_in_image ? (u_right / DOWNSAMPLE_BLOCK_COL_SIZE) : -1;
        int grid_v_right = right_in_image ? (v_right / DOWNSAMPLE_BLOCK_ROW_SIZE) : -1;

        bool left_is_closest = false;
        bool right_is_closest = false;
        float left_confidence = 0.0f;
        float right_confidence = 0.0f;

        if (left_in_image && grid_u_left >= 0 && grid_u_left < downsampled_cols_ &&
            grid_v_left >= 0 && grid_v_left < downsampled_rows_) {
            left_is_closest = (left_img_feature_.image_indexBuffer.at<int>(grid_v_left, grid_u_left) == static_cast<int>(i));
            if (left_is_closest) {
                left_confidence = left_img_feature_.image_confidenceBuffer.at<float>(grid_v_left, grid_u_left);
            }
        }

        if (right_in_image && grid_u_right >= 0 && grid_u_right < downsampled_cols_ &&
            grid_v_right >= 0 && grid_v_right < downsampled_rows_) {
            right_is_closest = (right_img_feature_.image_indexBuffer.at<int>(grid_v_right, grid_u_right) == static_cast<int>(i));
            if (right_is_closest) {
                right_confidence = right_img_feature_.image_confidenceBuffer.at<float>(grid_v_right, grid_u_right);
            }
        }

        // disparity and color_consistent check
        float disparity = disparity_values_[i];
        bool valid_disparity = (disparity >= MIN_DISPARITY && disparity <= MAX_DISPARITY);
        bool color_consistent = color_consistent_flags_[i];

        // main logic: get Union + Disparity optimization
        bool use_left = false;
        bool use_right = false;
        bool use_both = false;

        if (!left_in_image && right_in_image && right_is_closest) {
            // left not visable , use right (if right is closest）
            if (valid_disparity) {
                use_right = true;
                final_image_points_[i] = right_img_feature_.image_points[i];
                final_depth_values_[i] = right_img_feature_.points_depth[i];
                right_only_count++;
            }
        }
        else if (!right_in_image && left_in_image && left_is_closest) {
            // right not visable , use left (if left is closest）
            if (valid_disparity) {
                use_left = true;
                final_image_points_[i] = left_img_feature_.image_points[i];
                final_depth_values_[i] = left_img_feature_.points_depth[i];
                left_only_count++;
            }
        }
        else if (left_in_image && right_in_image) {
            if (left_is_closest && !right_is_closest) {
                // left is closest，right not -> use left
                if (valid_disparity) {
                    use_left = true;
                    final_image_points_[i] = left_img_feature_.image_points[i];
                    final_depth_values_[i] = left_img_feature_.points_depth[i];
                    left_only_count++;
                }
            }
            else if (!left_is_closest && right_is_closest) {
                // right is closest，left not -> use right
                if (valid_disparity) {
                    use_right = true;
                    final_image_points_[i] = right_img_feature_.image_points[i];
                    final_depth_values_[i] = right_img_feature_.points_depth[i];
                    right_only_count++;
                }
            }
            else if (left_is_closest && right_is_closest) {
                // all closest
                if (valid_disparity) {
                    if (color_consistent && color_consistency_scores_[i] > COLOR_CONSISTENCY_THRESHOLD) {
                        // color is consistent, get all
                        use_both = true;
                        final_image_points_[i] = left_img_feature_.image_points[i]; // use left
                        final_depth_values_[i] = (left_img_feature_.points_depth[i] + right_img_feature_.points_depth[i]) / 2.0f;
                        both_count++;
                    } 
                    // else {
                    //     // color is not consistent , delete
                    // }
                }
            }
            else {
                // neither closest (obscured)
                neither_count++;
                continue;
            }
        }
        else {
            // at least one not in or not closest
            neither_count++;
            continue;
        }

        // no select, use larger confidence
        if (!use_left && !use_right && !use_both && valid_disparity) {
            if (left_in_image && left_is_closest) {
                use_left = true;
                final_image_points_[i] = left_img_feature_.image_points[i];
                final_depth_values_[i] = left_img_feature_.points_depth[i];
                disparity_left_only_count++;
            } else if (right_in_image && right_is_closest) {
                use_right = true;
                final_image_points_[i] = right_img_feature_.image_points[i];
                final_depth_values_[i] = right_img_feature_.points_depth[i];
                disparity_right_only_count++;
            }
        }
        use_left_camera_[i] = use_left;
        use_right_camera_[i] = use_right;
        use_both_camera_[i] = use_both;
    }

    std::cout << "Left only: " << left_only_count
              << ", Right only: " << right_only_count
              << ", disparity_left_only_count: " << disparity_left_only_count
              << ", disparity_right_only_count: " << disparity_right_only_count
              << ", Both (fusion): " << both_count
              << ", Neither: " << neither_count << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoImageProjector::getFinalColorPoints(pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts, cv::Mat &left_img, cv::Mat &right_img)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (size_t i = 0; i < cm_pts->points.size(); ++i) {

        pcl::PointXYZRGB pt;
        // const auto& cm_pt = ori_pts->points[i]; //TODO
        const auto& cm_pt = cm_pts->points[i];
        pt.x = cm_pt.x;
        pt.y = cm_pt.y;
        pt.z = cm_pt.z;

        if (!use_left_camera_[i] && !use_right_camera_[i] && !use_both_camera_[i]) {
            continue;
        }

        int u = std::round(final_image_points_[i].x);
        int v = std::round(final_image_points_[i].y);

        cv::Vec3b pixel_value(0, 0, 0);

        if (use_left_camera_[i]) {
            if (u >= 0 && u < left_img.cols && v >= 0 && v < left_img.rows) {
                pixel_value = left_img.at<cv::Vec3b>(v, u);
            }
        }
        else if (use_right_camera_[i]) {
            if (u >= 0 && u < right_img.cols && v >= 0 && v < right_img.rows) {
                pixel_value = right_img.at<cv::Vec3b>(v, u);
            }
        }
        else if (use_both_camera_[i]) {
            // Weighted fusion： considering colors, depths, disparities, and consistencies
            int u_left = std::round(left_img_feature_.image_points[i].x);
            int v_left = std::round(left_img_feature_.image_points[i].y);
            int u_right = std::round(right_img_feature_.image_points[i].x);
            int v_right = std::round(right_img_feature_.image_points[i].y);
            cv::Vec3b left_color(0, 0, 0);
            cv::Vec3b right_color(0, 0, 0);
            bool left_valid = false;
            bool right_valid = false;
            if (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows) {
                left_color = left_img.at<cv::Vec3b>(v_left, u_left);
                left_valid = true;
            }
            if (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows) {
                right_color = right_img.at<cv::Vec3b>(v_right, u_right);
                right_valid = true;
            }
            if (left_valid && right_valid) {
                float left_depth = left_img_feature_.points_depth[i];
                float right_depth = right_img_feature_.points_depth[i];
                float disparity = disparity_values_[i];
                float consistency = color_consistency_scores_[i];

                float depth_weight_left = 1.0f / (1.0f + left_depth);
                float depth_weight_right = 1.0f / (1.0f + right_depth);

                // disparity larger, left weight is bigger
                float disparity_weight = 0.5f + (disparity / (disparity + 50.0f)) * 0.5f;

                float consistency_weight = consistency;

                float left_weight = depth_weight_left * disparity_weight * consistency_weight;
                float right_weight = depth_weight_right * (1.0f - disparity_weight) * consistency_weight;

                // weight normalization
                float total_weight = left_weight + right_weight;
                if (total_weight > 0) {
                    left_weight /= total_weight;
                    right_weight /= total_weight;
                    pixel_value[0] = static_cast<uchar>(left_color[0] * left_weight + right_color[0] * right_weight);
                    pixel_value[1] = static_cast<uchar>(left_color[1] * left_weight + right_color[1] * right_weight);
                    pixel_value[2] = static_cast<uchar>(left_color[2] * left_weight + right_color[2] * right_weight);
                } else {
                    // use average
                    pixel_value[0] = (left_color[0] + right_color[0]) / 2;
                    pixel_value[1] = (left_color[1] + right_color[1]) / 2;
                    pixel_value[2] = (left_color[2] + right_color[2]) / 2;
                }
            } else if (left_valid) {
                pixel_value = left_color;
            } else if (right_valid) {
                pixel_value = right_color;
            }
        }
        // set color
        pt.r = pixel_value[0];
        pt.g = pixel_value[1];
        pt.b = pixel_value[2];
        rgb_pts->push_back(pt);
    }
    std::cout << "Final point cloud size: " << rgb_pts->size() 
              << " (Retention rate: " << (rgb_pts->size() * 100.0 / cm_pts->points.size()) << "%)" << std::endl;

    return rgb_pts;
}

void StereoImageProjector::imgVisualization(cv::Mat &left_img, cv::Mat &right_img,
                                            cv::Mat &left_img_proj, cv::Mat &right_img_proj,
                                            pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts)
{
    if (!left_img_proj.empty() && !right_img_proj.empty()) {
        cv::Mat left_display = left_img.clone();
        cv::Mat right_display = right_img.clone();
        cv::Mat confidence_display = cv::Mat::zeros(left_img.size(), CV_8UC3);

        for (size_t i = 0; i < cm_pts->points.size(); ++i) {
            if (use_left_camera_[i] || use_right_camera_[i] || use_both_camera_[i]) {
                int u_left = std::round(left_img_feature_.image_points[i].x);
                int v_left = std::round(left_img_feature_.image_points[i].y);
                int u_right = std::round(right_img_feature_.image_points[i].x);
                int v_right = std::round(right_img_feature_.image_points[i].y);
                cv::Scalar point_color;
                if (use_left_camera_[i]) {
                    point_color = cv::Scalar(0, 0, 255); // red: left
                } else if (use_right_camera_[i]) {
                    point_color = cv::Scalar(0, 255, 0); // green: right
                } else if (use_both_camera_[i]) {
                    point_color = cv::Scalar(255, 0, 0); // blue: fusion
                }

                if (u_left >= 0 && u_left < left_img.cols && v_left >= 0 && v_left < left_img.rows) {
                    cv::circle(left_display, cv::Point(u_left, v_left), 2, point_color, -1);
                    if (use_both_camera_[i]) {
                        float confidence = color_consistency_scores_[i];
                        int intensity = static_cast<int>(confidence * 255);
                        cv::circle(confidence_display, cv::Point(u_left, v_left), 2, 
                                  cv::Scalar(intensity, intensity, intensity), -1);
                    }
                }
                if (u_right >= 0 && u_right < right_img.cols && v_right >= 0 && v_right < right_img.rows) {
                    cv::circle(right_display, cv::Point(u_right, v_right), 2, point_color, -1);
                }
            }
        }
        // cv::imshow("Left Image with Projected Points", left_display);
        // cv::imshow("Right Image with Projected Points", right_display);
        // cv::imshow("Confidence Map", confidence_display);
        // cv::waitKey(1);

        cv::Mat depthBuffer = cv::Mat::zeros(left_img_proj.size(), CV_32FC1);
        for (size_t i = 0; i < left_img_feature_.image_points.size(); ++i) {
          int u = left_img_feature_.image_points[i].x;
          int v = left_img_feature_.image_points[i].y;
          if (u<0 || u >= left_img_proj.cols || v<0 || v >= left_img_proj.rows) {
            continue;
          }
          float depth = left_img_feature_.points_depth[i];

          if (depthBuffer.at<float>(v, u) > depth)
          {
            left_img_feature_.image_points[i].x = -1;
            left_img_feature_.image_points[i].y = -1;
            continue;
          }
          depthBuffer.at<float>(v, u) = depth;
          int colormap_index = GetColormapIndex(depth, min_z_, max_z_, lut_size_);
          cv::Vec3b color = colormap_lut_.at<cv::Vec3b>(colormap_index, 0);
          cv::Scalar point_color(color[0], color[1], color[2]);
          cv::circle(left_img_proj, left_img_feature_.image_points[i], 3, point_color, -1);
        }
    }
}

int StereoImageProjector::GetColormapIndex(float depth, float min_z, float max_z, int lut_size) {
  depth = depth > max_z ? max_z : (depth<min_z ? min_z : depth);
  float normalized_depth = (depth - min_z) / (max_z - min_z);
  return static_cast<int>(normalized_depth * (lut_size - 1));
}

void StereoImageProjector::CreateColormapLUT(cv::Mat& colormap_lut, float min_z, float max_z) {
  int lut_size = 256;  // You can adjust the LUT size for more granularity
  cv::Mat colormap_lut_gray = cv::Mat(lut_size, 1, CV_8UC1);
  colormap_lut = cv::Mat(lut_size, 1, CV_8UC3);

  // Fill the LUT with values from 0 to 255
  for (int i = 0; i < lut_size; ++i)
  {
    colormap_lut_gray.at<uchar>(i, 0) = static_cast<uchar>(i);
  }

  // Apply the colormap to the LUT
  cv::applyColorMap(colormap_lut_gray, colormap_lut, cv::COLORMAP_JET);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoImageProjector::projectStereoImgToCloud(
    cv::Mat &left_img, cv::Mat &right_img,
    cv::Mat &left_img_proj, cv::Mat &right_img_proj,
    pcl::PointCloud<PointXYZIRT>::Ptr &cm_pts,
    Eigen::Matrix4d &T_cam_cl, Eigen::Matrix4d &T_camR_cl)
{

    int points_size = cm_pts->size();
    dataReset(left_img.cols, left_img.rows, points_size);

    transformAndProjectToStereoPlanes(cm_pts, T_cam_cl, T_camR_cl);

    calculateImageFeatureInfo(left_img_feature_);
    calculateImageFeatureInfo(right_img_feature_);

    calculateDisparityAndColorConsistent(cm_pts, left_img, right_img);

    selectFinalImageColor(cm_pts);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_pts = getFinalColorPoints(cm_pts, left_img, right_img);

    imgVisualization(left_img, right_img,
                     left_img_proj, right_img_proj, cm_pts);

    return rgb_pts;
}
} // namespace postprocess
}  // namespace robosense