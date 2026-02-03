#include "aruco_cluster_detect/process_logic.hpp"

#include <algorithm>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

ProcessLogic::ProcessLogic(double marker_length, int blue_id, int yellow_id, double cluster_radius, std::string camera_position, double smooth_alpha)
    : marker_length_(marker_length), blue_id_(blue_id), yellow_id_(yellow_id), cluster_radius_(cluster_radius), camera_position_(camera_position), smooth_alpha_(smooth_alpha) {}

ProcessLogic::ProcessLogic()
    : marker_length_(0.0), blue_id_(-1), yellow_id_(-1), cluster_radius_(0.3), camera_position_("front"), smooth_alpha_(0.3) {}

void ProcessLogic::select_clustered_aruco(
    const std::vector<int> &ids,
    const std::vector<cv::Vec3d> &rvecs,
    const std::vector<cv::Vec3d> &tvecs,
    std::vector<cv::Vec3d> &selected_rvecs,
    std::vector<cv::Vec3d> &selected_tvecs,
    std::vector<int> &selected_ids) {

    RCLCPP_INFO_ONCE(rclcpp::get_logger("ProcessLogic"), "Using cluster_radius: %.3f", cluster_radius_);

    int closest_idx = -1;
    double min_z = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tvecs.size(); i++) {
        if (tvecs[i][2] < min_z) {
            min_z = tvecs[i][2];
            closest_idx = (int)i;
        }
    }

    if (closest_idx == -1) return;

    struct MarkerCandidate { int original_index; double dist_to_anchor; };
    std::vector<MarkerCandidate> candidates;

    cv::Vec3d anchor_pos = tvecs[closest_idx];
    for (size_t i = 0; i < tvecs.size(); i++) {
        double dist = cv::norm(tvecs[i] - anchor_pos);
        if (dist <= cluster_radius_) candidates.push_back({(int)i, dist});
    }

    std::sort(candidates.begin(), candidates.end(), [](const MarkerCandidate &a, const MarkerCandidate &b){
        return a.dist_to_anchor < b.dist_to_anchor;
    });

    int limit = std::min((int)candidates.size(), 4);
    int count = 0;
    for (int i = 0; i < (int)candidates.size(); ++i) {
        int idx = candidates[i].original_index;
        if (ids[idx] != yellow_id_ && ids[idx] != blue_id_) continue;
        selected_ids.push_back(ids[idx]);
        selected_tvecs.push_back(tvecs[idx]);
        selected_rvecs.push_back(rvecs[idx]);
        if (++count >= limit) break;
    }
}

geometry_msgs::msg::PoseStamped ProcessLogic::compute_perpendicular_pose_from_floor_points(
    const std::vector<cv::Point2d> &floor_points,
    const rclcpp::Time &now,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub) {

    geometry_msgs::msg::PoseStamped result_pose;
    if (floor_points.empty()) return result_pose;

    // Center
    double sum_x = 0, sum_y = 0;
    for (const auto &p : floor_points) { sum_x += p.x; sum_y += p.y; }
    double center_x = sum_x / floor_points.size();
    double center_y = sum_y / floor_points.size();

    // PCA
    cv::Mat data_pts((int)floor_points.size(), 2, CV_64F);
    for (size_t i = 0; i < floor_points.size(); ++i) {
        data_pts.at<double>((int)i, 0) = floor_points[i].x;
        data_pts.at<double>((int)i, 1) = floor_points[i].y;
    }
    cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    double line_dx = pca_analysis.eigenvectors.at<double>(0, 0);
    double line_dy = pca_analysis.eigenvectors.at<double>(0, 1);

    double perp_dx = -line_dy;
    double perp_dy = line_dx;

    // 3. Adjust orientation based on Camera Position to minimize robot rotation
    double final_yaw = 0.0;
    double perp_angle = 0.0;

    if (camera_position_ == "left") {
        // Left Camera looks at +Y.
        // We want the normal pointing INTO the wall (towards +Y).
        if (perp_dy < 0) { perp_dx = -perp_dx; perp_dy = -perp_dy; }
        
        perp_angle = std::atan2(perp_dy, perp_dx);
        
        // Robot Yaw = Wall Normal - Camera Offset (+90 deg) 
        // This allows the robot to stay straight (Yaw 0) if the wall is directly to the left.
        final_yaw = perp_angle - M_PI_2; 

    } else if (camera_position_ == "right") {
        // Right Camera looks at -Y.
        // We want the normal pointing INTO the wall (towards -Y).
        if (perp_dy > 0) { perp_dx = -perp_dx; perp_dy = -perp_dy; }
        
        perp_angle = std::atan2(perp_dy, perp_dx);

        // Robot Yaw = Wall Normal - Camera Offset (-90 deg)
        // This allows the robot to stay straight (Yaw 0) if the wall is directly to the right.
        final_yaw = perp_angle + M_PI_2;

    } else if (camera_position_ == "back") {
        // Back Camera looks at -X.
        // We want normal pointing -X (Into wall behind us).
        if (perp_dx > 0) { perp_dx = -perp_dx; perp_dy = -perp_dy; }
        
        perp_angle = std::atan2(perp_dy, perp_dx);
        
        // Robot Yaw = Wall Normal - Camera Offset (180 deg)
        // If we are backing up to a wall, we want Yaw to be 0 (or 180 depending on your controller).
        // Usually, to face AWAY from the wall, Yaw matches the normal + 180.
        final_yaw = perp_angle + M_PI; 

    } else {
        // Front Camera (Default) looks at +X
        // We want normal pointing +X
        if (perp_dx < 0) { perp_dx = -perp_dx; perp_dy = -perp_dy; }
        
        final_yaw = std::atan2(perp_dy, perp_dx);
    }

    // Normalize angle to -PI ~ PI
    while (final_yaw < -M_PI) final_yaw += 2.0 * M_PI;
    while (final_yaw > M_PI) final_yaw -= 2.0 * M_PI;

    double final_yaw_deg = final_yaw * (180.0 / M_PI);
    RCLCPP_INFO(rclcpp::get_logger("ProcessLogic"), 
        "Cam: %s | Center=(%.2f,%.2f) | Normal=(%.2f,%.2f) | RobotYaw=%.1f deg", 
        camera_position_.c_str(), center_x, center_y, perp_dx, perp_dy, final_yaw_deg);

    result_pose.header.frame_id = "base_footprint";
    result_pose.header.stamp = now;
    result_pose.pose.position.x = center_x;
    result_pose.pose.position.y = center_y;
    result_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, final_yaw);
    q.normalize();
    result_pose.pose.orientation = tf2::toMsg(q);

    // Apply exponential smoothing filter
    if (!first_pose_) {
        // Smooth position
        result_pose.pose.position.x = smooth_alpha_ * result_pose.pose.position.x + 
                                      (1.0 - smooth_alpha_) * last_pose_.pose.position.x;
        result_pose.pose.position.y = smooth_alpha_ * result_pose.pose.position.y + 
                                      (1.0 - smooth_alpha_) * last_pose_.pose.position.y;
        
        // Smooth orientation (convert to angle, smooth, convert back)
        tf2::Quaternion q_last;
        tf2::fromMsg(last_pose_.pose.orientation, q_last);
        double roll_last, pitch_last, yaw_last;
        tf2::Matrix3x3(q_last).getRPY(roll_last, pitch_last, yaw_last);
        
        // Smooth yaw (handle angle wrapping)
        double yaw_diff = final_yaw - yaw_last;
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
        double smoothed_yaw = yaw_last + smooth_alpha_ * yaw_diff;
        
        // Set smoothed orientation
        q.setRPY(0, 0, smoothed_yaw);
        q.normalize();
        result_pose.pose.orientation = tf2::toMsg(q);
    } else {
        first_pose_ = false;
    }
    
    // Store for next iteration
    last_pose_ = result_pose;

    // Publish Visualization Marker (Optional: Show the calculated approach vector)
    if (marker_pub) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = result_pose.header.frame_id;
        arrow.header.stamp = now;
        arrow.ns = "approach_vector";
        arrow.id = 101;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;
        
        // Start point (Robot/Cluster Center)
        geometry_msgs::msg::Point p1, p2;
        p1.x = result_pose.pose.position.x; 
        p1.y = result_pose.pose.position.y; 
        p1.z = 0.0;
        
        // End point (pointing into the wall)
        double len = 0.5;
        p2.x = result_pose.pose.position.x + perp_dx * len; 
        p2.y = result_pose.pose.position.y + perp_dy * len; 
        p2.z = 0.0;
        
        arrow.points.push_back(p1);
        arrow.points.push_back(p2);
        arrow.scale.x = 0.05; arrow.scale.y = 0.1; arrow.scale.z = 0.1;
        arrow.color.r = 0.0f; arrow.color.g = 1.0f; arrow.color.b = 0.0f; arrow.color.a = 1.0f;
        marker_pub->publish(arrow);
    }

    return result_pose;
}
