#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>

class ProcessLogic {
public:
    ProcessLogic();
    ProcessLogic(double marker_length, int blue_id, int yellow_id, double cluster_radius, std::string camera_position = "front", double smooth_alpha = 0.3);

    // Select clustered aruco markers (closest to anchor and matching allowed ids)
    void select_clustered_aruco(
        const std::vector<int> &ids,
        const std::vector<cv::Vec3d> &rvecs,
        const std::vector<cv::Vec3d> &tvecs,
        std::vector<cv::Vec3d> &selected_rvecs,
        std::vector<cv::Vec3d> &selected_tvecs,
        std::vector<int> &selected_ids);

    // Compute perpendicular pose from points already transformed to the robot frame
    // `now` is used for marker timestamps; `marker_pub` may be nullptr to skip marker publishing.
    geometry_msgs::msg::PoseStamped compute_perpendicular_pose_from_floor_points(
        const std::vector<cv::Point2d> &floor_points,
        const cv::Point2d &camera_in_base,
        const rclcpp::Time &now);

private:
    double marker_length_;
    int blue_id_;
    int yellow_id_;
    double cluster_radius_;
    std::string camera_position_;  // "front", "back", "left", "right"
    
    // Smoothing filter parameters
    double smooth_alpha_;  // Exponential smoothing factor (0-1, lower = more smoothing)
    geometry_msgs::msg::PoseStamped last_pose_;
    bool first_pose_ = true;
};
