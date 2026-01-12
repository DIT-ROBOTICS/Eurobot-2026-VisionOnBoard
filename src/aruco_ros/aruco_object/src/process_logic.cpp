#include "my_package/process_logic.hpp"

#include <algorithm>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ProcessLogic::ProcessLogic(double marker_length, int blue_id, int yellow_id)
    : marker_length_(marker_length), blue_id_(blue_id), yellow_id_(yellow_id) {}

ProcessLogic::ProcessLogic()
    : marker_length_(0.0), blue_id_(-1), yellow_id_(-1) {}

void ProcessLogic::select_clustered_aruco(
    const std::vector<int> &ids,
    const std::vector<cv::Vec3d> &rvecs,
    const std::vector<cv::Vec3d> &tvecs,
    std::vector<cv::Vec3d> &selected_rvecs,
    std::vector<cv::Vec3d> &selected_tvecs,
    std::vector<int> &selected_ids) {

    int closest_idx = -1;
    double min_z = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tvecs.size(); i++) {
        if (tvecs[i][2] < min_z) {
            min_z = tvecs[i][2];
            closest_idx = (int)i;
        }
    }

    if (closest_idx == -1) return;

    double cluster_radius = 0.3; // 30 cm

    struct MarkerCandidate { int original_index; double dist_to_anchor; };
    std::vector<MarkerCandidate> candidates;

    cv::Vec3d anchor_pos = tvecs[closest_idx];
    for (size_t i = 0; i < tvecs.size(); i++) {
        double dist = cv::norm(tvecs[i] - anchor_pos);
        if (dist <= cluster_radius) candidates.push_back({(int)i, dist});
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

    double dot_product = (perp_dx * center_x) + (perp_dy * center_y);
    if (dot_product > 0) { perp_dx = -perp_dx; perp_dy = -perp_dy; }

    double final_yaw = std::atan2(perp_dy, perp_dx);

    result_pose.header.frame_id = "base_footprint";
    result_pose.header.stamp = now;
    result_pose.pose.position.x = center_x;
    result_pose.pose.position.y = center_y;
    result_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, final_yaw);
    q.normalize();
    result_pose.pose.orientation = tf2::toMsg(q);

    // Publish PCA principal axis as a LINE_STRIP marker if requested
    if (marker_pub) {
        visualization_msgs::msg::Marker line;
        line.header.frame_id = result_pose.header.frame_id;
        line.header.stamp = now;
        line.ns = "pca";
        line.id = 100;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        geometry_msgs::msg::Point p1, p2;
        double len = 0.3;
        p1.x = center_x + line_dx * len; p1.y = center_y + line_dy * len; p1.z = 0.0;
        p2.x = center_x - line_dx * len; p2.y = center_y - line_dy * len; p2.z = 0.0;
        line.points.push_back(p1);
        line.points.push_back(p2);
        line.scale.x = 0.02;
        line.color.r = 1.0f; line.color.g = 0.0f; line.color.b = 0.0f; line.color.a = 1.0f;
        marker_pub->publish(line);
    }

    return result_pose;
}
