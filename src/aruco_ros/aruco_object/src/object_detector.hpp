#ifndef OBJECT_DETECTOR_HPP
#define OBJECT_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

class ObjectDetectorNode : public rclcpp::Node
{
public:
    // Node called object_detector
    ObjectDetectorNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    //  return clustered four or less aruco
    void select_clustered_aruco(
        const std::vector<int> &ids,
        const std::vector<cv::Vec3d> &rvecs, 
        const std::vector<cv::Vec3d> &tvecs,
        std::vector<cv::Vec3d> &selected_rvecs, 
        std::vector<cv::Vec3d>& selected_tvecs, 
        std::vector<int>& selected_ids );

    // Goal is to make the robot go perpendicular to the 4 clustered hazelnut
    // PCA (Orientation) + Center (Translation)
    geometry_msgs::msg::PoseStamped compute_perpendicular_pose(const std::vector<cv::Vec3d>& tvecs_camera_frame);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_, dist_coeffs_;

    double MARKER_LENGTH;
    int BLUE_ID;
    int YELLOW_ID;
};

#endif // OBJECT_DETECTOR_HPP
