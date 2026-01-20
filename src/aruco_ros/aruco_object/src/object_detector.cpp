#include "my_package/my_node.hpp"

#include <chrono>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

MyNode::MyNode()
: Node("object_detector"), logic_(/*default constructed*/)
{
    using std::placeholders::_1;
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_rect_raw", 10,
        std::bind(&MyNode::image_callback, this, _1));

    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    MARKER_LENGTH_ = this->declare_parameter<double>("marker_length", 0.03);
    CLUSTER_RADIUS_ = this->declare_parameter<double>("cluster_radius", 0.3);
    BLUE_ID_ = this->declare_parameter<int>("blue_id", 36);
    YELLOW_ID_ = this->declare_parameter<int>("yellow_id", 47);
    CAMERA_POSITION_ = this->declare_parameter<std::string>("camera_position", "front");
    SMOOTH_ALPHA_ = this->declare_parameter<double>("smooth_alpha", 0.3);

    // configure logic with declared params
    logic_ = ProcessLogic(MARKER_LENGTH_, BLUE_ID_, YELLOW_ID_, CLUSTER_RADIUS_, CAMERA_POSITION_, SMOOTH_ALPHA_);

    RCLCPP_INFO(this->get_logger(), "Object detector started. cluster_radius: %.3f", CLUSTER_RADIUS_);
}

void MyNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);

    if (ids.empty()) return;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

    std::vector<cv::Vec3d> selected_rvecs, selected_tvecs;
    std::vector<int> selected_ids;
    logic_.select_clustered_aruco(ids, rvecs, tvecs, selected_rvecs, selected_tvecs, selected_ids);

    // publish TF and arrow markers for selected markers (in camera frame)
    for (size_t i = 0; i < selected_tvecs.size(); ++i) {
        cv::Mat R;
        cv::Mat rvec_mat = (cv::Mat1d(3,1) << selected_rvecs[i][0], selected_rvecs[i][1], selected_rvecs[i][2]);
        cv::Rodrigues(rvec_mat, R);
        tf2::Matrix3x3 m(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
        );
        tf2::Quaternion q; m.getRotation(q); q.normalize();
        geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "camera_color_optical_frame";
        t.child_frame_id = std::string("aruco_") + std::to_string(selected_ids[i]);
        t.transform.translation.x = selected_tvecs[i][0];
        t.transform.translation.y = selected_tvecs[i][1];
        t.transform.translation.z = selected_tvecs[i][2];
        t.transform.rotation = quat_msg;
        tf_broadcaster_->sendTransform(t);
    }

    // Transform selected points into base_footprint and compute perpendicular pose
    std::vector<cv::Point2d> floor_points;
    for (const auto &tvec : selected_tvecs) {
        geometry_msgs::msg::PointStamped pt_cam, pt_base;
        pt_cam.header.frame_id = "camera_color_optical_frame";
        pt_cam.header.stamp = rclcpp::Time(0);
        pt_cam.point.x = tvec[0]; pt_cam.point.y = tvec[1]; pt_cam.point.z = tvec[2];
        try {
            tf_buffer_->transform(pt_cam, pt_base, "base_footprint", tf2::durationFromSec(0.1));
            floor_points.emplace_back(pt_base.point.x, pt_base.point.y);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Point transform failed: %s", ex.what());
            return;
        }
    }

    if (floor_points.size() < 2) return;

    geometry_msgs::msg::PoseStamped target_pose = logic_.compute_perpendicular_pose_from_floor_points(
        floor_points, this->get_clock()->now(), marker_pub_);

    if (!target_pose.header.frame_id.empty()) {
        // Log key results: number of selected markers, center and yaw
        double cx = target_pose.pose.position.x;
        double cy = target_pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(target_pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double yaw_deg = yaw * (180.0 / M_PI);

        // selected_ids may be empty if none matched; print count
        RCLCPP_INFO(this->get_logger(), "Selected markers=%zu center=(%.3f,%.3f) yaw=%.3fdeg", selected_ids.size(), cx, cy, yaw_deg);

        object_pose_publisher_->publish(target_pose);
    }
}

void MyNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Optimization: If parameters don't change, we only need to read them once
    if (intrinsics_received_) {
        return; 
    }

    // 2. Convert Intrinsic Matrix (K) -> cv::Mat
    // msg->k is a std::array<double, 9>
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    
    // Manually copy data to ensure safety and correct ordering
    // (Row-major order: Row 0, then Row 1, then Row 2)
    for(int i=0; i<9; i++) {
        camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
    }

    // 3. Convert Distortion Coefficients (D) -> cv::Mat
    // msg->d is a std::vector<double>
    dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F);
    for(size_t i=0; i<msg->d.size(); i++) {
        dist_coeffs_.at<double>(0, i) = msg->d[i];
    }

    intrinsics_received_ = true;

    // Log to verify
    RCLCPP_INFO(this->get_logger(), "Camera Info received. K[0,0]: %f", camera_matrix_.at<double>(0,0));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
