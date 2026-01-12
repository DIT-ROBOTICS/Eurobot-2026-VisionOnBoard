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

    camera_matrix_ = (cv::Mat1d(3, 3) <<
        433.85614013671875, 0.0, 418.23370361328125,
        0.0, 433.11236572265625, 236.20132446289062,
        0.0, 0.0, 1.0);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    MARKER_LENGTH_ = this->declare_parameter<double>("marker_length", 0.03);
    BLUE_ID_ = this->declare_parameter<int>("blue_id", 36);
    YELLOW_ID_ = this->declare_parameter<int>("yellow_id", 47);

    // configure logic with declared params
    logic_ = ProcessLogic(MARKER_LENGTH_, BLUE_ID_, YELLOW_ID_);

    RCLCPP_INFO(this->get_logger(), "Object detector started.");
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

        visualization_msgs::msg::Marker mkr;
        mkr.header.stamp = this->get_clock()->now();
        mkr.header.frame_id = "camera_color_optical_frame";
        mkr.ns = "aruco";
        mkr.id = selected_ids[i];
        mkr.type = visualization_msgs::msg::Marker::ARROW;
        mkr.action = visualization_msgs::msg::Marker::ADD;
        mkr.pose.position.x = selected_tvecs[i][0];
        mkr.pose.position.y = selected_tvecs[i][1];
        mkr.pose.position.z = selected_tvecs[i][2];
        mkr.pose.orientation = quat_msg;
        mkr.scale.x = MARKER_LENGTH_;
        mkr.scale.y = MARKER_LENGTH_ * 0.2;
        mkr.scale.z = MARKER_LENGTH_ * 0.2;
        mkr.color.r = 0.0f; mkr.color.g = 1.0f; mkr.color.b = 0.0f; mkr.color.a = 1.0f;
        marker_pub_->publish(mkr);
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
        object_pose_publisher_->publish(target_pose);
    }
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
