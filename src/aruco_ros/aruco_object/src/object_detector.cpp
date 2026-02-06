#include "aruco_cluster_detect/object_detector_node.hpp"

#include <chrono>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

CameraOnBoardNode::CameraOnBoardNode()
: Node("aruco_detector_node"), logic_(/*default constructed*/)
{
    using std::placeholders::_1;
    
    // Declare parameters FIRST (before using them for topics)
    MARKER_LENGTH_ = this->declare_parameter<double>("marker_length", 0.03);
    CLUSTER_RADIUS_ = this->declare_parameter<double>("cluster_radius", 0.3);
    BLUE_ID_ = this->declare_parameter<int>("blue_id", 36);
    YELLOW_ID_ = this->declare_parameter<int>("yellow_id", 47);
    CAMERA_POSITION_ = this->declare_parameter<std::string>("camera_position", "front");
    SMOOTH_ALPHA_ = this->declare_parameter<double>("smooth_alpha", 0.3);
    
    // Map camera position string to dock_side number
    // dock_side: 0=front, 1=right, 2=back, 3=left
    position_to_side_ = {{"front", 0}, {"right", 1}, {"back", 2}, {"left", 3}};
    my_side_ = position_to_side_[CAMERA_POSITION_];
    
    // Build namespaced topic names based on camera position
    // e.g., "front" -> "/front/camera/color/image_rect_raw"
    std::string image_topic = "/" + CAMERA_POSITION_ + "/camera/color/image_rect_raw";
    std::string camera_info_topic = "/" + CAMERA_POSITION_ + "/camera/color/camera_info";
    
    RCLCPP_INFO(this->get_logger(), "Detector [%s] (side=%d) subscribing to: %s", 
        CAMERA_POSITION_.c_str(), my_side_, image_topic.c_str());
    
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10,
        std::bind(&CameraOnBoardNode::image_callback, this, _1));
    
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, 10,
        std::bind(&CameraOnBoardNode::camera_info_callback, this, _1));

    object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
    
    // Subscribe to dock_side topic for multi-camera activation control
    dock_side_sub_ = this->create_subscription<std_msgs::msg::Int16>(
        "/robot/dock_side", 10,
        [this](std_msgs::msg::Int16::SharedPtr msg) { 
            active_side_ = msg->data;
            bool is_active = (active_side_ == my_side_);
            RCLCPP_INFO(this->get_logger(), "Dock side: %d -> %s", 
                msg->data, is_active ? "ACTIVE" : "dormant");
        });
    

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    detector_params_ = cv::aruco::DetectorParameters::create();

    // configure logic with declared params
    logic_ = ProcessLogic(MARKER_LENGTH_, BLUE_ID_, YELLOW_ID_, CLUSTER_RADIUS_, CAMERA_POSITION_, SMOOTH_ALPHA_);

    // Pre-calculate marker object points (Center at 0,0,0)
    double half_len = MARKER_LENGTH_ * 0.5;
    marker_obj_points_ = {
        cv::Point3f(-half_len,  half_len, 0),
        cv::Point3f( half_len,  half_len, 0),
        cv::Point3f( half_len, -half_len, 0),
        cv::Point3f(-half_len, -half_len, 0)
    };

    RCLCPP_INFO(this->get_logger(), "Object detector [%s] (side=%d) started. cluster_radius: %.3f", 
        CAMERA_POSITION_.c_str(), my_side_, CLUSTER_RADIUS_);
}

void CameraOnBoardNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Multi-camera activation: dormant by default until dock_side matches our side
    if (active_side_ != my_side_) {
        return;
    }
    
    // get the camera's internal parameters
    if (!intrinsics_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for camera intrinsics...");
        return;
    }
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

    // Optimize TF lookup: Get transform ONCE for this frame timestamp
    geometry_msgs::msg::TransformStamped cam_to_base;
    try {
        cam_to_base = tf_buffer_->lookupTransform("base_footprint", "camera_color_optical_frame", 
                                                 msg->header.stamp, 
                                                 rclcpp::Duration::from_seconds(0.1)); 
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return; 
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Vec3d> rvecs, tvecs;
    rvecs.reserve(ids.size());
    tvecs.reserve(ids.size());

    for (size_t i = 0; i < ids.size(); ++i) {
        // 1. Sub-pixel refinement
        cv::cornerSubPix(
            gray, corners[i],
            cv::Size(5, 5),
            cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1)
        );

        // 2. Solve PnP using pre-calculated object points
        cv::Vec3d rvec, tvec;
        cv::solvePnP(marker_obj_points_, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
        
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }

    std::vector<cv::Vec3d> selected_rvecs, selected_tvecs;
    std::vector<int> selected_ids;
    logic_.select_clustered_aruco(ids, rvecs, tvecs, selected_rvecs, selected_tvecs, selected_ids);

    // // publish TF and arrow markers for selected markers (in camera frame)
    // for (size_t i = 0; i < selected_tvecs.size(); ++i) {
    //     cv::Mat R;
    //     cv::Mat rvec_mat = (cv::Mat1d(3,1) << selected_rvecs[i][0], selected_rvecs[i][1], selected_rvecs[i][2]);
    //     cv::Rodrigues(rvec_mat, R);
    //     tf2::Matrix3x3 m(
    //         R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
    //         R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
    //         R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    //     );
    //     tf2::Quaternion q; m.getRotation(q); q.normalize();
    //     geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);

    //     geometry_msgs::msg::TransformStamped t;
    //     t.header.stamp = this->get_clock()->now();
    //     t.header.frame_id = "camera_color_optical_frame";
    //     t.child_frame_id = std::string("aruco_") + std::to_string(selected_ids[i]);
    //     t.transform.translation.x = selected_tvecs[i][0];
    //     t.transform.translation.y = selected_tvecs[i][1];
    //     t.transform.translation.z = selected_tvecs[i][2];
    //     t.transform.rotation = quat_msg;
    //     tf_broadcaster_->sendTransform(t);
    // }

    // Transform selected points into base_footprint using the Cached Transform
    std::vector<cv::Point2d> floor_points;
    for (const auto &tvec : selected_tvecs) {
        geometry_msgs::msg::Point pt_cam;
        pt_cam.x = tvec[0]; pt_cam.y = tvec[1]; pt_cam.z = tvec[2];
        
        geometry_msgs::msg::Point pt_base;
        tf2::doTransform(pt_cam, pt_base, cam_to_base);
        
        floor_points.emplace_back(pt_base.x, pt_base.y);
    }

    if (floor_points.size() < 2) return;

    // Extract camera position in base_footprint from the transform
    cv::Point2d camera_in_base(cam_to_base.transform.translation.x, cam_to_base.transform.translation.y);

    geometry_msgs::msg::PoseStamped target_pose = logic_.compute_perpendicular_pose_from_floor_points(
        floor_points, camera_in_base, this->get_clock()->now());

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

void CameraOnBoardNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    // Optimization: If parameters don't change, we only need to read them once
    if (intrinsics_received_) {
        return; 
    }
    // Unpack the camera matrix from camera_info
    camera_matrix_ = cv::Mat(3, 3, CV_64F);

    for(int i=0; i<9; i++) {
        camera_matrix_.at<double>(i/3, i%3) = msg->k[i];
    }

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
    auto node = std::make_shared<CameraOnBoardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
