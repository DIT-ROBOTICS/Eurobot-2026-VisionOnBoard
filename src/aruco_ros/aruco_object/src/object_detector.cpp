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

class ObjectDetectorNode : public rclcpp::Node
{
public:
    // Node called object_detector
    ObjectDetectorNode() : Node("object_detector")
    {
        using std::placeholders::_1;
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", 10,
            std::bind(&ObjectDetectorNode::image_callback, this, _1));
        // publisher for sending Poststamp to detect_dock_pose topic
        object_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);
        // start TF System
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            // get_clock() is for synchronization
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            // subscribes to /tf and /tf_static and store into tf_buffer_
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Aruco setup
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();
    
        camera_matrix_ = (cv::Mat1d(3, 3) <<
            433.85614013671875, 0.0, 418.23370361328125,
            0.0, 433.11236572265625, 236.20132446289062,
            0.0, 0.0, 1.0);
        // here assume image is already perfectly rectified
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

        MARKER_LENGTH = this->declare_parameter<double>("marker_length", 0.03);
        BLUE_ID = this->declare_parameter<int>("blue_id", 36);
        YELLOW_ID = this->declare_parameter<int>("yellow_id", 47);

        RCLCPP_INFO(this->get_logger(), "Object detector started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // cv_bridge converts ros2 sensor::msgs to cv::Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // copy whole image sent by the message (originally it was a pointer)
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        // Detect each frame for pattern that match with dictionary_ (2D)
        cv::Mat frame = cv_ptr->image;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detector_params_, rejected);
        
        
        // publish marker tfs
        // Pose Estimation (3D)
        if(ids.size() > 0){
            std::vector<cv::Vec3d> rvecs, tvecs, selected_rvecs, selected_tvecs;
            // tvecs: Traslation vector (X, Y, Z from camera center)
            // rvecs: (Rotation Vectors): Orientation using Rodrigues notation.
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix_, dist_coeffs_, rvecs, tvecs);
            std::vector<int> selected_id;

            select_clustered_aruco(rvecs, tvecs, selected_rvecs, selected_tvecs, selected_id);

            int cnt = 0;
            for(size_t i=0; i<selected_id.size(); i++){
                if(ids[i] != BLUE_ID && ids[i] != YELLOW_ID) continue;
                tvecs_obj += tvecs[i];
                cnt++;
            }
            if(cnt > 0){
                tvecs_obj /= cnt;
                // Math Conversion (OpenCV to ROS TF2) OpenCV uses Matrices/Vectors; ROS uses Quaternions.
                tf2::Vector3 t_cm(tvecs_obj[0], tvecs_obj[1], tvecs_obj[2]);
                cv::Mat R_mat;
                cv::Rodrigues(rvecs_obj, R_mat);
                // map openCV matrix to TF2 matrix
                tf2::Matrix3x3 m(
                    R_mat.at<double>(0,0), R_mat.at<double>(0,1), R_mat.at<double>(0,2),
                    R_mat.at<double>(1,0), R_mat.at<double>(1,1), R_mat.at<double>(1,2),
                    R_mat.at<double>(2,0), R_mat.at<double>(2,1), R_mat.at<double>(2,2));
                
                // Extract Quaternion
                tf2::Quaternion q_cm; 
                m.getRotation(q_cm);
                
                // package tf and broadcast
                tf2::Transform T_cm(q_cm, t_cm);
                geometry_msgs::msg::TransformStamped tf_msg;
                tf_msg.header.stamp = this->get_clock()->now();
                tf_msg.header.frame_id = "camera_color_optical_frame";
                tf_msg.child_frame_id = "object_frame";
                tf_msg.transform = tf2::toMsg(T_cm);
                tf_broadcaster_->sendTransform(tf_msg);
                // cv::aruco::drawDetectedMarkers(frame, corners, ids);
                // cv::imshow("object Marker", frame);
                // cv::waitKey(1);

                try {
                    // Lookup where is the object relative to base_footprint
                    geometry_msgs::msg::TransformStamped base2obj_tf = tf_buffer_->lookupTransform("base_footprint", "object_frame", tf2::TimePointZero);
                    // Construct goal Pose (Should be the center of 4 hazelnuts)
                    geometry_msgs::msg::PoseStamped obj_pose_msg;
                    obj_pose_msg.header.stamp = this->get_clock()->now();
                    obj_pose_msg.header.frame_id = "object_frame";
                    obj_pose_msg.pose.position.x = base2obj_tf.transform.translation.x + 0.1;
                    obj_pose_msg.pose.position.y = base2obj_tf.transform.translation.y;
                    obj_pose_msg.pose.position.z = base2obj_tf.transform.translation.z;
                    obj_pose_msg.pose.orientation.x = base2obj_tf.transform.rotation.x;
                    obj_pose_msg.pose.orientation.y = base2obj_tf.transform.rotation.y;
                    obj_pose_msg.pose.orientation.z = base2obj_tf.transform.rotation.z;
                    obj_pose_msg.pose.orientation.w = base2obj_tf.transform.rotation.w;
                    object_pose_publisher_->publish(obj_pose_msg);

                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                }
            }
        }
    }

    //  return clustered four or less aruco 
    void select_clustered_aruco(const std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs
    std::vector<cv::Vec3d> &selected_rvecs, std::vector<cv::Vec3d>& selected_tvecs, std::vector<int>& selected_ids) {
        
        // 1. Find the "Anchor" (The marker closest to the camera)
        int closest_idx = -1;
        double min_z = std::numeric_limits<double>::max();

        for (size_t i = 0; i < tvecs.size(); i++) {
            // tvecs[i][2] is the depth (Z distance)
            if(tvecs[i][2] < min_z) {
                min_z = tvecs[i][2];
                closest_idx = i;
            }
        }

        if (closest_idx != -1) {
            // 3. Cluster: Find everyone close to the Anchor
            // Define a physical threshold (e.g., 3x marker size or fixed cm)
            // If blocks are 5cm, and they are in a line of 4, the furthest is ~15-20cm away.
            double cluster_radius = 0.3; // 30cm radius sphere around the closest block
            
            struct MarkerCandidate {
                int original_index;
                double dist_to_anchor;
            };
            std::vector<MarkerCandidate> candidates;

            cv::Vec3d anchor_pos = tvecs[closest_idx];

            for(size_t i = 0; i < tvecs.size(); i++) {
                // Euclidean distance between marker i and the anchor
                double dist = cv::norm(tvecs[i] - anchor_pos);

                if(dist <= cluster_radius) {
                    candidates.push_back({(int)i, dist});
                }
            }

            // 4. Filter: Keep closest 4 (or less)
            // Sort by distance to anchor (so we keep the tightest group)
            std::sort(candidates.begin(), candidates.end(), 
                [](const MarkerCandidate& a, const MarkerCandidate& b) {
                    return a.dist_to_anchor < b.dist_to_anchor;
                });

            // Limit to 4
            int limit = std::min((int)candidates.size(), 4);
            for(int i=0; i<limit; i++) {
                int idx = candidates[i].original_index;
                selected_ids.push_back(ids[idx]);
                selected_tvecs.push_back(tvecs[idx]);
                selected_rvecs.push_back(rvecs[idx]);
            }
        }
    }



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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
