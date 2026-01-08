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
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
        // visualization marker publisher (arrows, lines)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        
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
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
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
        if(ids.size() > 0){
            std::vector<cv::Vec3d> rvecs, tvecs, selected_rvecs, selected_tvecs;
            // tvecs: Traslation vector (X, Y, Z from camera center)
            // rvecs: (Rotation Vectors): Orientation using Rodrigues notation.
            // Using PnP
            cv::aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix_, dist_coeffs_, rvecs, tvecs);
            std::vector<int> selected_ids;

            select_clustered_aruco(ids, rvecs, tvecs, selected_rvecs, selected_tvecs, selected_ids);

            // publish TF and arrow marker for each selected detected marker
            for(size_t i=0; i<selected_tvecs.size(); ++i) {
                // rvec -> rotation matrix -> quaternion
                cv::Mat R;
                cv::Mat rvec_mat = (cv::Mat1d(3,1) << selected_rvecs[i][0], selected_rvecs[i][1], selected_rvecs[i][2]);
                cv::Rodrigues(rvec_mat, R);
                tf2::Matrix3x3 m(
                    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
                );
                tf2::Quaternion q;
                m.getRotation(q);
                q.normalize();
                geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(q);

                // broadcast TF (camera -> aruco_ID)
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "camera_color_optical_frame";
                t.child_frame_id = std::string("aruco_") + std::to_string(selected_ids[i]);
                t.transform.translation.x = selected_tvecs[i][0];
                t.transform.translation.y = selected_tvecs[i][1];
                t.transform.translation.z = selected_tvecs[i][2];
                t.transform.rotation = quat_msg;
                tf_broadcaster_->sendTransform(t);

                // publish arrow marker for visualization
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
                mkr.scale.x = MARKER_LENGTH; // arrow length ~ marker length
                mkr.scale.y = MARKER_LENGTH * 0.2;
                mkr.scale.z = MARKER_LENGTH * 0.2;
                mkr.color.r = 0.0f; mkr.color.g = 1.0f; mkr.color.b = 0.0f; mkr.color.a = 1.0f;
                RCLCPP_INFO(this->get_logger(), "Publishing ARROW marker ns=%s id=%d frame=%s pos=(%.3f,%.3f,%.3f) scale=(%.3f,%.3f,%.3f) color=(%.3f,%.3f,%.3f,%.3f)",
                    mkr.ns.c_str(), mkr.id, mkr.header.frame_id.c_str(),
                    mkr.pose.position.x, mkr.pose.position.y, mkr.pose.position.z,
                    mkr.scale.x, mkr.scale.y, mkr.scale.z,
                    mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a);
                marker_pub_->publish(mkr);
            }

            geometry_msgs::msg::PoseStamped target_pose = compute_perpendicular_pose(selected_tvecs);
            if (!target_pose.header.frame_id.empty()) {
                object_pose_publisher_->publish(target_pose);
            }
        }
    }

    //  return clustered four or less aruco 
    void select_clustered_aruco(
        const std::vector<int> &ids,
        const std::vector<cv::Vec3d> &rvecs, 
        const std::vector<cv::Vec3d> &tvecs,
        std::vector<cv::Vec3d> &selected_rvecs, 
        std::vector<cv::Vec3d>& selected_tvecs, 
        std::vector<int>& selected_ids) {
        
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
                // Euclidean distance between marker i and the anchor (In optical frame)
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
            int count = 0;
                
            for(int i=0; i<candidates.size(); i++) {
                int idx = candidates[i].original_index;
                if (ids[idx] != YELLOW_ID and ids[idx] != BLUE_ID) {
                    continue;
                }
                selected_ids.push_back(ids[idx]);
                selected_tvecs.push_back(tvecs[idx]);
                selected_rvecs.push_back(rvecs[idx]);
                count += 1;
                if (count >= limit) {
                    break;
                }
            }
        }
    }

    // Goal is to make the robot go perpendicular to the 4 clustered hazelnut
    // PCA (Orientation) + Center (Translation)  
    geometry_msgs::msg::PoseStamped compute_perpendicular_pose(
        const std::vector<cv::Vec3d>& tvecs_camera_frame) {
        // Initialize the final result_pose message 

        geometry_msgs::msg::PoseStamped result_pose;

        if (tvecs_camera_frame.empty()) return result_pose;

        // 1. Transform points to base_footprint (Robot Frame)
        // Transform to base_footprint first so easier to tell robot where to go
        std::vector<cv::Point2d> floor_points;
        double sum_x = 0, sum_y = 0;
        for (const auto& t : tvecs_camera_frame) {
            geometry_msgs::msg::PointStamped pt_cam, pt_base;
            pt_cam.header.frame_id = "camera_color_optical_frame";
            pt_cam.header.stamp = rclcpp::Time(0); // Use latest
            pt_cam.point.x = t[0];
            pt_cam.point.y = t[1];
            pt_cam.point.z = t[2];

            try {
                // transform pt_cam -> pt_base, frame_id becomes base_footprint
                // 0.1 second patience to wait for data delay
                tf_buffer_->transform(pt_cam, pt_base, "base_footprint", tf2::durationFromSec(0.1));
                floor_points.push_back(cv::Point2d(pt_base.point.x, pt_base.point.y));
                sum_x += pt_base.point.x;
                sum_y += pt_base.point.y;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Point transform failed: %s", ex.what());
                return result_pose; 
            }
        }

        if(floor_points.size() < 2) return result_pose;

        result_pose.header.frame_id = "base_footprint";
        result_pose.header.stamp = this->get_clock()->now();

        // 2. Calculate Center (Target Position)
        double center_x = sum_x / floor_points.size();
        double center_y = sum_y / floor_points.size();

        // 3. PCA for Line Direction
        // Needs matrix as input, so transform std::vector -> cv::Mat
        cv::Mat data_pts(floor_points.size(), 2, CV_64F);
        for(size_t i=0; i<floor_points.size(); i++) {
            data_pts.at<double>(i, 0) = floor_points[i].x;
            data_pts.at<double>(i, 1) = floor_points[i].y;
        }
        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

        // Normalized Line Vector (dx, dy) 
        // eigenvector row = 0 is primary eigenvector
        // (0,0) x component, (0,1) y component
        double line_dx = pca_analysis.eigenvectors.at<double>(0, 0);
        double line_dy = pca_analysis.eigenvectors.at<double>(0, 1);

        // 4. Calculate Perpendicular Vector (Normal)
        // Rotate line vector by 90 degrees: (x, y) -> (-y, x)
        double perp_dx = -line_dy;
        double perp_dy = line_dx;

        // 5. "Face the ArUco" Check (Dot Product)
        // Vector from Robot(0,0) to Target(center_x, center_y) is just (center_x, center_y)
        double dot_product = (perp_dx * center_x) + (perp_dy * center_y);

        if (dot_product < 0) {
            // If negative, our normal vector points AWAY from the blocks. Flip it.
            perp_dx = -perp_dx;
            perp_dy = -perp_dy;
        }

        // 6. Convert to Yaw Angle
        double final_yaw = std::atan2(perp_dy, perp_dx);

        // 7. Fill Result
        result_pose.pose.position.x = center_x;
        result_pose.pose.position.y = center_y;
        result_pose.pose.position.z = 0.0; // Stay on floor

        tf2::Quaternion q;
        // This takes your calculated angle (Yaw) and calculates Quaternion (x, y, z, w). 
        q.setRPY(0, 0, final_yaw);
        q.normalize();
        result_pose.pose.orientation = tf2::toMsg(q);

        // Publish PCA principal axis as a LINE_STRIP marker in base_footprint frame
        if (marker_pub_) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = result_pose.header.frame_id;
            line.header.stamp = this->get_clock()->now();
            // .ns and .id is for rviz to know whether create new object or update old one
            line.ns = "pca";
            line.id = 100;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            geometry_msgs::msg::Point p1, p2;
            double len = 0.3; // half-length of PCA line to visualize
            p1.x = center_x + line_dx * len;
            p1.y = center_y + line_dy * len;
            p1.z = 0.0;
            p2.x = center_x - line_dx * len;
            p2.y = center_y - line_dy * len;
            p2.z = 0.0;
            line.points.push_back(p1);
            line.points.push_back(p2);
            line.scale.x = 0.02; // line width
            line.color.r = 1.0f; line.color.g = 0.0f; line.color.b = 0.0f; line.color.a = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Publishing LINE_STRIP marker ns=%s id=%d frame=%s p1=(%.3f,%.3f,%.3f) p2=(%.3f,%.3f,%.3f) scale=%.3f color=(%.3f,%.3f,%.3f,%.3f)",
                line.ns.c_str(), line.id, line.header.frame_id.c_str(),
                p1.x, p1.y, p1.z,
                p2.x, p2.y, p2.z,
                line.scale.x,
                line.color.r, line.color.g, line.color.b, line.color.a);
            marker_pub_->publish(line);
        }

        return result_pose;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
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
