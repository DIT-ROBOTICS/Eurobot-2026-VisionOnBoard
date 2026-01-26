#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <map>
#include <deque>

class ArucoRowScannerNode : public rclcpp::Node {
public:
    ArucoRowScannerNode() : Node("aruco_row_scanner") {
        // Parameters
        this->declare_parameter<std::string>("team_color", "yellow");
        this->declare_parameter<int64_t>("blue_id", 36);
        this->declare_parameter<int64_t>("yellow_id", 47);
        this->declare_parameter<int>("votes_needed", 5);

        team_color_ = this->get_parameter("team_color").as_string();
        votes_needed_ = this->get_parameter("votes_needed").as_int();

        // logic: if team_color is blue, use yellow_id, else use blue_id
        int64_t id = (team_color_ == "blue") ? 
            this->get_parameter("yellow_id").as_int() : 
            this->get_parameter("blue_id").as_int();

        target_ids_.push_back(static_cast<int>(id));

        RCLCPP_INFO(this->get_logger(), "Scanner started for team: %s. Votes needed: %d", team_color_.c_str(), votes_needed_);

        // Initialize ArUco dictionary
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        detector_params_ = cv::aruco::DetectorParameters::create();

        // Subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", 10,
            std::bind(&ArucoRowScannerNode::image_callback, this, std::placeholders::_1));

        // Publisher
        mask_pub_ = this->create_publisher<std_msgs::msg::Int8MultiArray>("target_flip_mask", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (task_completed_) return;

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_);

        // Phase 2: Perfect 4 Check
        if (ids.size() != 4) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Seen %zu markers (need 4)", ids.size());
            return; // Ignore frame
        }
        struct Marker {
            int id;
            float x;
        };

        std::vector<Marker> sorted_markers;
        // calculate center x of each marker
        for (size_t i = 0; i < ids.size(); ++i) {
            float cx = 0;
            for (const auto& p : corners[i]) cx += p.x;
            cx /= 4.0;
            sorted_markers.push_back({ids[i], cx});
        }
        
        // sort by ascending x (left to right)
        std::sort(sorted_markers.begin(), sorted_markers.end(), 
            [](const Marker& a, const Marker& b) { return a.x < b.x; });

        // Generate Instant Array
        std::vector<int8_t> current_vote;
        bool all_valid_ids = true;
        
        // iterate through all marker
        for (const auto& m : sorted_markers) {
            bool is_target = false;
            // iterate through all target id
            for (int t_id : target_ids_) {
                if (m.id == t_id) {
                    is_target = true;
                    break;
                }
            }
            current_vote.push_back(is_target ? 1 : 0);
        }

        RCLCPP_ERROR(this->get_logger(), "Current vote: [%d, %d, %d, %d]", 
            current_vote[0], current_vote[1], current_vote[2], current_vote[3]);

        // Phase 3: Voting
        vote_buffer_.push_back(current_vote);
        if (vote_buffer_.size() > (size_t)votes_needed_) {
            vote_buffer_.pop_front();
        }

        if (vote_buffer_.size() == (size_t)votes_needed_) {
            if (check_consensus()) {
                 publish_result(vote_buffer_.front());
                 // Optional: task_completed_ = true; 
                 vote_buffer_.clear(); // Reset buffer after success to avoid rapid re-triggers
            } else {
                // A strict sliding window is better than clearing fully, TODO
                if (!check_consensus()) {
                     vote_buffer_.clear();
                }
            }
        }
    }

    bool check_consensus() {
        if (vote_buffer_.empty()) return false;
        const auto& first = vote_buffer_.front();
        for (const auto& v : vote_buffer_) {
            if (v != first) return false;
        }
        return true;
    }

    void publish_result(const std::vector<int8_t>& result) {
        std_msgs::msg::Int8MultiArray msg;
        msg.data = result;
        mask_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Consensus reached! Published mask: [%d, %d, %d, %d]", 
            result[0], result[1], result[2], result[3]);
    }

    std::string team_color_;
    std::vector<int> target_ids_;
    int votes_needed_;
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int8MultiArray>::SharedPtr mask_pub_;
    
    std::deque<std::vector<int8_t>> vote_buffer_;
    bool task_completed_ = false; // Could be used if we want single-shot behavior
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoRowScannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
