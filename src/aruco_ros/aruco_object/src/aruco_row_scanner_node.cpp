    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <std_msgs/msg/int16.hpp>
    #include <std_msgs/msg/int32_multi_array.hpp>
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
            // Declare camera_position FIRST (before using it for topics)
            this->declare_parameter<std::string>("camera_position", "front");
            camera_position_ = this->get_parameter("camera_position").as_string();
            // Build namespaced topic name based on camera position
            // Topic format: /{camera_namespace}/{camera_name}/color/image_rect_raw
            // Both camera_namespace and camera_name are set to camera_position in launch
            std::string image_topic = "/" + camera_position_ + "/" + camera_position_ + "/color/image_rect_raw";
            RCLCPP_INFO(this->get_logger(), "Scanner [%s] (side=%d) subscribing to: %s", camera_position_.c_str(), my_side_, image_topic.c_str());

            // Map camera position string to dock_side number
            // dock_side: 0=front, 1=right, 2=back, 3=left 
            position_to_side_ = {{"front", 0}, {"right", 1}, {"back", 2}, {"left", 3}};
            my_side_ = position_to_side_[camera_position_];
            
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

            // Initialize ArUco dictionary
            dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            detector_params_ = cv::aruco::DetectorParameters::create();

            // Image subscriber with namespaced topic
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                image_topic, 10,
                std::bind(&ArucoRowScannerNode::image_callback, this, std::placeholders::_1));

            // Publisher: /robot/vision/hazelnut/flip (Int32MultiArray with 5 elements)
            flip_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/robot/vision/hazelnut/flip", 10);
            
            // Subscribe to dock_side topic for multi-camera activation control
            dock_side_sub_ = this->create_subscription<std_msgs::msg::Int16>(
                "/robot/dock_side", 10,
                [this](std_msgs::msg::Int16::SharedPtr msg) { 
                    active_side_ = msg->data;
                    bool is_active = (active_side_ == my_side_);
                RCLCPP_INFO(this->get_logger(), "Dock side: %d -> %s", msg->data, is_active ? "ACTIVE" : "dormant");
                });
                
            RCLCPP_INFO(this->get_logger(), "Scanner [%s] (side=%d) started for team: %s. Votes needed: %d", camera_position_.c_str(), my_side_, team_color_.c_str(), votes_needed_);
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
            // Multi-camera activation: dormant by default until dock_side matches our side
            if (active_side_ != my_side_) {
                return;
            }

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
                float y;
            };

            std::vector<Marker> sorted_markers;
            // calculate center y of each marker
            for (size_t i = 0; i < ids.size(); ++i) {
                float cy = 0;
                for (const auto& p : corners[i]) cy += p.y;
                cy /= 4.0;
                sorted_markers.push_back({ids[i], cy});
            }
            
            // sort by ascending y (small to large) -> Top to Bottom in image
            std::sort(sorted_markers.begin(), sorted_markers.end(), 
                [](const Marker& a, const Marker& b) { return a.y < b.y; });

            // Generate Instant Array (4 flip values)
            std::vector<int32_t> current_vote;
            
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
                // 0 = NO NEED to flip, 1 = NEED to flip
                current_vote.push_back(is_target ? 1 : 0);
            }

            RCLCPP_INFO(this->get_logger(), "Current vote: [%d, %d, %d, %d]", current_vote[0], current_vote[1], current_vote[2], current_vote[3]);

            // Phase 3: Voting
            vote_buffer_.push_back(current_vote);
            if (vote_buffer_.size() > (size_t)votes_needed_) {
                vote_buffer_.pop_front();
            }

            if (vote_buffer_.size() == (size_t)votes_needed_) {
                if (check_consensus()) {
                    publish_result(vote_buffer_.front());
                    vote_buffer_.clear(); // Reset buffer after success to avoid rapid re-triggers
                } else {
                    // Clear buffer if no consensus
                    vote_buffer_.clear();
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

        void publish_result(const std::vector<int32_t>& result) {
            // Format: [flip0, flip1, flip2, flip3, side]
            std_msgs::msg::Int32MultiArray msg;
            msg.data.resize(5);
            msg.data[0] = result[0];  // hazelnut 0
            msg.data[1] = result[1];  // hazelnut 1
            msg.data[2] = result[2];  // hazelnut 2
            msg.data[3] = result[3];  // hazelnut 3
            msg.data[4] = my_side_;   // which side to do the action
            
            flip_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Consensus reached! Published flip: [%d, %d, %d, %d, side=%d]", 
                result[0], result[1], result[2], result[3], my_side_);
        }

        std::string team_color_;
        std::vector<int> target_ids_;
        int votes_needed_;
        
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
        // pub, sub
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr flip_pub_;
        
        std::deque<std::vector<int32_t>> vote_buffer_;
        std::string camera_position_;
        int16_t my_side_ = -1;           // This node's side number
        int16_t active_side_ = -1;       // Currently active side from /robot/dock_side
        std::map<std::string, int16_t> position_to_side_;
        rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr dock_side_sub_;
    };

    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ArucoRowScannerNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
