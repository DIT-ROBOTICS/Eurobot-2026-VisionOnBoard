#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>


class FeedbackAll : public rclcpp::Node {
public:
    FeedbackAll() : Node("feedback_all_node") {

        this->declare_parameter<std::string>("left_topic", "/left/left/obj_distance");
        this->declare_parameter<std::string>("right_topic", "/right/right/obj_distance");
        this->declare_parameter<std::string>("front_topic", "/front/front/obj_distance");
        this->declare_parameter<std::string>("back_topic", "/back/back/obj_distance");
        this->declare_parameter<std::string>("success_topic", "/robot/vision/onTakeSuccess");
        this->declare_parameter<std::string>("take_topic", "/robot/on_take");
        this->declare_parameter<std::string>("put_topic", "/robot/on_put");
        this->declare_parameter<int32_t>("take_threshold_mm", 100);
        this->declare_parameter<int32_t>("put_threshold_mm", 100);

        left_topic_ = this->get_parameter("left_topic").as_string();
        right_topic_ = this->get_parameter("right_topic").as_string();
        front_topic_ = this->get_parameter("front_topic").as_string();
        back_topic_ = this->get_parameter("back_topic").as_string();
        take_topic_ = this->get_parameter("take_topic").as_string();
        put_topic_ = this->get_parameter("put_topic").as_string();

        take_threshold_mm_ = this->get_parameter("take_threshold_mm").as_int();
        put_threshold_mm_ = this->get_parameter("put_threshold_mm").as_int();

        success_topic_ = this->get_parameter("success_topic").as_string();
        success_publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(success_topic_, 10);

        left_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            left_topic_, 10, 
            std::bind(&FeedbackAll::left_dist_callback, this, std::placeholders::_1));
        right_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            right_topic_, 10, 
            std::bind(&FeedbackAll::right_dist_callback, this, std::placeholders::_1));
        front_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            front_topic_, 10, 
            std::bind(&FeedbackAll::front_dist_callback, this, std::placeholders::_1));
        back_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            back_topic_, 10, 
            std::bind(&FeedbackAll::back_dist_callback, this, std::placeholders::_1));
        mission_take_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            take_topic_, 10, 
            std::bind(&FeedbackAll::mission_take_callback, this, std::placeholders::_1));
        mission_put_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
            put_topic_, 10, 
            std::bind(&FeedbackAll::mission_put_callback, this, std::placeholders::_1));
    }

private:
    void mission_take_callback(const std_msgs::msg::Int16::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received take mission signal: %d", msg->data);
        current_mission_state_ = MissionState::TAKING;
    }

    void mission_put_callback(const std_msgs::msg::Int16::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received put mission signal: %d", msg->data);
        current_mission_state_ = MissionState::PUTTING;
    }

    void left_dist_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if(current_mission_state_ == MissionState::TAKING) {
            distances_[LEFT] = (msg->data < take_threshold_mm_) ? 1 : 0;
        } else if(current_mission_state_ == MissionState::PUTTING) {
            distances_[LEFT] = (msg->data < put_threshold_mm_) ? 1 : 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Received left distance: %d mm", msg->data);
        check_success();
    }

    void right_dist_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if(current_mission_state_ == MissionState::TAKING) {
            distances_[RIGHT] = (msg->data < take_threshold_mm_) ? 1 : 0;
        } else if(current_mission_state_ == MissionState::PUTTING) {
            distances_[RIGHT] = (msg->data < put_threshold_mm_) ? 1 : 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Received right distance: %d mm", msg->data);
        check_success();
    }

    void front_dist_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if(current_mission_state_ == MissionState::TAKING) {
            distances_[FRONT] = (msg->data < take_threshold_mm_) ? 1 : 0;
        } else if(current_mission_state_ == MissionState::PUTTING) {
            distances_[FRONT] = (msg->data < put_threshold_mm_) ? 1 : 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Received front distance: %d mm", msg->data);
        check_success();
    }

    void back_dist_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        if(current_mission_state_ == MissionState::TAKING) {
            distances_[BACK] = (msg->data < take_threshold_mm_) ? 1 : 0;
        } else if(current_mission_state_ == MissionState::PUTTING) {
            distances_[BACK] = (msg->data < put_threshold_mm_) ? 1 : 0;
        }
        // RCLCPP_INFO(this->get_logger(), "Received back distance: %d mm", msg->data);
        check_success();
    }

    void check_success() {
        std_msgs::msg::Int32MultiArray result_msg;
        result_msg.data = distances_;
        success_publisher_->publish(result_msg);
        // RCLCPP_INFO(this->get_logger(), "Published success status: %d, %d, %d, %d", distances_[FRONT], distances_[RIGHT], distances_[BACK], distances_[LEFT]);
        RCLCPP_INFO(this->get_logger(), "Current mission state: %s", (current_mission_state_ == MissionState::TAKING) ? "TAKING" : (current_mission_state_ == MissionState::PUTTING) ? "PUTTING" : "UNKNOWN");
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr front_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr back_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_take_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mission_put_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr success_publisher_;
    std::string left_topic_;
    std::string right_topic_;
    std::string front_topic_;
    std::string back_topic_;
    std::string take_topic_;
    std::string put_topic_;
    std::string success_topic_;
    std::int32_t take_threshold_mm_;
    std::int32_t put_threshold_mm_;
    std::int32_t current_mission_state_ = -1; // -1: unknown, 0: taking, 1: putting
    std::vector<std::int32_t> distances_{0, 0, 0, 0}; // front, right, back, left
    enum Direction { FRONT = 0, RIGHT = 1, BACK = 2, LEFT = 3 };
    enum MissionState { TAKING = 0, PUTTING = 1 };
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeedbackAll>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}