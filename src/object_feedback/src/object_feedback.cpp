#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ObjectFeedback : public rclcpp::Node {
public:
    ObjectFeedback() : Node("object_feedback_node") {
        this->declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
        this->declare_parameter<std::string>("dist_topic", "/camera/camera/obj_distance");
        depth_topic_ = this->get_parameter("depth_topic").as_string();
        dist_topic_ = this->get_parameter("dist_topic").as_string();
        dist_publisher_ = this->create_publisher<std_msgs::msg::Int32>(dist_topic_, 10);
        depth_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10, 
            std::bind(&ObjectFeedback::depth_img_callback, this, std::placeholders::_1));
    }

private:
    void depth_img_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat depth_frame = cv_ptr->image;

            int target_x = depth_frame.cols / 4;
            double sum = 0.0;
            int count = 0;

            for(int v = 0; v < depth_frame.rows; ++v) {
                if (msg->encoding == "16UC1") {
                    uint16_t depth_value = depth_frame.at<uint16_t>(v, target_x);
                    if (depth_value > 0) {
                        sum += depth_value;
                        count++;
                    }
                } 
                else if (msg->encoding == "32FC1") {
                    float depth_value = depth_frame.at<float>(v, target_x);
                    if (depth_value > 0.0f) {
                        sum += depth_value;
                        count++;
                    }
                }
            }

            if (count > 0) {
                double average = sum / count;
                std_msgs::msg::Int32 out_msg;
                out_msg.data = static_cast<int32_t>(average);
                dist_publisher_->publish(out_msg);
                // RCLCPP_INFO(this->get_logger(), "Average depth at column %d: %.2f mm", target_x, average);
            }

            // cv::Point pt1(depth_frame.cols / 4, 0);
            // cv::Point pt2(depth_frame.cols / 4, depth_frame.rows);
            // cv::Scalar line_color(255, 0, 0);
            // int thickness = 2;
            // cv::line(depth_frame, pt1, pt2, line_color, thickness);

            // cv::imshow("Depth View", depth_frame * 100);
            // cv::waitKey(1);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dist_publisher_;
    std::string depth_topic_;
    std::string dist_topic_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown(); 
    return 0;
}