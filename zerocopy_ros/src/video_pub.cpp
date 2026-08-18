#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class VideoPub : public rclcpp::Node {
public:
    VideoPub() : Node("video_pub") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .durability_volatile()
                   .best_effort()
                   .liveliness(rclcpp::LivelinessPolicy::Automatic)
                   .liveliness_lease_duration(1000ms)
                   .deadline(500ms);

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);

        video_path_ = std::string(getenv("HOME")) + "/test_video.mp4";
        cap_.open(video_path_);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "❌ Error: Could not open %s", video_path_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ Publisher Started. QoS: Auto-Liveliness (1s)");

        cap_ >> frame_;
        if (!frame_.empty()) frame_size_ = frame_.total() * frame_.elemSize();

        timer_ = this->create_wall_timer(33ms, std::bind(&VideoPub::publish_frame, this));
    }

private:
    void publish_frame() {
        cap_ >> frame_;
        if (frame_.empty()) {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            cap_ >> frame_;
        }

        rclcpp::Time now = this->now();

        // 1. Try Shared Memory
        if (publisher_->can_loan_messages()) {
            auto loaned_msg = publisher_->borrow_loaned_message();
            sensor_msgs::msg::Image& msg = loaned_msg.get();

            msg.header.stamp = now;
            msg.header.frame_id = std::to_string(frame_count_++);
            msg.height = frame_.rows;
            msg.width = frame_.cols;
            msg.encoding = "bgr8";
            msg.is_bigendian = false;
            msg.step = frame_.step;
            
            if (msg.data.size() != frame_size_) msg.data.resize(frame_size_);
            memcpy(msg.data.data(), frame_.data, frame_size_);

            publisher_->publish(std::move(loaned_msg));
        } 
        // 2. Fallback
        else {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = now;
            msg.header.frame_id = std::to_string(frame_count_++);
            msg.height = frame_.rows;
            msg.width = frame_.cols;
            msg.encoding = "bgr8";
            msg.is_bigendian = false;
            msg.step = frame_.step;
            msg.data.resize(frame_size_);
            memcpy(msg.data.data(), frame_.data, frame_size_);
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::Mat frame_;
    size_t frame_size_;
    std::string video_path_;
    int frame_count_ = 0;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPub>());
    rclcpp::shutdown();
    return 0;
}
