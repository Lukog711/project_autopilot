#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>

using namespace std::chrono_literals;

class DecisionNode : public rclcpp::Node {
public:
    DecisionNode() : Node("decision_node"), running_(true), dist_(999.0f), safe_(0.0f), is_healthy_(true) {
        rclcpp::QoS matched_qos(10);
        matched_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        matched_qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
        matched_qos.liveliness_lease_duration(200ms);

        sub_dist_ = this->create_subscription<std_msgs::msg::Float32>("/closest_distance", matched_qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_); dist_ = msg->data;
        });
        sub_safe_ = this->create_subscription<std_msgs::msg::Float32>("/safe_braking_distance", matched_qos, [this](const std_msgs::msg::Float32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_); safe_ = msg->data;
        });
        sub_health_ = this->create_subscription<std_msgs::msg::Bool>("/system_health", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(data_mutex_); is_healthy_ = msg->data;
        });

        pub_status_ = this->create_publisher<std_msgs::msg::Bool>("/ui_status", matched_qos);

        heartbeat_timer_ = this->create_wall_timer(50ms, [this]() {
            if (pub_status_->can_loan_messages()) {
                auto loaned_msg = pub_status_->borrow_loaned_message();
                loaned_msg.get().data = true;
                pub_status_->publish(std::move(loaned_msg));
            } else {
                std_msgs::msg::Bool msg; msg.data = true;
                pub_status_->publish(msg);
            }
        });
    }
    ~DecisionNode() { running_ = false; cv::destroyAllWindows(); }

    void run_ui_loop() {
        cv::namedWindow("Autopilot HUD", cv::WINDOW_AUTOSIZE);
        while (running_ && rclcpp::ok()) {
            float cur_dist, cur_safe; bool cur_health;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                cur_dist = dist_; cur_safe = safe_; cur_health = is_healthy_;
            }

            cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            std::string status; cv::Scalar color;

            if (!cur_health) { color = cv::Scalar(0, 0, 255); status = "SYSTEM FAILURE - BRAKE!"; } 
            else if (cur_dist > cur_safe) { color = cv::Scalar(0, 200, 0); status = "SAFE - PATH CLEAR"; } 
            else { color = cv::Scalar(0, 0, 255); status = "DANGER - BRAKE NOW!"; }

            img.setTo(color);
            cv::putText(img, status, cv::Point(20, 150), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 4);
            char dist_str[64], safe_str[64];
            std::snprintf(dist_str, sizeof(dist_str), "Obstacle:    %.2f m", cur_dist);
            std::snprintf(safe_str,  sizeof(safe_str), "Braking Req: %.2f m", cur_safe);
            cv::putText(img, dist_str, cv::Point(50, 260), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);
            cv::putText(img, safe_str, cv::Point(50, 330), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 3);

            cv::imshow("Autopilot HUD", img); cv::waitKey(50);
        }
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dist_, sub_safe_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_health_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    std::mutex data_mutex_;
    std::atomic<bool> running_;
    float dist_, safe_; bool is_healthy_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionNode>();
    std::thread ros_thread([&node]() { rclcpp::spin(node); });
    node->run_ui_loop();
    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();
    return 0;
}
