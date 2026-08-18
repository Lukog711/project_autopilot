#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <filesystem>
#include <fstream>
#include <future>
#include <string>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class HealthMonitorNode : public rclcpp::Node {
public:
    HealthMonitorNode() : Node("health_monitor_node"), system_ok_(true), proc_check_pending_(false) {
        start_time_ = this->now();
        last_data_time_ = this->now();
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("/system_health", 10);

        rclcpp::QoS watch_qos(10);
        watch_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        watch_qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
        watch_qos.liveliness_lease_duration(200ms);

        rclcpp::SubscriptionOptions opts_dist;
        opts_dist.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo & info) {
            if (info.alive_count == 0 && is_past_grace_period()) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: Liveliness LOST on /closest_distance!");
                system_ok_ = false;
            }
        };
        sub_dist_ = this->create_subscription<std_msgs::msg::Float32>("/closest_distance", watch_qos, [this](const std_msgs::msg::Float32::SharedPtr) { last_data_time_ = this->now(); }, opts_dist);

        rclcpp::SubscriptionOptions opts_safe;
        opts_safe.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo & info) {
            if (info.alive_count == 0 && is_past_grace_period()) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: Liveliness LOST on /safe_braking_distance!");
                system_ok_ = false;
            }
        };
        sub_safe_ = this->create_subscription<std_msgs::msg::Float32>("/safe_braking_distance", watch_qos, [](const std_msgs::msg::Float32::SharedPtr) {}, opts_safe);

        rclcpp::SubscriptionOptions opts_ui;
        opts_ui.event_callbacks.liveliness_callback = [this](rclcpp::QOSLivelinessChangedInfo & info) {
            if (info.alive_count == 0 && is_past_grace_period()) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: Liveliness LOST on /ui_status!");
                system_ok_ = false;
            }
        };
        sub_ui_ = this->create_subscription<std_msgs::msg::Bool>("/ui_status", watch_qos, [](const std_msgs::msg::Bool::SharedPtr) {}, opts_ui);

        timer_ = this->create_wall_timer(100ms, std::bind(&HealthMonitorNode::check_health, this));
    }

private:
    bool is_past_grace_period() const { return (this->now() - start_time_).seconds() > 5.0; }

    bool check_daemon_running(const std::string & process_name) {
        try {
            for (const auto & entry : fs::directory_iterator("/proc")) {
                if (!entry.is_directory()) continue;
                std::ifstream comm(entry.path() / "comm");
                std::string name;
                if (comm >> name && name == process_name) return true;
            }
        } catch (...) {}
        return false;
    }
    
    void publish_health() {
        if (publisher_->can_loan_messages()) {
            auto loaned_msg = publisher_->borrow_loaned_message();
            loaned_msg.get().data = system_ok_;
            publisher_->publish(std::move(loaned_msg));
        } else {
            std_msgs::msg::Bool msg; msg.data = system_ok_;
            publisher_->publish(msg);
        }
    }

    void check_health() {
        if (!is_past_grace_period()) {
            last_data_time_ = this->now();
            publish_health();
            return;
        }

        if ((this->now() - last_data_time_).seconds() > 0.5) {
            RCLCPP_ERROR(this->get_logger(), "FATAL: LiDAR sensor silent for > 500 ms!");
            system_ok_ = false;
        }

        if (!proc_check_pending_) {
            proc_check_pending_ = true;
            proc_future_ = std::async(std::launch::async, [this]() { return check_daemon_running("iox-roudi"); });
        }
        if (proc_future_.valid() && proc_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            bool roudi_ok = proc_future_.get();
            proc_check_pending_ = false;

            if (!roudi_ok) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: iox-roudi daemon is DEAD!");
                system_ok_ = false;
            }

            auto nodes = this->get_node_names();
            bool found = false;
            for (const auto & n : nodes) {
                if (n == "/livox_lidar_publisher" || n == "livox_lidar_publisher") { found = true; break; }
            }
            if (!found) {
                RCLCPP_ERROR(this->get_logger(), "FATAL: livox_lidar_publisher node is DEAD!");
                system_ok_ = false;
            }
        }

        publish_health();
    }
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_dist_, sub_safe_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ui_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_, last_data_time_;
    bool system_ok_;
    std::future<bool> proc_future_;
    bool proc_check_pending_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthMonitorNode>());
    rclcpp::shutdown(); return 0;
}
