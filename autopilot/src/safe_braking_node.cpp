#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class SafeBrakingNode : public rclcpp::Node {
public:
    SafeBrakingNode() : Node("safe_braking_node") {
        rclcpp::QoS qos(10);
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
        qos.liveliness_lease_duration(200ms);

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/safe_braking_distance", qos);

        constexpr float v_mph = 5.0f;
        constexpr float v_ms = v_mph * 0.44704f;
        constexpr float decel_ms2 = 15.0f;
        constexpr float t_reaction_s = 0.2f;
        d_total_ = (v_ms * t_reaction_s) + (v_ms * v_ms) / (2.0f * decel_ms2);

        timer_ = this->create_wall_timer(100ms, std::bind(&SafeBrakingNode::publish_distance, this));
    }
private:
    void publish_distance() {
        // --- TRUE ZERO-COPY PUBLISH ---
        if (publisher_->can_loan_messages()) {
            auto loaned_msg = publisher_->borrow_loaned_message();
            loaned_msg.get().data = d_total_;
            publisher_->publish(std::move(loaned_msg));
        } else {
            std_msgs::msg::Float32 msg; msg.data = d_total_;
            publisher_->publish(msg);
        }
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float d_total_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeBrakingNode>());
    rclcpp::shutdown(); return 0;
}
