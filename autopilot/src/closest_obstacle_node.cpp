#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class ClosestObstacleNode : public rclcpp::Node {
public:
    ClosestObstacleNode() : Node("closest_obstacle_node") {
        rclcpp::QoS pub_qos(10);
        pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        pub_qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
        pub_qos.liveliness_lease_duration(200ms);
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/closest_distance", pub_qos);

        rclcpp::QoS sensor_qos(10);
        sensor_qos.best_effort();
        sensor_qos.durability_volatile();
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", sensor_qos, std::bind(&ClosestObstacleNode::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        const auto stamp = rclcpp::Time(msg->header.stamp);
        if ((this->now() - stamp).seconds() > 0.2) return;

        float min_dist = std::numeric_limits<float>::max();
        for (const auto & point : msg->points) {
            float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (dist > 0.05f && dist < min_dist) min_dist = dist;
        }

        if (min_dist != std::numeric_limits<float>::max()) {
            // --- TRUE ZERO-COPY PUBLISH ---
            if (publisher_->can_loan_messages()) {
                auto loaned_msg = publisher_->borrow_loaned_message();
                loaned_msg.get().data = min_dist;
                publisher_->publish(std::move(loaned_msg));
            } else {
                std_msgs::msg::Float32 out; out.data = min_dist;
                publisher_->publish(out);
            }
        }
    }
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClosestObstacleNode>());
    rclcpp::shutdown(); return 0;
}
