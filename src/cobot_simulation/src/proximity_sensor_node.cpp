#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class ProximitySensorNode : public rclcpp::Node {
public:
  ProximitySensorNode() : Node("proximity_sensor_node") {
    publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("proximity", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ProximitySensorNode::timer_callback, this));

    // Random number generator for distance
    std::random_device rd;
    gen_ = std::mt19937(rd());
    dist_ = std::uniform_real_distribution<>(200.0, 1000.0);
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::Float64();
    message.data = dist_(gen_);
    RCLCPP_INFO(this->get_logger(), "Publishing proximity: '%f'", message.data);
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dist_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProximitySensorNode>());
  rclcpp::shutdown();
  return 0;
}
