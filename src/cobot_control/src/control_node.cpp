#include "cobot_control/speed_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>

class ControlNode : public rclcpp::Node {
public:
  ControlNode() : Node("control_node") {
    // Initialize the core controller
    controller_ = std::make_unique<cobot_control::SpeedController>();

    // Subscription to proximity sensor data
    proximity_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        "proximity", 10,
        std::bind(&ControlNode::proximity_callback, this,
                  std::placeholders::_1));

    // Subscription to emergency stop signal
    estop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10,
        std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));

    // Publisher for the current speed state
    state_publisher_ =
        this->create_publisher<std_msgs::msg::String>("speed_state", 10);
  }

private:
  void proximity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    auto new_state = controller_->process_proximity(msg->data);
    publish_state(new_state, msg->data);
  }

  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    auto new_state = controller_->process_emergency_stop(msg->data);
    if (msg->data == true) {
      RCLCPP_INFO(this->get_logger(), "Estop triggered ");
    }

    publish_state(new_state, -1.0);
  }

  void publish_state(cobot_control::SpeedState state,
                     const _Float64 proximity = -1.0) {
    auto message = std_msgs::msg::String();
    message.data = to_string(state);
    RCLCPP_INFO(this->get_logger(), "Current State: %s, Proximity: %.2f",
                message.data.c_str(), proximity);
    state_publisher_->publish(message);
  }

  std::string to_string(cobot_control::SpeedState state) {
    switch (state) {
    case cobot_control::SpeedState::FULL_SPEED:
      return "FULL_SPEED";
    case cobot_control::SpeedState::SLOW:
      return "SLOW";
    case cobot_control::SpeedState::STOP:
      return "STOP";
    case cobot_control::SpeedState::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    default:
      return "UNKNOWN";
    }
  }

  std::unique_ptr<cobot_control::SpeedController> controller_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      proximity_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
