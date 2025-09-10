#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>

// Function to check for keyboard input without blocking
int getch() {
  static struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

class EmergencyStopNode : public rclcpp::Node {
public:
  EmergencyStopNode() : Node("emergency_stop_node"), is_estopped_(false) {
    publisher_ =
        this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&EmergencyStopNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Emergency Stop Node has been initialized.");
    RCLCPP_INFO(this->get_logger(), "Press 'e' to TRIGGER emergency stop.");
    RCLCPP_INFO(this->get_logger(), "Press 'c' to CLEAR emergency stop.");
  }

private:
  void timer_callback() {
    int key = getch(); // Using a non-blocking getch
    auto message = std_msgs::msg::Bool();

    if (key == 'e' || key == 'E') {
      if (!is_estopped_) {
        is_estopped_ = true;
        RCLCPP_WARN(this->get_logger(), "Emergency Stop TRIGGERED!");
      }
    } else if (key == 'c' || key == 'C') {
      if (is_estopped_) {
        is_estopped_ = false;
        RCLCPP_INFO(this->get_logger(),
                    "Emergency Stop CLEARED. Resuming normal operation.");
      }
    }

    message.data = is_estopped_;
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_estopped_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStopNode>());
  rclcpp::shutdown();
  return 0;
}
