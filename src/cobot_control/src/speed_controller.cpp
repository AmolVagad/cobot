#include "cobot_control/speed_controller.hpp"

namespace cobot_control {

SpeedController::SpeedController() : current_state_(SpeedState::FULL_SPEED) {}

void SpeedController::reset() { current_state_ = SpeedState::FULL_SPEED; }

SpeedState SpeedController::get_current_state() const { return current_state_; }

SpeedState SpeedController::process_proximity(double distance) {
  // The state machine logic is only active if not in emergency stop
  if (current_state_ != SpeedState::EMERGENCY_STOP) {
    switch (current_state_) {
    case SpeedState::FULL_SPEED:
      if (distance < full_speed_threshold_) {
        current_state_ = SpeedState::SLOW;
      }
      break;

    case SpeedState::SLOW:
      if (distance > full_speed_threshold_ + hysteresis_margin_) {
        current_state_ = SpeedState::FULL_SPEED;
      } else if (distance < slow_speed_threshold_) {
        current_state_ = SpeedState::STOP;
      }
      break;

    case SpeedState::STOP:
      if (distance > slow_speed_threshold_ + hysteresis_margin_) {
        current_state_ = SpeedState::SLOW;
      }
      break;

    default:
      // Should not happen
      break;
    }
  }
  return current_state_;
}

SpeedState SpeedController::process_emergency_stop(bool estop_active) {
  if (estop_active) {
    current_state_ = SpeedState::EMERGENCY_STOP;
  } else {
    // If e-stop was active and is now cleared, reset to a safe state
    if (current_state_ == SpeedState::EMERGENCY_STOP) {
      reset();
    }
  }
  return current_state_;
}

} // namespace cobot_control
