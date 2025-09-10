#ifndef COBOT_CONTROL__SPEED_CONTROLLER_HPP_
#define COBOT_CONTROL__SPEED_CONTROLLER_HPP_

namespace cobot_control {

// Enum for the different speed states of the cobot
enum class SpeedState { FULL_SPEED, SLOW, STOP, EMERGENCY_STOP };

class SpeedController {
public:
  SpeedController();

  // Processes the proximity data and returns the new speed state
  SpeedState process_proximity(double distance);

  // Processes the emergency stop signal and returns the new speed state
  SpeedState process_emergency_stop(bool estop_active);

  // Resets the controller to its initial state
  void reset();

  // Gets the current speed state
  SpeedState get_current_state() const;

private:
  SpeedState current_state_;

  // Hysteresis thresholds to prevent rapid state changes
  const double full_speed_threshold_ = 800.0; // mm
  const double slow_speed_threshold_ = 400.0; // mm
  const double hysteresis_margin_ = 50.0;     // mm
};

} // namespace cobot_control

#endif // COBOT_CONTROL__SPEED_CONTROLLER_HPP_
