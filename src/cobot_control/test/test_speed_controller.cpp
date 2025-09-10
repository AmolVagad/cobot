#include "cobot_control/speed_controller.hpp"
#include <gtest/gtest.h>

using namespace cobot_control;

// Test fixture for SpeedController tests
class SpeedControllerTest : public ::testing::Test {
protected:
  void SetUp() override { controller = std::make_unique<SpeedController>(); }

  std::unique_ptr<SpeedController> controller;

  // Define the same constants as in your header file
  static constexpr double full_speed_threshold_ = 800.0; // mm
  static constexpr double slow_speed_threshold_ = 400.0; // mm
  static constexpr double hysteresis_margin_ = 50.0;     // mm
};

// ===== BASIC INITIALIZATION TESTS =====

TEST_F(SpeedControllerTest, InitialStateIsFullSpeed) {
  EXPECT_EQ(controller->get_current_state(), SpeedState::FULL_SPEED);
}

TEST_F(SpeedControllerTest, ResetSetsStateToFullSpeed) {
  // Change to a different state first
  controller->process_proximity(300.0); // Should go to SLOW then STOP
  controller->process_proximity(300.0); // Now in STOP
  ASSERT_EQ(controller->get_current_state(), SpeedState::STOP);

  // Reset should return to FULL_SPEED
  controller->reset();
  EXPECT_EQ(controller->get_current_state(), SpeedState::FULL_SPEED);
}

// ===== PROXIMITY TRANSITION TESTS =====

TEST_F(SpeedControllerTest, FullSpeedToSlowTransition) {
  // Start in FULL_SPEED
  EXPECT_EQ(controller->get_current_state(), SpeedState::FULL_SPEED);

  // Object enters slow zone (< 800mm)
  EXPECT_EQ(controller->process_proximity(799.0), SpeedState::SLOW);
}

TEST_F(SpeedControllerTest, FullSpeedRemainsAtLargeDistance) {
  // Should stay in FULL_SPEED when distance >= 800mm
  EXPECT_EQ(controller->process_proximity(1000.0), SpeedState::FULL_SPEED);
}

TEST_F(SpeedControllerTest, SlowToStopTransition) {
  // First transition to SLOW
  controller->process_proximity(600.0);
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);

  // Then to STOP when < 400mm
  EXPECT_EQ(controller->process_proximity(399.0), SpeedState::STOP);
}

TEST_F(SpeedControllerTest, SlowToFullSpeedTransition) {
  // First go to SLOW
  controller->process_proximity(600.0);
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);

  // Return to FULL_SPEED with hysteresis (> 800 + 50 = 850)
  EXPECT_EQ(controller->process_proximity(851.0), SpeedState::FULL_SPEED);
}

TEST_F(SpeedControllerTest, StopToSlowTransition) {
  // First go to STOP
  controller->process_proximity(600.0); // SLOW
  controller->process_proximity(300.0); // STOP
  ASSERT_EQ(controller->get_current_state(), SpeedState::STOP);

  // Return to SLOW with hysteresis (> 400 + 50 = 450)
  EXPECT_EQ(controller->process_proximity(451.0), SpeedState::SLOW);
}

// ===== HYSTERESIS TESTS =====

TEST_F(SpeedControllerTest, HysteresisPreventsBouncing_SlowToFull) {
  // Go to SLOW state
  controller->process_proximity(600.0);
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);

  // Distance just above threshold but within hysteresis should stay SLOW
  // 800 + 20 = 820, which is < 800 + 50 = 850, so should stay SLOW
  EXPECT_EQ(controller->process_proximity(820.0), SpeedState::SLOW);

  // Only when above hysteresis threshold should it go to FULL_SPEED
  EXPECT_EQ(controller->process_proximity(851.0), SpeedState::FULL_SPEED);
}

TEST_F(SpeedControllerTest, HysteresisPreventsBouncing_StopToSlow) {
  // Go to STOP state
  controller->process_proximity(600.0); // SLOW
  controller->process_proximity(300.0); // STOP
  ASSERT_EQ(controller->get_current_state(), SpeedState::STOP);

  // Distance just above threshold but within hysteresis should stay STOP
  // 400 + 30 = 430, which is < 400 + 50 = 450, so should stay STOP
  EXPECT_EQ(controller->process_proximity(430.0), SpeedState::STOP);

  // Only when above hysteresis threshold should it go to SLOW
  EXPECT_EQ(controller->process_proximity(451.0), SpeedState::SLOW);
}

// ===== EMERGENCY STOP TESTS =====

TEST_F(SpeedControllerTest, EmergencyStopFromAnyState) {
  // Test emergency stop from FULL_SPEED
  EXPECT_EQ(controller->get_current_state(), SpeedState::FULL_SPEED);
  EXPECT_EQ(controller->process_emergency_stop(true),
            SpeedState::EMERGENCY_STOP);

  // Reset and test from SLOW
  controller->reset();
  controller->process_proximity(600.0); // SLOW
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);
  EXPECT_EQ(controller->process_emergency_stop(true),
            SpeedState::EMERGENCY_STOP);

  // Reset and test from STOP
  controller->reset();
  controller->process_proximity(600.0); // SLOW
  controller->process_proximity(300.0); // STOP
  ASSERT_EQ(controller->get_current_state(), SpeedState::STOP);
  EXPECT_EQ(controller->process_emergency_stop(true),
            SpeedState::EMERGENCY_STOP);
}

TEST_F(SpeedControllerTest, EmergencyStopOverridesProximity) {
  // Activate E-Stop
  EXPECT_EQ(controller->process_emergency_stop(true),
            SpeedState::EMERGENCY_STOP);

  // Proximity should now be ignored - state should remain EMERGENCY_STOP
  EXPECT_EQ(controller->process_proximity(1000.0), SpeedState::EMERGENCY_STOP);
  EXPECT_EQ(controller->process_proximity(500.0), SpeedState::EMERGENCY_STOP);
  EXPECT_EQ(controller->process_proximity(100.0), SpeedState::EMERGENCY_STOP);
}

TEST_F(SpeedControllerTest, ClearingEmergencyStopResetsToFullSpeed) {
  // Trigger E-Stop
  controller->process_emergency_stop(true);
  ASSERT_EQ(controller->get_current_state(), SpeedState::EMERGENCY_STOP);

  // Clear E-Stop should reset to FULL_SPEED
  EXPECT_EQ(controller->process_emergency_stop(false), SpeedState::FULL_SPEED);

  // Subsequent 'false' signals should not change the state
  EXPECT_EQ(controller->process_emergency_stop(false), SpeedState::FULL_SPEED);
}

// ===== EDGE CASE TESTS =====

TEST_F(SpeedControllerTest, ExactThresholdValues) {
  // Test exact threshold values - your controller uses < comparisons, so exact
  // values stay in current state
  EXPECT_EQ(controller->process_proximity(800.0),
            SpeedState::FULL_SPEED); // Exactly at threshold stays FULL_SPEED
  EXPECT_EQ(controller->process_proximity(799.9),
            SpeedState::SLOW); // Just below threshold goes to SLOW

  // From SLOW, test exact threshold to STOP
  EXPECT_EQ(controller->process_proximity(400.0),
            SpeedState::SLOW); // Exactly at threshold stays SLOW
  EXPECT_EQ(controller->process_proximity(399.9),
            SpeedState::STOP); // Just below threshold goes to STOP
}

TEST_F(SpeedControllerTest, NegativeDistanceHandling) {
  // Test with negative distances (sensor error case)
  EXPECT_EQ(controller->process_proximity(-10.0),
            SpeedState::SLOW); // Should transition to SLOW first
  EXPECT_EQ(controller->process_proximity(-10.0),
            SpeedState::STOP); // Then to STOP
}

TEST_F(SpeedControllerTest, VeryLargeDistanceHandling) {
  // Test with very large distances
  controller->process_proximity(600.0); // Go to SLOW first
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);

  controller->process_proximity(10000.0); // Very large distance
  EXPECT_EQ(controller->get_current_state(), SpeedState::FULL_SPEED);
}

// ===== COMPLEX SCENARIO TESTS =====

TEST_F(SpeedControllerTest, ComplexProximityScenario) {
  // Simulate a complex approach and retreat scenario

  // Object approaching from far
  EXPECT_EQ(controller->process_proximity(1000.0), SpeedState::FULL_SPEED);

  // Object enters slow zone
  EXPECT_EQ(controller->process_proximity(700.0), SpeedState::SLOW);

  // Object continues approaching
  EXPECT_EQ(controller->process_proximity(500.0), SpeedState::SLOW);

  // Object enters stop zone
  EXPECT_EQ(controller->process_proximity(300.0), SpeedState::STOP);

  // Object moves away slightly but not enough to clear hysteresis
  EXPECT_EQ(controller->process_proximity(430.0), SpeedState::STOP);

  // Object moves away enough to enter slow zone
  EXPECT_EQ(controller->process_proximity(500.0), SpeedState::SLOW);

  // Object moves away but not enough to clear hysteresis for full speed
  EXPECT_EQ(controller->process_proximity(830.0), SpeedState::SLOW);

  // Object moves away enough to return to full speed
  EXPECT_EQ(controller->process_proximity(900.0), SpeedState::FULL_SPEED);
}

TEST_F(SpeedControllerTest, EmergencyStopDuringProximitySequence) {
  // Start normal proximity sequence
  controller->process_proximity(600.0); // SLOW
  ASSERT_EQ(controller->get_current_state(), SpeedState::SLOW);

  // Emergency stop during sequence
  EXPECT_EQ(controller->process_emergency_stop(true),
            SpeedState::EMERGENCY_STOP);

  // Try proximity changes during emergency stop
  EXPECT_EQ(controller->process_proximity(1000.0), SpeedState::EMERGENCY_STOP);

  // Clear emergency stop
  EXPECT_EQ(controller->process_emergency_stop(false), SpeedState::FULL_SPEED);

  // Resume normal operation
  EXPECT_EQ(controller->process_proximity(600.0), SpeedState::SLOW);
}

// Main function for running tests
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}