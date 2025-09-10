# Cobot Proximity Speed Controller
This ROS2 workspace contains a modular system for controlling a collaborative robot's speed based on proximity sensor data and an emergency stop signal. The core control logic is implemented in a ROS-agnostic C++ library, ensuring portability and ease of testing. This project is compatible with ROS2 Foxy Fitzroy.

## Architecture
The system is divided into three main packages, following ROS2 best practices for separation of concerns:

```cobot_control```: Contains the core, ROS-agnostic speed control logic implemented as a C++ library. This library features a state machine with hysteresis to prevent rapid state changes. A ROS2 wrapper node (control_node) interfaces this library with the ROS2 ecosystem, subscribing to sensor and e-stop topics and publishing the robot's state.

```cobot_simulation```: Provides nodes to simulate hardware inputs for development and testing. It includes a proximity_sensor_node that publishes random distance data and an emergency_stop_node that allows toggling the e-stop signal via keyboard input.

```cobot_bringup```: Contains the main launch files to start the entire system, including the simulation and control nodes.

## Building the Workspace
Ensure you have a ROS2 Foxy Fitzroy environment sourced.

Clone this repository into your ROS2 workspace's src directory.

Navigate to the root of your workspace at cobot/src

### Build the packages using colcon:

```colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug```

### Run the tests as 
```colcon test --packages-select cobot_control --event-handlers console_direct+ --pytest-args -v```

## Running the System
Source your workspace's setup file from the root of the workspace:

```source install/setup.bash```

Use the main launch file to bring up all the nodes:

```ros2 launch cobot_bringup bringup.launch.py```

Two new terminal windows will open: one for the emergency stop input and one for the control node.

The control node terminal will log the current state of the cobot (FULL_SPEED, SLOW, STOP, EMERGENCY_STOP).

The emergency stop terminal will prompt you to press e to enable the emergency stop status and press c to clear emergency stop.

## State Machine Logic
The SpeedController class implements a state machine with the following states and transitions:

### FULL_SPEED: Default state. Active when distance > 800mm.

### SLOW: Active when distance is between 400mm and 800mm.

### STOP: Active when distance < 400mm.

### EMERGENCY_STOP: Overrides all other states when the emergency stop is active. The system returns to FULL_SPEED once the e-stop is cleared.

A hysteresis margin of 50mm is used to prevent rapid, flickering state changes when the proximity sensor reading is near a threshold.

Code Generator used : Gemini for code cleanup, unit tests, documentation and ROS2 file templates 
