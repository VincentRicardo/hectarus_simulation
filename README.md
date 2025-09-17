# Six Legged Robot - Hectarus - Simulation with ROS2 Jazzy and Gazebo Harmonic
A ROS2-based simulation of a six-legged (hexapod) robot using Gazebo Harmonic, built for research and learning in robotics and mechatronics.

This project features a real-time teleoperation using keyboard input, torque monitoring for each joint, and IMU-based static self-stabilization.

## Features
### 3D Hexapod Model

### Keyboard Teleoperation

### Torque Monitoring

### Basic IMU Integration


## Prerequisites

## Installation
1. Clone the repository
```ruby
git clone https://github.com/VincentRicardo/hectarus_simulation.git
```
2. Build the workspace
```ruby
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage
1. Launch the Simulation
```ruby
ros2 launch hectarus_controller controller.launch.py
```
2. Control the Robot
Open another terminal and run the keyboard teleoperation node:
```ruby
ros2 run hectarus_controller hectarus_teleop_key
``` 
4. Monitor Joint Torque
Open another terminal and run the torque monitoring to monitor each joint torque
```ruby
ros2 launch hectarus_controller torque.launch.py
```
5. Activate Imu Self-Stabilization
To activate self-stabilization using IMU, open another terminal and set use_imu parameter to true
```ruby
ros2 param set /gazebo_joint_publisher use_imu true
```

