# Six Legged Robot - Hectarus - Simulation with ROS2 Jazzy and Gazebo Harmonic
A ROS2-based simulation of a six-legged (hexapod) robot using Gazebo Harmonic with visualization in RViz, built for research and learning in robotics and mechatronics.

The main purpose of this project is to analyze and simulated the best walking pattern for hexapod robot either flat terrain or uneven terrain such as stairs.

This project features a real-time teleoperation using keyboard input, torque monitoring for each joint, and IMU-based static self-stabilization.

## Features
### 3D Hexapod Model
A detailed Xacro-based 3D model of a six-legged robot generated and modified by fusion-to-urdf plugin. 
- Each leg is modeled with 3 joints (3 DOF) representing a spider leg.
![](/asset/walking_rviz.gif =500x500)
- The model includes [collision meshes and visual meshes](https://github.com/VincentRicardo/hectarus_simulation/blob/4ed12e8d3b8fdac9c984391c6abc5447614acf5c/ros_ws/src/hectarus_sim/description/hectarus_core.xacro#L21C5-L33C17) for realistic simulation in Gazebo

### Keyboard Teleoperation
The robot can be controlled in real-time using a simple keyboard interface with tripod gait implemented.

The supported motions are:
- Forward / Backward movement with W and X key
- Stop with S key
- Left / Right Strafing with Z and C key
- Rotational movement (turn left and right) with A and D for default turning degree and Q and E for smaller turning degree

This features include controlling robot forward and strafe distance with arrow keyboard.
![](/asset/teleopkey.png = 500x500)

### Torque Monitoring
Real-time monitoring of each joint torque using torque sensor in gazebo plugin to measures joint torque values during operation.
- Simulates the physical load experienced by each actuator as the robot moves.
- This torque monitoring can be used to analyze:
  1. Physical hardware torque needed to implement the robot into a hardware
  2. Optimizing gait design for energy efficiency
  3. Identifying potential failure points in hardware design
![](/asset/torque_monitoring.webm)

### Basic IMU Integration for Self-Stabilization
An Inertial Measurement Unit (IMU) is integrated into the simulation using gazebo sensor plugin to track the robot's roll, pitch and yaw value.
- The IMU is implemented until static self-stabilization.
- When the parameter "use_imu" is true, the robot automatically adjusts its leg heights to maintain its posture based on its terrain.
![](/asset/self_stabilization.gif = 500x500)

## Flaw

## Prerequisites
The ROS2, colcon and Gazebo needed to be installed on the PC to run the program.
1. The ROS2 Jazzy can be downloaded in this [link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) tutorial
2. The colcon can downloaded with this script
```ruby
sudo apt install python3-colcon-common-extensions
```
3. The Gazebo Harmonic can be downloaded in this [link](https://gazebosim.org/docs/harmonic/install_ubuntu/) tutorial

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
The robot then can be controller with W, A, S, D, to move forward, turn left and right and move backward. Q and E to turn left and right the robot with smaller degree. Z and C controlling the robot to move strafe. 
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

