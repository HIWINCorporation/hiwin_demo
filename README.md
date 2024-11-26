# HIWIN ROS2 Example
[![License - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

This repository demonstrates the usage of `trajectory_msgs::msg::JointTrajectory` and the Kinematics and Dynamics Library (KDL) with a HIWIN robot.

## Features
- **Trajectory Control:** Publish joint trajectories using ROS2.
- **Kinematic Computations:** Use KDL for forward and inverse kinematics.
- **Integration with HIWIN Robots:** Designed for seamless operation with HIWIN industrial robots.

## Prerequisites
- **HIWIN ROS 2 Driver:** Ensure you have the hiwin_ros2 package installed.

## Usage
**On the HIWIN ROS 2 Side**
1. Launch the HIWIN MoveIt configuration:
```bash
ros2 launch hiwin_ra6_moveit_config ra6_moveit.launch.py ra_type:=ra605_710 use_fake_hardware:=false robot_ip:=<robot ip>
```
Replace `<robot_ip>` with the IP address of your HIWIN robot.

**Launch the Trajectory Example**
2. Start the trajectory sender node:
```bash
ros2 launch hiwin_demo send_trajectory.launch.py
```
This will send trajectory commands to the HIWIN robot.
