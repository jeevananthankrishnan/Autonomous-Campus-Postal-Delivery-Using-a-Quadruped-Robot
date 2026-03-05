# Autonomous-Campus-Postal-Delivery-Using-a-Quadruped-Robot
This project demonstrates that practical outdoor navigation for quadruped robots can be implemented using low-cost microcontrollers and sensor fusion without requiring high-performance onboard computers. The open and modular ESP32-based architecture makes it suitable for research, education, and robotics prototyping.
# 📌 Overview

 Project presents a low-cost, fully embedded autonomous navigation system for a quadruped robot designed for campus-scale postal delivery.

Unlike traditional robotics systems that rely on ROS or SLAM, this system is implemented entirely on dual ESP32 microcontrollers, using onboard sensor fusion and waypoint navigation.

# The system integrates:

GPS-based waypoint navigation

IMU-based heading correction

LiDAR and ultrasonic obstacle avoidance

Dual-core FreeRTOS task architecture

Web-based real-time monitoring dashboard

Custom DAC-based remote controller emulation

The platform is implemented on a Unitree Go2 quadruped robot and demonstrates reliable campus navigation.

# 🎯 Key Contributions

Fully embedded ROS-free navigation stack

Multi-sensor fusion (GPS + IMU + LiDAR + Ultrasonic)

Dual-ESP32 distributed architecture using ESP-NOW

Real-time web dashboard without infrastructure dependency

Custom analog controller emulation interface

Cost-effective and scalable autonomous quadruped framework

# 🏗 System Architecture
Hardware Architecture

The system uses a dual ESP32 architecture communicating via ESP-NOW 


# ESP32 Dev Module

Main navigation logic

Sensor fusion

Waypoint computation

FreeRTOS dual-core tasking

# ESP32-CAM Module

Web dashboard interface

HTTP command handling

Telemetry broadcasting


# 🧠 Software Architecture

The ESP32 Dev module runs FreeRTOS with task partitioning across both cores 

# Core 0

Mission state machine

Waypoint logic

Display updates (50 ms loop)

# Core 1

Sensor acquisition (20 ms loop)

Motor command execution

Obstacle avoidance override

# Data Management

All sensor data is stored in a shared structure (SharedData) and transmitted to the web dashboard as JSON packets.

# 🚀 Navigation Strategy

1️⃣ Waypoint Navigation

GPS used for global positioning

Exponential filtering reduces GPS jitter

Waypoints guide campus-to-campus travel 

2️⃣ IMU-Based Blind Mode

When GPS signal degrades:

IMU heading correction maintains direction

Used for short trajectory segments 

3️⃣ Obstacle Avoidance Logic

Using 5 ultrasonic sensors:

Front threshold: 60 cm

Side clearance: 40 cm

Hysteresis control prevents oscillation

Automatically selects side with maximum clearance

## Navigation Commands:
NAV_FORWARD
NAV_TURN_LEFT
NAV_TURN_RIGHT
NAV_STRAFE_LEFT
NAV_STRAFE_RIGHT
NAV_WAIT
NAV_STOPPED

# 🌐 Web Dashboard

A self-hosted HTML interface provides:

Live GPS coordinates

IMU yaw heading

Ultrasonic distances

Navigation state

Mission control commands 

![image](https://github.com/Jeevananthan-RA/Autonomous-Campus-Postal-Delivery-Using-a-Quadruped-Robot/blob/2b12b4c945f0473d9ce744c03afced250989fdda/Dashboard.jpeg)



## 🔄 Mission State Machine

# Implemented as a finite state machine:

Idle

Navigate to Waypoint

Obstacle Avoidance

Pause

Resume

Return to Home

# Designed for robust operation during:

GPS interruption

Dynamic obstacle detection

Manual override via dashboard

# Excepted Outcomes

The robot consistently navigates between predefined landmarks ( Post Office → H-Building) with repeatable arrival accuracy under normal outdoor conditions.

![image](https://github.com/Jeevananthan-RA/Autonomous-Campus-Postal-Delivery-Using-a-Quadruped-Robot/blob/f45eb4ee126aea48dfe1b3db93b62bfefb865e00/Outcomes.png)


Advanced terrain classification

As concluded in the study, the architecture provides a cost-effective and scalable foundation for autonomous quadruped deployment


