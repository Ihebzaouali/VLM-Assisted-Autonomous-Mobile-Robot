# VLM-Assisted Autonomous Mobile Robot  
Autonomous mobile robot for target detection, collection and transport, combining LiDAR-based obstacle avoidance with vision-based target recognition. Integrated a VLM for high-level strategic adaptation and mission supervision, on top of a classical navigation stack for robust execution. Validated the complete autonomous navigation system in Gazebo simulation and on a physical robot.

# VLM-Assisted Autonomous Mobile Robot

An autonomous mobile robot that detects, collects and transports targets. A vision-language model (VLM) supervises the mission and adapts the high-level strategy, while a classical navigation stack handles motion for robust execution.

> TODO: add a demo GIF or video link here (simulation and real robot).

## Features

- **Autonomous navigation** with SLAM and Nav2
- **LiDAR-based obstacle avoidance**
- **Vision-based target detection** (YOLO) for target recognition and collection
- **VLM supervision** (LLaVA) for high-level strategy adaptation and mission monitoring
- **Sim-to-real validation:** tested in Gazebo simulation and on a physical robot

## Architecture

```
VLM (LLaVA)  →  high-level strategy and mission supervision
      ↓
Nav2 + SLAM  →  path planning and localization
      ↓
LiDAR / camera / YOLO  →  obstacle avoidance and target detection
```

The VLM never drives the robot directly: it adapts the mission at a high level, and the classical stack executes the motion.

> TODO: replace this sketch with a real diagram (nodes and topics) if you have one.

## Stack

ROS 2, Nav2, SLAM, Gazebo, YOLO, LLaVA, Docker, Raspberry Pi 5, LiDAR, sensor fusion

## Getting started

> TODO: fill in with your real steps. For example:
> - ROS 2 distribution and required packages
> - How to build the Docker image
> - How to launch the Gazebo simulation
> - How to run on the physical robot

## Hardware

> TODO: robot platform, LiDAR model, camera, and any other hardware used.

## Results

The complete navigation system was validated in Gazebo simulation and on a physical robot.

> TODO: add a few concrete results if you have them (success rate, test scenarios, limitations).

## Project structure

> TODO: short description of the main folders and packages.

## Limitations and future work

> TODO: for example, the project currently runs a single robot; multi-robot coordination is not implemented.

## Author

Iheb Zaouali, [LinkedIn](https://www.linkedin.com/in/iheb-zaouali-673974352)
