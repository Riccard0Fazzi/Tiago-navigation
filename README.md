# ROS-Based Autonomous Navigation and Exploration for TIAGo Robot

## Overview

This project implements an autonomous navigation and exploration pipeline for the TIAGo robot using ROS1 in a simulated environment.

The system enables the robot to:
- explore an unknown environment
- detect AprilTags placed on objects
- estimate their positions in the map frame

The focus is on **autonomous exploration and perception**, combining sensor-driven navigation with vision-based target detection.

---

## Demo

TIAGo robot performing autonomous exploration and AprilTag detection in simulation:



https://github.com/user-attachments/assets/55a13fbe-2bb6-4187-9f3c-5ffd60f0d1bc



---

## Problem

The goal is to navigate inside an environment and detect a set of AprilTags placed on objects, returning their positions in the map reference frame.

The IDs of the AprilTags to detect are provided by a ROS service at runtime.

The system must:
- explore the environment autonomously  
- detect AprilTags using the onboard camera  
- transform detections into the map frame  
- return the positions of all requested targets  

As described in the assignment, the robot must operate in an environment composed of multiple rooms and narrow passages, requiring robust navigation and perception.

---

## System Architecture

The system is structured around two main components:

- **Node A (Client / Coordination)**
  - Requests AprilTag IDs from a ROS service
  - Sends goals to the navigation system
  - Receives feedback and final results

- **Node B (Navigation & Perception)**
  - Implements the autonomous exploration strategy
  - Detects AprilTags and estimates their poses
  - Manages the overall navigation loop

Communication is implemented through a custom ROS action:
- goal → list of AprilTag IDs  
- feedback → robot status during exploration  
- result → detected AprilTag poses  

---

## Pipeline

1. Initialize robot posture and camera orientation  
2. Perform a **360° rotation** to scan the environment  
3. Detect AprilTags and store their IDs and poses  
4. Identify candidate exploration directions using laser scan clustering  
5. Select a valid exploration vector based on:
   - previously visited areas  
   - obstacle avoidance  
   - table position constraints  
6. Send navigation goal to `move_base`  
7. Repeat exploration until all target AprilTags are found  

---

## Key Design Choices

- **Exploration based on laser scan clustering**
  - Identification of free-space directions from sensor data  

- **Memory-based navigation**
  - Avoid revisiting already explored areas  

- **Hybrid navigation approach**
  - `move_base` for global navigation  
  - custom control law for narrow corridor traversal  

- **Randomized exploration strategy**
  - Ensures eventual discovery of all targets  

---

## Challenges

Key challenges encountered during development:

- **Navigation in narrow corridors**
  - Addressed using a custom control law based on laser data  

- **Exploration strategy design**
  - Required balancing randomness and efficiency  

- **Pose transformations**
  - Managed using TF with retry mechanisms to handle delays  

- **Simulation variability**
  - Different behaviors observed between local environments and VLAB  

---

## Technologies

- ROS1  
- Navigation Stack (`move_base`)  
- Laser scan processing  
- AprilTag detection  
- TF transformations  

---

## Environment

This project is **not fully standalone**.

It was developed within a specific simulation environment provided for the course assignment and depends on external ROS packages and simulation assets.

To run the project, the full TIAGo simulation setup described in the assignment is required.

---

## Authors

This project was developed as part of a group assignment:

- Riccardo Fazzi  
- Matteo Baldoni  
- Luca Grigolin  

Each member contributed to different components of the system, including navigation, perception, and system integration.
