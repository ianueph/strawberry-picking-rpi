# Strawberry Picking Robot (Raspberry Pi)

This project implements an autonomous robot arm capable of perceiving and picking strawberries along a ridgeline using a Raspberry Pi and computer vision.

## Overview

The system combines visual perception with a robotic arm to locate and pick strawberries in a structured environment. It uses ROS 2 for coordination, Docker for containerized development, and a combination of Raspberry Pi and Arduino for hardware control.

---

## Requirements

### Hardware

- Raspberry Pi 5
- 2× Pi Camera Module 3 (1 wide-angle, 1 normal)
- Arduino Uno
- 5-DOF Robotic Arm

### Software

- A host machine capable of running Docker
- ROS 2 (inside the dev container)
- Git Bash or similar terminal

---

## Installation

The development environment is containerized using a devcontainer.

1. Clone the repository:

    ```bash
    git clone https://github.com/ianueph/strawberry-picking-rpi.git
    cd strawberry-picking-rpi
    ```

2. Open the project in Visual Studio Code with the **Dev Containers** extension installed.

3. The devcontainer will automatically install all required dependencies when it starts.

---

## Deployment

After setup and connecting all hardware:

1. Grant access to the Arduino serial device:

    ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

2. Launch the robot system:

    ```bash
    ros2 launch sprpi_arm_bringup main_launch.launch.py
    ```

---

## Notes

- Ensure both Pi Cameras are connected and recognized by the Raspberry Pi.
- The Arduino should be flashed with the correct firmware for arm control.
- This project assumes basic familiarity with ROS 2, Linux, and Docker.

---
