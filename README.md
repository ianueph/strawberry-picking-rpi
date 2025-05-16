# Strawberry Picking Robot Arm

This project implements an autonomous robot arm capable of perceiving and picking strawberries along a ridgeline using a Raspberry Pi and computer vision.

## Overview

The system combines visual perception with a robotic arm to locate and pick strawberries in a structured environment. It uses ROS 2 Jazzy for coordination, Docker for containerized development, and a combination of Raspberry Pi and Arduino for hardware control.

<div align="center">
  <table>
    <thead>
      <tr>
        <th>Hardware Overview</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><img src="media/system%20overview.drawio.png" alt="System Overview" /></td>
      </tr>
    </tbody>
  </table>
  </p/>
</div>

<div align="center">
  <table>
    <thead>
      <tr>
        <th>Top</th>
        <th>Front</th>
        <th>Inside</th>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td><img src="media/top_view.png" alt="Top view image" width="250" /></td>
        <td><img src="media/front_view.png" alt="Front view image" width="250" /></td>
        <td><img src="media/inside_view.png" alt="Inside view image" width="250" /></td>
      </tr>
    </tbody>
  </table>
</div>

## Demo Videos

<table>
  <thead>
    <tr>
      <th>Not damaged, Successful Pick</th>
      <th>Not damaged, Partially Successful Pick</th>
      <th>Not damaged, Unsuccessful Pick</th>
      <th>Damaged, Successful Pick</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>
        <a href="https://www.youtube.com/shorts/Jc2jJJqsEMs" target="_blank">
          <img src="https://i.ytimg.com/vi/Jc2jJJqsEMs/hqdefault.jpg" alt="ND S" />
        </a>
      </td>
      <td>
        <a href="https://www.youtube.com/shorts/nUUJdWZODPo" target="_blank">
          <img src="https://i.ytimg.com/vi/nUUJdWZODPo/hqdefault.jpg" alt="ND PS" />
        </a>
      </td>
      <td>
        <a href="https://www.youtube.com/shorts/5XrlYmzagXs" target="_blank">
          <img src="https://i.ytimg.com/vi/5XrlYmzagXs/hqdefault.jpg" alt="ND US" />
        </a>
      </td>
      <td>
        <a href="https://www.youtube.com/shorts/FgwHGqu-LLM" target="_blank">
          <img src="https://i.ytimg.com/vi/FgwHGqu-LLM/hqdefault.jpg" alt="D S" />
        </a>
      </td>
    </tr>
  </tbody>
</table>

---

## Requirements

### Hardware

- Raspberry Pi 5
- 2× Pi Camera Module 3 (1 wide-angle, 1 normal)
- Arduino Uno
- 5-DOF Robotic Arm

### Software

- RaspberryPi OS
- ROS 2 Jazzy (inside the dev container)
- Git Bash or similar terminal

---

## Installation

### Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- Docker installed and running on your system

### Steps

1. Clone the repository:
    ```bash
    git clone https://github.com/ianueph/strawberry-picking-rpi.git
    cd strawberry-picking-rpi
    ```

2. Open the project folder in VS Code.

3. When prompted by VS Code, **"Reopen in Container"**.

4. The container will build and install all dependencies automatically via:
   - `installations.sh`
   - `postCreateCommand.sh`

---

## Deployment

After setup and connecting all hardware:

1. Upload YOLO Model Weights

    Before launching the robot system, ensure the required YOLOv8 model weights are available. Place the following files in the specified directory:

    ```bash
    src/
    └── yolo_ros/
        ├── ...
        ├── yolo_bringup/
        │   ├── launch/
        │   │   ├── strawberry-object-tracking.pt
        │   │   ├── strawberry-image-class.pt
        │   │   └── ...
        │   └── ...
        ├── yolo_msgs/
        │   └── ...
        ├── yolo_ros/
        │   └── ...
        └── ...
    ```

1. Grant access to the Arduino serial device:

    ```bash
    sudo chmod 666 /dev/ttyACM0
    ```

2. Launch the robot system:

    ```bash
    ros2 launch sprpi_arm_bringup main_launch.launch.py
    ```
> **Note:** Ensure you have X11 forwarding set up properly if running GUI apps like RViz2. This is handled via mounted volumes and environment variables in `devcontainer.json`.

---

## Notes

- Ensure both Pi Cameras are connected and recognized by the Raspberry Pi.
- The Arduino should be flashed with the correct firmware for arm control.
- This project assumes basic familiarity with ROS 2, Linux, and Docker.

---
