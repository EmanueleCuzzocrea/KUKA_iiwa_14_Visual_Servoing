
# KUKA iiwa 14 Visual-Servoing

## Overview

This project is developed as part of the Robotics Lab course assignment to implement a vision-based task using ROS (Robot Operating System) and
Gazebo simulation environment. The task involves detecting a circular object using OpenCV, controlling a simulated robot to align with an ArUco marker,
and implementing advanced control algorithms to track and align the robot's camera with a moving object.

## Objectives

The primary objectives of this project are:
1. **Object Detection**: Construct a Gazebo world with a circular object and detect it using the `opencv_ros` package.
2. **Look-at-point Control**: Implement a vision-based control node to align the robot's camera with an ArUco marker using a position and orientation offset.
3. **Innovative look-at-point strategies**:  Implement and test an improved look-at-point control algorithm based on the fact the task is belonging to $S^2$
3. **Trajectory tracking + Visual-Servoing**: The robot has to follow a trajectory, while keeping the eyes on the ArUco Marker.

## Project Structure

The primary components of the project include:

1. **Gazebo World Construction**: 
    - A new model representing a 15 cm radius colored circular object is created and inserted into a Gazebo world.
    - The robot is configured in the Gazebo world to view the object through its camera.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d35baa42-7e54-4601-8188-af63e6c277c7" alt="Pseudocode" width="400"/>
</p>


2. **Object Detection with OpenCV**:
    - The `opencv_ros_node` is modified to subscribe to the camera feed, detect the circular object using blob detection, and republish the processed image with the detected object highlighted.

<p align="center">
  <img src="https://github.com/user-attachments/assets/d6a1596e-1600-4aaa-9343-bdff6d51af66" alt="Pseudocode" width="400"/>
</p>


3. **Vision-Based Robot Control**:
    - The `kdl_robot_vision_control` node is modified to implement a vision-based task that aligns the robot's camera with an ArUco marker.
    - The robot's tracking capability is demonstrated by moving the ArUco marker and plotting the velocity commands sent to the robot.

[LAP_offset.webm](https://github.com/user-attachments/assets/5443e272-7fb6-41cb-a0a5-aa3651b967ab)


4. **Improved Look-At-Point Algorithm**:
    - An innovative look-at-point algorithm is implemented, on the basis that the task is defined in $S^2$.
    - The algorithm is tested to verify that the robot maintains alignment with the marker while executing joint space movements.

[LAP_improved.webm](https://github.com/user-attachments/assets/3ac16d3f-0ae9-4856-b943-a5d1edf5c617)


5. **Trajectory tracking + Visual-Servoing**:
    - A dynamic version of the vision-based controller is developed. The roboth has to track a given trajectory using joint space and cartesian space inverse dynamics controllers, while keeping the eyes on the ArUco marker.
    - The results are analyzed in terms of commanded joint torques and Cartesian error norms along the performed trajectories.

<p align="center">
  <img src="https://github.com/user-attachments/assets/de7a610b-af59-4d7e-9f5f-cd68f52cf757" alt="Pseudocode" width="400"/>
</p>

### Linear trajectory
[a1_js.webm](https://github.com/user-attachments/assets/7e2163fe-f057-48a2-84f3-2fd94a12441e)

### Circular trajectory
[a2_js.webm](https://github.com/user-attachments/assets/1b380116-fc7d-41d7-ba7c-2b85e491e9a3)


## Installation

To set up the project and run the simulation, follow these steps:

1. **Install ROS Noetic**:
    Ensure that ROS Noetic is installed on your system.

2. **Clone the Repository**:
    ```bash
    cd .../catkin_ws/src
    git clone https://github.com/EmanueleCuzzocrea/Homework3.git
    ```

3. **Build the Package**:
    Compile the ROS workspace:
    ```bash
    cd .../catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Key Results and Observations

- **Object Detection**: The circular object was successfully detected in the camera feed using blob detection, and the processed image was republished with the detected object highlighted.
- **Look-at-point Control**: The robot successfully aligned its camera with the ArUco marker, demonstrating effective use of the vision-based control node.
- **Dynamic Control**: The dynamic vision-based controller effectively tracked reference velocities, while showing at the same time consistent alignment of the robot's camera with the marker.

Thank you!
