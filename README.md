Two Robot MPC Simulation in Gazebo
This project demonstrates how to spawn and simulate two robots in the Gazebo environment using ROS Noetic. The robots are described using URDF files, and their states are published with robot_state_publisher.

Features
Spawns two robots in Gazebo.
Utilizes URDF to describe the robots.
Uses robot_state_publisher for joint states.
Easy-to-follow setup and launch.

📋 Prerequisites
Install ROS Noetic
Follow the official ROS installation guide.
Install Required ROS Packages
Install necessary dependencies:
    sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-gazebo-ros

Set Up Your Workspace
Create and configure your catkin workspace:
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
Install Gazebo
Gazebo comes with ROS Noetic.

🗂️ Project Structure
two_robot_mpc/
├── launch/
│   ├── robots.launch    # Launch file for Gazebo
├── urdf/
│   ├── robot1.urdf               # Robot 1 URDF file
│   ├── robot2.urdf               # Robot 2 URDF file
└── README.md                     # Project documentation

🚀 Getting Started
Clone the Repository Clone the repository into your catkin workspace:
Build the Workspace Build the project using catkin_make
Launch the Simulation Run the launch file to spawn robots in Gazebo:
    roslaunch two_robot_mpc robots.launch

Visualize in Gazebo
Open Gazebo and adjust the camera view to see the robots.
Use Gazebo's axes and grid as reference for positioning.



This project focuses on integrating YOLO inference with ROS 2, enabling seamless object detection capabilities for robotics applications. The development process involved leveraging Google Colab for training and inferencing due to hardware limitations, as my laptop does not support GPU acceleration. Below are the key steps and accomplishments:

Training and Inferencing:

The YOLO model was trained and tested on Google Colab, utilizing its GPU resources to expedite the process.
Model inferencing was also successfully executed on Colab, ensuring the effectiveness of the trained model.
ROS 2 Node Development:

A dedicated ROS 2 node was developed to integrate YOLO inference using Ultralytics YOLOv5.
The node is designed for real-time object detection in robotics systems.
TensorRT Conversion:

An attempt was made to convert the trained model to TensorRT for optimized deployment.
Due to resource limitations on my laptop, this step could not be completed locally. However, TensorRT conversion remains a prospective enhancement for deployment on edge devices.
This repository provides the ROS 2 node code, along with instructions for reproducing the training and inferencing setup on Google Colab.
