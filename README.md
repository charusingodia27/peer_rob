Two Robot MPC Simulation in Gazebo

This project demonstrates how to spawn and simulate two robots in the Gazebo environment using ROS Noetic. The robots are described using URDF files, and their states are published with robot_state_publisher.

Features

Spawns two robots in Gazebo.

Utilizes URDF to describe the robots.

Uses robot_state_publisher for joint states.

Easy-to-follow setup and launch.


 ______________________________________________________________________________________________________

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

___________________________________________________________________________________________________________

🚀 Getting Started

Clone the repository into your catkin workspace:

Build the Workspace Build the project using catkin_make

Launch the Simulation Run the launch file to spawn robots in Gazebo:

    roslaunch two_robot_mpc robots.launch

Visualize in Gazebo

Open Gazebo and adjust the camera view to see the robots.

Use Gazebo's axes and grid as reference for positioning.

__________________________________________________________________________________________________________________
__________________________________________________________________________________________________________________

Pallet Detection with ROS 2

This repository contains a ROS 2-based pallet detection system that uses a pre-trained YOLO model to detect pallets in image data. The project subscribes to an image topic, performs detection, and republishes the results with bounding boxes.

Features

Detect pallets in image streams using YOLO.

Subscribes to a ROS 2 image topic and publishes detection results to another topic.

Processes data from a sample ROS 2 bag file.

______________________________________________________________________________________________________________________
📋Prerequisites

Ensure the following are installed on your system:

ROS 2 (Foxy, Galactic, or Humble)
Python 3.x

Packages:

cv2

cv_bridge

ultralytics

Pre-trained YOLO model file: trained_models/best.pt.

____________________________________________________________________________________________________
Installation
1. Clone this Repository

2. Install ROS 2

Follow the ROS 2 installation guide for your platform. After installation, source the ROS 2 setup file:

	source /opt/ros/<ros-distro>/setup.bash

3. Install Dependencies
Install the required packages:

		sudo apt update
		sudo apt install ros-<ros-distro>-cv-bridge python3-opencv
		pip install ultralytics

  
4. Add the YOLO Model
    
Place your pre-trained YOLO model (best.pt) in the trained_models folder:

<repository-folder>/pallet_detection/src/trained_models/best.pt

__________________________________________________________________________________________________________________
Usage

Step 1: Run the ROS 2 Bag File

Ensure the provided bag file (internship_assignment_sample_bag) is accessible. Play the bag file:

	ros2 bag play <path-to-bag-file>/internship_assignment_sample_bag

Step 2: Run the Pallet Detection Node

Launch the detection script while remapping the input image topic from the bag file:

	ros2 run pallet_detection pallet_detection-ros2.py

Step 3: View Results

The detection results (images with bounding boxes) will be published to /pallet_detection/output_image. Use a tool like rqt_image_view to visualize the output:

	rqt_image_view

___________________________________________________________________________________________________________________________
Testing

Confirm the bag file publishes the image topic.

Verify the YOLO model is correctly placed in the trained_models folder.

Run the detection node and confirm results are published to /pallet_detection/output_image.

_____________________________________________________________________________________________________________________________________________________________

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
