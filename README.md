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
