

# Automatic Annotation of Human 3D Poses in 2D Images Using MoCap

This project provides a set of modules for automating the annotation of human 3D poses in 2D images with Motion Capture (MoCap) data. The pipeline includes scripts for MoCap dataloader node, debugging checks, converting different formats to ROS bags, projectors and synchronisers.

## Project Structure

The repository contains the following Python scripts:

### `mocap_node.py`
This node establishes a TCP connection to the MoCap device and retrieve the information for all 32 joints, such as position, speed and acceleration. Then, it publishes the information with Float32MultiArray format to the selected topic. The Float32MultiArray is manually designed by me to efficiently transmit and receive skeleton data and timestamps.

### `Automized_recorder.py`
This script automates the recording process for 3D pose data using a camera and MoCap system. It subscribes to the topics of 2D images from camera node and 3D pose information from MoCap node. It then operates the synchronization process and find the best match of image/pose in real-time. The final image/pose matches are then stored for further processing.

### `jpg2rosbag.py`
This script converts a series of JPEG images into a ROS bag format, which allows for easy playback and analysis within the ROS framework. 

### `projector_jpg_offline.py`
This script handles the offline projection of 3D MoCap data onto 2D images. It takes recorded 3D poses from csv files and their match (jpg files). It also loads the camera calibration file. Afterwards, with all these information in hand, it finds each joint on the 2D image plane. The body hierarchy is defined, so the script can connect the joints to each other and draw the complete skeleton on the image. Some rotation/translation shifts caused by initial drift can also be compensated by manual setting or mouse calibration. The overlaied images with skeleton will be saved for qualitative evaluation.

### `projector_online.py`
This node is performing very similarly to the previous one, but it does not use the jpg and csv files which have been saved before! This script directly subscribes to the camera, MoCap and calibration nodes and retrieve the fresh data online and it does all steps in real-time.

### `projector_rviz_offline.py`
For debugging purposes, sometimes we need to see reconstructed 3D poses in ROS RVIZ environment, so that we can detect any device miscalibration, drifts or coordination misconfiguration. This node creates marker for joints and publish markers data to a topic which RVIZ can subscribe to.

### `rosbag2RVIZ.py` and `rosbag2video`
These scripts are intended to convert recorded ROS bag files to either marker in RVIZ to replay the saved motions or replay the saved images as video for visualization outputs.

### Other Python Scripts
Often used and served as debugging tools for different development stages.

## Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/mohammadkhsh/RP-MoCap.git
   ```

2. Install dependencies.
   

## Acknowledgments

This project was developed in collaboration between the University of Stuttgart and Mercedes-Benz.

## Confidentiality Notice

The contents of this repository are confidential and proprietary. Unauthorized copying or distribution of the materials within this repository is strictly prohibited.
