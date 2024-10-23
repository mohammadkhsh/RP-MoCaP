import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import cv2
import yaml
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import csv
import time
import readchar
import math
from collections import deque
import json
import copy
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel



def project_point(point, camera_info):
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    try:
        # Perform the projection using the camera model
        projected_point = camera_model.project3dToPixel((point.point.x, point.point.y, point.point.z))
        u = int(projected_point[0])
        v = int(projected_point[1])
        return u, v

    except Exception as e:
        rospy.logwarn(str(e))
        return None




if __name__ == '__main__':
    rospy.init_node('point_projection')


    camera_info = CameraInfo()
    with open('ost.yaml', 'r') as file:
        data = yaml.safe_load(file)
        camera_info.width = data['image_width']
        camera_info.height = data['image_height']
        camera_info.K = data['camera_matrix']['data']
        camera_info.D = data['distortion_coefficients']['data']
        camera_info.R = data['rectification_matrix']['data']
        camera_info.P = data['projection_matrix']['data']


    # Load the calibration parameters from the YAML file
    with open('ost.yaml', 'r') as file:
        calibration_data = yaml.safe_load(file)

    # Extract the calibration parameters from the loaded data
    camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape(3, 3)
    rectification_matrix = np.array(calibration_data['rectification_matrix']['data']).reshape(3, 3)
    projection_matrix = np.array(calibration_data['projection_matrix']['data']).reshape(3, 4)
    print(projection_matrix," \n")
    # Compute the inverse of the camera matrix
    camera_matrix_inv = np.linalg.inv(camera_matrix)

    # Derive the extrinsic matrix
    extrinsic_matrix = np.dot(camera_matrix_inv, projection_matrix)

    # Modify the extrinsic matrix to change the camera position
    new_translation = np.array([0, -1.32, 2.6])  # Desired new camera position
    extrinsic_matrix[:, 3] = new_translation

    # Recompose the projection matrix
    new_projection_matrix = np.dot(camera_matrix, extrinsic_matrix)

    # Print the new projection matrix
    print("New Projection Matrix:")
    print(new_projection_matrix)


