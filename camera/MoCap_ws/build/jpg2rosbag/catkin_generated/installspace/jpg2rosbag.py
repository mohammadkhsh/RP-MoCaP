import os
import sys
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
from geometry_msgs.msg import PointStamped
import numpy as np
from std_msgs.msg import Float32MultiArray
from custom_msg_f32.msg import Float32MultiArrayPlusStamp
from std_msgs.msg import MultiArrayDimension

from sensor_msgs.msg import CameraInfo
import yaml


def yaml_to_camera_info(yaml_file):
    with open(yaml_file, 'r') as file:
        calib_data = yaml.safe_load(file)
    camera_info = CameraInfo()
    camera_info.width = calib_data['image_width']
    camera_info.height = calib_data['image_height']
    camera_info.distortion_model = calib_data['distortion_model']  # Corrected key name
    camera_info.K = calib_data['camera_matrix']['data']
    camera_info.D = calib_data['distortion_coefficients']['data']
    camera_info.R = calib_data['rectification_matrix']['data']
    camera_info.P = calib_data['projection_matrix']['data']

    return camera_info

def convert_images_and_points_to_bag(image_directory, points_file, output_bag, camera_info):
    bridge = CvBridge()
    bag = rosbag.Bag(output_bag, 'w')
    # image_files = sorted(os.listdir(image_directory))

    with open(points_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip the header row
        for row in enumerate(reader):
            joints_data = row
            timestamp = float(row[1][0])
            points_arr = np.array(row[1]).flatten()
            # remove the first extra timestamp element
            points_arr = points_arr[1:]
            # Convert all other elements to float
            points_arr = points_arr.astype(float)

            points_msg = Float32MultiArrayPlusStamp()


            points_msg.data = points_arr.tolist()
            points_msg.header.stamp = rospy.Time.now()
            points_msg.header.frame_id = "body_joints"


            image_path = image_directory + str(row[1][0])+".jpg"
            image = cv2.imread(image_path)
            image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_msg.header.stamp = rospy.Time.now() #rospy.Time.from_sec(os.path.getmtime(image_path))
            #print(image_msg.header.stamp)
            image_msg.header.frame_id = 'Camera_FLIR'

            camera_info.header.stamp = rospy.Time.now()


            bag.write('/ROSbag/image_rect', image_msg)
            bag.write('/ROSbag/body_joints', points_msg)
            bag.write('/ROSbag/camera_info', camera_info)

    bag.close()

image_directory = "OUTPUT/output_"
output_bag = "OUTPUT/output.bag"
points_file = "OUTPUT/mocap_data.csv"
yaml_file = 'ost.yaml'
camera_info = yaml_to_camera_info(yaml_file)


rospy.init_node('image_to_bag')
convert_images_and_points_to_bag(image_directory, points_file, output_bag, camera_info)
