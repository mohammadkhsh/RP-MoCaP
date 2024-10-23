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

from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import math
import tf





def yaml_to_camera_info(yaml_file):
    yaml_file = os.path.join(current_dir, "ost.yaml")
    with open(yaml_file, 'r') as file:
        calib_data = yaml.safe_load(file)
    #print(calib_data)
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

        # Running a loop on each row in the .CSV file
        # Each row contains the information of the imestamp and the joints 3D positions
        for row in enumerate(reader):

            joints_data = row
            # reading the timestamp from the csv file, but not using it  :D
            timestamp = float(row[1][0])
            # "1" is missing in the csv file for each timestamp... so we need to add "1" at the beginning to fix that issue!
            timestamp_int = int("1" + str(int(timestamp)))
            seconds = int(timestamp_int // 1e9)
            nanoseconds = int(timestamp_int % int(1e9))

            # Creating a rospy.Time object
            timestamp_standard = rospy.Time(seconds, nanoseconds)
            ####### Points topic:
            points_arr = np.array(row[1]).flatten()
            # remove the first extra timestamp element
            points_arr = points_arr[1:]
            # Convert all other elements to float
            points_arr = points_arr.astype(float)

            points_msg = Float32MultiArrayPlusStamp()
            points_msg.data = points_arr.tolist()
            points_msg.header.stamp = timestamp_standard #rospy.Time.now()

            points_msg.header.frame_id = "body_joints"
            ####### Images topic:
            # image file name is set like "output_{timestamp}.jpg

            image_path = image_directory + str(row[1][0])+".jpg"
            image = cv2.imread(image_path)
            image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            image_msg.header.stamp = points_msg.header.stamp
            image_msg.header.frame_id = 'Camera_FLIR'

            ####### camera_info topic:
            camera_info.header = image_msg.header

            ####### tf topic:
            tf_message = TFMessage()
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = points_msg.header.stamp
            print(points_msg.header.stamp)
            print(transform_stamped.header.stamp)
            transform_stamped.header.frame_id = 'Camera_FLIR'
            transform_stamped.child_frame_id = 'body_joints'
            transform_stamped.transform.translation.x = -0.04  # Set the translation values
            transform_stamped.transform.translation.y = 1.4
            transform_stamped.transform.translation.z = 2.7
            roll = 0
            pitch = np.pi
            yaw = np.pi
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            transform_stamped.transform.rotation.x = quaternion[0] #normalized_rotation_x    # Set the rotation values
            transform_stamped.transform.rotation.y = quaternion[1] #normalized_rotation_y
            transform_stamped.transform.rotation.z = quaternion[2] #normalized_rotation_z
            transform_stamped.transform.rotation.w = quaternion[3] #rotation_w

            tf_message.transforms.append(transform_stamped)

            #print(timestamp_standard)

            bag.write('image_rect', image_msg, timestamp_standard)
            bag.write('body_joints', points_msg, timestamp_standard)
            bag.write('camera_info', camera_info, timestamp_standard)
            bag.write('/tf', tf_message, timestamp_standard)

    bag.close()

current_dir = os.path.dirname(os.path.abspath(__file__))
print(current_dir)
image_directory = "OUTPUT/output_"
image_directory = os.path.join(current_dir, image_directory)
output_bag = "OUTPUT/output.bag"
output_bag = os.path.join(current_dir, output_bag)
points_file = "OUTPUT/mocap_data.csv"
points_file = os.path.join(current_dir, points_file)
yaml_file = 'ost.yaml'
camera_info = yaml_to_camera_info(yaml_file)


rospy.init_node('image_to_bag')
convert_images_and_points_to_bag(image_directory, points_file, output_bag, camera_info)
