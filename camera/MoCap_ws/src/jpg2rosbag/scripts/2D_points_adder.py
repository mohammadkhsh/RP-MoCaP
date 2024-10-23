from visualization_msgs.msg import Marker, MarkerArray
from custom_msg_f32.msg import Float32MultiArrayPlusStamp
from std_msgs.msg import MultiArrayDimension
import os
import numpy as np


import rospy
import rosbag
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
from tf import TransformListener
from image_geometry import PinholeCameraModel
from std_msgs.msg import Float32MultiArray

date_version = "_23-5-2023_2"
current_dir = os.path.dirname(os.path.abspath(__file__))
output_bag = "OUTPUT" + date_version + "/full_output.bag"
input_bag = "OUTPUT" + date_version + "/output.bag"
output_bag = os.path.join(current_dir, output_bag)
input_bag = os.path.join(current_dir, input_bag)

init_rotation = 0.28  # 2.6.2023 - T-pose = 9.7s

# Create a new bag file
new_bag = rosbag.Bag(output_bag, 'w')

rospy.init_node('projected_points_adder')

# Initialize necessary variables and objects
tf_listener = TransformListener()
camera_model = PinholeCameraModel()

projected_points_pub = rospy.Publisher('/projected_points', Float32MultiArrayPlusStamp, queue_size=10)


def body_rotation(joints_list, angle,x,y,z):
    # Define the rotation axis and angle (in radians)
    axis = np.array([x, y, z])  # Y-axis
    #angle = np.pi / 4  # Rotate 45 degrees

    # Define the 3D points as a NumPy array
    points = np.array(joints_list)
    # Define the rotation matrix
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c
    x, y, z = axis
    R = np.array([[x*x*C+c, x*y*C-z*s, x*z*C+y*s],
                  [y*x*C+z*s, y*y*C+c, y*z*C-x*s],
                  [z*x*C-y*s, z*y*C+x*s, z*z*C+c]])

    # Apply the rotation to the points
    rotated_points = np.dot(R, points.T).T
    return rotated_points



# Define the callback function for processing body joints and images
def process_data(body_joints_msg, image_msg, camera_info_msg, tf_msg):
    # Perform transformations and project 3D points to pixel coordinates
    try:
        # Wait for the transformation to be available
        tf_listener.setTransform(tf_msg.transforms[0])
        camera_model.fromCameraInfo(camera_info_msg)

        # Create a new message to store the projected points
        projected_points_msg = Float32MultiArrayPlusStamp()
        projected_points_msg.data = []
        projected_points_msg.header = image_msg.header

        # All timestamps must be the same
        print(body_joints_msg.header.stamp)
        print(image_msg.header.stamp)
        print(camera_info_msg.header.stamp)

        joint_list = [list(body_joints_msg.data[i:i+3]) for i in range(0, len(body_joints_msg.data), 3)]
        # we need this angle at T-pose moment to fix the initial orientation issue!
        rotated_joints = body_rotation(joint_list , init_rotation, 0, 1, 0)
        points_arr = np.array(rotated_joints)
        points = points_arr.reshape(-1, 3)


        # Loop through each body joint in the message
        for  point in points:
            # Create a PointStamped message with the body joint information
            body_joint = PointStamped()
            body_joint.header = body_joints_msg.header
            body_joint.header.stamp = body_joints_msg.header.stamp
            body_joint.point.x = point[0]
            body_joint.point.y = point[1]
            body_joint.point.z = point[2]

            # Transform the body joint to the camera frame
            transformed_point = tf_listener.transformPoint('/Camera_FLIR', body_joint)

            # Calculate the pixel coordinates using PinholeCameraModel
            u, v = camera_model.project3dToPixel((transformed_point.point.x, transformed_point.point.y, transformed_point.point.z))

            # Add the projected point to the array
            projected_points_msg.data.append(float(u))
            projected_points_msg.data.append(float(v))

        # Publish the projected points on the new topic
        projected_points_pub.publish(projected_points_msg)

        # Write the updated projected points to the new bag file


    except Exception as e:
        rospy.logerr('Failed to transform body joints: {}'.format(e))

    # Write the original messages to the new bag file
    new_bag.write('/projected_points', projected_points_msg, body_joints_msg.header.stamp)
    new_bag.write('/body_joints', body_joints_msg, body_joints_msg.header.stamp)
    new_bag.write('/image_rect', image_msg, image_msg.header.stamp)
    new_bag.write('/camera_info', camera_info_msg, camera_info_msg.header.stamp)
    new_bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)

# Open the original bag file for reading
body_joints_msg = None
image_msg = None
camera_info_msg = None
tf_msg = None
with rosbag.Bag(input_bag, 'r') as bag:
    # Iterate over the messages in the bag file
    for topic, msg, t in bag.read_messages():

        if topic == 'body_joints':
            body_joints_msg = msg
        elif topic == 'image_rect':
            image_msg = msg
        elif topic == 'camera_info':
            camera_info_msg = msg
        elif topic == '/tf':
            tf_msg = msg

        # Process the body joints, image, and camera info together
        if (body_joints_msg and image_msg and camera_info_msg and tf_msg) and (body_joints_msg.header.stamp == image_msg.header.stamp and image_msg.header.stamp == camera_info_msg.header.stamp):
            process_data(body_joints_msg, image_msg, camera_info_msg, tf_msg)
            body_joints_msg = None
            image_msg = None
            camera_info_msg = None
            tf_msg = None

        # Write the original messages to the new bag file
        #new_bag.write(topic, msg, t)

# Close the new bag file

new_bag.close()
