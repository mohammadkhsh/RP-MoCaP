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


# Initialize the ROS node
rospy.init_node('projector_node')

# Create a TF buffer
tf_buffer = tf2_ros.Buffer()

# Create a TF listener
tf_listener = tf2_ros.TransformListener(tf_buffer)



bridge = CvBridge()
image = np.zeros((480, 640, 3), dtype=np.uint8)
image_width = 0
image_height = 0
image_processed = 0

data_dir = "OUTPUT_2.6_3/"


# 2/6/2023 _ 1
chest_click_x = 2010
chest_click_y = 1144
initial_y_rotation = -0.2 * np.pi
initial_x_rotation = -0.02 * np.pi

body_thickness = 0.05
T_pose_frame = 61

"""
# 2/6/2023 _ 1
chest_click_x = 2030
chest_click_y = 1144
initial_y_rotation = -0.2 * np.pi
initial_x_rotation = -0.02 * np.pi

body_thickness = 0.05
T_pose_frame = 61
"""


"""
# 24/5/2023 _ 1
chest_click_x = 2030
chest_click_y = 1144
initial_y_rotation = -0.08 * np.pi
initial_x_rotation = -0.02 * np.pi

body_thickness = 0.05
T_pose_frame = 43
"""

"""
# 24/5/2023 _ 2
chest_click_x = 1990
chest_click_y = 1150
initial_y_rotation = -0.11 * np.pi
body_thickness = -0.05
T_pose_frame = 61

"""

"""
# 23/5/2023 _ 2
chest_click_x = 1984
chest_click_y = 1146
initial_y_rotation = -0.082  * np.pi #-0.082 * np.pi    -0.31
initial_x_rotation = -0.0 * np.pi
body_thickness = 0.05
T_pose_frame = 56
"""

"""
# 23/5/2023 _ 1
chest_click_x = 2005
chest_click_y = 1146
initial_y_rotation = -0.14*np.pi
body_thickness = -0.09
"""

"""
# 9/5/2023
chest_click_x = 1998
chest_click_y = 1141
body_thickness = -0.09
"""
offset_chest_x = 0
offset_chest_y = 0
z_offset = 0 # 2.6
y_offset = 0 # 1.32

pause_ok = 0

joints_info = {
        "Body.cx": 0,
        "Chest.cx": 1,
        "Head.cx": 2,
        "HeadEnd.cx": 3,
        "Hips.cx": 4,
        "LeftArm.cx": 5,
        "LeftFinger.cx": 6,
        "LeftFingerEnd.cx": 7,
        "LeftFoot.cx": 8,
        "LeftForearm.cx": 9,
        "LeftHand.cx": 10,
        "LeftHeel.cx": 11,
        "LeftLeg.cx": 12,
        "LeftShoulder.cx": 13,
        "LeftThigh.cx": 14,
        "LeftToe.cx": 15,
        "LeftToeEnd.cx": 16,
        "Neck.cx": 17,
        "RightArm.cx": 18,
        "RightFinger.cx": 19,
        "RightFingerEnd.cx": 20,
        "RightFoot.cx": 21,
        "RightForearm.cx": 22,
        "RightHand.cx": 23,
        "RightHeel.cx": 24,
        "RightLeg.cx": 25,
        "RightShoulder.cx": 26,
        "RightThigh.cx": 27,
        "RightToe.cx": 28,
        "RightToeEnd.cx": 29,
        "SpineLow.cx": 30,
        "SpineMid.cx": 31}

interpolate_flag = -1
pause_flag = -1

upper_body_ids = [3, 2, 1, 31, 0, 30] # n=4
handsLeft_ids = [6, 10, 9, 5, 13, 1]
handsRight_ids = [1,26, 18, 22, 23, 19] #n=9 (8+1)
legs_ids = [15, 11, 14, 30, 27, 24, 28]
no_connection_ids = [4, 7, 8, 12, 16, 17, 20, 21, 25, 29]
initial_points = []
current_normal = [0, 0, 1]


### Result As a Py Dictionary In a JSON File

xs = 0
ys = 0
zs = 0
x_img = 0
y_img = 0


motion_capture_data = {
    "instance_type": "MotionCaptureData",
    "recording_date" : "9/5/2023",
    "camera_config":
        {
        "camera_matrix": [],
        "distortion_matrix": [],
        "rectification_matrix": [],
        "projection_matrix": [],
        "camera_height": y_offset,
        "frame_rate": 10,
        "camera_partnumber": "FLIR-GS3"
        },
    "skeleton_initial_settings":
        {
        "target_distance": z_offset,
        "initial_x_rotation": 0,
        "initial_y_rotation": 0,
        "initial_z_rotation": 0,
        "chest_init_x_img": chest_click_x,
        "chest_init_y_img": chest_click_y,
        "skeleton_height": 1.87
        },

    "frames": []
}

image_frame = {
    "file": "path_to_image.jpg",
    "image_size_w*h": [0, 0],
    "label_info": {
        "frame_number": 0,
        "format_version": "2.0",
        "timestamp": 2023,
        "comments": []
    },
    "object_annotations": [
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "Body",
            "joint_id": 0,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False

        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "Chest",
            "joint_id": 1,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False

        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "Head",
            "joint_id": 2,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False

        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "HeadEnd",
            "joint_id": 3,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False

        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "Hips",
            "joint_id": 4,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False

        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftArm",
            "joint_id": 5,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftFinger",
            "joint_id": 6,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftFingerEnd",
            "joint_id": 7,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftFoot",
            "joint_id": 8,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftForearm",
            "joint_id": 9,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftHand",
            "joint_id": 10,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftHeel",
            "joint_id": 11,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftLeg",
            "joint_id": 12,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftShoulder",
            "joint_id": 13,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftThigh",
            "joint_id": 14,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftToe",
            "joint_id": 15,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instan0ce_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "LeftToeEnd",
            "joint_id": 16,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "Neck",
            "joint_id": 17,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightArm",
            "joint_id": 18,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightFinger",
            "joint_id": 19,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightFingerEnd",
            "joint_id": 20,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightFoot",
            "joint_id": 21,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightForearm",
            "joint_id": 22,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightHand",
            "joint_id": 23,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightHeel",
            "joint_id": 24,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightLeg",
            "joint_id": 25,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightShoulder",
            "joint_id": 26,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightThigh",
            "joint_id": 27,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightToe",
            "joint_id": 28,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "RightToeEnd",
            "joint_id": 29,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "SpineLow",
            "joint_id": 30,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        },
        {
            "instance_type": "Point",
            "hierarchy_class": "JointPoint",
            "joint_name": "SpineMid",
            "joint_id": 31,
            "position_3d": [xs, ys, zs],
            "position_2d": [x_img, y_img],
            "visible" : False
        }
    ]

}


###

def mouse_callback(event, x, y, flags, param):

    global chest_click_x, chest_click_y, interpolate_flag, pause_flag
    if event == cv2.EVENT_LBUTTONDOWN:

        print("Clicked pixel:", x, y)
        chest_click_x = x
        chest_click_y = y

        #interpolate_flag *= -1
    if event == cv2.EVENT_RBUTTONDOWN:
        pause_flag *= -1




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



def round_point(point):
    return Point(round(point[0], 4), round(point[1], 4), round(point[2], 4))



def calculate_body_rotation(current_joints):
    global initial_points, current_normal

    # Calculate the initial body normal vector
    initial_shoulderLeft, initial_shoulderRight, initial_chest = initial_points[13], initial_points[26], initial_points[1]
    initial_normal = np.cross(initial_shoulderLeft - initial_shoulderRight, initial_shoulderLeft - initial_chest)

    # Calculate the current body normal vector
    current_shoulderLeft, current_shoulderRight, current_chest = current_joints[13], current_joints[26], current_joints[1]
    current_normal = np.cross(current_shoulderLeft - current_shoulderRight, current_shoulderLeft - current_chest)

    # Calculate the rotation angle
    dot_product = np.dot(initial_normal, current_normal)
    magnitude_product = np.linalg.norm(initial_normal) * np.linalg.norm(current_normal)
    cosine_angle = dot_product / magnitude_product

    if cosine_angle < -1.0:
        cosine_angle = -1.0
    elif cosine_angle > 1.0:
        cosine_angle = 1.0

    rotation_angle = math.acos(cosine_angle)

    # Convert the angle to degrees if needed
    rotation_angle_degrees = math.degrees(rotation_angle)

    return rotation_angle_degrees



def move_points_in_normal_direction(points, normal_vector, displacement):

    #normal vector to obtain a unit vector
    unit_vector = normal_vector / np.linalg.norm(normal_vector)

    # Calculate the displacement vector
    displacement_vector = unit_vector * displacement

    # Add the displacement vector to all points
    moved_points = points + displacement_vector

    return moved_points


def manual_projection(point_3d, projection_mat):

    mark_point = np.array([point_3d[0], point_3d[1], point_3d[2], 1])

    # apply projection matrix to 3D point
    proj_point = projection_mat.dot(mark_point)

    # divide by Z-coordinate to convert to pixel coordinates
    u = int(proj_point[0] / proj_point[2] + 0.5)
    v = int(proj_point[1] / proj_point[2] + 0.5)
    v = image_height - v
    u -= offset_chest_x
    v -= offset_chest_y


    return (u, v)


def tf_projection(points_3d, camera_info):
    camera_model = PinholeCameraModel()
    camera_model.fromCameraInfo(camera_info)

    point = PointStamped()
    point.header.frame_id = 'camera_frame'  # Set the frame ID of the point
    point.point.x = points_3d[0]
    point.point.y = points_3d[1]
    point.point.z = points_3d[2]



    # Perform the projection using the camera model
    projected_point = camera_model.project3dToPixel((point.point.x, point.point.y, point.point.z))
    u = int(projected_point[0])
    v = int(projected_point[1])
    v = image_height - v
    u -= offset_chest_x
    v -= offset_chest_y
    return (u, v)


def joint_input(joint_state,trans_f = 0):
    global pause_ok, offset_chest_x, offset_chest_y, initial_points
    x_joint_image = []
    y_joint_image = []

    # convert list of joints to numpy array + add Z offset
    points_arr = np.array(joint_state)
    points = points_arr.reshape(-1, 3)
    points[:, 2] += z_offset

    rounded_points = [round_point(point) for point in points]

    # select chest point to fix :
    if i==T_pose_frame :      # frame No. "T_pose_frame" is the first frame after T-pose reset
        initial_points = joint_state
        # Recap: points[1] is the chest !
        #(pixel_x, pixel_y) = manual_projection(points[1], proj_matrix)
        (pixel_x, pixel_y) = tf_projection(points[1], camera_info)

        offset_chest_x = pixel_x - chest_click_x
        offset_chest_y = pixel_y - chest_click_y
    elif i<T_pose_frame :
        initial_points = joint_state

    orientation = round(calculate_body_rotation(joint_state),2)
    cv2.putText(img=image, text="Rotation: " + str(orientation),org=(10,60) ,
    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.8, color=(0, 0, 255),thickness=1)
    #cv2.putText(img=image, text='Body distance is = ' + str(z_offset),org=(5,20) ,fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.8, color=(0, 0, 255),thickness=1)
    cv2.putText(img=image, text="Mocap Time: "+str(buffer_mocap_time[i] + "  |  Frame No. "+str(i)),org=(10,25) ,
    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.8, color=(0, 0, 255),thickness=1)

    # move the whole skeleton towards the body normal vector
    #points = move_points_in_normal_direction(points, current_normal, body_thickness * np.sin(orientation/180*np.pi) )

    mark_point = Point()

    '''
    # calibrating line:
    point_5_meter_1 = np.array([1,2,5,1])
    point_5_meter_2 = np.array([0,2,5,1])
    proj_p5m_1 = proj_matrix.dot(point_5_meter_1)
    proj_p5m_2 = proj_matrix.dot(point_5_meter_2)
    p5m_1_x = int(proj_p5m_1[0] / proj_p5m_1[2] + 0.5)
    p5m_2_x = int(proj_p5m_2[0] / proj_p5m_2[2] + 0.5)
    p5m_1_y = int(proj_p5m_1[1] / proj_p5m_1[2] + 0.5)
    p5m_2_y = int(proj_p5m_2[1] / proj_p5m_2[2] + 0.5)
    cv2.line(image, (p5m_2_x,p5m_2_y),(p5m_1_x,p5m_1_y),(0,0,255),2)

    '''

    #Axis :
    end_x = np.array([1,0,2.85,1])
    end_y = np.array([0,1,2.85,1])
    end_z = np.array([0,0,3.85,1])
    origin = np.array([0,0,2.85,1])
    p_end_x = proj_matrix.dot(end_x)
    p_end_y = proj_matrix.dot(end_y)
    p_end_z = proj_matrix.dot(end_z)
    p_origin = proj_matrix.dot(origin)
    pixel_x_end_x = int(p_end_x[0] / p_end_x[2] + 0.5)
    pixel_y_end_x = int(p_end_x[1] / p_end_x[2] + 0.5)
    pixel_x_end_y = int(p_end_y[0] / p_end_y[2] + 0.5)
    pixel_y_end_y = int(p_end_y[1] / p_end_y[2] + 0.5)
    pixel_x_end_z = int(p_end_z[0] / p_end_z[2] + 0.5)
    pixel_y_end_z = int(p_end_z[1] / p_end_z[2] + 0.5)
    pixel_x_origin = int(p_origin[0] / p_origin[2] + 0.5)
    pixel_y_origin = int(p_origin[1] / p_origin[2] + 0.5)

    #cv2.line(image, (pixel_x_end_x,pixel_y_end_x),(pixel_x_origin,pixel_y_origin),(0,0,255),2)
    #cv2.line(image, (pixel_x_end_y,pixel_y_end_y),(pixel_x_origin,pixel_y_origin),(0,255,0),2)
    #cv2.line(image, (pixel_x_end_z,pixel_y_end_z),(pixel_x_origin,pixel_y_origin),(255,0,0),2)
    #cv2.putText(img=image, text='X',org=(pixel_x_end_x,pixel_y_end_x) ,
    #fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 0, 255),thickness=1)
    #cv2.putText(image, text='Y',org=(pixel_x_end_y,pixel_y_end_y) ,
    #fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=1, color=(0, 255, 0),thickness=1)

    image_frame["label_info"]["frame_number"] = i

    cont = 0

    for point in points:

        #(pixel_x, pixel_y) = manual_projection(point, proj_matrix)

        (pixel_x, pixel_y) = tf_projection(point, camera_info)


        #print("Manual Projection :", pixel_x, " , ", pixel_y)
        #print("GeoMet Projection :", new_pixel_x, " , ", new_pixel_y)



        x_joint_image.append(pixel_x)
        y_joint_image.append(pixel_y)

        #print("\n\nPixel pos: ",pixel_x , pixel_y,"\n")
        center_point = (int(image_width/2) , int(image_height/2))        
        if not(cont in no_connection_ids):
            image_frame["object_annotations"][cont]["position_2d"] = [pixel_x, pixel_y]
            cv2.circle(image, (pixel_x, pixel_y), 4, (0, 255, 0), -1)

        cont += 1

    line_color = (0,255,255,255)
    lineLeft_color = (255,0,255,255)
    lineRight_color = (255,255,0,255)

    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[upper_body_ids[j]], y_joint_image[upper_body_ids[j]]), (x_joint_image[upper_body_ids[j+1]], y_joint_image[upper_body_ids[j+1]]),line_color,2)
    for j in range(len(handsLeft_ids)-1):
        cv2.line(image, (x_joint_image[handsLeft_ids[j]], y_joint_image[handsLeft_ids[j]]), (x_joint_image[handsLeft_ids[j+1]], y_joint_image[handsLeft_ids[j+1]]),lineLeft_color,2)
    for j in range(len(handsRight_ids)-1):
        cv2.line(image, (x_joint_image[handsRight_ids[j]], y_joint_image[handsRight_ids[j]]), (x_joint_image[handsRight_ids[j+1]], y_joint_image[handsRight_ids[j+1]]),lineRight_color,2)
    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[legs_ids[j]], y_joint_image[legs_ids[j]]), (x_joint_image[legs_ids[j+1]], y_joint_image[legs_ids[j+1]]),line_color,2)


    # display image
    cv2.imshow('Image', image)
    #out.write(image)
    cv2.setMouseCallback('Image', mouse_callback)
    key = cv2.waitKey(1)


# load camera calibration file
calibration_file = 'ost.yaml'
with open(calibration_file) as f:
    calib_data = yaml.safe_load(f)

# create camera matrix and distortion coefficients
camera_matrix = np.array(calib_data['camera_matrix']['data']).reshape((3, 3))
dist_coeffs = np.array(calib_data['distortion_coefficients']['data'])

# create rectification matrix (if applicable)
if 'rectification_matrix' in calib_data:
     rect_matrix = np.array(calib_data['rectification_matrix']['data']).reshape((3, 3))
else:
     rect_matrix = None

# create projection matrix
proj_matrix = np.array(calib_data['projection_matrix']['data']).reshape((3, 4))

print("camera matrix :  ",camera_matrix,"\ndistortion matrix :  ",dist_coeffs,"\nrectification matrix :  ",rect_matrix,"\nProjection matrix :  ",proj_matrix)

# add calibration matrices to the label file
motion_capture_data["camera_config"]["camera_matrix"] = calib_data['camera_matrix']['data']
motion_capture_data["camera_config"]["distortion_matrix"] = calib_data['distortion_coefficients']['data']
motion_capture_data["camera_config"]["rectification_matrix"] = calib_data['rectification_matrix']['data']
motion_capture_data["camera_config"]["projection_matrix"] = calib_data['projection_matrix']['data']


# load calibration file in another way in order to feed it to ROS_TF

rospy.init_node('projector_node')

camera_info = CameraInfo()
with open('ost.yaml', 'r') as file:
    data = yaml.safe_load(file)
    camera_info.width = data['image_width']
    camera_info.height = data['image_height']
    camera_info.K = data['camera_matrix']['data']
    camera_info.D = data['distortion_coefficients']['data']
    camera_info.R = data['rectification_matrix']['data']
    camera_info.P = data['projection_matrix']['data']


#cap = cv2.VideoCapture('record/record1.avi')

file_path = str(data_dir) + "mocap_data.csv"

# regression :
offset = 0

buffer_mocap_data = deque(maxlen = 1000)
buffer_mocap_time= deque(maxlen = 1000)


fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 10.0, (4096, 2160))

with open(file_path, 'r') as file:
    reader = csv.reader(file)
    csv_joints = []
    first_row = next(reader)
    for i in range(len(first_row)):
        if first_row[i] in joints_info:
            csv_joints.append([joints_info[first_row[i]],i])

    print(csv_joints)
    time.sleep(1)

    # Skip the header row

    #for i in range(1):
    #    next(reader)
    # Initialize the list to store the position data
    joint_data = [[] for i in range(32)]

    # Loop through each row of the CSV file

    for i, row in enumerate(reader):
        for local_pos, csv_pos in csv_joints:
            joint_data[local_pos].append(float(row[csv_pos+2]))
            joint_data[local_pos].append(float(row[csv_pos+1]))
            joint_data[local_pos].append(float(row[csv_pos]))
        for i in range(len(joint_data)):   # Fix skeleton ref. point --> move from feet to the point on optical axis!
            joint_data[i][1] -= y_offset


        buffer_mocap_data.append(joint_data)
        buffer_mocap_time.append(row[0])
        joint_data = [[] for i in range(32)]



    for i in range(T_pose_frame - 1, len(buffer_mocap_data)):

        if pause_flag == 1:
            while pause_flag == 1 :
                cv2.waitKey(1)

        #k = readchar.readkey()
        image = cv2.imread(str(data_dir)+"output_"+str(buffer_mocap_time[i])+'.jpg')
        image_width = image.shape[1]
        image_height = image.shape[0]
        print(i, "   Mocap Time: ", buffer_mocap_time[i])
        # Extract the position data (assuming it's in the first 3 columns)

        joint_data = buffer_mocap_data[i].copy()


        #rot1 = body_rotation(joint_data,130*np.pi/200,0,1,0)  # 1 = 13
        rot1 = body_rotation(joint_data, initial_y_rotation, 0, 1, 0)
        rot1 = body_rotation(rot1, initial_x_rotation, 1, 0, 0)
        k=0
        print("Raw data")
        for ji in joint_data :
            print( k , " : ", ji)
            k += 1
        k =0
        print("\nRotated data")
        for ji in rot1:
            print( k , " : ", ji)
            k += 1

        joint_input(rot1,0)

        # Edit Dictionary:
        image_frame["file"] = "output_" + str(buffer_mocap_time[i]) + ".jpg"
        image_frame["image_size_w*h"] = [image_width, image_height]
        image_frame["label_info"]["timestamp"] = buffer_mocap_time[i]


        for k in range(32):
            image_frame["object_annotations"][k]["position_3d"] = joint_data[k]
            if not(k in no_connection_ids):
                image_frame["object_annotations"][k]["visible"] = True
        #print(image_frame)
        new_data = copy.deepcopy(image_frame)
        motion_capture_data["frames"].append(new_data)

        time.sleep(0.05)

    motion_capture_data["skeleton_initial_settings"]["initial_x_rotation"] = 0
    motion_capture_data["skeleton_initial_settings"]["initial_y_rotation"] = -0.147 * np.pi
    motion_capture_data["skeleton_initial_settings"]["initial_z_rotation"] = 0

    text = input("Save json label file? y/n \n")
    if text == "y":
        json_data = json.dumps(motion_capture_data, indent=4)
        # Write the JSON data to a file
        with open("label_data.json", "w") as file:
            file.write(json_data)





    time.sleep(1)
    # Rotation Debugging Part:
    out.release()
    cv2.destroyAllWindows()


