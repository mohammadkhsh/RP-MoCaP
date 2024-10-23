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


bridge = CvBridge()
image = np.zeros((480, 640, 3), dtype=np.uint8)
image_width = 0
image_height = 0
image_processed = 0
# define joint callback function
chest_click_x = 2128
chest_click_y = 1123 #975
z_offset = 3 #2.6
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
i+offse
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


upper_body_ids = [3, 2, 1, 31, 0, 30] # n=4
hands_ids = [6, 10, 9, 5, 13, 1, 26, 18, 22, 23, 19] #n=9 (8+1)
legs_ids = [15, 11, 14, 30, 27, 24, 28]
no_connection_ids = [4, 7, 8, 12, 16, 17, 20, 21, 25, 29]

def joint_input(joint_state,trans_f = 0):
    global pause_ok
    x_joint_image = []
    y_joint_image = []

    # perform any necessary transformations to joint positions to bring them into camera coordinate system



    points_arr = np.array(joint_state)
    points = points_arr.reshape(-1, 3)
    #points = joint_state
    points[:, 2] += z_offset

    rounded_points = [round_point(point) for point in points]
    #print (rounded_points)
    # apply any necessary transformations to bring point into camera coordinate system

    # project 3D points onto image plane and draw circles
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

    #cv2.putText(img=image, text='Body distance is = ' + str(z_offset),org=(5,20) ,fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.8, color=(0, 0, 255),thickness=1)
    cv2.putText(img=image, text="Mocap Time: "+str(buffer_mocap_time[i]),org=(10,25) ,
    fontFace=cv2.FONT_HERSHEY_TRIPLEX, fontScale=0.8, color=(0, 0, 255),thickness=1)



    mark_point = np.array([points[1][0], points[1][1], points[1][2] ,1])


# apply projection matrix to 3D point
    proj_point = proj_matrix.dot(mark_point)


    # divide by Z-coordinate to convert to pixel coordinates
    pixel_x = int(proj_point[0] / proj_point[2] + 0.5)
    pixel_y = int(proj_point[1] / proj_point[2] + 0.5)
    pixel_y = image_height - pixel_y
    offset_chest_x = pixel_x - chest_click_x
    offset_chest_y = pixel_y - chest_click_y
    cont = 0
    for point in points:

        mark_point = np.array([point[0], point[1], point[2] ,1])


    # apply projection matrix to 3D point
        proj_point = proj_matrix.dot(mark_point)


        # divide by Z-coordinate to convert to pixel coordinates
        pixel_x = int(proj_point[0] / proj_point[2] + 0.5)
        pixel_y = int(proj_point[1] / proj_point[2] + 0.5)
        pixel_y = image_height - pixel_y

        pixel_x -= offset_chest_x
        pixel_y -= offset_chest_y
        x_joint_image.append(pixel_x)
        y_joint_image.append(pixel_y)

        center_point = (int(image_width/2) , int(image_height/2))

        # apply rectification matrix (if applicable)
    #    if rect_matrix is not None:
    #        rect_point = rect_matrix.dot(joint_point)
    #        pixel_x = int(rect_point[0] / rect_point[2] + 0.5)
    #        pixel_y = int(rect_point[1] / rect_point[2] + 0.5)

            # draw circle at joint position
        if trans_f == 0:
            if not(cont in no_connection_ids) :
                cv2.circle(image, (pixel_x, pixel_y), 4, (255, 0, 255), -1)
        #cv2.circle(image, center_point, 7, (255, 0, 0), -1)
        cont += 1
    # Joints Connections :
    if trans_f == 0:
        line_color = (0,255,255,255)
    else:
        line_color = (255,255,255,100)
    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[upper_body_ids[j]], y_joint_image[upper_body_ids[j]]), (x_joint_image[upper_body_ids[j+1]], y_joint_image[upper_body_ids[j+1]]),line_color,2)
        #print((x_joint_image[upper_body_ids[j]], y_joint_image[upper_body_ids[j]]))
    for j in range(len(hands_ids)-1):
        cv2.line(image, (x_joint_image[hands_ids[j]], y_joint_image[hands_ids[j]]), (x_joint_image[hands_ids[j+1]], y_joint_image[hands_ids[j+1]]),line_color,2)
    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[legs_ids[j]], y_joint_image[legs_ids[j]]), (x_joint_image[legs_ids[j+1]], y_joint_image[legs_ids[j+1]]),line_color,2)

    #print("\n\nPixel pos: ",pixel_x , pixel_y,"\n")
    # display image
    cv2.imshow('Image', image)
    cv2.setMouseCallback('Image', mouse_callback)
    #cv2.imwrite('OUTPUT/output '+str(frame_num)+'.jpg',image)
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

print("camera matrix :  ",camera_matrix,"\ndistortion matrix :  ",dist_coeffs,"\nrectification matrix :  ",rect_matrix,"\ncamera matrix :  ",proj_matrix)

cap = cv2.VideoCapture('record/record1.avi')
file_path = "OUTPUT/mocap_data.csv"

# regression :

offset = 17

buffer_mocap_data = deque(maxlen = 1000)
buffer_mocap_time= deque(maxlen = 1000)



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
            joint_data[local_pos].append(float(row[csv_pos]))
            joint_data[local_pos].append(float(row[csv_pos+1]))
            joint_data[local_pos].append(float(row[csv_pos+2]))
        buffer_mocap_data.append(joint_data)
        buffer_mocap_time.append(row[0])
        joint_data = [[] for i in range(32)]

    for i in range(len(buffer_mocap_data)):

        if pause_flag == 1:
            while pause_flag == 1 :
                cv2.waitKey(1)

        #k = readchar.readkey()
        image = cv2.imread("OUTPUT/output_"+str(buffer_mocap_time[i+offset])+'.jpg')

        print("Mocap Time: ",buffer_mocap_time[i])
        # Extract the position data (assuming it's in the first 3 columns)



        joint_data = buffer_mocap_data[i]
        rot1 = body_rotation(joint_data,13*np.pi/20,0,1,0)
        rot2 = body_rotation(rot1,90*np.pi/500 * (-1),1,0,0)
        joint_input(rot2,0)

        # Do something with the position data (e.g. process it)
        time.sleep(0.1)

    # Do something with the list of position data (e.g. plot it)
    rot = rot2
    j=0
    #rot = body_rotation(rot,13*np.pi/20,0,1,0)
    while (False):
        j +=1
           ## rotate s.t. Y axis
        rot = body_rotation(rot,j*np.pi/500 * (-1) ,1,0,0)    ## rotate s.t. X axis
        #rot = body_rotation(rot,0,0,0,1)    ## rotate s.t. Z axis
        image = b_img.copy()
        joint_input(rot)
        print("Y Rot : ",j," * ","-pi/500")
        k = readchar.readkey()


