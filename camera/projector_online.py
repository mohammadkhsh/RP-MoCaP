import rospy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer

import numpy as np
import cv2
import yaml
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import os

bridge = CvBridge()
image = np.zeros((480, 640, 3), dtype=np.uint8)
image_width = 0
image_height = 0
image_processed = 0
chest_click_x = 1998
chest_click_y = 1141
z_offset = 2.6
y_offset = 1.32
pause_ok = 0

upper_body_ids = [3, 2, 1, 31, 0, 30] # n=4
hands_ids = [6, 10, 9, 5, 13, 1, 26, 18, 22, 23, 19] #n=9 (8+1)
legs_ids = [15, 11, 14, 30, 27, 24, 28]
no_connection_ids = [4, 7, 8, 12, 16, 17, 20, 21, 25, 29]

buffer_mocap_data = deque(maxlen = 1000)
buffer_mocap_time= deque(maxlen = 1000)


def mouse_callback(event, x, y, flags, param):
    global chest_click_x, chest_click_y, interpolate_flag, pause_flag
    if event == cv2.EVENT_LBUTTONDOWN:

        print("Clicked pixel:", x, y)
        chest_click_x = x
        chest_click_y = y

        #interpolate_flag *= -1
    if event == cv2.EVENT_RBUTTONDOWN:
        pause_flag *= -1

#############################################################################

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

###############################################################################

def round_point(point):
    return Point(round(point[0], 4), round(point[1], 4), round(point[2], 4))

###############################################################################

def visualizer(joint_state, image, time_stamp, trans_f = 0):
    global pause_ok, z_offset, chest_click_x, chest_click_y
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
    cv2.putText(img=image, text="Mocap Time: "+str(time_stamp),org=(10,25) ,
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
    #out.write(image)

    now_secs = rospy.Time.now().to_sec()
    now_nsecs = rospy.Time.now().to_nsec()
    time_now_ns = now_nsecs % 1000000000000000000
    rospy.logwarn("Visualization Latency : %d milli seconds" % round(abs(time_stamp - time_now_ns )/100000))

    cv2.setMouseCallback('Image', mouse_callback)

    key = cv2.waitKey(1)

###############################################################################

class ImageRecorder:

    def __init__(self):
        self.mocap_timestamp = None
        self.latest_image = None
        self.mocap_points = None
        self.image_sub = rospy.Subscriber('/camera/image_rect_color/compressed', CompressedImage, self.image_callback)
        self.mocap_sub = rospy.Subscriber('/MocapSkeleton_msg', Float32MultiArray, self.mocap_callback)
        #self.image_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        #self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_raw_callback)

        self.bridge = CvBridge()

        self.buffer_mocap_data = deque(maxlen = 1000)
        self.matched_mocap = [[0]*3]*32
        self.matched_image = None
        self.matched_timestamp = [0,0]

###############################################################################

    def find_nearest_time_stamp(self, time_stamp):
        # Binary search in the mocap time stamps
        lo = 0
        hi = len(self.buffer_mocap_data) - 1
        while lo <= hi:
            mid = (lo + hi) // 2
            if self.buffer_mocap_data[mid][32][1] < time_stamp:
                lo = mid + 1
            elif self.buffer_mocap_data[mid][32][1] > time_stamp:
                hi = mid - 1
            else:
                return mid
        # If the exact time stamp is not found, return the closest one
        if hi < 0:
            return 0
        elif lo >= len(self.buffer_mocap_data):
            return len(self.buffer_mocap_data) - 1
        elif time_stamp - self.buffer_mocap_data[hi][32][1] < self.buffer_mocap_data[lo][32][1] - time_stamp:
            return hi
        else:
            return lo

###############################################################################

    def camera_info_callback(self, msg):
        print("CameraInfo Time_stamp: ", msg.header.stamp, )

##############################

    def camera_raw_callback(self, msg):
        print("Camera image RAW Time_stamp: ", msg.header.stamp, )

###############################################################################
    def image_callback(self, msg):
        print("Image Received Time_stamp: ", rospy.Time.now().to_nsec())
        secs = msg.header.stamp.secs
        nsecs = msg.header.stamp.nsecs

        # Combining the seconds and nanoseconds into a single integer
        time_in_ns = ((secs%1000000000) * 1000000000) + nsecs

        # Converting the time stamp to an integer
        self.image_timestamp = int(time_in_ns)
        self.latest_image = msg.data #.data !!! for compressed
        matched_index = self.find_nearest_time_stamp(self.image_timestamp)
        self.matched_timestamp[0] = int(self.buffer_mocap_data[matched_index][32][1])
        self.matched_timestamp[1] = self.image_timestamp

        self.matched_mocap = self.buffer_mocap_data[matched_index]
        self.matched_image = self.latest_image
        print("Image containing Time_stamp : ",self.image_timestamp )

        self.process_data()

###############################################################################

    def mocap_callback(self, msg):
        #print("Received mocap data time : ", rospy.Time.now().to_nsec())
        self.mocap_points = msg.data
        self.joint_arr = np.array(msg.data)
        self.joint_list = self.joint_arr.reshape(-1, 3)
        self.mocap_timestamp = int(int(self.joint_list[32][0])*1000000000000 + int(self.joint_list[32][1])*1000000  + int(self.joint_list[32][2]))
        self.joint_list [32] = self.mocap_timestamp
        self.buffer_mocap_data.append(self.joint_list)
        #print("Sent Mocap data Time_stamp : ", self.mocap_timestamp)

###############################################################################

    def process_data(self):
        rospy.logwarn("process_data")
        global begin_flag, time_begin, end_code, joint_dict
        if self.matched_image is not None and self.matched_mocap is not None:
            # Check the time difference between the image and the mocap data
            rospy.logwarn("Time difference between image and mocap data is : %d milli seconds" % round(abs(self.matched_timestamp[0] - self.matched_timestamp[1])/1000000))
            #img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")   # For: camera/image_raw
            np_arr = np.fromstring(self.matched_image, np.uint8)   # For: camera/image_rect_color/compressed
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv2.imwrite('OUTPUT/output_'+str(int(self.matched_timestamp[0]))+'.jpg',img)  #save nearest mocap time stamp for the image!
            joint_values = []
            for sublist in self.matched_mocap:
                joint_values.extend(sublist)
            #mocap_writer.writerow([str(self.matched_timestamp[0])] + joint_values)
            ### Initial Y Offset Compensation
            for i in range(len(self.matched_mocap)):   # Fix skeleton ref. point --> move from feet to the point on optical axis!
                self.matched_mocap[i][1] -= y_offset
            ### Initial Rotation Compensation
            rotation_1 = body_rotation(self.matched_mocap,0,0,1,0)
            rotated_joint_values = body_rotation(rotation_1,-0.147*np.pi,0,1,0)
            visualizer(rotated_joint_values, img, self.matched_timestamp[0])

            # Reset the latest image and mocap timestamp to wait for the next data
            self.latest_image = None
            self.mocap_timestamp = None



###############################################################################
############ MAIN CODE ##############
###############################################################################


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

# initialize ROS node
rospy.init_node('Online_Projector_V1',anonymous=True)

# start ROS node

recorder = ImageRecorder()

rospy.spin()
