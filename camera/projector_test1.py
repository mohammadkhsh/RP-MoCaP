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

bridge = CvBridge()
image = np.zeros((480, 640, 3), dtype=np.uint8)
image_width = 0
image_height = 0
image_processed = 0
# define joint callback function
chest_click_x = 0
chest_click_y = 0
z_offset = 2.6


def mouse_callback(event, x, y, flags, param):
    global chest_click_x, chest_click_y
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked pixel:", x, y)
        chest_click_x = x
        chest_click_y = y


def image_callback(image_data):

    global image, image_width, image_height, image_processed
    try:
        image = bridge.imgmsg_to_cv2(image_data, "bgr8")
        image_width = image_data.width
        image_height = image_data.height
        image_processed = 0
    except CvBridgeError as e:
        print(e)

def round_point(point):
    return Point(round(point[0], 4), round(point[1], 4), round(point[2], 4))


upper_body_ids = [3, 2, 1, 31, 0, 30] # n=4
hands_ids = [6, 10, 9, 5, 13, 1, 26, 18, 22, 23, 19] #n=9 (8+1)
legs_ids = [15, 11, 14, 30, 27, 24, 28]

def joint_callback(joint_state):
    global image_processed
    # extract joint positions
    if image_processed == 1 :
        return None
    x_joint_image = []
    y_joint_image = []

    # perform any necessary transformations to joint positions to bring them into camera coordinate system


    # create 3D point using joint position
    points_arr = np.array(joint_state.data)
    points = points_arr.reshape(-1, 3)
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

    cv2.putText(img=image, text='Body distance is = ' + str(z_offset),org=(5,20) ,
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
        cv2.circle(image, (pixel_x, pixel_y), 4, (255, 0, 255), -1)
        cv2.circle(image, center_point, 7, (255, 0, 0), -1)

    # Joints Connections :

    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[upper_body_ids[j]], y_joint_image[upper_body_ids[j]]), (x_joint_image[upper_body_ids[j+1]], y_joint_image[upper_body_ids[j+1]]),(0,255,255),2)
        #print((x_joint_image[upper_body_ids[j]], y_joint_image[upper_body_ids[j]]))
    for j in range(len(hands_ids)-1):
        cv2.line(image, (x_joint_image[hands_ids[j]], y_joint_image[hands_ids[j]]), (x_joint_image[hands_ids[j+1]], y_joint_image[hands_ids[j+1]]),(0,255,255),2)
    for j in range(len(upper_body_ids)-1):
        cv2.line(image, (x_joint_image[legs_ids[j]], y_joint_image[legs_ids[j]]), (x_joint_image[legs_ids[j+1]], y_joint_image[legs_ids[j+1]]),(0,255,255),2)

    #print("\n\nPixel pos: ",pixel_x , pixel_y,"\n")
    # display image
    cv2.imshow('Image', image)
    cv2.setMouseCallback('Image', mouse_callback)
    #image_processed = 1
    cv2.waitKey(1)


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
rospy.init_node('joint_projection_node',anonymous=True)

# create subscribers
joint_sub = rospy.Subscriber('/joint_local_position', Float32MultiArray, joint_callback,queue_size=1)
image_sub = rospy.Subscriber("/camera/image_rect_color",Image,image_callback)
# start ROS node

rospy.spin()
