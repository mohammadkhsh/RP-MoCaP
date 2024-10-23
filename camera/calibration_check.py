import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import sys
import os
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge, CvBridgeError


# Distance and Length for the square  (calibration check!)
l = 0.5
d = 2.5


click_x = 2110
click_y = 1075


# create bridge object
bridge = CvBridge()

# create window for displaying image
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

# define size of the checkerboard pattern
CHECKERBOARD_SIZE = (9, 7)
square_size = 0.02


# define termination criteria for corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# create object points
objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)

# create empty arrays to store object points and image points
object_points = []
image_points = []

# load camera calibration data
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

proj_matrix = np.array(calib_data['projection_matrix']['data']).reshape((3, 4))


def mouse_callback(event, x, y, flags, param):
    global click_x, click_y, interpolate_flag, pause_flag
    if event == cv2.EVENT_LBUTTONDOWN:

        print("Clicked pixel:", x, y)
        click_x = x
        click_y = y

        #interpolate_flag *= -1
    if event == cv2.EVENT_RBUTTONDOWN:
        pause_flag *= -1

# define image callback function
def image_callback(msg):
    global click_x, click_y
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # calibrating line:
    point_tl = np.array([-l/2,-l/2,d+2,1])
    point_tr = np.array([-l/2,l/2,d,1])
    point_bl = np.array([l/2,-l/2,d+2,1])
    point_br = np.array([l/2,l/2,d,1])
    proj_tl = proj_matrix.dot(point_tl)
    proj_tr = proj_matrix.dot(point_tr)
    proj_bl = proj_matrix.dot(point_bl)
    proj_br = proj_matrix.dot(point_br)

    ptl_x = int(proj_tl[0] / proj_tl[2] + 0.5) #- offset_chest_x
    ptl_y = int(proj_tl[1] / proj_tl[2] + 0.5) #- offset_chest_y

    offset_x = ptl_x - click_x
    offset_y = ptl_y - click_y

    ptl_x = int(proj_tl[0] / proj_tl[2] + 0.5) - offset_x
    ptl_y = int(proj_tl[1] / proj_tl[2] + 0.5) - offset_y
    ptr_x = int(proj_tr[0] / proj_tr[2] + 0.5) - offset_x
    ptr_y = int(proj_tr[1] / proj_tr[2] + 0.5) - offset_y
    pbl_x = int(proj_bl[0] / proj_bl[2] + 0.5) - offset_x
    pbl_y = int(proj_bl[1] / proj_bl[2] + 0.5) - offset_y
    pbr_x = int(proj_br[0] / proj_br[2] + 0.5) - offset_x
    pbr_y = int(proj_br[1] / proj_br[2] + 0.5) - offset_y



    cv2.line(image, (ptl_x,ptl_y),(ptr_x,ptr_y),(0,0,255),2)
    cv2.line(image, (ptr_x,ptr_y),(pbr_x,pbr_y),(0,0,255),2)
    cv2.line(image, (pbr_x,pbr_y),(pbl_x,pbl_y),(0,0,255),2)
    cv2.line(image, (pbl_x,pbl_y),(ptl_x,ptl_y),(0,0,255),2)

    cv2.imshow('Image', image)
    cv2.setMouseCallback('Image', mouse_callback)
    cv2.waitKey(1)



    '''
    try:
        # convert ROS image to OpenCV image
        img = bridge.imgmsg_to_cv2(msg, "bgr8")

        # resize image
        img = cv2.resize(img, (640, 480))

        # find chessboard corners
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

        # refine corner location
        if ret == True:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(img, CHECKERBOARD_SIZE, corners_refined, ret)

            # store object points and image points
            object_points.append(objp)
            image_points.append(corners_refined)

            # check calibration
            error, _, _, _, _ = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], camera_matrix, dist_coeffs)
            print("Reprojection error:", error)

        # display image
        cv2.imshow("Image", img)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)

    '''

# initialize node and subscribe to camera topic
rospy.init_node('calibration_check', anonymous=True)
rospy.Subscriber('/camera/image_rect_color', Image, image_callback, queue_size=1)
rospy.Rate(5)
# spin node
rospy.spin()

# close window
cv2.destroyAllWindows()
