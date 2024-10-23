import cv2
import yaml
import numpy as np


def mouse_callback(event, x, y, flags, param):
    global chest_click_x, chest_click_y
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Clicked pixel:", x, y)
        click_x = x
        click_y = y
        point_2_meter_1 = np.array([1,2,2,1])
        point_2_meter_2 = np.array([0,2,2,1])
        proj_p2m_1 = proj_matrix.dot(point_2_meter_1)
        proj_p2m_2 = proj_matrix.dot(point_2_meter_2)
        p2m_1_x = int(proj_p2m_1[0] / proj_p2m_1[2] + 0.5)
        p2m_2_x = int(proj_p2m_2[0] / proj_p2m_2[2] + 0.5)
        p2m_1_y = int(proj_p2m_1[1] / proj_p2m_1[2] + 0.5)
        p2m_2_y = int(proj_p2m_2[1] / proj_p2m_2[2] + 0.5)
        click_x -= p2m_2_x
        click_y -= p2m_2_y
        p2m_1_x += click_x
        p2m_1_y += click_y

        cv2.line(frame, (x,y),(p2m_1_x,p2m_1_y),(0,0,255),2)
        cv2.imshow('Image', frame)




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


cap = cv2.VideoCapture('output.avi')

# Set the frame index to 99 (since frame indexing starts at 0)
cap.set(cv2.CAP_PROP_POS_FRAMES, 120)

# Read the frame
ret, frame = cap.read()

# calibrating line:
point_5_meter_1 = np.array([1,2,5,1])
point_5_meter_2 = np.array([0,2,5,1])
proj_p5m_1 = proj_matrix.dot(point_5_meter_1)
proj_p5m_2 = proj_matrix.dot(point_5_meter_2)
p5m_1_x = int(proj_p5m_1[0] / proj_p5m_1[2] + 0.5)
p5m_2_x = int(proj_p5m_2[0] / proj_p5m_2[2] + 0.5)
p5m_1_y = int(proj_p5m_1[1] / proj_p5m_1[2] + 0.5)
p5m_2_y = int(proj_p5m_2[1] / proj_p5m_2[2] + 0.5)

cv2.line(frame, (p5m_2_x,p5m_2_y),(p5m_1_x,p5m_1_y),(0,0,255),2)


# Save the frame as an image file
cv2.imshow('Image', frame)
cv2.setMouseCallback('Image', mouse_callback)
#image_processed = 1
cv2.waitKey(1)

# Release the video file
cap.release()


while(True):
    cv2.waitKey(1)
