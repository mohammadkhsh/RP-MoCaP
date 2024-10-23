import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VideoRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.video = None

    def image_callback(self, msg):
        # Convert ROS image message to numpy array
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # If video writer is not initialized, create one with maximum possible resolution
        if self.video is None:
            height, width, channels = img.shape
            self.video = cv2.VideoWriter('mohammad3.avi', cv2.VideoWriter_fourcc(*'MJPG'), 7, (width, height))

        # Write the image to the video file
        self.video.write(img)

    def __del__(self):
        # Release the video writer and close the video file
        if self.video is not None:
            self.video.release()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('video_recorder')
    recorder = VideoRecorder()
    rospy.Subscriber('/camera/image_rect_color', Image, recorder.image_callback)
    rospy.spin()
