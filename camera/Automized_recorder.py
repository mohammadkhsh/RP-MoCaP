import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
from cv_bridge import CvBridge
import cv2
import csv
import numpy as np
import sys
from collections import deque
import os

begin_flag = 0
time_begin = 0
end_code = 0


joint_dict = []
joint_dict.append("Body")               # 0 = SpineBase
joint_dict.append("Chest")              # 1 = SpineMid
joint_dict.append("Head")               # 2* = Neck
joint_dict.append("HeadEnd")            # 3* = Head
joint_dict.append("Hips")               # 4 = ShoulderLeft
joint_dict.append("LeftArm")            # 5* = ElbowLeft
joint_dict.append("LeftFinger")         # 6* = WristLeft
joint_dict.append("LeftFingerEnd")      # 7 = HandLeft
joint_dict.append("LeftFoot")           # 8 = ShoulderRight
joint_dict.append("LeftForearm")        # 9* = ElbowRight
joint_dict.append("LeftHand")           # 10* = WristRight
joint_dict.append("LeftHeel")           # 11*= HandRight
joint_dict.append("LeftLeg")            # 12 = HipLeft
joint_dict.append("LeftShoulder")       # 13* = KneeLeft
joint_dict.append("LeftThigh")          # 14* = AnkleLeft
joint_dict.append("LeftToe")            # 15* = FootLeft
joint_dict.append("LeftToeEnd")         # 16 = HipRight
joint_dict.append("Neck")               # 17 = KneeRight
joint_dict.append("RightArm")           # 18 = AnkleRight
joint_dict.append("RightFinger")        # 19* = FootRight
joint_dict.append("RightFingerEnd")     # 20 = SpineShoulder
joint_dict.append("RightFoot")          # 21 = HandTipLeft
joint_dict.append("RightForearm")       # 22* = ThumbLeft
joint_dict.append("RightHand")          # 23* = HandTipRight
joint_dict.append("RightHeel")          # 24* = ThumbRight
joint_dict.append("RightLeg")           # 25 =
joint_dict.append("RightShoulder")      # 26* =
joint_dict.append("RightThigh")         # 27* =
joint_dict.append("RightToe")           # 28* =
joint_dict.append("RightToeEnd")        # 29 =
joint_dict.append("SpineLow")           # 30 =
joint_dict.append("SpineMid")           # 31 =


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
            return len(mocap_time_stamps) - 1
        elif time_stamp - self.buffer_mocap_data[hi][32][1] < self.buffer_mocap_data[lo][32][1] - time_stamp:
            return hi
        else:
            return lo

    def camera_info_callback(self, msg):
        print("CameraInfo Time_stamp: ", msg.header.stamp, )

    def camera_raw_callback(self, msg):
        print("Camera image RAW Time_stamp: ", msg.header.stamp, )

    def image_callback(self, msg):
        rospy.logwarn("image_callback")
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
        #print("Nearest time stamp from mocap is :", int(self.buffer_mocap_data[matched_index][32][1]))

        self.process_data()

    def mocap_callback(self, msg):
        #print("Received mocap data time : ", rospy.Time.now().to_nsec())
        self.mocap_points = msg.data
        self.joint_arr = np.array(msg.data)
        self.joint_list = self.joint_arr.reshape(-1, 3)
        self.mocap_timestamp = int(int(self.joint_list[32][0])*1000000000000 + int(self.joint_list[32][1])*1000000  + int(self.joint_list[32][2]))
        self.joint_list [32] = self.mocap_timestamp
        self.buffer_mocap_data.append(self.joint_list)
        #print("Sent Mocap data Time_stamp : ", self.mocap_timestamp)
        #self.process_data()

    def process_data(self):
        rospy.logwarn("process_data")
        global begin_flag, time_begin, end_code, joint_dict
        if self.matched_image is not None and self.matched_mocap is not None:
            # Check the time difference between the image and the mocap data



            rospy.logwarn("Time difference between image and mocap data is : %f milli seconds" % round(abs(self.matched_timestamp[0] - self.matched_timestamp[1])/1000000))



            #img = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")   # For: camera/image_raw

            np_arr = np.fromstring(self.matched_image, np.uint8)   # For: camera/image_rect_color/compressed
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


            cv2.imwrite('OUTPUT/output_'+str(int(self.matched_timestamp[0]))+'.jpg',img)  #save nearest mocap time stamp for the image!

                    # Create a CSV writer object
            mocap_writer = csv.writer(mocap_file)

                    # Write a new row to the CSV file
            joint_values = []
            for sublist in self.matched_mocap:
                joint_values.extend(sublist)
            mocap_writer.writerow([str(self.matched_timestamp[0])] + joint_values)


            # Reset the latest image and mocap timestamp to wait for the next data
            self.latest_image = None
            self.mocap_timestamp = None





# Define a callback function to process synchronized messages
def process_message(camera_msg, mocap_msg):
    # Get the image from the camera message
    img = cv2.cvtColor(camera_msg.data, cv2.COLOR_BGR2RGB)
    # Het the MoCap date from skeleton message
    joint_arr = np.array(mocap_msg.data)
    joint_list = points_arr.reshape(-1, 3)
    cv2.imwrite('OUTPUT/output_'+str(rospy.Time.now)+'.jpg',image)

            # Create a CSV writer object
    mocap_writer = csv.writer(mocap_file)

            # Write a new row to the CSV file
    mocap_writer.writerow([rospy.Time.now(), mocap_msg.data])

# Initialize the joints name according to the standard token from MoCap shadowmotion

new_joint_dict = []  # Create a new list to store the modified elements
suffixes = ['.cx', '.cy', '.cz']

for elem in joint_dict:
    for suffix in suffixes:
        new_joint_dict.append(elem + suffix)  # Add the modified element to the new list
new_joint_dict.insert(0, 'time')
print(new_joint_dict)

# Initialize the ROS node
rospy.init_node('Automised_recorder')

filename = "OUTPUT/mocap_data.csv"

if os.path.exists(filename):
    os.remove(filename)
    print("File deleted!")
else:
    print("File does not exist and it will be created!")
mocap_file = open('OUTPUT/mocap_data.csv', mode='a')
mocap_writer = csv.writer(mocap_file)
mocap_writer.writerow(jname for jname in new_joint_dict)

recorder = ImageRecorder()


# Create subscribers for the camera images and motion capture data
#camera_sub = rospy.Subscriber('/camera/image_rect_color', Image)
#mocap_sub = rospy.Subscriber('/MocapSkeleton_msg', Float32MultiArray)

# Create an approximate time synchronizer
#ts = ApproximateTimeSynchronizer([camera_sub, mocap_sub], queue_size=10, slop=0.1)
#ts.registerCallback(process_message)

# Start the ROS node
rospy.spin()
