import rospy
import rosbag
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sensor_msgs.msg import CameraInfo
from custom_msg_f32.msg import Float32MultiArrayPlusStamp
from geometry_msgs.msg import Quaternion
import math
from geometry_msgs.msg import Vector3, Point


init_rotation = 0.416

upper_body_ids = [3, 2, 1, 31, 0, 30] # n=4
handsLeft_ids = [6, 10, 9, 5, 13, 1]
handsRight_ids = [1,26, 18, 22, 23, 19] #n=9 (8+1)
legs_ids = [15, 11, 14, 30, 27, 24, 28]
no_connection_ids = [4, 7, 8, 12, 16, 17, 20, 21, 25, 29]

def calculate_rhand_rotation(hand_joint):

    tan_angle = hand_joint[0] / hand_joint[2]

    rotation_angle = math.atan(tan_angle)

    # Convert the angle to degrees if needed
    rotation_angle_degrees = math.degrees(rotation_angle)

    return rotation_angle


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


def callback(msg):
    print(msg.header.stamp)
    markers_pub = rospy.Publisher('markers_array', MarkerArray, queue_size=10)
    joint_list = [list(msg.data[i:i+3]) for i in range(0, len(msg.data), 3)]
    # we need this angle at T-pose moment to fix the initial orientation issue!
    print(calculate_rhand_rotation(joint_list[23]))
    rotated_joints = body_rotation(joint_list , init_rotation, 0, 1, 0)
    points_arr = np.array(rotated_joints)
    points = points_arr.reshape(-1, 3)

    for i in range(len(points)):
        temp = points[i][0]
        points[i][0] = points[i][2]
        points[i][2] = -temp

    marker_id = 0
    marker_array = MarkerArray()

    # Create a marker for the lines
    line_marker = []

    for i in range(4):
        temp_marker = Marker()
        temp_marker.header.frame_id = 'body_joints'  # Set the frame ID
        temp_marker.ns = 'lines'
        temp_marker.id = i
        temp_marker.action = Marker.ADD
        temp_marker.type = Marker.LINE_STRIP
        temp_marker.scale.x = 0.02
        temp_marker.scale.y = 0.02
        temp_marker.color.r = 0.0   # Set the line color to green
        temp_marker.color.g = 1.0
        temp_marker.color.b = 0.0
        temp_marker.color.a = 1.0
        line_marker.append(temp_marker)




    line_marker[0].points = [Point(
        points[id][0],
        points[id][1],
        points[id][2]) for id in upper_body_ids]
    line_marker[1].points = [Point(
        points[id][0],
        points[id][1],
        points[id][2]) for id in handsRight_ids]
    line_marker[2].points = [Point(
            points[id][0],
            points[id][1],
            points[id][2]) for id in handsLeft_ids]
    line_marker[3].points = [Point(
        points[id][0],
        points[id][1],
        points[id][2]) for id in legs_ids]
    for body_joint in points:  # Assuming 'body_joints' is an array of 3D points
        body_point_stamped = PointStamped()

        # Z values and X values must be exchanged!
        body_point_stamped.point.x = body_joint[0]
        body_point_stamped.point.y = body_joint[1]
        # Z axis direction must be reversed
        body_point_stamped.point.z = body_joint[2]

        # Create a marker for the projected point
        marker = Marker()
        marker.id = marker_id
        marker_id += 1
        #marker.header.stamp = msg.header.stamp
        marker.header.frame_id = "body_joints"
        marker.header.stamp = msg.header.stamp

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = body_point_stamped.point.x
        marker.pose.position.y = body_point_stamped.point.y
        marker.pose.position.z = body_point_stamped.point.z
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.a = 1.0
        quaternion = Quaternion()
        quaternion.x = 0.0  # Set the x component of the quaternion
        quaternion.y = 0.0  # Set the y component of the quaternion
        quaternion.z = 0.0  # Set the z component of the quaternion
        quaternion.w = 1.0  # Set the w component of the quaternion
        marker.pose.orientation = quaternion
        marker.lifetime = rospy.Duration(1)  # 1 second


        marker_array.markers.append(marker)


    marker_array.markers.append(line_marker[0])
    marker_array.markers.append(line_marker[1])
    marker_array.markers.append(line_marker[2])
    marker_array.markers.append(line_marker[3])
    print(marker.header.stamp)
    markers_pub.publish(marker_array)

    #print(marker_array)




if __name__ == '__main__':
    bag_file = 'OUTPUT/output.bag'
    rospy.init_node("Visualization", anonymous=True)
    rospy.Subscriber("/body_joints", Float32MultiArrayPlusStamp, callback)
    try:

        while(True):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
