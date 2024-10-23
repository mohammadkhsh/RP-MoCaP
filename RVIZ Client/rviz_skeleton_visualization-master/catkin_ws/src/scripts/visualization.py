#!/usr/bin/env python
# -*- coding: utf-8 -*-

# visualization.py: code for skeleton visualizatio.
#                   skeleton info is taken from another ROS node.
# Author: Mohammad Khoshkdahan
# Date: 11.1.2023


# import modules
import numpy as np
import yaml
import rospy
import json
import ast
import operator
import time
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray

from math import sqrt

'''
The skeleton is considered as a combination of line strips.
Hence, the skeleton is decomposed into three LINE_STRIP as following:'String' object has no attribute 'strip'
	1) upper_body : [In Red Color] from head to spine base
        2) hands : [In Green Color] frofrom std_msgs.msg import Float32MultiArraym left-hand tip to right-hand tip
	3) legs : [In Blue Color] from left foot to right foot
/sys/module/usbcore/parameters/usbfs_memory_mb
See the link below to get the id of each joint as defined in Kinect v2
source: https://msdn.microsoft.com/en-us/library/microsoft.kinect.jointtype.aspx
new https://docs.microsoft.com/en-us/previous-versions/windows/kinect/dn758662%28v%3dieb.10%29
from std_msgs.msg import Float32MultiArray

upper_body:
	head 3, neck 2, spine shoulder 20,
	spine mid 1, spine base 0

hands:
	hand tip left 21, hand left 7, wrist left 6, elbow left 5
	shoulder left 4, shoulder right 8, elbow right 9
	wrist right 10, hand right 11, hand tip right 23

legs:
	foot left 15, ankle left 1"4, knee left 13
	hip left 12, spine base 0, hip right 16
	knee right 17, ankle right 18, foot right 19
'''

counter = 0
"""
#OLD and ORIGINAL joints info:

joints_info = {'AnkleLeft': 14,
               'AnkleRight': 18,
               'ElbowLeft': 5,
               'ElbowRight': 9,
               'FootLeft': 15,
               'FootRight': 19,
               'HandLeft': 7,
               'HandRight': 11,
               'HandTipLeft': 21,
               'HandTipRight': 23,
               'Head': 3,
               'HipLeft': 12,
               'HipRight': 16,
               'KneeLeft': 13,
               'KneeRight': 17,
               'Neck': 2,
               'ShoulderLeft': 4,sqrt
               'ShoulderRight': 8,
               'SpineBase': 0,
               'SpineMid': 1,
               'SpineShoulder': 20,
               'ThumbLeft': 22,
               'ThumbRight': 24,
               'WristLeft': 6,
               'WristRight': 10}
"""
joints_info = {
        "Body": 0,
        "Chest": 1,
        "Head": 2,
        "HeadEnd": 3,
        "Hips": 4,
        "LeftArm": 5,
        "LeftFinger": 6,
        "LeftFingerEnd": 7,
        "LeftFoot": 8,
        "LeftForearm": 9,
        "LeftHand": 10,
        "LeftHeel": 11,
        "LeftLeg": 12,
        "LeftShoulder": 13,
        "LeftThigh": 14,
        "LeftToe": 15,
        "LeftToeEnd": 16,
        "Neck": 17,
        "RightArm": 18,
        "RightFinger": 19,
        "RightFingerEnd": 20,
        "RightFoot": 21,
        "RightForearm": 22,
        "RightHand": 23,
        "RightHeel": 24,
        "RightLeg": 25,
        "RightShoulder": 26,
        "RightThigh": 27,
        "RightToe": 28,
        "RightToeEnd": 29,
        "SpineLow": 30,
        "SpineMid": 31}


ignore_id = [26,13,0,30]
def axis_change(j_list,p1,p2):
    for joint in j_list:
        joint[p1], joint[p2] = joint[p2], joint[p1]
    return j_list

def callback_comingdata(data):
    global counter
    global skeleton_joints
    clean_data = str(data).replace("\\", "")
    clean_data = clean_data.replace("\n", "")
    # listed_data = json.loads(clean_data[6:])
    listed_data = ast.literal_eval(clean_data[6:])
    listed_data = ast.literal_eval(listed_data)
    rec_t = int(clean_data[clean_data.find('#')+1 : len(clean_data)-1] )
    print("Data No. ",rec_t, type(rec_t))

    # split # !

    #print(type(listed_data), listed_data)

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)




    joint_list = [None] * len(joints_info)
    joint_list = listed_data
    print(len(joint_list))
    #print(joint_list,"\n****")

    # axis change to test if stretching is gone!
    joint_list = axis_change(joint_list,0,2)   ### SWAP axis !

    #skeleton_joints =  [list(map(operator.add, x,y)) for x, y in zip(scaled_init_joint_list , joint_list)]
    skeleton_joints = joint_list
    counter = counter + 1
    print("Reciever counter= ",counter,"   ")#,type(skeleton_joints),"   Data :\n", skeleton_joints)
    #print("msg recieved!")

    print(type(skeleton_joints),skeleton_joints)


class Visualization():
    def __init__(self, ns, skeleton_frame, body_id_text_size, skeleton_line_width, file_name):
        self.ns = ns
        skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=2)

        # define the colors
        colors = [ColorRGBA(0.98, 0.30, 0.30, 1.00),
                  ColorRGBA(0.12, 0.63, 0.42, 1.00),
                  ColorRGBA(0.26, 0.09, 0.91, 1.00),
                  ColorRGBA(0.77, 0.44, 0.14, 1.00),
                  ColorRGBA(0.92, 0.73, 0.14, 1.00),
                  ColorRGBA(0.00, 0.61, 0.88, 1.00),
                  ColorRGBA(1.00, 0.65, 0.60, 1.00),
                  ColorRGBA(0.59, 0.00, 0.56, 1.00)]

        body_id_color = ColorRGBA(0.62, 0.93, 0.14, 1.00)

        #upper_body_ids = [3, 2, 20, 1, 0]
        #hands_ids = [21, 7, 6, 5, 4, 20, 8, 9, 10, 11, 23]
        #legs_ids = [15, 14, 13, 12, 0, 16, 17, 18, 19]

        # new body composition :
        upper_body_ids = [3, 2, 1, 31,0,30] # n=4
        hands_ids = [6, 10, 9,5, 13, 1, 26, 18,22, 23, 19] #n=9 (8+1)
        legs_ids = [15, 11, 14, 30, 27, 24, 28] #n=7 (6+1)

        # define other joint ids
        head_id = 2

        # for the demonstration, I am just using one person recorded data



        #print(bodies)
        rospy.Subscriber("MocapSkeleton", String, callback_comingdata)
        rospy.sleep(1)


        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():

            #rospy.Subscriber("MocapSkeleton", String, callback_comingdata)
            #print(type(skeleton_joints))
            bodies = [skeleton_joints]
            marker_index = 0
            person_index = 1
            marker_array = MarkerArray()

            for body in bodies:
                now = rospy.Time.now()

                marker_index += 1
                upper_body = self.create_marker(
                    marker_index,
                    colors[person_index],
                    Marker.LINE_STRIP,
                    skeleton_line_width,
                    now,
                    skeleton_frame)

                marker_index += 1
                hands = self.create_marker(
                    marker_index,
                    colors[person_index],
                    Marker.LINE_STRIP,
                    skeleton_line_width,
                    now,
                    skeleton_frame)

                marker_index += 1
                legs = self.create_marker(
                    marker_index,
                    colors[person_index],
                    Marker.LINE_STRIP,
                    skeleton_line_width,
                    now,
                    skeleton_frame)

                all_joints = body
                upper_body.points = [Point(
                    all_joints[id][0],
                    all_joints[id][1],
                    all_joints[id][2]) for id in upper_body_ids]
                hands.points = [Point(
                    all_joints[id][0],
                    all_joints[id][1],
                    all_joints[id][2]) for id in hands_ids]
                legs.points = [Point(
                    all_joints[id][0],
                    all_joints[id][1],
                    all_joints[id][2]) for id in legs_ids]

                marker_index += 1
                head_id_marker = self.create_marker(
                    marker_index,
                    body_id_color,
                    Marker.TEXT_VIEW_FACING,
                    body_id_text_size,
                    now,
                    skeleton_frame)
                head_id_marker.text = str(person_index)
                head_id_marker.pose.position = Point(
                    all_joints[head_id][0],
                    all_joints[head_id][1],
                    all_joints[head_id][2])
                marker_index += 1
                '''
                hand_len_marker = self.create_marker(
                        marker_index,
                        body_id_color,
                        Marker.TEXT_VIEW_FACING,
                        hand_len_text_size,
                        now,
                        skeleton_frame)
                hand_len = "{0:.3f}".format(sqrt( pow(skeleton_joints[26][0] - skeleton_joints[22][0], 2) + pow(skeleton_joints[26][1] - skeleton_joints[22][1], 2) + pow(skeleton_joints[26][2] - skeleton_joints[22][2], 2)  ))
                hand_len_marker.text = str(hand_len)
                hand_len_marker.pose.position = Point(
                        (skeleton_joints[26][0]+skeleton_joints[22][0])/2,
                        (skeleton_joints[26][1]+skeleton_joints[22][1])/2,
                        (skeleton_joints[26][2]+skeleton_joints[22][2])/2 + 0.15)
                        marker_array.markers.append(hand_len_marker)
                '''


                marker_array.markers.append(head_id_marker)
                marker_array.markers.append(upper_body)
                marker_array.markers.append(hands)
                marker_array.markers.append(legs)

                for j in body:
                    marker_index += 1
                    only_joint = self.create_marker_sphere(
                        marker_index,
                        colors[person_index+2],
                        Marker.SPHERE,
                        skeleton_line_width+0.03,
                        now,
                        skeleton_frame)
                    only_joint.pose.position.x = j[0]
                    only_joint.pose.position.y = j[1]
                    only_joint.pose.position.z = j[2]
                    marker_array.markers.append(only_joint)

                person_index += 1


            points_arr = np.array(skeleton_joints).flatten()
            points_msg = Float32MultiArray()
            points_msg.data = points_arr.tolist()

            joint_pub.publish(points_msg)
            skeleton_pub.publish(marker_array)
            rate.sleep()



    def create_marker(self, index, color, marker_type, size, time, frame_id):
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.header.stamp = time
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker

    def create_marker_sphere(self, index, color, marker_type, size, time, frame_id):
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.header.stamp = time
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker


if __name__ == '__main__':
    # define some constants
    ns = 'visualization_marker_array'
    body_id_text_size = 0.2
    hand_len_text_size = 0.08
    skeleton_line_width = 0.02


    # initialize ros node
    rospy.init_node(ns, anonymous=True)
    file_name = rospy.get_param('~file')
    skeleton_frame = rospy.get_param('~skeleton_frame')
    rospy.Subscriber("MocapSkeleton", String, callback_comingdata)
    joint_pub = rospy.Publisher('/joint_local_position', Float32MultiArray, queue_size=10)


    # read yaml file as a dictionary
    with open(file_name) as yaml_file:
        init_joints = yaml.safe_load(yaml_file) #,  Loader=yaml.FullLoader)

    init_joint_list = [None] * len(joints_info)
    for joint_name in init_joints:
        init_joint_list[joints_info[joint_name]] = init_joints[joint_name]

    scaled_init_joint_list = [[i/100 for i in row] for row in init_joint_list]

    print(skeleton_frame)
    Visualization(ns, skeleton_frame, body_id_text_size,
                  skeleton_line_width, file_name)
