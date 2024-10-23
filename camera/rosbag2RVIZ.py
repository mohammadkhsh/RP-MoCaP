import rospy
import rosbag
import tf2_ros
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sensor_msgs.msg import CameraInfo





def publish_image_and_markers(bag_file):
    rospy.init_node('image_and_markers_publisher', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    image_pub = rospy.Publisher('image_sequence', Image, queue_size=10)
    markers_pub = rospy.Publisher('markers_array', MarkerArray, queue_size=10)
    camera_info_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=10)
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            print(t)
            if topic == '/ROSbag/camera_info':
                # Publish the camera calibration details
                camera_info_pub.publish(msg)

            if topic == '/ROSbag/image_rect':
                # Publish the image
                image_pub.publish(msg)

            elif topic == '/ROSbag/body_joints':
                points_arr = np.array(msg.data)
                points = points_arr.reshape(-1, 3)
                marker_id = 0
                for body_joint in points:  # Assuming 'body_joints' is an array of 3D points
                    body_point_stamped = PointStamped()
                    #body_point_stamped.header = msg.header
                    body_point_stamped.point.x = body_joint[0]
                    body_point_stamped.point.y = body_joint[1]
                    body_point_stamped.point.z = body_joint[2]

                    try:
                        #camera_point_stamped = tf_buffer.transform(body_point_stamped, 'camera_frame', rospy.Duration(1.0))
                        #camera_point = camera_point_stamped.point

                        # Create a marker for the projected point
                        marker = Marker()
                        marker.id = marker_id
                        marker_id += 1
                        #marker.header.stamp = msg.header.timestamp
                        marker.header.frame_id = "body_joints"
                        #marker.header = msg.header
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        #marker.pose.position = camera_point
                        marker.pose.position.x = body_point_stamped.point.x
                        marker.pose.position.y = body_point_stamped.point.y
                        marker.pose.position.z = body_point_stamped.point.z
                        marker.scale.x = 0.1
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1
                        marker.color.r = 1.0
                        marker.color.a = 1.0

                        # Publish the marker
                        marker_array = MarkerArray()
                        marker_array.header.stamp = msg.header.stamp
                        marker_array.markers.append(marker)


                    except tf2_ros.TransformException as e:
                        rospy.logwarn('Failed to transform point: {}'.format(e))

                markers_pub.publish(marker_array)




if __name__ == '__main__':
    bag_file = 'OUTPUT/output1.bag'
    try:

        while(True):
            publish_image_and_markers(bag_file)
    except rospy.ROSInterruptException:
        pass
