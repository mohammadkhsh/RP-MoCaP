import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import csv
import copy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray, ImageMarker
from shape_msgs.msg import Mesh
from geometry_msgs.msg import Point
from shape_msgs.msg import MeshTriangle
from geometry_msgs.msg import Point32
from std_msgs.msg import ColorRGBA




def project_points_to_image(image_path, points):
    # Load the image using OpenCV
    image = cv2.imread(image_path)

    # Check if the image was loaded successfully
    if image is None:
        rospy.logerr("Failed to load the image: %s", image_path)
        return None

    # Create a copy of the image to draw projected points
    projected_image = image.copy()

    # Iterate over the 3D points
    for point in points:
        # Project the 3D point to 2D image coordinates
        projected_point = (int(point[0]), int(point[1]))

        # Draw a circle at the projected point on the image
        cv2.circle(projected_image, projected_point, 5, (0, 255, 0), -1)

    return projected_image





def publish_image_to_rviz(image, transform):
    marker_array = MarkerArray()

    # Create a marker for the mesh
    marker = Marker()
    marker.header.frame_id = "/world"
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.id = 0

    # Create a mesh and set its vertices
    mesh = Mesh()
    mesh.vertices = [
        Point(x=0, y=0, z=0),
        Point(x=1, y=0, z=0),
        Point(x=0, y=1, z=0),
        Point(x=1, y=1, z=0)
    ]

    # Create mesh triangles
    mesh.triangles = [
        MeshTriangle(vertex_indices=[0, 1, 2]),
        MeshTriangle(vertex_indices=[2, 1, 3])
    ]

    # Assign colors to the vertices
    mesh.colors = [
        ColorRGBA(r=1, g=0, b=0, a=1),
        ColorRGBA(r=0, g=1, b=0, a=1),
        ColorRGBA(r=0, g=0, b=1, a=1),
        ColorRGBA(r=1, g=1, b=0, a=1)
    ]

    marker.mesh = mesh
    marker.pose = transform
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    marker_array.markers.append(marker)

    marker_publisher.publish(marker_array)






if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("image_projection_node")

    # Create a publisher for the image messages
    image_publisher = rospy.Publisher("/projected_image", Image, queue_size=10)

    # Create a publisher for the MarkerArray messages
    marker_publisher = rospy.Publisher("image_markers", MarkerArray, queue_size=1)

    # Convert the projected image to ROS image message

    # Create a TF broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Set the image frame and world frame
    image_frame = "image_frame"
    world_frame = "world"
    bridge = CvBridge()

    # Create a transform between the image frame and world frame
    transform = TransformStamped()
    transform.header.frame_id = world_frame
    transform.child_frame_id = image_frame
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    # Specify the image directory
    image_directory = "OUTPUT"

    # Specify the 3D points for each image
    points_list = [
       [[100, 100], [200, 200], [300, 300]],
       [[200, 100], [300, 200], [400, 300]],
       # Add more points for each image as needed
    ]

    # Iterate over the images and their corresponding points
    while(True): #for i, points in enumerate(points_list):
       points = points_list[0]
       # Construct the image path
       image_path = f"{image_directory}/output.jpg"

       # Project the points onto the image
       projected_image = project_points_to_image(image_path, points)

       ros_image = bridge.cv2_to_imgmsg(projected_image, "bgr8")

       if projected_image is not None:
           # Publish the projected image to RVIZ
           publish_image_to_rviz(projected_image, transform)

       # Wait for a short duration before processing the next image
       rospy.sleep(0.5)
