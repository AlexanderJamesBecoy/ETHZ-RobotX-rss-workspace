#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose

from object_detection_msgs.msg import ObjectDetectionInfoArray



import numpy as np

tf_listener = None
detection_info_global_pub = rospy.Publisher('/object_detector/detection_info_global', ObjectDetectionInfoArray, queue_size=10)


def tf_callback(msg):  
    for transform in msg.transforms:
        # world -> base_link
        if transform.header.frame_id == "odom_graph_msf" and transform.child_frame_id == "world_graph_msf":
            tf_odomGraph2worldGraph = transform
        if transform.header.frame_id == "base_link" and transform.child_frame_id == "odom_graph_msf":
            tf_base2odomGraph = transform
           
def tf_static_callback(msg):
    for transform in msg.transforms:
        # base_link -> rgb_camera_link
        if transform.header.frame_id == "base_link" and transform.child_frame_id == "rgb_camera_link":
            tf_base2cam = transform
        # rgb_camera_link -> rgb_camera_optical_link
        if transform.header.frame_id == "rgb_camera_link" and transform.child_frame_id == "rgb_camera_optical_link":
            tf_cam2opt = transform


def detection_callback(msg):
    global detection_info_global_pub
    detection_info_global = msg

    for obj_i, obj_info in enumerate(msg.info):
        try: 
            # Transform the camera frame to the world frame
            object_pose = tf2_geometry_msgs.PoseStamped()
            object_pose.header.frame_id = "rgb_camera_optical_link"
            object_pose.pose.position = obj_info.position # in frame of rgb_camera_optic_link
            object_pose.pose.orientation.w = 1.0 # Don't care about the orientation
            transform = tf_buffer.lookup_transform("world_graph_msf", "rgb_camera_optical_link", rospy.Time(0))
            transformed_pose = do_transform_pose(object_pose, transform)

            # Write transformed pos to global msg
            detection_info_global.info[obj_i].class_id = msg.info[obj_i].class_id
            detection_info_global.info[obj_i].position = transformed_pose.pose.position

            # Log
            #rospy.loginfo(f"Object {detection_info_global.info[obj_i].class_id} in world frame: {detection_info_global.info[obj_i].position}")
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF transform error: {e}")

    # Publish
    detection_info_global_pub.publish(detection_info_global)

    


def main():
    global tf_buffer

    rospy.init_node('tf_listener', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
  
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    rospy.Subscriber('/tf_static', TFMessage, tf_static_callback)
    rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, detection_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
