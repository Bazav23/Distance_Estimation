#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import math

class ObjectDistanceEstimator:
    def __init__(self):
        self.camera_height = 1.5  # Height of the camera from the ground in meters
        self.camera_tilt_angle = 0.02  # Tilt angle of the camera in radians (0 if the camera is parallel to the ground)
        self.image_height = 1024  # Height of the image in pixels
        self.focal_length = 1739  # Focal length in pixels

        rospy.init_node('object_distance_estimator', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        rospy.spin()

    def bounding_boxes_callback(self, msg):
        for box in msg.bounding_boxes:
            if box.Class in ["car", "person"]:
                distance = self.calculate_distance(box)
                rospy.loginfo("Detected object: %s at distance: %.2f meters", box.Class, distance)

    def calculate_distance(self, box):
        pixel_y = box.ymax  # y-coordinate of the bottom of the bounding box
        rospy.logdebug(f"pixel_y: {pixel_y}, image_height/2: {self.image_height / 2}, focal_length: {self.focal_length}")

        # Calculate the angle of depression/elevation
        angle = math.atan2((pixel_y - self.image_height / 2), self.focal_length)
        total_angle = angle - self.camera_tilt_angle
        rospy.logdebug(f"angle: {angle}, total_angle: {total_angle}")

        # Ensure the angle is not zero or negative to avoid unrealistic distance
        if total_angle <= 0:
            rospy.logwarn("Total angle is non-positive, returning infinity as distance")
            return float('inf')

        # Calculate the distance using the height of the camera and the angle
        distance = self.camera_height / math.tan(total_angle)
        rospy.logdebug(f"distance: {distance}")

        return distance

if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ObjectDistanceEstimator node")
        ObjectDistanceEstimator()
    except rospy.ROSInterruptException:
        pass

