#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes

class ObjectDistanceEstimator:
    def __init__(self):
        self.focal_length = 1739  # Assuming a typical focal length in pixels
        self.principal_point = (512,612)  # Assuming the principal point is at the center of a 2048x2448 image

        rospy.init_node('object_distance_estimator', anonymous=True)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        rospy.spin()

    def bounding_boxes_callback(self, msg):
        for box in msg.bounding_boxes:
            known_width = self.get_known_width(box.Class)
            if known_width is not None:
                distance = self.calculate_distance(box, known_width)
                rospy.loginfo("Detected object: %s at distance: %.2f meters", box.Class, distance)

    def get_known_width(self, object_class):
        known_widths = {
            "person": 0.5,  # Average shoulder width in meters
            "car": 1.8,     # Average width of a car
            
        }
        return known_widths.get(object_class)

    def calculate_distance(self, box, known_width):
        pixel_width = box.xmax - box.xmin
        distance = (known_width * self.focal_length) / pixel_width
        return distance

if __name__ == '__main__':
    try:
        ObjectDistanceEstimator()
    except rospy.ROSInterruptException:
        pass

