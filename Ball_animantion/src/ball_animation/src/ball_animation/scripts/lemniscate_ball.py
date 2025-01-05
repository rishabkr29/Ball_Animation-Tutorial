#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def generate_motion():
    rospy.init_node('motion_generation', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    image_pub = rospy.Publisher('/image/ball_animation', Image, queue_size=10)
    bridge = CvBridge()

    width, height = 500, 500
    center_x, center_y = width // 2, height // 2
    a, b = 150, 75
    t = 0

    while not rospy.is_shutdown():
        # Calculate lemniscate curve
        x = center_x + a * np.cos(t) / (1 + np.sin(t)**2)
        y = center_y + b * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)

        # Draw green ball on image
        image = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.circle(image, (int(x), int(y)), 10, (0, 255, 0), -1)

        # Convert image to ROS message and publish
        img_msg = bridge.cv2_to_imgmsg(image, "bgr8")
        image_pub.publish(img_msg)

        # Increment angle
        t += 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        generate_motion()
    except rospy.ROSInterruptException:
        pass
