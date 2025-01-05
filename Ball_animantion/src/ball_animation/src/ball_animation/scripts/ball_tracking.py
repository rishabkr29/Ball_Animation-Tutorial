#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_tracker', anonymous=True)
        self.image_sub = rospy.Subscriber('/image/ball_animation', Image, self.image_callback)
        self.ball_pub = rospy.Publisher('/ball_position', PoseStamped, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range of green color in HSV
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([80, 255, 255])

        # Threshold the HSV image to get only green colors
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Find contours in the mask and initialize the current (x, y) center of the ball
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None

        # Proceed if at least one contour was found
        if len(contours) > 0:
            # Find the largest contour in the mask, then use it to compute the minimum enclosing circle
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # Only proceed if the radius meets a minimum size
            if radius > 10:
                # Draw the circle and centroid on the frame
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(cv_image, (int(x), int(y)), 5, (0, 255, 255), -1)

                # Publish the center coordinates of the ball
                ball_pose = PoseStamped()
                ball_pose.header.stamp = rospy.Time.now()
                ball_pose.header.frame_id = "base_link"  # Adjust if necessary
                ball_pose.pose.position.x = x
                ball_pose.pose.position.y = y
                ball_pose.pose.position.z = 0.0  # Adjust if necessary
                self.ball_pub.publish(ball_pose)

        # Display the image with contours and masked image (optional for debugging)
        cv2.imshow("Ball Tracking", cv_image)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main():
    tracker = BallTracker()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
