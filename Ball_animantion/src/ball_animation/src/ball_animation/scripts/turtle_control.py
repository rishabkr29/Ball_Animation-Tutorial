#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from turtlesim.msg import Pose
import math

# Global variable to store the current pose of the turtle
turtle_pose = None

# Callback function to update the turtle's pose
def pose_callback(data):
    global turtle_pose
    turtle_pose = data

# Callback function to move the turtle based on the pose received from the ball tracking node
def move_callback(data):
    global turtle_pose
    if turtle_pose is None:
        return

    cmd_vel = Twist()
    goal_x = data.pose.position.x
    goal_y = data.pose.position.y

    inc_x = goal_x - turtle_pose.x
    inc_y = goal_y - turtle_pose.y

    angle_to_goal = math.atan2(inc_y, inc_x)
    distance_to_goal = math.sqrt(inc_x**2 + inc_y**2)

    # Check if the turtle is near the boundary
    if turtle_pose.x < 1.0 or turtle_pose.x > 10.0 or turtle_pose.y < 1.0 or turtle_pose.y > 10.0:
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
    else:
        # Control logic to move the turtle
        if distance_to_goal > 0.1:
            cmd_vel.linear.x = 1.5 * distance_to_goal
            cmd_vel.angular.z = 6 * (angle_to_goal - turtle_pose.theta)
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0

    cmd_vel_pub.publish(cmd_vel)

# Main function to initialize the node and set up subscribers and publishers
def control_turtle():
    global cmd_vel_pub
    rospy.init_node('turtle_control', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/turtle/move_pose', PoseStamped, move_callback)
    cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        control_turtle()
    except rospy.ROSInterruptException:
        pass
