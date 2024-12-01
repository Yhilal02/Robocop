#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import threading, queue
from GPIOSetup import ChassisQueue, CHassisQueuePreset

# Callback function to process the velocity commands from /cmd_vel
def cmd_vel_callback(msg):
    # Extract linear and angular velocities from the Twist message
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    # Print out the received values for debugging purposes
    rospy.loginfo(f"Linear Velocity: {linear_velocity} m/s")
    rospy.loginfo(f"Angular Velocity: {angular_velocity} rad/s")



def listener():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_subscriber', anonymous=True)

    # Subscribe to the /cmd_vel topic
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    # Keep the node running
    rospy.spin()

def lidarCommandthread():
    print("LidarCommand Thread Started")
    
    while True:
        ChassisQueuePreset.SteeringAngle = angular_velocity
        ChassisQueuePreset.MotorSpeed = linear_velocity
        ChassisQueue.put(ChassisQueuePreset.LidarAction)



if __name__ == '__main__':
    try:
        listener()
    # except rospy.ROSInterruptException:
    #    pass
