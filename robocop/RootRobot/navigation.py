#!/usr/bin/env python

import rospy
import subprocess
import time

def navigation():
    # Initialize ROS node
    rospy.init_node('slam_to_navigation')

    # Step 1: Start Hector SLAM
    rospy.loginfo("Starting Hector SLAM for mapping...")
    subprocess.Popen(["roslaunch", "hector_slam_launch", "tutorial.launch"])

    # Give some time for SLAM to build the map
    time.sleep(60)  # Adjust time based on the environment size

    # Step 2: Save the map
    rospy.loginfo("Saving the map...")
    subprocess.call(["rosrun", "map_server", "map_saver", "-f", "/path_to_save/map"])

    # Step 3: Start move_base for navigation using the saved map
    rospy.loginfo("Starting move_base for navigation...")
    subprocess.Popen(["roslaunch", "my_robot_package", "navigation.launch"])

def lidarStartThread():
    print("LidarStart Thread")
    
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
