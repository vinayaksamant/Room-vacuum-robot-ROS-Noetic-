#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

def scan_callback(scan_data):
    detected_angles = []
    for i, distance in enumerate(scan_data.ranges):
        if distance < 0.5:
            # Calculate the angle in degrees
            angle = scan_data.angle_min + i * scan_data.angle_increment
            angle_deg = math.degrees(angle)
            detected_angles.append(angle_deg)

    if detected_angles:
        rospy.loginfo(f"Object detected within 0.5 meters at angles: {detected_angles}")

def main():
    rospy.init_node('lidar_detection_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
