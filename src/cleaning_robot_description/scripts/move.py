#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import threading
import time

class Movement:
    def __init__(self):
        rospy.init_node('Movement', anonymous=True)
        self.lock = threading.Lock()
        
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.current_pose = np.zeros(3)  # Matrix to store x, y, and yaw

        self.current_pose_mov = (0.5,0.5,0)     # initial condition

        self.k_rho = 0.2
        self.k_alpha = 0.3
        self.k_beta = -0.1

        self.trajectory_points = ((1.6, 0.5, 0), 
                                (2.0, 0.5, 0),
                                (2.5, 0.5, 0),
                                (3.0, 0.5, 0),
                                (3.5, 0.5, 0),
                                (4.0, 0.5, 0),
                                (4.5, 0.5, 0),
                                (5.0, 0.5, 0),
                                (5.5, 0.5, 0), 
                                (5.5, 1.0, 1.57),
                                (5.0, 1.0, 3.14),
                                (4.5, 1.0, 3.14),
                                (4.0, 1.0, 3.14),
                                (3.5, 1.0, 3.14),
                                (3.0, 1.0, 3.14),
                                (2.5, 1.0, 3.14),
                                (2.0, 1.0, 3.14),
                                (1.6, 1.0, 3.14),
                                (1.6, 1.5, 1.57),
                                (2.0, 1.5, 0),
                                (2.5, 1.5, 0),
                                (3.0, 1.5, 0),
                                (3.5, 1.5, 0),
                                (4.0, 1.5, 0),
                                (4.5, 1.5, 0),
                                (5.0, 1.5, 0),
                                (5.5, 1.5, 0),
                                (5.5, 2.0, 1.57),
                                (5.0, 2.0, 3.14),
                                (4.5, 2.0, 3.14),
                                (4.0, 2.0, 3.14),
                                (3.5, 2.0, 3.14),
                                (3.0, 2.0, 3.14),
                                (2.5, 2.0, 3.14),
                                (2.0, 2.0, 3.14),
                                (1.6, 2.0, 3.14),
                                (1.6, 2.5, 1.57),
                                (2.0, 2.5, 0),
                                (2.5, 2.5, 0),
                                (3.0, 2.5, 0),
                                (3.5, 2.5, 0),
                                (4.0, 2.5, 0),
                                (4.5, 2.5, 0),
                                (5.0, 2.5, 0),
                                (5.5, 2.5, 0),
                                (5.5, 3.0, 1.57),
                                (5.0, 3.0, 3.14),
                                (4.5, 3.0, 3.14),
                                (4.0, 3.0, 3.14),
                                (3.5, 3.0, 3.14),
                                (3.0, 3.0, 3.14),
                                (2.5, 3.0, 3.14),
                                (2.0, 3.0, 3.14),
                                (1.6, 3.0, 3.14),
                                (1.6, 3.5, 1.57),
                                (2.0, 3.5, 0),
                                (2.5, 3.5, 0),
                                (3.0, 3.5, 0),
                                (3.5, 3.5, 0),
                                (4.0, 3.5, 0),
                                (4.5, 3.5, 0),
                                (5.0, 3.5, 0),
                                (5.5, 3.5, 0),
                                (5.5, 4.0, 1.57),
                                (5.0, 4.0, 3.14),
                                (4.5, 4.0, 3.14),
                                (4.0, 4.0, 3.14),
                                (3.5, 4.0, 3.14),
                                (3.0, 4.0, 3.14),
                                (2.5, 4.0, 3.14),
                                (2.0, 4.0, 3.14),
                                (1.6, 4.0, 3.14),
                                (1.6, 4.3, 1.57),
                                (2.0, 4.3, 0),
                                (2.5, 4.3, 0),
                                (3.0, 4.3, 0),
                                (3.5, 4.3, 0),
                                (4.0, 4.3, 0),
                                (4.5, 4.3, 0),
                                (5.0, 4.3, 0),
                                (5.5, 4.3, 0))
        self.obstacle_coordinates = ()

        self.rate = rospy.Rate(10)
        self.obstacle_detected = False

    def odom_callback(self, data):
        with self.lock:
            # Update the robot's position (x, y) and orientation (yaw) based on odometry data
            self.current_pose[0] = data.pose.pose.position.x
            self.current_pose[1] = data.pose.pose.position.y
            self.current_pose[2] = self.get_yaw_from_quaternion(data.pose.pose.orientation)

    def lidar_callback(self, data):
        with self.lock:
            # Check if any distance in the Lidar data is within 0.5 meters
            self.obstacle_detected = any(0 < d < 0.6 for d in data.ranges) 
            # print(self.obstacle_detected)

    def get_yaw_from_quaternion(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]
    
    def publish_velocity(self, v, w): 
        vel_msg = Twist()
        vel_msg.linear.x = v
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        
        # Angular velocity in the z direction
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = w

        # Publish the velocity
        self.velocity_publisher.publish(vel_msg)
        self.rate.sleep()

    def stop_movement(self):
        while not rospy.is_shutdown():
            self.publish_velocity(v=0,w=0)
            break

    def move_angular(self, goal_theta):
        if abs(goal_theta - self.current_pose_mov[2]) == 0:
            return True

        if (goal_theta - self.current_pose_mov[2]) > 0:
            while not rospy.is_shutdown():
                angle_diff = goal_theta - self.current_pose[2]
                if abs(angle_diff) < 0.02:
                    self.current_pose_mov = (self.current_pose_mov[0], self.current_pose_mov[1], goal_theta)
                    break
                self.publish_velocity(v=0, w=-0.3)
            self.stop_movement()
            return True

        elif (goal_theta - self.current_pose_mov[2]) < 0:
            while not rospy.is_shutdown():
                angle_diff = self.current_pose[2] - goal_theta
                if abs(angle_diff) < 0.02:
                    self.current_pose_mov = (self.current_pose_mov[0], self.current_pose_mov[1], goal_theta)
                    break
                self.publish_velocity(v=0, w=0.3)
            self.stop_movement()
            return True
        
    def move_x(self, goal_x): 
        if abs(goal_x - self.current_pose_mov[0]) < 0.1:
            return True

        if (goal_x - self.current_pose_mov[0]) > 0: 
            if self.current_pose_mov[2] != 0:
                self.move_angular(0)
                if self.obstacle_detected:
                    self.Obstacle_avoiding_for_x()
                    self.move_angular(0)
            while not rospy.is_shutdown():
                angle_diff = goal_x - self.current_pose[0]
                if abs(angle_diff) < 0.03:
                    self.current_pose_mov = (goal_x, self.current_pose_mov[1], self.current_pose_mov[2])
                    break 
                self.publish_velocity(v=0.5, w=0) 
            self.stop_movement()
            return True

        elif (goal_x - self.current_pose_mov[0]) < 0.01: 
            if self.current_pose_mov[2] != 3.14:
                self.move_angular(3.14) 
                if self.obstacle_detected:
                    self.Obstacle_avoiding_for_x()
                    self.move_angular(3.14)
            while not rospy.is_shutdown():
                angle_diff = self.current_pose[0] - goal_x
                if abs(angle_diff) < 0.03:
                    self.current_pose_mov = (goal_x, self.current_pose_mov[1], self.current_pose_mov[2])
                    break
                self.publish_velocity(v=0.5, w=0)
            self.stop_movement()
            return True

    def move_y(self, goal_y):
        if abs(goal_y - self.current_pose_mov[1]) < 0.1:
            return True
        
        if (goal_y - self.current_pose_mov[1]) > 0:
            if self.current_pose_mov[2] != 1.57:
                self.move_angular(1.57)
                if self.obstacle_detected:
                    self.Obstacle_avoiding_for_y()
                    self.move_angular(1.57)
            while not rospy.is_shutdown():
                angle_diff = goal_y - self.current_pose[1]
                if abs(angle_diff) < 0.03:
                    self.current_pose_mov = (self.current_pose_mov[0], goal_y, self.current_pose_mov[2])
                    break
                self.publish_velocity(v=0.5, w=0)
            self.stop_movement()
            return True

        elif (goal_y - self.current_pose_mov[1]) < 0:
            if self.current_pose_mov[2] != -1.57:
                self.move_angular(-1.57)
                if self.obstacle_detected:
                    self.Obstacle_avoiding_for_y()
                    self.move_angular(-1.57)
            while not rospy.is_shutdown():
                angle_diff = self.current_pose[1] - goal_y
                if abs(angle_diff) < 0.03:
                    self.current_pose_mov = (self.current_pose_mov[0], goal_y, self.current_pose_mov[2])
                    break
                self.publish_velocity(v=0.5, w=0) 
            self.stop_movement()
            return True

    def step_movement(self):
        distance_to_move = 0.5  # Distance to move in meters

        # Calculate linear velocity needed to move 0.5 meters
        v = 0.5  # Set linear velocity to 0.5 m/s (adjust if needed)
        duration = distance_to_move / v  # Calculate time to move distance

        # Move forward for calculated duration
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            self.publish_velocity(v=v, w=0)

        # Stop the robot after reaching the desired distance
        self.stop_movement()

    def Obstacle_avoiding_for_x(self):  
        if self.current_pose_mov[2]==0:
            self.move_angular(1.57)
            self.move_y(self.current_pose_mov[1]+0.5)
            # print("Reached y increment")
        else:
            self.move_angular(0)
            self.move_y(self.current_pose_mov[1]-0.5)
            # print("Reached y increment")

    def Obstacle_avoiding_for_y(self):
        # print("Yoooooo")
        if self.current_pose_mov[2]==1.57:
            self.move_angular(3.14)
            self.move_x(self.current_pose_mov[0]-0.5)
            # print("Reached x increment")
        else:
            self.move_angular(0)
            self.move_x(self.current_pose_mov[0]+0.5)
            # print("Reached x increment")

    def move_to_goal(self, point): 
        # if self.obstacle_detected == True:
        #     return None
        self.move_x(point[0])
        # if self.obstacle_detected == True:
        #     return False
        self.move_y(point[1])
        self.move_angular(point[2])
        return True

    def main_loop(self):
        for point in self.trajectory_points:
            if not self.obstacle_detected:
                if self.move_to_goal(point) == None:
                    self.Obstacle_avoiding_for_x()
                elif self.move_to_goal(point) == False:
                    self.Obstacle_avoiding_for_y()
                else:
                    continue
                self.move_to_goal(point)
            else:
                self.Obstacle_avoiding_for_x()

if __name__ == '__main__':
    try: 
        mov = Movement() 
        # mov.main_loop()
        
        mov.move_to_goal(point=(2,1,1.57))
    except rospy.ROSInterruptException:
        pass
