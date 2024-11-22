# # # #!/usr/bin/env python3

# # # import rospy
# # # import math
# # # from geometry_msgs.msg import Twist
# # # from nav_msgs.msg import Odometry
# # # from tf.transformations import euler_from_quaternion

# # # class DifferentialDriveRobot:
# # #     def __init__(self):
# # #         rospy.init_node('differential_drive_robot', anonymous=True)
# # #         self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# # #         self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
# # #         self.pose = Odometry().pose.pose
# # #         self.rate = rospy.Rate(10)  # 10 Hz

# # #     def odom_callback(self, data):
# # #         self.pose = data.pose.pose

# # #     def move_to_goal(self, goal_x, goal_y, goal_theta):
# # #         vel_msg = Twist()
# # #         while not rospy.is_shutdown():
# # #             # Calculate the distance to the goal
# # #             distance = math.sqrt((goal_x - self.pose.position.x) ** 2 + (goal_y - self.pose.position.y) ** 2)
            
# # #             # Calculate the angle to the goal
# # #             quaternion = (
# # #                 self.pose.orientation.x,
# # #                 self.pose.orientation.y,
# # #                 self.pose.orientation.z,
# # #                 self.pose.orientation.w)
# # #             _, _, yaw = euler_from_quaternion(quaternion)
# # #             angle_to_goal = math.atan2(goal_y - self.pose.position.y, goal_x - self.pose.position.x)
# # #             angle_error = angle_to_goal - yaw

# # #             # Control the robot to move towards the goal
# # #             if distance > 0.1:
# # #                 # Angular velocity based on distance
# # #                 vel_msg.angular.z = 0.5 * distance

# # #                 # Linear velocity based on angle error
# # #                 vel_msg.linear.x = 0.2 * angle_error
# # #             else:
# # #                 # Stop the robot
# # #                 vel_msg.linear.x = 0
# # #                 vel_msg.angular.z = 0
                
# # #                 # Check if the orientation is correct
# # #                 if abs(angle_error) > 0.1:
# # #                     vel_msg.angular.z = 0.5 * angle_error
# # #                 else:
# # #                     vel_msg.angular.z = 0
# # #                     break  # Goal reached

# # #             self.velocity_publisher.publish(vel_msg)
# # #             self.rate.sleep()

# # # if __name__ == '__main__':
# # #     try:
# # #         x_goal = float(input("Set your x goal: "))
# # #         y_goal = float(input("Set your y goal: "))
# # #         theta_goal = float(input("Set your theta goal (in radians): "))
# # #         robot = DifferentialDriveRobot()
# # #         robot.move_to_goal(x_goal, y_goal, theta_goal)
# # #     except rospy.ROSInterruptException:
# # #         pass

# # #!/usr/bin/env python

# # import rospy
# # import actionlib
# # from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# # def movebase_client():
# #     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
# #     client.wait_for_server()

# #     goal = MoveBaseGoal()
# #     goal.target_pose.header.frame_id = "map"
# #     goal.target_pose.header.stamp = rospy.Time.now()
# #     goal.target_pose.pose.position.x = 2.0
# #     goal.target_pose.pose.position.y = 1.0
# #     goal.target_pose.pose.orientation.w = 1.0

# #     client.send_goal(goal)
# #     wait = client.wait_for_result()

# #     if not wait:
# #         rospy.logerr("Action server not available!")
# #         rospy.signal_shutdown("Action server not available!")
# #     else:
# #         return client.get_result()

# # if __name__ == '__main__':
# #     try:
# #         rospy.init_node('movebase_client_py')
# #         result = movebase_client()
# #         if result:
# #             rospy.loginfo("Goal execution done!")
# #     except rospy.ROSInterruptException:
# #         rospy.loginfo("Navigation test finished.")


# #!/usr/bin/env python3

# # import rospy
# # from sensor_msgs.msg import LaserScan

# # class LidarObstacleDetection:
# #     def __init__(self):
# #         rospy.init_node('lidar_obstacle_detection_node', anonymous=True)
        
# #         # Subscriber to the /scan topic
# #         self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
# #         # Flag to indicate if an obstacle is detected
# #         self.obstacle_detected = False

# #     def scan_callback(self, scan_data):
# #         detected = any(distance < 0.5 for distance in scan_data.ranges if not rospy.is_shutdown())
# #         if detected:
# #             rospy.loginfo("Obstacle detected within 0.5 meters")
# #             self.obstacle_detected = True
# #         else:
# #             self.obstacle_detected = False

# # if __name__ == '__main__':
# #     try:
# #         LidarObstacleDetection()
# #         rospy.spin()
# #     except rospy.ROSInterruptException:
# #         pass


# import rospy
# from sensor_msgs.msg import LaserScan

# # Callback function to process the LIDAR scan data
# def scan_callback(scan_data):
#     detected = any(distance < 0.5 for distance in scan_data.ranges if not rospy.is_shutdown())
#     if detected:
#         rospy.loginfo("Obstacle detected within 0.5 meters")

# # Initialize the ROS node
# rospy.init_node('lidar_obstacle_detection_node', anonymous=True)

# # Subscriber to the /scan topic
# scan_subscriber = rospy.Subscriber('/scan', LaserScan, scan_callback)

# # Keep the script running
# rospy.spin()

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
            self.obstacle_detected = any(d < 0.5 for d in data.ranges if d > 0)

    def get_yaw_from_quaternion(self, quaternion):
        euler = tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return euler[2]
    
    def publish_velocity(self, v, w):  
        if self.obstacle_detected:
            self.stop_movement()
            return

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
            while not rospy.is_shutdown():
                angle_diff = self.current_pose[1] - goal_y
                if abs(angle_diff) < 0.03:
                    self.current_pose_mov = (self.current_pose_mov[0], goal_y, self.current_pose_mov[2])
                    break
                self.publish_velocity(v=0.5, w=0) 
            self.stop_movement()
            return True

    def move_to_goal(self, goal_pose):
        goal_pose_x = goal_pose[0]
        goal_pose_y = goal_pose[1]
        goal_pose_theta = goal_pose[2]

        self.move_x(goal_pose[0])
        self.move_y(goal_pose[1])
        self.move_angular(goal_pose[2])

if __name__ == '__main__':
    try: 
        mov = Movement()
        goal = (1.5, 0.5, 0) 

        # mov.move_to_goal(goal)     
        # mov.stop_movement()

        # while goal[1] <= 4.5:
        #     goal = (goal[0], goal[1], 0)   
        #     while goal[0] <= 5:
        #         # print(goal)
        #         goal = (goal[0] + 0.5, goal[1], goal[2])  
        #         mov.move_x(goal[0])     
        #         mov.stop_movement()
        #         print(goal)
        #     print("End of while")

        #     if goal[1] == 4.5:
        #         break
        #     mov.move_angular(1.57)
        #     goal = (goal[0], goal[1] + 0.5, goal[2])  
        #     mov.move_y(goal[1]) 
        #     mov.move_angular(3.14)

        #     while goal[0] >= 2:
        #         goal = (goal[0] - 0.5, goal[1], goal[2])  
        #         mov.move_x(goal[0])     
        #         mov.stop_movement()
        #         print(goal)
        #     print("End of while")
 
        #     mov.move_angular(1.57)
        #     goal = (goal[0], goal[1] + 0.5, goal[2])  
        #     mov.move_y(goal[1]) 
        #     mov.move_angular(0)
            
    except rospy.ROSInterruptException:
        pass
