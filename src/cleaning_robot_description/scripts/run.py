#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal
import tf
import math

class RobotMover:
    def __init__(self):
        rospy.init_node('robot_mover')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
        self.goal_pose = Pose()
        
        # Set the desired goal position
        self.goal_pose.position.x = 2.0
        self.goal_pose.position.y = 1.0
        self.goal_pose.position.z = 0.0
        
        # Set the desired goal orientation (quaternion for no rotation)
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.goal_pose.orientation = Quaternion(*quaternion)
        
        self.current_pose = None

    def publish_goal(self):
        goal_msg = MoveBaseActionGoal()
        goal_msg.goal.target_pose.header.frame_id = "map"
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.pose = self.goal_pose
        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Goal published: (%s, %s, %s)" % (self.goal_pose.position.x, self.goal_pose.position.y, self.goal_pose.position.z))
        self.move_to_goal()

    def move_to_goal(self):
        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue

            # Calculate the distance and angle to the goal
            delta_x = self.goal_pose.position.x - self.current_pose.position.x
            delta_y = self.goal_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(delta_x**2 + delta_y**2)
            angle_to_goal = math.atan2(delta_y, delta_x)

            # Calculate current robot orientation
            orientation_q = self.current_pose.orientation
            _, _, current_yaw = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

            # Calculate the angle difference
            angle_diff = angle_to_goal - current_yaw

            # Set linear and angular velocities (switching as per your robot's configuration)
            if distance > 0.1:
                twist.linear.x = 0  # No linear movement
                twist.angular.z = 1*distance  # Use angular.z for linear movement
            else:
                twist.linear.x = 0  # Stop the robot
                twist.angular.z = 0

            if abs(angle_diff) > 0.1:
                twist.linear.x = 2*angle_diff  # Use linear.x for angular movement
                twist.angular.z = 0  # No angular movement
            else:
                twist.linear.x = 0  # Stop the robot
                twist.angular.z = 0

            # Publish the velocity
            self.cmd_vel_pub.publish(twist)

            # Check if the goal is reached
            if distance < 0.1 and abs(angle_diff) < 0.1:
                rospy.loginfo("Goal reached!")
                break

            rate.sleep()

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def run(self):
        rospy.Subscriber('/amcl_pose', PoseStamped, self.current_pose_callback)
        rospy.sleep(2)  # Wait for subscribers to connect
        self.publish_goal()
        rospy.spin()

if __name__ == '__main__':
    mover = RobotMover()
    mover.run()