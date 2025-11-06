#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan # 
from nav_msgs.msg import Odometry # 
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive # 

class SafetyNode(Node):

    def __init__(self):
    
        super().__init__('lab2_safety_node')

        #sub to odom and scan topics
        self.odom_subscriber = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 1000) 
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1000)
        #pub to drive topic
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, 'drive', 1000)

        self.get_logger().info("Safety Node has been started.")
        self.speed = 0.5  # current velocity 
        
    def odom_callback(self, msg):
        self.get_speed = msg.twist.twist.linear.x  #(Vx) 

    def scan_callback(self, msg):

        self.distance = msg.ranges  # distance array
        self.length_array = len(msg.ranges)  # length of the distance array 
        
        array_angles = []  # to store angles corresponding to each distance measurement
        array_distance = []  # to store distance measurements
        
        self.angle_min = msg.angle_min  # min angle is the start angle 
        self.angle_max = msg.angle_max # max angle is the end angle

        self.angle_increment = msg.angle_increment  # angle increment        
        
        for i in range(self.length_array):
            angle = self.angle_min + i * self.angle_increment # angle at index i
            distance = self.distance[i] # distance at that angle (r)
            if angle < -np.pi/2 or angle > np.pi/2:  # consider only angles outside of -45 to 45 degrees
                array_angles.append(angle)
                array_distance.append(distance)
        
        array_angles = np.array(array_angles)
        array_distance = np.array(array_distance)
    
        if len(array_distance) == 0:
            return
        
        min_idx = np.argmin(array_distance) 
        min_distance = array_distance[min_idx]
        angle = array_angles[min_idx]

        r_dot = -self.speed * np.cos(angle)
        ttc = min_distance / max(abs(r_dot),0.01)  # time to collision (stopped the division by zero)

        if ttc >= 3.0:
            drive_speed = self.speed
            self.get_logger().info(f"✅ Safe | TTC={ttc:.2f}s | Speed={drive_speed:.2f} m/s")
        else:
            drive_speed = 0.0
            self.get_logger().warn(f"⚠️ Braking! TTC={ttc:.2f}s < 3.0s | Stopping the car.")

        # --- Publish ---
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = drive_speed
        drive_msg.drive.steering_angle = 0.0
        self.drive_publisher.publish(drive_msg)
            
def main(args=None):
    rclpy.init(args=args)

    safety_node = SafetyNode()

    rclpy.spin(safety_node)

    safety_node.destroy_node()
    rclpy.shutdown()    
