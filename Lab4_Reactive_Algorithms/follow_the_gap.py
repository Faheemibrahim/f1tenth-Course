import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap_node')
        
        # subscribe to LIDAR scan data
        self.scan_callback_handlr = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    
        # publish to akermann steering commands
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )


        # Array Variables
        self.distances = None
        self.angles = None
        self.ranges = None
        self.f_angles = None   # filtered angles (after mask)

        # Lidar Variables
        self.min_angle = None
        self.max_angle = None
        self.angle_increment = None
        self.max_range = None
        self.min_range = None

        # Car Varibales 
        self.max_steering_angle = 0.4189  # rad


    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)  # converts the list to a numpy array
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.max_range = msg.range_max
        self.min_range = msg.range_min

        self.filter_ranges()
        self.get_closest_distance()
        self.find_best_gap()
        self.select_best_angle()
        self.steering()

    def filter_ranges(self):
        length = len(self.ranges) # size of the array eg; 1080
        length_array = np.arange(length) # creates an array from 0 to length-1 (1080-1)
        self.angles = self.min_angle + length_array * self.angle_increment   # array stores angles
        # self.distances = self.ranges.copy()  # makes a clone of the ranges array and stores distances (enable this line to test without angle filtering)
        # self.f_angles = self.angles.copy()

        #filter 
        mask = (self.angles > -np.pi/4) & (self.angles < np.pi/4) # returns a boolean array where the condition is met (angle filter)
        self.distances = self.ranges[mask].copy()  # makes a clone of the ranges array and stores distances 
        self.f_angles = self.angles[mask].copy() 

        self.distances[np.isinf(self.distances)] = self.max_range  # set inf values to max_range
        self.distances[np.isnan(self.distances)] = 0.0  # set nan values to 0.0
        
        self.distances = np.clip(self.distances, self.min_range, self.max_range)  # clip distances between min_range and max_range

        # test with and without smoothening !!!!!! 
        kernel = np.ones(3)/3
        self.distances = np.convolve(self.distances, kernel, mode='same')

        return self.f_angles, self.distances


    def get_closest_distance(self):
        min_dist = np.min(self.distances) # get minimum distance value
        min_dist_idx = np.argmin(self.distances) # get index of minimum distance value
        
        #bubble 
        bubble_radius = 10 # number of indices to zero out on each side of the closest point

        start_idx = max(0, min_dist_idx - bubble_radius)   # max(least indx value (0), closest index - bubble radius)

        end_idx = min(len(self.distances) - 1, min_dist_idx + bubble_radius)
        
        self.distances[start_idx:end_idx + 1] = 0.0      

    def find_best_gap(self):
        free = self.distances > 2.0 # this sets the minimum distance 
        free_padded = np.concatenate(([False], free, [False]))
        free_int = free_padded.astype(np.int8)

        diff = np.diff(free_int)  # np.diff basically does free_int[i+1] - free_int[i] and gives one less element than the padded 

        start_idx = np.where(diff == 1)[0]  # first element in the list 
        end_idx = np.where(diff == -1)[0]   

        size = end_idx - start_idx
        max_gap_idx = np.argmax(size)

        self.best_start = start_idx[max_gap_idx]      # inclusive   
        self.best_end   = end_idx[max_gap_idx] - 1    # inclusive 

        return self.best_start, self.best_end

    def select_best_angle(self):
        if self.best_start >= self.best_end:
            self.best_angle = 0.0
            return self.best_angle

        center_idx = (self.best_start + self.best_end) // 2
        self.best_angle = self.f_angles[center_idx]

        return self.best_angle

    def steering(self):
        nor_angle = self.best_angle / (np.pi/4) # depedns on the angle filter 
        steering_angle = np.clip(nor_angle,-1,1) * self.max_steering_angle

        ten_deg = np.deg2rad(10.0)

        abs_sa = abs(steering_angle) # (converts negetive values to positive)temporary absolute-value copy for checking how large the turn is

        if 0 < abs_sa < ten_deg:
            speed = 3.5
        else:
            speed = 1.5

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#https://github.com/bluejamis/f1tenth_labs/blob/main/lab4/lab4/gap_follow_node.py  refer to this guys code too its better i think 