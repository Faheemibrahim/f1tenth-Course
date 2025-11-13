#!/usr/bin/env python3

# ros2 python library 
import rclpy 
from rclpy.node import Node

# numpy for calculations
import numpy as np

# import matplotlib for plotting graphs (running on docker)
import matplotlib
matplotlib.use("Qt5Agg")     # GUI backend
import matplotlib.pyplot as plt

# ros messages
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# ros services 
from std_srvs.srv import Trigger

class WallFollow(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        #publisher 
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        #subscriber
        self.scan_callback_handle = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        

        self.srv = self.create_service(
            Trigger,
            'stop',
            self.stop)

        
        # set PID gains (needs tuning)
        # https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method (tuning method)
        self.kp = 1.0
        self.ki = 0
        self.kd = 0.1
        self.wind_up = 1.0  # anti-windup limit

        # wall following params
        self.l = 0.1 # lookahead distance (play with this to see how the car oscillates) reduce to reduce oscillations increase to make car more responsive
        self.desired_distance = 1.0 # (m) desired distance from wall
        self.speed = 0.0  # initial speed (m/s)

        #steering angle 
        self.steering_angle = 0.0
        self.max_steering_angle = 0.4189  # rad
        
        # PID state (starts at zero)
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.PID = 0.0

        # time variables for discrete time PID
        self.prev_time = None

        # server stop flag
        self.stop_requested = False 
        
        # lidar params
        self.range_data = [] # lidare cashe distance data
        self.angle_min = 0.0
        self.angle_increment = 0.0

        #parameters for plotting graphs
        self.time_data = []
        self.error_data = []
        self.p_data = []
        self.i_data = []
        self.d_data = []
        self.pid_data = []
        
    def stop(self, request, response):
        self.stop_requested = True      
        response.success = True
        response.message = "Stopping & plotting"
        return response
    
    def set_speed(self,speed,angle):

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, msg: LaserScan):

        # gets msg from lidar and converted to parameters
        self.range_data = msg.ranges  # lidar all the distances 
        self.angle_min = msg.angle_min  # start angle of the scan [rad]
        self.angle_max = msg.angle_max  # end angle of the scan [rad]
        self.angle_increment = msg.angle_increment  # angular distance between measurements [rad]

        if not self.stop_requested:  # if services not triggerd will continue
            self.get_error()      
            self.pid_control()   
            self.steer()    
        else:                       # if service triggered will stop and plot graphs
            self.create_graphs(
                self.time_data,
                self.error_data,
                self.p_data,
                self.i_data,
                self.d_data,
                self.pid_data)
            return

    def get_distance(self,angle):

        index = int((angle - self.angle_min) / self.angle_increment)
        if 0 <= index < len(self.range_data):
            distance = self.range_data[index]
               
            if np.isnan(distance) or np.isinf(distance):  # protextion against invalid readings
                return 10.0
            else:
                return distance
        else:        
            return 10.0

    def get_error(self):        

        a_angle = 60 * np.pi/180
        b_angle = 20 * np.pi/180
        theta = a_angle - b_angle # angle is (40) fixed 

        a_distance = self.get_distance(a_angle) 
        b_distance = self.get_distance(b_angle)  
        
        alpha = np.arctan(( b_distance - a_distance*np.cos(theta))/( a_distance*np.sin(theta))) # 1 angle bettwen car and perpendicular to wall
        D_t = b_distance * np.cos(alpha) # 2 perpendicular distance to wall
        D_t_plus_1 = D_t + self.l * np.sin(alpha) # 3 lookahead distance

        self.error = self.desired_distance - D_t_plus_1 # 4 error between desired and actual distance

        return self.error 

    def pid_control(self):

        # calculate (dt) as system is descrete time (If needed add EMA(Exponential Moving Average) to smooth dt)
        current_time = self.get_clock().now().nanoseconds / 1e9  # 1second (convert to nanoseconds)
        if self.prev_time is None:
            dt = 0.004                                      #  250 Hz clock rate
        else:
            dt = current_time - self.prev_time
            dt = float(np.clip(dt, 0.005, 0.05)) # preventing dt from being unrealistically small or large delay (minimum 200hz and 20hz maximum)
        self.prev_time = current_time

        P = self.kp * self.error  

        self.integral += self.error * dt # integral of error over time
        I = self.ki * max(min(self.integral, self.wind_up), -self.wind_up)  # anti-windup

        derivation = (self.error - self.prev_error)/dt  # rate of change of error over time
        D = self.kd * derivation
        self.prev_error = self.error

        self.PID = P + I + D

        # appedning data for plotting graphs
        self.time_data.append(current_time)
        self.error_data.append(self.error)
        self.p_data.append(P)
        self.i_data.append(I)
        self.d_data.append(D)
        self.pid_data.append(self.PID)

        return self.PID

    def steer(self):
    
        self.steering_angle = np.clip(self.PID, -1.0, 1.0) * self.max_steering_angle # normalize and then scale to max steering angle

        ten_deg = np.deg2rad(10.0)
        twenty_deg = np.deg2rad(20.0)
        abs_sa = abs(self.steering_angle) # (converts negetive values to positive)temporary absolute-value copy for checking how large the turn is

        if 0 < abs_sa < ten_deg:
            self.speed = 1.5
        elif ten_deg < abs_sa < twenty_deg:
            self.speed = 1.0
        else:
            self.speed = 0.5

        self.set_speed(self.speed, -self.steering_angle)  # added negative sign to steer correctly


    def create_graphs(self, time_data, error_data, p_data, i_data, d_data, pid_data):

        # stop the car
        self.set_speed(0.0,0.0)
        self.get_logger().info("stopped-Car-Now-Plotting-Graphs")

        # plotting the graphs
        fig, axs = plt.subplots(5, 1, layout='constrained')

        axs[0].set_title('P Component')
        axs[0].plot(time_data, p_data, label='PID Output', color='red')
        
        axs[1].set_title('I Component')
        axs[1].plot(time_data, i_data, label='Integral', color='green')

        axs[2].set_title('D Component')
        axs[2].plot(time_data, d_data, label='Derivative', color='blue')

        axs[3].set_title('PID Output')
        axs[3].plot(time_data, pid_data, label='PID', color='purple')
        
        axs[4].set_title('Error over Time')
        axs[4].plot(time_data, error_data, label='Error', color='blue')

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()