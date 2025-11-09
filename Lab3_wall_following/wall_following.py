import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
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
        
        # set PID gains (needs tuning)
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
        
        # lidar params
        self.range_data = [] # lidare cashe distance data
        self.angle_min = 0.0
        self.angle_increment = 0.0
        
        # TODO: store any necessary values you think you'll need

    def scan_callback(self, msg: LaserScan):

        # gets msg from lidar and converted to parameters
        self.range_data = msg.ranges  # lidar all the distances 
        self.angle_min = msg.angle_min  # start angle of the scan [rad]
        self.angle_max = msg.angle_max  # end angle of the scan [rad]
        self.angle_increment = msg.angle_increment  # angular distance between measurements [rad]

        # Compute control + publish
        self.get_error()      
        self.pid_control()   
        self.steer()           

    def get_distance(self,angle):

        index = int((angle - self.angle_min) / self.angle_increment)
        if 0 <= index < len(self.range_data):
            distance = self.range_data[index]
               
            if np.isnan(distance) or np.isinf(distance): 
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
        
        alpha = np.arctan(( a_distance*np.cos(theta) - b_distance )/( a_distance*np.sin(theta))) # 1 (check this value for max)
        D_t = b_distance * np.cos(alpha) # 2 perpendicular distance to wall
        D_t_plus_1 = D_t + self.l * np.sin(alpha) # 3 lookahead distance

        self.error = self.desired_distance - D_t_plus_1 #4

        return alpha

    def pid_control(self):

        # calculate (dt) as system is descrete time (If needed add EMA(Exponential Moving Average) to smooth dt)
        current_time = self.get_clock().now().nanoseconds / 1e9  # 1second (convert to nanoseconds)
        if self.prev_time is None:
            dt = 0.004                                      #  250 Hz clock rate
        else:
            dt = current_time - self.prev_time
            dt = float(np.clip(dt, 0.005, 0.05))
        self.prev_time = current_time

        P = self.kp * self.error  

        self.integral += self.error * dt # integral of error over time
        I = self.ki * max(min(self.integral, self.wind_up), -self.wind_up)  # anti-windup

        derivation = (self.error - self.prev_error)/dt  # rate of change of error over time
        D = self.kd * derivation
        self.prev_error = self.error

        self.PID = P + I + D

        return self.PID



    def steer(self):
    
        self.steering_angle = np.clip(self.PID, -1.0, 1.0) * self.max_steering_angle

        ten_deg = np.deg2rad(10.0)
        twenty_deg = np.deg2rad(20.0)
        abs_sa = abs(self.steering_angle) # temporary absolute-value copy for checking how large the turn is

        if 0 < abs_sa < ten_deg:
            self.speed = 1.5
        elif ten_deg < abs_sa < twenty_deg:
            self.speed = 1.0
        else:
            self.speed = 0.5

        # publishes msg to drive topic
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = -self.steering_angle # added negative sign to steer correctly
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()