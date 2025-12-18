import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.publisher_ = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # store PID gains
        # we will start by setting kd, ki = 0 and kp = 1, then tune kp until the car can oscillate without crashing
        self.kp = 0.5
        self.kd = 0.05
        self.ki = 0.0

        # store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # store any necessary values you think you'll need
        self.lookahead_dist = 1.75
        self.desired_distance = 1 # meters from wall

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        wedge_size = math.radians(5)

        start_angle = angle - wedge_size
        end_angle = angle + wedge_size

        start_index = int((start_angle - range_data.angle_min) / range_data.angle_increment)
        end_index = int((end_angle - range_data.angle_min) / range_data.angle_increment)
        start_index = max(0, start_index)
        end_index = min(len(range_data.ranges) - 1, end_index)

        if start_index >= end_index:
            return 10.0

        current_slice = range_data.ranges[start_index : end_index]

        valid_ranges = [r for r in current_slice if not np.isnan(r) and not np.isinf(r)]
        
        if not valid_ranges:
            return 10.0

        return min(valid_ranges)

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        angle_b = np.pi / 2 
        angle_a = np.pi / 4 
        theta = angle_b - angle_a 

        b = self.get_range(range_data, angle_b)
        a = self.get_range(range_data, angle_a)

        alpha = np.arctan2((a * np.cos(theta) - b), (a * np.sin(theta)))

        d = b * np.cos(alpha)
        d_1 = d + self.lookahead_dist * np.sin(alpha)
        error = dist - d_1

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        p = self.kp * error
        d = self.kd * (error - self.prev_error)
        self.integral += error
        i = self.ki * self.integral

        steering_angle = -(p + d + i)

        if steering_angle > 0.85: steering_angle = 0.85
        elif steering_angle < -0.85: steering_angle = -0.85

        # Max Speed 2.0, decreasing as steering angle increases
        velocity = 2.0 - (abs(steering_angle) * 1.0) 
        if velocity < 0.5: velocity = 0.5 # Safety floor

        self.prev_error = error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity

        self.publisher_.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """

        error = self.get_error(msg, self.desired_distance)

        # if abs(error) < 0.1:
        #     velocity = 1.5
        # elif abs(error) < 0.2:
        #     velocity = 1.0
        # else:
        #     velocity = 0.5

        self.pid_control(error, 0.0)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()