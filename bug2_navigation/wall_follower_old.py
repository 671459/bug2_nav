import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class wall_followerController(Node):
    def __init__(self):
        super().__init__('wall_follower_Controller')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.clbk_laser,
            10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize sensor readings
        self.lidar_left = 100
        self.lidar_right = 100
        self.lidar_front = 100

        # Timer callback period
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Desired distance from the wall on the right (in meters)
        self.desired_wall_distance = 0.5
        # Safe distance to avoid obstacles directly in front
        self.safe_distance_front = 0.5

    # Callback function for the robot's LIDAR topic /scan
    def clbk_laser(self, msg):
        num_ranges = len(msg.ranges)
        front_index = int((0 - msg.angle_min) / msg.angle_increment)

        left_index = int((math.pi / 2 - msg.angle_min) / msg.angle_increment)

        right_index = int((-math.pi / 2 - msg.angle_min) / msg.angle_increment)

        front_index = front_index % num_ranges
        left_index = left_index % num_ranges
        right_index = right_index % num_ranges

        self.lidar_front = msg.ranges[front_index]
        self.lidar_left = msg.ranges[left_index]
        self.lidar_right = msg.ranges[right_index]

    def timer_callback(self):
        # Create a Twist message
        vel_msg = Twist()

        # Logic for wall following
        if self.lidar_front < self.safe_distance_front:
            # Obstacle ahead, turn left to avoid it
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5
        elif self.lidar_right > self.desired_wall_distance + 0.5:
            # Right corner, turn hard right
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = -0.6
        elif self.lidar_right > self.desired_wall_distance + 0.1:
            # Too far from the right wall, turn right
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = -0.2
        elif self.lidar_right < self.desired_wall_distance - 0.1:
            # Too close to the right wall, turn left
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.2
        else:
            # At a good distance from the wall, move forward
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0

        # Publish the velocity command
        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = wall_followerController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
