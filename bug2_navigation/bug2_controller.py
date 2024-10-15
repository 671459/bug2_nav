import math
from bug2_interfaces.srv import GoToPoint
from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

class Bug2Controller(Node):
    def __init__(self):
        super().__init__('Bug2_Controller')

        # Goal position (set your goal here)
        self.goal = Point(x=5.0, y=5.0, z=0.0)  # Set the goal coordinates

        # Start position (initial robot position)
        self.start = Point(x=0.0, y=0.0, z=0.0)  # Set your starting coordinates

        # Current position
        self.position = Point()
        self.active_go_to_point = False

        # Subscribers for position and laser scan
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Create a client for the GoToPoint service
        self.GTPcli = self.create_client(GoToPoint, 'go_to_point_service')
        while not self.GTPcli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GoToPoint service not available, waiting again...')
        self.GTPreq = GoToPoint.Request()

        # Create a client for the WallFollower service
        self.WFcli = self.create_client(SetBool, 'wall_follower_service')
        while not self.WFcli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('WallFollower service not available, waiting again...')
        self.WFreq = SetBool.Request()

        # State: 0 for GoToPoint, 1 for WallFollower
        self.state = 0

        # Threshold for m-line proximity and obstacle detection
        self.m_line_precision = 0.05  # How close to the m-line we need to be
        self.obstacle_threshold = 0.5  # Distance threshold to consider an obstacle

    def odom_callback(self, msg):
        """Update the current position of the robot."""
        self.position = msg.pose.pose.position
        self.check_m_line()

    def laser_callback(self, msg):
        """Check for obstacles using LaserScan."""
        front_distance = min(min(msg.ranges[0:10]), min(msg.ranges[350:359]))  # Front of the robot
        if front_distance < self.obstacle_threshold:
            # Switch to WallFollower if an obstacle is detected
            self.call_wall_follower(True)
        else:
            self.call_wall_follower(False)

    def check_m_line(self):
        """Check if the robot is on or near the m-line."""
        distance_to_m_line = self.calculate_distance_to_m_line(self.position)

        if distance_to_m_line < self.m_line_precision:
            self.get_logger().info('On the m-line, switching to GoToPoint.')
            self.call_go_to_point(self.goal.x, self.goal.y, self.goal.z)
        else:
            self.get_logger().info('Not on the m-line, switching to WallFollower.')
            self.call_wall_follower(True)

    def calculate_distance_to_m_line(self, position):
        """Calculate the perpendicular distance from the robot to the m-line."""
        x_r, y_r = position.x, position.y
        x_s, y_s = self.start.x, self.start.y
        x_g, y_g = self.goal.x, self.goal.y

        # Formula for the perpendicular distance from point to line
        numerator = abs((y_g - y_s) * x_r - (x_g - x_s) * y_r + x_g * y_s - y_g * x_s)
        denominator = math.sqrt((y_g - y_s)**2 + (x_g - x_s)**2)
        distance = numerator / denominator

        return distance

    def call_go_to_point(self, x, y, z):
        """Call the GoToPoint service to move towards the goal."""
        if self.state == 0:  # Already using GoToPoint, no need to call again
            return

        self.GTPreq.target_position = Point(x=x, y=y, z=z)
        self.GTPreq.activate = True
        self.get_logger().info(f"Sending GoToPoint request to move to ({x}, {y}, {z})")

        future = self.GTPcli.call_async(self.GTPreq)
        future.add_done_callback(self.go_to_point_response_callback)

        # Set state to GoToPoint
        self.state = 0

    def go_to_point_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("GoToPoint service successfully activated.")
            else:
                self.get_logger().error("GoToPoint service call failed.")
        except Exception as e:
            self.get_logger().error(f"GoToPoint service call failed with error: {str(e)}")

    def call_wall_follower(self, activate):
        """Call the WallFollower service to avoid obstacles."""
        if activate and self.state == 1:  # Already using WallFollower, no need to call again
            return

        self.WFreq.data = activate
        self.get_logger().info(f"{'Activating' if activate else 'Deactivating'} wall follower")

        future = self.WFcli.call_async(self.WFreq)
        future.add_done_callback(self.wall_follower_response_callback)

        # Set state to WallFollower
        if activate:
            self.state = 1

    def wall_follower_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Wall follower {'activated' if self.WFreq.data else 'deactivated'} successfully.")
            else:
                self.get_logger().error("Wall follower service call failed.")
        except Exception as e:
            self.get_logger().error(f"Wall follower service call failed with error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)

    controller = Bug2Controller()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
