import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class go_to_pointController(Node):
    def __init__(self):
        super().__init__('go_to_point_controller')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.clbk_odom,
            10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        #Creates a timer definition -> during rclpy.spin() the function self.timer_callback() will be executed every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        self.objective = {'x': 5.0, 'y': 5.0}  #example objective

        #initialize
        self.position = None
        self.yaw = None


    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position

        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]


    def timer_callback(self):
        objective_x = self.objective['x']
        objective_y = self.objective['y']

        distance_to_objective = math.sqrt((objective_x - self.position.x) ** 2 + (objective_y - self.position.y) ** 2)

        angle_to_objective = math.atan2(objective_y - self.position.y, objective_x - self.position.x)

        vel_msg = Twist()

        #angle difference between current yaw and angle to objective
        angle_difference = angle_to_objective - self.yaw
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))  # Normalize angle to [-pi, pi]

        if distance_to_objective < 0.1:
            #on objective, stand still
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        elif abs(angle_difference) > 0.2:
            #not pointing towards objective, turn in place
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5 if angle_difference > 0 else -0.5
        else:
            #pointing towards objective, move forward
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = 0.0

        self.publisher_.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = go_to_pointController()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()