import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('linear_vel', 1.0)
        self.declare_parameter('angular_vel', 0.0)

        self.linear_vel_ = self.get_parameter('linear_vel').get_parameter_value().double_value
        self.angular_vel_ = self.get_parameter('angular_vel').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Twist, 'controller', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_vel_
        msg.angular.z = self.angular_vel_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Controller: lin {msg.linear.x}, ang {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    ControllerPublisher = Controller()
    rclpy.spin(ControllerPublisher)

    ControllerPublisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()