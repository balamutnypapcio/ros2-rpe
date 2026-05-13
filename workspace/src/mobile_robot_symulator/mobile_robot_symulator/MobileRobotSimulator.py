import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (Twist, PoseStamped)
import math

class MobileRobotState(Node):
    def __init__(self):
        super().__init__('mobile_state_robot_node')
        self.subscriber_ = self.create_subscription(
            Twist,
            'controller',
            self.velocity_callback,
            10
            )

    def velocity_callback(self, msg):
        i = 0


def main(args=None):
    rclpy.init(args=args)

    MobileRobotStatePublisher = MobileRobotState()
    rclpy.spin(MobileRobotStatePublisher)

    MobileRobotStatePublisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()