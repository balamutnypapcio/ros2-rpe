import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Bool

class Sensor(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.declare_parameter('sensor_name', 'sensor1')
        self.name_ = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.topic_name_ = f'/{self.name_}_state'
        self.publisher_ = self.create_publisher(Bool, self.topic_name_, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = bool(random.randint(0,1))
        self.publisher_.publish(msg)
        self.get_logger().info('%s publishing: "%s"' % (self.name_, msg.data))


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = Sensor()
    rclpy.spin(sensor_publisher)

    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

