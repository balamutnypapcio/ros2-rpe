from rob_mon_interfaces.srv import RestartSensor
from std_srvs.srv import SetBool 

import rclpy
from rclpy.node import Node
import random
from std_msgs.msg import Bool

class Sensor(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.sensor_state = True

        self.declare_parameter('sensor_name', 'sensor1')
        self.declare_parameter('sensor_publish_period', 1.0)

        self.name = self.get_parameter('sensor_name').get_parameter_value().string_value
        self.publish_period = self.get_parameter('sensor_publish_period').get_parameter_value().double_value

        self.topic_name = f'/{self.name}_state'
        self.publisher_ = self.create_publisher(Bool, self.topic_name, 10)
        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.srv = self.create_service(RestartSensor, f"/{self.name}_restart", self.restart_callback)
        self.srv = self.create_service(SetBool, f"/{self.name}_set_state", self.set_state_callback)

    def timer_callback(self):
        msg = Bool()
        msg.data = self.sensor_state
        self.publisher_.publish(msg)
        self.get_logger().info('%s publishing: "%s"' % (self.name, msg.data))

    def restart_callback(self, request, response):
        self.get_logger().info(f"Error: Restarting sensor: {self.name}")
        self.sensor_state = True
        response.success = True
        response.message = f'{self.name} restarted successfully'
        return response
    
    def set_state_callback(self, request, response):
        self.sensor_state = request.data
        response.success = True
        response.message = f'{self.name} state set to {request.data}'
        return response
    


def main(args=None):
    rclpy.init(args=args)

    sensor_publisher = Sensor()
    rclpy.spin(sensor_publisher)

    sensor_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

