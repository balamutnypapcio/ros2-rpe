from rob_mon_interfaces.srv import NotifyError

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class StateMonitor(Node):
    def __init__(self):
        super().__init__('StateMonitor')
    
        self.declare_parameter('monitored_sensors', ['sensor1'])
        self.declare_parameter('timeout_period', 1.0)
        self.declare_parameter('publish_period', 1.0)

        self.sensors_list = self.get_parameter('monitored_sensors').get_parameter_value().string_array_value
        self.timeout_period = self.get_parameter('timeout_period').get_parameter_value().double_value
        self.publish_period = self.get_parameter('publish_period').get_parameter_value().double_value
        
        self.sensors_states = {name: None for name in self.sensors_list}

        self.subscriptions_list = []
        for sensor_name in self.sensors_list:
            topic_name = f'/{sensor_name}_state'
            sub = self.create_subscription(
                Bool, 
                topic_name, 
                lambda msg, s_name = sensor_name: self.state_sensor_callback(msg,s_name),
                10
            )
            self.subscriptions_list.append(sub)

        self.publisher = self.create_publisher(Bool, 'robot_state', 10)
        self.timer = self.create_timer(self.publish_period, self.state_robot_callback)

        self.cli = self.create_client(NotifyError, "ComponentError")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not avaible, waiting again...")
        self.req = NotifyError.Request()


    def state_sensor_callback(self, msg, sensor_name):
        self.get_logger().info(f'Received state: {sensor_name}, {msg.data}')
        self.sensors_states[sensor_name] =  msg.data

        if msg.data is False:
            self.get_logger().warn(f'Sensor {sensor_name} error! Notifying ErrorHandler...')
            self.send_sensor_restart_request(sensor_name)

    def state_robot_callback(self):
        msg = Bool()
        msg.data = self.check_robot_state()
        self.publisher.publish(msg)
        self.get_logger().info('StateMonitor publishing: "%s"' % msg.data)

    def check_robot_state(self) -> bool:
        return all(state is True for state in self.sensors_states.values())
    
    def send_sensor_restart_request(self, sensor_name :str):
        self.req.sensor_name = sensor_name
        return self.cli.call_async(self.req)
    

def main(args=None):
    rclpy.init(args=args)
    state_monitor = StateMonitor()
    rclpy.spin(state_monitor)
    state_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
