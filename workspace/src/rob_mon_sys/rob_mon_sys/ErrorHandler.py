from rob_mon_interfaces.srv import NotifyError, RestartSensor

import rclpy
from rclpy.node import Node


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('ErrorHandler')
        self.srv = self.create_service(NotifyError, "ComponentError", self.component_error_callback)
        

    def send_sensor_restart_request(self, sensor_name :str):
        cli = self.create_client(RestartSensor, f'/{sensor_name}_restart')
        cli.wait_for_service(timeout_sec=1.0)
        req = RestartSensor.Request()
        req.sensor_name = sensor_name
        return cli.call_async(req)
    

    def component_error_callback(self, request, response):
        self.get_logger().info(f'Error: Received for sensor: {request.sensor_name}')
        self.send_sensor_restart_request(request.sensor_name)
        response.success = True
        return response



def main():

    rclpy.init()
    error_handler = ErrorHandler()
    rclpy.spin(error_handler)
    error_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()