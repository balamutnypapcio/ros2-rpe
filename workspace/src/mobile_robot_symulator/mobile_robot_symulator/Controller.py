import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math


class PIDController:
    """A pure mathematical implementation of a Proportional-Integral-Derivative controller with output saturation."""
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        
        # Clamps the controller output within safe physical actuator limits
        return max(min(output, self.max_output), self.min_output)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class MotionPlanner:
    """A state-machine-driven motion planner handling sequential logic for square trajectories and basic kinematics for circular paths."""
    def __init__(self, mode, square_side, circle_radius, circle_linear_vel, stop_duration=0.6):
        self.mode = mode
        self.current_wp_index = 0
        self.state = 'ROTATING_IN_PLACE'
        self.state_timer = 0.0
        self.stop_duration = stop_duration
        
        self.circle_radius = circle_radius
        self.circle_linear_vel = circle_linear_vel

        self.waypoints = [
            (square_side, 0.0),
            (square_side, square_side),
            (0.0, square_side),
            (0.0, 0.0)
        ]
        
        self.dist_threshold = 0.04
        self.ang_threshold = 0.03

    def update(self, x, y, phi, dt, linear_pid, angular_pid, logger):
        if self.mode == 'circle':
            u1 = self.circle_linear_vel
            # Differential drive constraint for drawing a circle: w = v / R
            u2 = self.circle_linear_vel / self.circle_radius  
            return u1, u2

        dist_error, ang_error = self._calculate_errors(x, y, phi)

        if self.state == 'ROTATING_IN_PLACE':
            return self._handle_rotating(ang_error, dt, angular_pid, logger)
        elif self.state == 'STOP_BEFORE_DRIVE':
            return self._handle_stop_before_drive(dt, logger)
        elif self.state == 'DRIVING':
            return self._handle_driving(dist_error, ang_error, dt, linear_pid, angular_pid, logger)
        elif self.state == 'STOP_AT_WP':
            return self._handle_stop_at_wp(dt, logger)

        return 0.0, 0.0

    def _calculate_errors(self, x, y, phi):
        target_x, target_y = self.waypoints[self.current_wp_index]
        dist_error = math.sqrt((target_x - x)**2 + (target_y - y)**2)
        desired_phi = math.atan2(target_y - y, target_x - x)
        ang_error = desired_phi - phi
        
        # Normalizes the heading error to [-pi, pi] radians to guarantee the shortest turn direction
        ang_error = math.atan2(math.sin(ang_error), math.cos(ang_error))
        return dist_error, ang_error

    def _handle_rotating(self, ang_error, dt, angular_pid, logger):
        u2 = angular_pid.compute(ang_error, dt)
        if abs(ang_error) < self.ang_threshold:
            self.state = 'STOP_BEFORE_DRIVE'
            self.state_timer = 0.0
            angular_pid.reset()
        return 0.0, u2

    def _handle_stop_before_drive(self, dt, logger):
        self.state_timer += dt
        if self.state_timer >= self.stop_duration:
            self.state = 'DRIVING'
        return 0.0, 0.0

    def _handle_driving(self, dist_error, ang_error, dt, linear_pid, angular_pid, logger):
        u1 = linear_pid.compute(dist_error, dt)
        u2 = angular_pid.compute(ang_error, dt)
        if dist_error < self.dist_threshold:
            self.state = 'STOP_AT_WP'
            self.state_timer = 0.0
            linear_pid.reset()
            angular_pid.reset()
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
        return u1, u2

    def _handle_stop_at_wp(self, dt, logger):
        self.state_timer += dt
        if self.state_timer >= self.stop_duration:
            self.state = 'ROTATING_IN_PLACE'
        return 0.0, 0.0


class Controller(Node):
    """A ROS 2 node that manages parameter loading, environment state subscriptions, and velocity command publishing."""
    def __init__(self):
        super().__init__('controller_node')

        self.publisher_ = self.create_publisher(Twist, 'controller', 10)
        self.pose_subscriber = self.create_subscription(
            PoseStamped, 'robot_pose', self.pose_callback, 10)

        self.declare_parameter('mode', 'square')
        self.declare_parameter('square_side_length', 1.5)
        self.declare_parameter('square_linear_vel', 0.5)
        self.declare_parameter('square_angular_vel', 1.5)
        self.declare_parameter('circle_radius', 1.0)
        self.declare_parameter('circle_linear_vel', 0.4)

        mode = self.get_parameter('mode').value
        sq_side = self.get_parameter('square_side_length').value
        sq_lin_vel = self.get_parameter('square_linear_vel').value
        sq_ang_vel = self.get_parameter('square_angular_vel').value
        circ_radius = self.get_parameter('circle_radius').value
        circ_lin_vel = self.get_parameter('circle_linear_vel').value

        self.get_logger().info(f'Uruchomiono kontroler w trybie: {mode.upper()}')

        self.planner = MotionPlanner(mode, sq_side, circ_radius, circ_lin_vel)
        
        self.linear_pid = PIDController(kp=1.5, ki=0.1, kd=0.05, max_output=sq_lin_vel, min_output=-sq_lin_vel)
        self.angular_pid = PIDController(kp=3.5, ki=0.1, kd=0.15, max_output=sq_ang_vel, min_output=-sq_ang_vel)

        self.x, self.y, self.phi = 0.0, 0.0, 0.0
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # Extracts the planar yaw orientation angle from a 2D-constrained quaternion representation
        self.phi = 2.0 * math.atan2(qz, qw)

    def control_loop(self):
        u1, u2 = self.planner.update(
            self.x, self.y, self.phi, self.dt, 
            self.linear_pid, self.angular_pid, self.get_logger()
        )

        cmd_msg = Twist()
        cmd_msg.linear.x = u1
        cmd_msg.angular.z = u2
        self.publisher_.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    ControllerPublisher = Controller()
    rclpy.spin(ControllerPublisher)
    ControllerPublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()