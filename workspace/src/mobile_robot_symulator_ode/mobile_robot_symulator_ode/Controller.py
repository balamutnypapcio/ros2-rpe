import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
import math


class ProportionalController:
    """
    Proportional feedback controller that outputs force F and torque tau.

    Linear control:   F   = Kp_d   * e_dist  - Kv  * v
    Angular control:  tau = Kp_phi * e_phi   - Kw  * omega

    The controller drives the robot toward a target (x_t, y_t).
    """

    def __init__(self, Kp_d=1.5, Kp_phi=2.0, Kv=0.5, Kw=0.3,
                 F_max=5.0, tau_max=2.0):
        self.Kp_d   = Kp_d
        self.Kp_phi = Kp_phi
        self.Kv     = Kv
        self.Kw     = Kw
        self.F_max   = F_max
        self.tau_max = tau_max

    def compute(self, x, y, phi, v, omega, x_t, y_t):
        """
        Compute force F and torque tau toward target (x_t, y_t).

        Returns:
          F    -- driving force [N]
          tau  -- steering torque [N*m]
          e_d  -- distance error [m]  (used for waypoint switching)
        """
        # Distance error
        e_d = math.sqrt((x_t - x) ** 2 + (y_t - y) ** 2)

        # Heading error (normalized to [-pi, pi])
        phi_d = math.atan2(y_t - y, x_t - x)
        e_phi = math.atan2(math.sin(phi_d - phi), math.cos(phi_d - phi))

        # Proportional control laws (with velocity damping)
        alignment = max(0.0, math.cos(e_phi))
        F   = self.Kp_d * e_d * alignment - self.Kv * v
        tau = self.Kp_phi * e_phi - self.Kw  * omega

        # Saturate outputs
        F   = max(-self.F_max,   min(self.F_max,   F))
        tau = max(-self.tau_max, min(self.tau_max, tau))

        return F, tau, e_d


class WaypointNavigator:
    """
    Sequences the robot through a list of (x, y) waypoints.
    Switches to the next waypoint when the robot is within `tolerance` metres.
    """

    def __init__(self, waypoints, tolerance=0.15):
        self.waypoints   = waypoints
        self.tolerance   = tolerance
        self.current_idx = 0

    @property
    def target(self):
        return self.waypoints[self.current_idx]

    def update(self, e_d, logger):
        """Advance to next waypoint if close enough."""
        if e_d < self.tolerance:
            prev = self.current_idx
            self.current_idx = (self.current_idx + 1) % len(self.waypoints)
            logger.info(
                f'Waypoint {prev} reached → moving to waypoint {self.current_idx} '
                f'{self.waypoints[self.current_idx]}'
            )


class Controller(Node):
    """
    ROS 2 controller node.

    Subscribes to:
      /pose           (geometry_msgs/PoseStamped) -- robot state from simulator

    Publishes:
      /control_input  (std_msgs/Float64MultiArray) -- [F, tau]
    """

    def __init__(self):
        super().__init__('controller_node')

        # --- Parameters ---
        self.declare_parameter('Kp_d',   1.5)
        self.declare_parameter('Kp_phi', 2.0)
        self.declare_parameter('Kv',     0.5)
        self.declare_parameter('Kw',     0.3)
        self.declare_parameter('F_max',  5.0)
        self.declare_parameter('tau_max', 2.0)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('waypoints', [2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0])

        Kp_d      = self.get_parameter('Kp_d').value
        Kp_phi    = self.get_parameter('Kp_phi').value
        Kv        = self.get_parameter('Kv').value
        Kw        = self.get_parameter('Kw').value
        F_max     = self.get_parameter('F_max').value
        tau_max   = self.get_parameter('tau_max').value
        tolerance = self.get_parameter('waypoint_tolerance').value
        wp_flat   = self.get_parameter('waypoints').value

        # Parse flat [x0, y0, x1, y1, ...] list into (x, y) tuples
        waypoints = [
            (wp_flat[i], wp_flat[i + 1])
            for i in range(0, len(wp_flat) - 1, 2)
        ]

        self.get_logger().info(f'Waypoints: {waypoints}')
        self.get_logger().info(
            f'Controller gains: Kp_d={Kp_d}, Kp_phi={Kp_phi}, Kv={Kv}, Kw={Kw}'
        )

        # --- Controller & navigator ---
        self.ctrl = ProportionalController(
            Kp_d=Kp_d, Kp_phi=Kp_phi, Kv=Kv, Kw=Kw,
            F_max=F_max, tau_max=tau_max
        )
        self.nav = WaypointNavigator(waypoints, tolerance=tolerance)

        # --- Robot state (updated by subscriber) ---
        self.x     = 0.0
        self.y     = 0.0
        self.phi   = 0.0
        self.v     = 0.0   # estimated from consecutive poses
        self.omega = 0.0
        self._prev_phi = 0.0
        self._prev_x   = 0.0
        self._prev_y   = 0.0

        self.dt = 0.05

        # --- ROS interfaces ---
        self.pub_ = self.create_publisher(Float64MultiArray, 'control_input', 10)
        self.sub_ = self.create_subscription(
            PoseStamped, 'pose', self.pose_callback, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def pose_callback(self, msg):
        """Extract pose and estimate velocities from consecutive poses."""
        new_x   = msg.pose.position.x
        new_y   = msg.pose.position.y
        qz      = msg.pose.orientation.z
        qw      = msg.pose.orientation.w
        new_phi = 2.0 * math.atan2(qz, qw)

        # Finite-difference velocity estimation
        self.v     = math.sqrt((new_x - self._prev_x) ** 2 +
                               (new_y - self._prev_y) ** 2) / self.dt
        dphi       = math.atan2(math.sin(new_phi - self._prev_phi),
                                math.cos(new_phi - self._prev_phi))
        self.omega = dphi / self.dt

        self.x, self.y, self.phi = new_x, new_y, new_phi
        self._prev_x, self._prev_y, self._prev_phi = new_x, new_y, new_phi

    def control_loop(self):
        """Compute control and publish [F, tau]."""
        x_t, y_t = self.nav.target

        F, tau, e_d = self.ctrl.compute(
            self.x, self.y, self.phi,
            self.v, self.omega,
            x_t, y_t
        )

        # Update waypoint navigator
        self.nav.update(e_d, self.get_logger())

        # Publish control inputs
        msg = Float64MultiArray()
        msg.data = [F, tau]
        self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()