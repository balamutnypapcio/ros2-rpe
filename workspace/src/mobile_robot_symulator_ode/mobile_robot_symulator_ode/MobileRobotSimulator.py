import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import math
import tf2_ros
from scipy.integrate import solve_ivp


def robot_ode(t, X, F, tau, m, I, b, c):
    """
    Right-hand side of the robot dynamic ODE system.

    State vector X = [x, y, phi, v, omega]:
      x, y   -- position [m]
      phi    -- heading [rad]
      v      -- linear velocity [m/s]
      omega  -- angular velocity [rad/s]

    Inputs:
      F   -- driving force [N]
      tau -- steering torque [N*m]
      m   -- robot mass [kg]
      I   -- rotational inertia [kg*m^2]
      b   -- linear damping coefficient
      c   -- angular damping coefficient
    """
    x, y, phi, v, omega = X
    dx     = v * math.cos(phi)        # kinematic: x-dot
    dy     = v * math.sin(phi)        # kinematic: y-dot
    dphi   = omega                    # kinematic: phi-dot
    dv     = (F - b * v) / m         # dynamic: linear acceleration
    domega = (tau - c * omega) / I   # dynamic: angular acceleration
    return [dx, dy, dphi, dv, domega]


class DynamicRobotModel:
    """
    Dynamic unicycle model integrated with SciPy solve_ivp (RK45).

    State: [x, y, phi, v, omega]
    Control inputs: force F [N] and torque tau [N*m].
    """

    def __init__(self, m=1.0, I=0.1, b=0.5, c=0.1, dt=0.05):
        self.m   = m    # mass [kg]
        self.I   = I    # rotational inertia [kg*m^2]
        self.b   = b    # linear damping
        self.c   = c    # angular damping
        self.dt  = dt

        # Initial state: [x, y, phi, v, omega]
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0]

    def update(self, F, tau):
        """Advance the state by one dt using RK45."""
        t_span = [0.0, self.dt]
        sol = solve_ivp(
            robot_ode,
            t_span,
            self.state,
            args=(F, tau, self.m, self.I, self.b, self.c),
            method='RK45',
            max_step=self.dt,
            dense_output=False,
        )
        # Take the last column (state at t = dt)
        self.state = [sol.y[i][-1] for i in range(5)]

    @property
    def x(self):
        return self.state[0]

    @property
    def y(self):
        return self.state[1]

    @property
    def phi(self):
        return self.state[2]

    @property
    def v(self):
        return self.state[3]

    @property
    def omega(self):
        return self.state[4]

    def get_quaternion(self):
        """Return 2D rotation as a quaternion (rotation around Z-axis)."""
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.phi / 2.0)
        qw = math.cos(self.phi / 2.0)
        return qx, qy, qz, qw


class MobileRobotState(Node):
    """
    ROS 2 node responsible for robot state integration and publishing.

    Subscribes to:
      /control_input  (std_msgs/Float64MultiArray) -- [F, tau] from controller

    Publishes:
      /pose           (geometry_msgs/PoseStamped)
      /path           (nav_msgs/Path)
      TF: world -> base_link
    """

    def __init__(self):
        super().__init__('mobile_robot_simulator_node')

        # --- Parameters ---
        self.declare_parameter('mass', 1.0)
        self.declare_parameter('inertia', 0.1)
        self.declare_parameter('lin_damping', 0.5)
        self.declare_parameter('ang_damping', 0.1)

        m = self.get_parameter('mass').value
        I = self.get_parameter('inertia').value
        b = self.get_parameter('lin_damping').value
        c = self.get_parameter('ang_damping').value

        self.dt = 0.05

        # --- Robot model ---
        self.robot_model = DynamicRobotModel(m=m, I=I, b=b, c=c, dt=self.dt)

        # Current control inputs (updated by subscriber)
        self.F   = 0.0
        self.tau = 0.0

        # --- ROS interfaces ---
        self.control_sub_ = self.create_subscription(
            Float64MultiArray, 'control_input', self.control_callback, 10)

        self.pose_publisher_ = self.create_publisher(PoseStamped, 'pose', 10)
        self.path_publisher_ = self.create_publisher(Path, 'path', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = 'world'

        self.timer = self.create_timer(self.dt, self.update_robot_state)

        self.get_logger().info(
            f'Dynamic robot model started: m={m}, I={I}, b={b}, c={c}'
        )

    def control_callback(self, msg):
        """Receive [F, tau] from the controller node."""
        if len(msg.data) >= 2:
            self.F   = msg.data[0]
            self.tau = msg.data[1]

    def update_robot_state(self):
        """Integrate one ODE step and publish pose/path/TF."""
        self.robot_model.update(self.F, self.tau)

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.robot_model.get_quaternion()

        self._publish_pose(now, qx, qy, qz, qw)
        self._broadcast_tf(now, qx, qy, qz, qw)

    def _publish_pose(self, timestamp, qx, qy, qz, qw):
        pose_msg = PoseStamped()
        pose_msg.header.stamp     = timestamp
        pose_msg.header.frame_id  = 'world'
        pose_msg.pose.position.x  = self.robot_model.x
        pose_msg.pose.position.y  = self.robot_model.y
        pose_msg.pose.position.z  = 0.0
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_publisher_.publish(pose_msg)

        self.path_msg_.poses.append(pose_msg)
        self.path_msg_.header.stamp = timestamp
        self.path_publisher_.publish(self.path_msg_)

    def _broadcast_tf(self, timestamp, qx, qy, qz, qw):
        t = TransformStamped()
        t.header.stamp    = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.robot_model.x
        t.transform.translation.y = self.robot_model.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster_.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MobileRobotState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
