import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
import math
import tf2_ros 


class UnicycleModel:
    """Class responsible for robot's state and mathematics."""
    def __init__(self, dt=0.05):
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0
        self.dt = dt

    def update_kinematics(self, u1, u2):
        self.x += math.cos(self.phi) * u1 * self.dt
        self.y += math.sin(self.phi) * u1 * self.dt
        self.phi += u2 * self.dt

    def get_quaternion(self):
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.phi / 2.0)
        qw = math.cos(self.phi / 2.0)
        return qx, qy, qz, qw


class MobileRobotState(Node):
    """ROS2 NODE  Class responsible for communication with ros2."""
    def __init__(self):
        super().__init__('mobile_state_robot_node')
        self.subscriber_ = self.create_subscription(
            Twist, 'controller', self.velocity_callback, 10)
        self.pose_publisher_ = self.create_publisher(
            PoseStamped, 'robot_pose', 10)
        self.path_publisher_ = self.create_publisher(
            Path, 'robot_path', 10)
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster(self)

        self.path_msg_ = Path()
        self.path_msg_.header.frame_id = 'world'

        self.dt = 0.05
        self.robot_model = UnicycleModel(dt=self.dt)

        self.u1 = 0.0
        self.u2 = 0.0

        self.timer = self.create_timer(self.dt, self.update_robot_position)


    def velocity_callback(self, msg):
        self.u1 = msg.linear.x
        self.u2 = msg.angular.z


    def update_robot_position(self):
        self.robot_model.update_kinematics(self.u1, self.u2)
        
        self.u1 = 0.0
        self.u2 = 0.0
        
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.robot_model.get_quaternion()
        
        self._publish_pose(now, qx, qy, qz, qw)
        self._broadcast_tf(now, qx, qy, qz, qw)


    def _publish_pose(self, timestamp, qx, qy, qz, qw):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.robot_model.x
        pose_msg.pose.position.y = self.robot_model.y
        pose_msg.pose.position.z = 0.0
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
        t.header.stamp = timestamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
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
    MobileRobotStatePublisher = MobileRobotState()
    rclpy.spin(MobileRobotStatePublisher)
    MobileRobotStatePublisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()