#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

# Parâmetros do sistema
WHEEL_RADIUS = 0.1            # metros, conforme URDF
SERVO_MIN = 0.26              # valor mínimo lido do tópico servo
SERVO_MAX = 0.87              # valor máximo lido do tópico servo
STEER_MIN_RAD = -0.5236       # ângulo mínimo (URDF)
STEER_MAX_RAD = 0.5236        # ângulo máximo (URDF)
TIMER_PERIOD = 0.1            # segundos entre publicações

class MovRobotModel(Node):
    def __init__(self):
        super().__init__('mov_robot_model')
        self.get_logger().info('Initializing MovRobotModel Node')

        # Estados internos
        self.odom = None
        self.speed = 0.0
        self.raw_steer = SERVO_MIN
        self.steer_angle = 0.0
        self.wheel_angles = [0.0] * 4

        # Distância acumulada
        self.last_position = None
        self.distance_traveled = 0.0

        # TF broadcaster e listener
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers de joint_states e path
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)

        # Inicializa Path
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Nomes de juntas do URDF
        self.joint_names = [
            'back_left_wheel_joint',
            'back_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'front_left_steering_joint',
            'front_right_steering_joint',
        ]

        # Subscrições
        self.create_subscription(Odometry, '/vesc/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/commands/motor/speed', self.speed_callback, 10)
        self.create_subscription(Float64, '/commands/servo/position', self.steer_callback, 10)
        self.get_logger().info('Subscribed to /vesc/odom, /commands/motor/speed, /commands/servo/position')

        # Timer periódico
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        self.odom = msg
        # Atualiza distância percorrida
        pos = msg.pose.pose.position
        if self.last_position is not None:
            dx = pos.x - self.last_position.x
            dy = pos.y - self.last_position.y
            self.distance_traveled += math.hypot(dx, dy)
        self.last_position = pos

        # Broadcast TF odom->base_link
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = pos.x
        tf.transform.translation.y = pos.y
        tf.transform.translation.z = pos.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        # Atualiza Path (cauda da odometria)
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)

    def speed_callback(self, msg: Float64):
        self.speed = msg.data
        # Atualiza ângulo das rodas
        delta = (self.speed * TIMER_PERIOD) / WHEEL_RADIUS
        delta = -delta  # ajustar sinal se necessário
        for i in range(4):
            self.wheel_angles[i] += delta

    def steer_callback(self, msg: Float64):
        self.raw_steer = msg.data
        ratio = (self.raw_steer - SERVO_MIN) / (SERVO_MAX - SERVO_MIN)
        ratio = min(max(ratio, 0.0), 1.0)
        self.steer_angle = STEER_MIN_RAD + ratio * (STEER_MAX_RAD - STEER_MIN_RAD)
        self.steer_angle = -self.steer_angle


    def timer_callback(self):
        # Publica JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = self.wheel_angles + [self.steer_angle, self.steer_angle]
        self.joint_pub.publish(js)

        # Publica Path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

        # Logs no terminal
        if self.odom:
            p = self.odom.pose.pose.position
            yaw = self._get_yaw(self.odom.pose.pose.orientation)
            self.get_logger().info(f'[ODOM] x={p.x:.3f}, y={p.y:.3f}, yaw≈{yaw:.3f}')
            self.get_logger().info(f'[DIST] traveled: {self.distance_traveled:.3f} m')
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            t = trans.transform.translation
            q = trans.transform.rotation
            yaw_tf = self._get_yaw(q)
            self.get_logger().info(f'[TF]  x={t.x:.3f}, y={t.y:.3f}, yaw≈{yaw_tf:.3f}')
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn('TF map->base_link not available yet')
        self.get_logger().info(f'[STEER] raw={self.raw_steer:.3f} -> {self.steer_angle:.3f} rad')

    @staticmethod
    def _get_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = MovRobotModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MovRobotModel')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
