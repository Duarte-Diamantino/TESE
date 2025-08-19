#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')

        # --- deslocamento horizontal no eixo y (ajuste manualmente) ---
        #self.h_offset = 3.0  # basta mudar este valor para transladar a spline

        # Declare os parâmetros da spline
        for name, default in [
            ('a',       1.0),
            ('b',       0.0),
            ('c',       0.0),
            ('x_min',   0.0),
            ('x_max',   5.0),
            ('h_offset',   2.0),
            ('frame_id','map'),
        ]:
            self.declare_parameter(name, default)

        # Publishers e timer
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub   = self.create_publisher(Path,   'reference_path',    10)
        self.timer      = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Lê parâmetros
        a      = self.get_parameter('a').value
        b      = self.get_parameter('b').value
        c      = self.get_parameter('c').value
        xmin   = self.get_parameter('x_min').value
        xmax   = self.get_parameter('x_max').value
        hoffset   = self.get_parameter('h_offset').value
        frame  = self.get_parameter('frame_id').value
        now    = self.get_clock().now().to_msg()

        # DEBUG: log dos parâmetros recebidos
        self.get_logger().info(
            f"[DEBUG] spline params: a={a}, b={b}, c={c}, x_min={xmin}, x_max={xmax}, h_offset={hoffset}"
        )

        # Prepara o Marker
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp    = now
        m.ns     = 'spline'
        m.id     = 0
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.g = 1.0
        m.color.a = 1.0

        # Prepara o Path
        path = Path()
        path.header = m.header

        # Gera os pontos com translação em y
        N  = 100
        dx = (xmax - xmin) / N
        for i in range(N + 1):
            # coordenada "natural" em y
            y0 = xmin + i * dx
            # polinômio em y0
            x  = a * y0 * y0 + b * y0 + c
            # aplica deslocamento horizontal (no eixo y)
            y  = y0 + hoffset

            p = Point(x=x, y=y, z=0.0)
            m.points.append(p)

            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position = p
            path.poses.append(ps)

        # Publica Marker e Path
        self.marker_pub.publish(m)
        self.path_pub.publish(path)

def main():
    rclpy.init()
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()