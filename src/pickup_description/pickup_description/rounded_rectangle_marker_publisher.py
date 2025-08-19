import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped

class RoundedRectanglePublisher(Node):
    def __init__(self):
        super().__init__('rounded_rectangle_marker_publisher')

        # Declare parameters
        for name, default in [
            ('width',    10.0),
            ('height',   5.0),
            ('radius',   1.0),
            ('center_x', 30.0),
            ('center_y', -30.0),
            ('frame_id', 'map'),
        ]:
            self.declare_parameter(name, default)

        # Publishers and timer
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub   = self.create_publisher(Path,   'reference_path',    10)
        self.timer      = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Read parameters
        w       = self.get_parameter('width').value
        h       = self.get_parameter('height').value
        r       = self.get_parameter('radius').value
        cx      = self.get_parameter('center_x').value
        cy      = self.get_parameter('center_y').value
        frame   = self.get_parameter('frame_id').value
        now     = self.get_clock().now().to_msg()

        # Prepare Marker
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp    = now
        m.ns     = 'rounded_rectangle'
        m.id     = 0
        m.type   = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.r = 1.0
        m.color.a = 1.0

        # Prepare Path
        path = Path()
        path.header = m.header

        # Compute corner centers relative to rectangle center
        half_w = w / 2.0
        half_h = h / 2.0
        corners = [
            ( half_w - r, -half_h + r),  # bottom-right
            ( half_w - r,  half_h - r),  # top-right
            (-half_w + r,  half_h - r),  # top-left
            (-half_w + r, -half_h + r),  # bottom-left
        ]
        angles = [(-math.pi/2, 0), (0, math.pi/2), (math.pi/2, math.pi), (math.pi, 3*math.pi/2)]

        # Number of points per corner and edge
        N_edge = 20

        # Bottom edge (from bottom-left to bottom-right)
        for i in range(N_edge + 1):
            x = -half_w + r + (w - 2*r) * i / N_edge
            y = -half_h
            p = Point(x=x + cx, y=y + cy, z=0.0)
            m.points.append(p)
            ps = PoseStamped(header=m.header)
            ps.pose.position = p
            path.poses.append(ps)

        # Corners and subsequent edges
        for (ccx, ccy), (ang0, ang1) in zip(corners, angles):
            # Arc for corner
            for j in range(N_edge + 1):
                theta = ang0 + (ang1 - ang0) * j / N_edge
                x = ccx + r * math.cos(theta)
                y = ccy + r * math.sin(theta)
                p = Point(x=x + cx, y=y + cy, z=0.0)
                m.points.append(p)
                ps = PoseStamped(header=m.header)
                ps.pose.position = p
                path.poses.append(ps)

        # Close loop by repeating first point
        if m.points:
            m.points.append(m.points[0])
            path.poses.append(path.poses[0])

        # Publish
        self.marker_pub.publish(m)
        self.path_pub.publish(path)


def main():
    rclpy.init()
    node = RoundedRectanglePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()