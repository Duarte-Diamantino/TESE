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
            ('frame_id', 'map'),
        ]:
            self.declare_parameter(name, default)

        # Publishers and timer
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub   = self.create_publisher(Path,   'reference_path',    10)
        self.timer      = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Read parameters
        w      = self.get_parameter('width').value
        h      = self.get_parameter('height').value
        r      = self.get_parameter('radius').value
        frame  = self.get_parameter('frame_id').value
        now    = self.get_clock().now().to_msg()

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

        # Compute corner centers
        cx = w / 2.0 - r
        cy = h / 2.0 - r
        centers = [
            ( cx, -cy),  # bottom-right
            ( cx,  cy),  # top-right
            (-cx,  cy),  # top-left
            (-cx, -cy),  # bottom-left
        ]
        # Angles for corners
        angles = [(-math.pi/2, 0), (0, math.pi/2), (math.pi/2, math.pi), (math.pi, 3*math.pi/2)]

        # Generate points
        N_corner = 20
        # Start at bottom straight segment start
        start_x = -cx
        start_y = -h/2.0
        # Bottom edge
        for i in range(N_corner + 1):
            x = -cx + (w - 2*r) * i / N_corner
            y = -h/2.0
            p = Point(x=x, y=y, z=0.0)
            m.points.append(p)
            ps = PoseStamped(header=m.header)
            ps.pose.position = p
            path.poses.append(ps)

        # Corners and edges
        for idx, ((ccx, ccy), (ang0, ang1)) in enumerate(zip(centers, angles)):
            # corner arc
            for j in range(N_corner + 1):
                theta = ang0 + (ang1 - ang0) * j / N_corner
                x = ccx + r * math.cos(theta)
                y = ccy + r * math.sin(theta)
                p = Point(x=x, y=y, z=0.0)
                m.points.append(p)
                ps = PoseStamped(header=m.header)
                ps.pose.position = p
                path.poses.append(ps)
            # edge after corner (skip last corner point to avoid duplication)
            # next straight segment handled implicitly by next corner's start

        # Close the loop by connecting back to first point
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
