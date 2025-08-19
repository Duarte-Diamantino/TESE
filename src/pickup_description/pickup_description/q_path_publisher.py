import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped

def circle_intersections(x1, y1, x2, y2, r):
    dx, dy = x2 - x1, y2 - y1
    d = math.hypot(dx, dy)
    if d == 0 or d > 2*r:
        return []
    mx, my = x1 + dx*0.5, y1 + dy*0.5
    h = math.sqrt(max(r*r - (d*0.5)**2, 0.0))
    ux, uy = -dy/d, dx/d
    p1 = (mx + h*ux, my + h*uy)
    p2 = (mx - h*ux, my - h*uy)
    return [p1] if math.isclose(h, 0.0, abs_tol=1e-12) else [p1, p2]

def angle_0_2pi(a):
    a = math.fmod(a, 2*math.pi)
    if a < 0:
        a += 2*math.pi
    return a

class QPathPublisher(Node):
    def __init__(self):
        super().__init__('q_path_publisher')
        # parâmetros novos para translação global
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('trans_x', -0.4)  # deslocamento global em X (metros)
        self.declare_parameter('trans_y', 0.0)  # deslocamento global em Y (metros)

        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.path_pub   = self.create_publisher(Path, 'reference_path', 10)
        self.timer      = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        frame_id = self.get_parameter('frame_id').value
        # lê a translação global (podes mudar em runtime com `ros2 param set`)
        tx = float(self.get_parameter('trans_x').value)
        ty = float(self.get_parameter('trans_y').value)

        now = self.get_clock().now().to_msg()

        # Parâmetros (geometria base sem translação)
        reta_len     = 5.5
        radius       = 2.2
        inner_offset = 0.2
        inner_radius = radius - inner_offset
        y_reta       = 0.0

        # Marker
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp    = now
        m.ns = 'q_trajectory'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.g = 1.0
        m.color.a = 1.0

        path = Path()
        path.header = m.header
        points = []

        # 1) Reta inicial (y=0)
        for i in range(50):
            x = i * reta_len / 49.0
            points.append((x, y_reta))

        # 2) Arco grande 270º CCW
        c1x, c1y = reta_len, y_reta + inner_radius
        start1 = -math.pi/2
        end1   = start1 + 1.5*math.pi
        N1 = 120
        arc1 = []
        for i in range(N1 + 1):
            ang = start1 + (end1 - start1) * i / N1
            arc1.append((c1x + inner_radius*math.cos(ang),
                         c1y + inner_radius*math.sin(ang)))
        if math.isclose(points[-1][0], arc1[0][0], abs_tol=1e-9) and math.isclose(points[-1][1], arc1[0][1], abs_tol=1e-9):
            points.pop()
        points.extend(arc1)

        # 3) Contra-curva (90º) — unir via interseção e varrer no sentido HORÁRIO
        c2x, c2y = 1.5, 2.0
        r = inner_radius

        inters = circle_intersections(c1x, c1y, c2x, c2y, r)

        def ang_at(p): return angle_0_2pi(math.atan2(p[1]-c2y, p[0]-c2x))
        if inters:
            # preferir interseção no setor 270º–360º
            cands = [(p[0], p[1], ang_at(p)) for p in inters]
            cands_sector = [q for q in cands if 1.5*math.pi - 1e-6 <= q[2] <= 2.0*math.pi + 1e-6]
            lx, ly = points[-1]
            if cands_sector:
                cands_sector.sort(key=lambda q: (q[0]-lx)**2 + (q[1]-ly)**2)
                px, py, start2 = cands_sector[0]
            else:
                cands.sort(key=lambda q: (q[0]-lx)**2 + (q[1]-ly)**2)
                px, py, start2 = cands[0]
            join = (px, py)
        else:
            start2 = 1.5*math.pi
            join = (c2x + r*math.cos(start2), c2y + r*math.sin(start2))

        # substituir o último ponto pelo ponto de junção
        points[-1] = join

        # Gerar SÓ 90º, mas agora no sentido horário (start2 → start2 - π/2)
        sweep = math.pi / 2.0
        N2 = 40
        for i in range(1, N2 + 1):  # i=1 para não repetir o join
            ang = start2 - sweep * i / N2
            x = c2x + r * math.cos(ang)
            y = c2y + r * math.sin(ang)
            points.append((x, y))

        # 4) Aplicar translação global apenas na publicação (todos os pontos)
        for x, y in points:
            p = Point(x=x + tx, y=y + ty, z=0.0)
            m.points.append(p)

            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position = p
            path.poses.append(ps)

        self.marker_pub.publish(m)
        self.path_pub.publish(path)

def main():
    rclpy.init()
    node = QPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
