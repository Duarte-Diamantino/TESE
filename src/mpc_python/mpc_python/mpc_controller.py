#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy
import numpy as np
import math, time, hashlib
from scipy.optimize import minimize
from scipy.spatial import KDTree

# ===== Timing / Horizonte =====
dt = 0.3
H_SHORT = 6
H_LONG  = 16
WINDOW_AHEAD = 50

# ===== Veículo / limites =====
V_MAX = 1.2
V_MIN = 0.2
GAMMA_MAX = math.pi / 8.0
WHEEL_BASE = 0.335
ALIGN_HEADING_THRESH_DEG = 8.0
ALIGN_CTE_THRESH = 0.08
V_MAX_NOT_ALIGNED = 0.8
ACCEL_LIMIT = 0.5
DECEL_LIMIT = 1.8
OMEGA_MAX = 0.7

# ===== Servo =====
SERVO_CENTER = 0.5751
SERVO_MAX = 0.8760
SERVO_MIN = 0.2639

# ===== Pesos MPC =====
w_cross_track = 15.0
w_heading = 5.0
w_control_effort = 0.1
w_control_smooth = 2.0
w_terminal = 15.0

# ===== Pista / segurança =====
TRACK_HALF_WIDTH = 0.50
MAX_LAT_ACC = 1.2
V_SAFETY_MARGIN = 0.15
PATH_TUBE_LIMIT = 0.55   # <<< novo: “tubo” máximo à volta da trajetória para rejeitar cortes

# ===== Obstáculos =====
REQUIRE_OBSTACLES_TO_START = True
OBSTACLE_RADIUS = 0.5
PATH_FILTER_MARGIN = 0.10
OBS_APPEARS_THRESH_FOR_SLOW = 0.15
LOCAL_SCAN_MULT = 6

# Retângulo branco (apenas visual)
OBSTACLE_RECT_SIZE = (0.5, 0.5, 0.1)

# ===== Loop / fim =====
LOOP_CLOSE_TOL = 1.0
WRAP_NEAR_START_DIST = 0.9
END_SLOWDOWN_REMAIN = 2.0
END_STOP_REMAIN = 0.80
END_FORCE_STOP_RADIUS = 0.50

# ===== Progresso exigido =====
PROGRESS_SLOWDOWN_MIN = 0.50
PROGRESS_STOP_MIN     = 0.80

# ===== Resampling =====
RESAMPLE_STEP = 0.15

# ===== Velocidade→RPM (hard min 3000) =====
RPM_CMD_MIN = 3000.0
RPM_MAX = 9000.0
GAIN_RPM_PER_MS = 7000.0
def v_to_rpm(v_mps: float) -> float:
    if v_mps <= 0.0: return 0.0
    rpm = max(RPM_CMD_MIN, GAIN_RPM_PER_MS * v_mps)
    return float(np.clip(rpm, RPM_CMD_MIN, RPM_MAX))

# ===== Rejoin tuning =====
POST_OBS_BIAS_STEPS = 18
REJOIN_GAIN = 1.2                 # ligeiramente mais baixo para não “cortar esquina”
FILTER_MARGIN_AFTER_FACTOR = 0.5

# ===== Rejoin sequencial =====
REJOIN_CANDIDATES = 25
REJOIN_STEP = 2

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # Estado
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0; self.velocity = 0.0
        self.have_odom = False

        # Trajetória
        self.base_path = []; self.ref_path = []
        self.path_headings = []; self.path_curvatures = []; self.path_cumdist = []
        self.path_kdtree = None; self.current_segment = 0
        self._last_path_hash = None
        self.closed_loop = False

        # Progresso
        self.max_index_reached = 0
        self.progress_ratio = 0.0

        # Obstáculos
        self.obstacles = []; self.have_obstacles = False
        self.active_obstacles = []

        # Joystick
        self.joy_override = False

        # Horizonte/warm start
        self.H = H_SHORT
        self.control_sequence_v = [0.8] * self.H
        self.control_sequence_gamma = [0.0] * self.H
        self.prev_v_cmd = 0.0

        # Override & rejoin bias
        self.ref_start_override = None
        self.post_obs_bias_count = 0

        # Log throttle
        self._last_log_time = 0.0
        self._log_period = 1.0

        # ROS I/O
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/predicted_trajectories', 10)

        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Path, '/reference_path', self.path_cb, 10)
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_subscription(PoseArray, '/obstacles', self.obstacles_cb, 10)

        self.create_timer(dt, self.update)

    # ---------- Callbacks ----------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.have_odom = True

    def joy_cb(self, msg: Joy):
        self.joy_override = any(button > 0 for button in msg.buttons)

    def obstacles_cb(self, msg: PoseArray):
        self.obstacles = [(p.position.x, p.position.y) for p in msg.poses]
        self.have_obstacles = True
        if self.base_path:
            self.rebuild_path_structures(self.base_path)

    def _hash_points(self, pts):
        if not pts: return None
        s = ';'.join(f'{x:.3f},{y:.3f}' for (x,y) in pts)
        return hashlib.md5(s.encode('utf-8')).hexdigest()

    def path_cb(self, msg: Path):
        raw = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        new_base = self.resample_path(raw) if len(raw) >= 2 else raw
        new_hash = self._hash_points(new_base)
        if new_hash == self._last_path_hash:
            return
        self._last_path_hash = new_hash
        self.base_path = new_base
        self.rebuild_path_structures(self.base_path)
        self.get_logger().info(f"Trajetória recebida com {len(self.ref_path)} pontos (base {len(self.base_path)})")

    # ---------- Trajetória ----------
    def resample_path(self, points, step=RESAMPLE_STEP):
        if len(points) < 2: return points
        points_arr = np.array(points)
        seg = np.diff(points_arr, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        total = float(np.sum(seg_len))
        if total <= 0.0: return points
        num_points = max(2, int(total / step))
        cum = np.insert(np.cumsum(seg_len), 0, 0.0)
        targets = np.linspace(0.0, total, num_points)
        out = []
        for d in targets:
            i = int(np.searchsorted(cum, d) - 1)
            i = max(0, min(i, len(seg_len) - 1))
            s = points_arr[i]; e = points_arr[i+1]
            t = (d - cum[i]) / max(seg_len[i], 1e-9)
            p = s + t * (e - s)
            out.append((float(p[0]), float(p[1])))
        return out

    def rebuild_path_structures(self, pts):
        self.ref_path = list(pts)
        self.closed_loop = False
        if len(self.ref_path) >= 3:
            d_cl = math.hypot(self.ref_path[0][0]-self.ref_path[-1][0],
                              self.ref_path[0][1]-self.ref_path[-1][1])
            if d_cl <= LOOP_CLOSE_TOL:
                self.closed_loop = True

        # headings
        self.path_headings = []
        for i in range(len(self.ref_path) - 1):
            dx = self.ref_path[i+1][0] - self.ref_path[i][0]
            dy = self.ref_path[i+1][1] - self.ref_path[i][1]
            self.path_headings.append(math.atan2(dy, dx))
        self.path_headings.append(self.path_headings[-1] if self.path_headings else 0.0)

        # curvaturas
        self.path_curvatures = [0.0] * len(self.ref_path)
        for i in range(1, len(self.ref_path) - 1):
            x1,y1 = self.ref_path[i-1]; x2,y2 = self.ref_path[i]; x3,y3 = self.ref_path[i+1]
            a = math.hypot(x2-x1, y2-y1); b = math.hypot(x3-x2, y3-y2); c = math.hypot(x3-x1, y3-y1)
            s = 0.5*(a+b+c); area_sq = max(s*(s-a)*(s-b)*(s-c), 0.0)
            if a*b*c > 1e-6 and area_sq > 0.0:
                R = (a*b*c)/(4.0*math.sqrt(area_sq))
                self.path_curvatures[i] = 1.0/max(R,1e-6)

        # cumdist
        self.path_cumdist = [0.0]
        for i in range(1, len(self.ref_path)):
            d = math.hypot(self.ref_path[i][0]-self.ref_path[i-1][0], self.ref_path[i][1]-self.ref_path[i-1][1])
            self.path_cumdist.append(self.path_cumdist[-1] + d)

        # KD
        self.path_kdtree = KDTree(self.ref_path) if self.ref_path else None
        if self.have_odom and self.path_kdtree:
            _, idx = self.path_kdtree.query([self.x, self.y])
            self.current_segment = int(idx)
        else:
            self.current_segment = 0

        # reset progresso / rejoin
        self.max_index_reached = 0
        self.progress_ratio = 0.0
        self.post_obs_bias_count = 0

    # ---------- Obstáculo utils ----------
    def obstacles_relevant_for_points(self, pts, extra=PATH_FILTER_MARGIN):
        if not pts or not self.obstacles: return []
        r = OBSTACLE_RADIUS + max(0.0, extra)
        r2 = r * r
        relevant = []
        for (ox, oy) in self.obstacles:
            min_d2 = min((ox - x)**2 + (oy - y)**2 for (x, y) in pts)
            if min_d2 <= r2:
                relevant.append((ox, oy))
        return relevant

    def states_hit_obstacle(self, states, obstacles, appear_extra=0.0):
        if not obstacles: return False
        rad = OBSTACLE_RADIUS + max(0.0, appear_extra)
        r2 = rad * rad
        for (x, y, _, _) in states:
            for (ox, oy) in obstacles:
                dx = x - ox; dy = y - oy
                if (dx*dx + dy*dy) < r2:
                    return True
        return False

    def is_ref_idx_unsafe(self, i, obstacles, extra=PATH_FILTER_MARGIN):
        if not obstacles: return False
        rx, ry = self.ref_path[i]
        r2 = (OBSTACLE_RADIUS + extra) ** 2
        # >>> FIX: usar a lista 'obstacles' passada como argumento <<<
        for (ox, oy) in obstacles:
            dx = rx - ox; dy = ry - oy
            if (dx*dx + dy*dy) <= r2:
                return True
        return False

    # ---------- Distância ao caminho / tubo ----------
    def nearest_path_dist(self, x, y):
        if not self.path_kdtree: return 0.0
        d, idx = self.path_kdtree.query([x, y])
        return float(d)

    def states_outside_tube(self, states, tube=PATH_TUBE_LIMIT):
        """True se algum estado se afasta demasiado do caminho (corta a pista)."""
        t2 = tube
        for (x, y, _, _) in states:
            if self.nearest_path_dist(x, y) > t2:
                return True
        return False

    # ---------- Ordenação: menor índice válido à frente ----------
    def find_closest_point_ordered(self, x, y):
        if not self.ref_path: return (0.0, 0.0), 0, 0.0
        HEADING_GATE_RAD = math.radians(60.0)
        FORWARD_DOT_MIN = -0.05
        n = len(self.ref_path); i0 = self.current_segment
        i1 = min(n - 1, i0 + WINDOW_AHEAD)
        if i1 <= i0: i1 = min(n - 1, i0 + 1)
        fx, fy = math.cos(self.yaw), math.sin(self.yaw)
        chosen = i0
        for i in range(i0, i1 + 1):
            rx, ry = self.ref_path[i]
            if i < len(self.path_headings):
                if abs(self.normalize_angle(self.path_headings[i] - self.yaw)) > HEADING_GATE_RAD:
                    continue
            vx, vy = (rx - x), (ry - y)
            if (vx*fx + vy*fy) < FORWARD_DOT_MIN:
                continue
            chosen = i
            break
        if chosen > self.current_segment:
            self.current_segment = chosen
        if self.closed_loop and self.current_segment >= len(self.ref_path) - 5:
            d0 = math.hypot(x - self.ref_path[0][0], y - self.ref_path[0][1])
            if d0 < WRAP_NEAR_START_DIST:
                self.current_segment = 0
        idx = self.current_segment
        return self.ref_path[idx], idx, (self.path_headings[idx] if idx < len(self.path_headings) else 0.0)

    # ---------- Bloco proibido à frente ----------
    def find_unsafe_block(self, start_idx, obstacles, max_scan=80):
        if not obstacles: return None
        n = len(self.ref_path)
        jmax = min(n - 1, start_idx + max_scan)
        in_block = False; first = None; last = None
        for j in range(start_idx, jmax + 1):
            unsafe = self.is_ref_idx_unsafe(j, obstacles)
            if unsafe:
                if not in_block: first = j
                in_block = True; last = j
            elif in_block:
                break
        if in_block:
            return first, last
        return None

    def first_safe_after(self, idx, obstacles, max_scan=80):
        n = len(self.ref_path)
        jmax = min(n - 1, idx + max_scan)
        for j in range(idx + 1, jmax + 1):
            if not self.is_ref_idx_unsafe(j, obstacles):
                return j
        return idx

    # ---------- Dinâmica / custo ----------
    def simulate_vehicle_dynamics(self, x0, y0, yaw0, vs, gs, H=None):
        H = H if H is not None else self.H
        states = []
        x, y, yaw = x0, y0, yaw0
        for k in range(H):
            v = float(vs[k]); g = float(gs[k])
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += (v / WHEEL_BASE) * math.tan(g) * dt
            yaw = self.normalize_angle(yaw)
            states.append((x, y, yaw, v))
        return states

    def mpc_cost_function(self, u):
        H = self.H
        control_v = u[:H]; control_g = u[H:]
        if any((v < V_MIN) or (v > V_MAX) for v in control_v): return 1e8
        if any(abs(g) > GAMMA_MAX for g in control_g): return 1e8
        states = self.simulate_vehicle_dynamics(self.x, self.y, self.yaw, control_v, control_g, H=H)
        if self.ref_start_override is not None:
            start_idx = int(self.ref_start_override)
        else:
            _, start_idx, _ = self.find_closest_point_ordered(self.x, self.y)
        base_pts, base_heads, base_curv = self.get_local_reference_base(start_idx, H=H)
        margin = PATH_FILTER_MARGIN * (FILTER_MARGIN_AFTER_FACTOR if self.post_obs_bias_count > 0 else 1.0)
        relevant_obs = self.obstacles_relevant_for_points(base_pts, extra=margin)
        self.active_obstacles = relevant_obs
        if relevant_obs:
            ref_pts, ref_heads, ref_curv = self.filter_ref_points_near_obstacles(
                base_pts, base_heads, base_curv, relevant_obs, margin=margin
            )
        else:
            ref_pts, ref_heads, ref_curv = base_pts, base_heads, base_curv
        if not ref_pts:
            return 1e8
        rejoin_factor = (1.0 + REJOIN_GAIN) if self.post_obs_bias_count > 0 else 1.0
        total = 0.0; big = 1e9
        for k, (st, rp, rh, curv) in enumerate(zip(states, ref_pts, ref_heads, ref_curv)):
            x, y, yaw, vk = st; rx, ry = rp
            if self.states_hit_obstacle([(x, y, 0.0, 0.0)], relevant_obs, appear_extra=0.0): return big
            dist = math.hypot(x - rx, y - ry); total += w_cross_track * dist * dist
            if dist > TRACK_HALF_WIDTH: total += big * (dist - TRACK_HALF_WIDTH + 1.0)
            hd = abs(self.normalize_angle(yaw - rh)); total += w_heading * hd
            total += rejoin_factor * (k / max(1, H)) * w_terminal * dist
            # v_ref
            eps = 1e-6
            v_curv = V_MAX if abs(curv) < eps else math.sqrt(max(MAX_LAT_ACC/(abs(curv)+eps), 0.0))
            v_curv = max(V_MIN, min(V_MAX, v_curv))
            heading_scaler = max(0.2, 1.0 - 0.95 * (hd / (math.pi / 2.0)))
            cte_scaler = max(0.25, 1.0 - 1.5 * dist)
            v_ref = max(V_MIN, min(V_MAX, v_curv * heading_scaler * cte_scaler))
            total += 3.5 * (vk - v_ref) ** 2
            v_soft_max = v_ref * (1.0 + V_SAFETY_MARGIN)
            if vk > v_soft_max: total += 300.0 * (vk - v_soft_max) ** 2
        total += w_control_effort * sum(float(v) ** 2 for v in control_v)
        total += w_control_effort * sum(float(g) ** 2 for g in control_g)
        for k in range(H - 1):
            total += w_control_smooth * (control_v[k+1] - control_v[k]) ** 2
            total += w_control_smooth * (control_g[k+1] - control_g[k]) ** 2
        return float(total)

    # ---------- Local ref helpers ----------
    def get_local_reference_base(self, start_idx, H=None):
        if not self.ref_path: return [], [], []
        H = H if H is not None else self.H
        end_idx = min(len(self.ref_path), start_idx + H * 3)
        pts = self.ref_path[start_idx:end_idx]
        heads = self.path_headings[start_idx:end_idx]
        curv = self.path_curvatures[start_idx:end_idx] if self.path_curvatures else [0.0] * len(pts)
        step = max(1, len(pts) // H)
        base_pts, base_heads, base_curv = [], [], []
        for i in range(0, len(pts), step):
            base_pts.append(pts[i]); base_heads.append(heads[i]); base_curv.append(curv[i])
            if len(base_pts) >= H: break
        while len(base_pts) < H:
            base_pts.append(self.ref_path[-1])
            base_heads.append(self.path_headings[-1] if self.path_headings else 0.0)
            base_curv.append(self.path_curvatures[-1] if self.path_curvatures else 0.0)
        return base_pts[:H], base_heads[:H], base_curv[:H]

    def filter_ref_points_near_obstacles(self, pts, heads, curv, obstacles, margin=PATH_FILTER_MARGIN):
        if not obstacles: return pts, heads, curv
        r2 = (OBSTACLE_RADIUS + margin) ** 2
        f_pts, f_heads, f_curv = [], [], []
        for (p, h, c) in zip(pts, heads, curv):
            rx, ry = p
            close = False
            for (ox, oy) in obstacles:
                dx = rx - ox; dy = ry - oy
                if (dx*dx + dy*dy) <= r2:
                    close = True; break
            if not close:
                f_pts.append(p); f_heads.append(h); f_curv.append(c)
        if not f_pts:
            return pts, heads, curv
        while len(f_pts) < len(pts):
            f_pts.append(f_pts[-1]); f_heads.append(f_heads[-1]); f_curv.append(f_curv[-1])
        return f_pts[:len(pts)], f_heads[:len(pts)], f_curv[:len(pts)]

    # ---------- Otimização ----------
    def run_optimization(self, H, warm_v, warm_g):
        self.H = H
        v0 = list(warm_v); g0 = list(warm_g)
        if len(v0) < H: v0 += [v0[-1] if v0 else 0.8] * (H - len(v0)); g0 += [g0[-1] if g0 else 0.0] * (H - len(g0))
        elif len(v0) > H: v0 = v0[:H]; g0 = g0[:H]
        # warm shift + clamp dentro dos bounds
        v0 = v0[1:] + [v0[-1]]; g0 = g0[1:] + [g0[-1]]
        epsb = 1e-9
        v0 = [float(np.clip(v, V_MIN + epsb, V_MAX - epsb)) for v in v0]
        g0 = [float(np.clip(g, -GAMMA_MAX + epsb, GAMMA_MAX - epsb)) for g in g0]
        init = v0 + g0
        bounds = [(V_MIN, V_MAX)] * H + [(-GAMMA_MAX, GAMMA_MAX)] * H
        result = minimize(self.mpc_cost_function, init, method='L-BFGS-B', bounds=bounds,
                          options={'maxiter': 32, 'ftol': 0.1})
        if result.success:
            v_seq = [float(np.clip(v, V_MIN, V_MAX)) for v in result.x[:H]]
            g_seq = [float(np.clip(g, -GAMMA_MAX, GAMMA_MAX)) for g in result.x[H:]]
        else:
            v_seq, g_seq = v0, g0
        states = self.simulate_vehicle_dynamics(self.x, self.y, self.yaw, v_seq, g_seq, H=H)
        return bool(result.success), v_seq, g_seq, states

    # ---------- Visualização ----------
    def publish_markers(self, predicted_states):
        ma = MarkerArray()
        clear = Marker(); clear.header.frame_id = "map"; clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        # ref laranja→castanho
        ref_down = self.ref_path[::3] if self.ref_path else []
        n = len(ref_down)
        start_rgb = (1.0, 0.55, 0.0); end_rgb = (0.40, 0.26, 0.13)
        for i, (x, y) in enumerate(ref_down):
            t = 0.0 if n <= 1 else i/(n-1)
            r = (1.0-t)*start_rgb[0] + t*end_rgb[0]
            g = (1.0-t)*start_rgb[1] + t*end_rgb[1]
            b = (1.0-t)*start_rgb[2] + t*end_rgb[2]
            m = Marker(); m.header.frame_id = "map"; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 1.0; m.color.r = r; m.color.g = g; m.color.b = b
            ma.markers.append(m)
        # janela azul
        _, current_idx, _ = self.find_closest_point_ordered(self.x, self.y)
        ref_points, _, _ = self.get_local_reference_base(current_idx, H=self.H)
        for i, (x, y) in enumerate(ref_points):
            m = Marker(); m.header.frame_id = "map"; m.id = 10000+i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 1.0; m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0
            ma.markers.append(m)
        # previsão
        for i, (x, y, _, _) in enumerate(predicted_states):
            coll = self.states_hit_obstacle([(x, y, 0.0, 0.0)], self.active_obstacles, appear_extra=0.0)
            m = Marker(); m.header.frame_id = "map"; m.id = 20000+i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position = Point(x=float(x), y=float(y), z=0.0)
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.a = 1.0
            if coll: m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            else:    m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            ma.markers.append(m)
        # obstáculos
        for i, (ox, oy) in enumerate(self.obstacles):
            m = Marker(); m.header.frame_id = "map"; m.id = 40000+i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position = Point(x=float(ox), y=float(oy), z=0.0)
            m.scale.x, m.scale.y, m.scale.z = OBSTACLE_RECT_SIZE
            m.color.a = 0.95; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0
            ma.markers.append(m)
        # pose atual
        m = Marker(); m.header.frame_id = "map"; m.id = 30000
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = Point(x=float(self.x), y=float(self.y), z=0.0)
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a = 1.0; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    # ---------- Planeamento auxiliar ----------
    def plan_from_index(self, start_idx, warm_v, warm_g, H=H_LONG):
        """Tenta planear a partir de start_idx (override) e valida: sem colisão e dentro do 'tubo'."""
        self.ref_start_override = int(start_idx)
        ok, v, g, states = self.run_optimization(H, warm_v, warm_g)
        base_pts, _, _ = self.get_local_reference_base(start_idx, H=H)
        margin = PATH_FILTER_MARGIN * (FILTER_MARGIN_AFTER_FACTOR if self.post_obs_bias_count > 0 else 1.0)
        rel = self.obstacles_relevant_for_points(base_pts, extra=margin)
        self.ref_start_override = None
        # validações
        if self.states_hit_obstacle(states, rel, appear_extra=0.0):
            return False, v, g, states, rel
        if self.states_outside_tube(states, tube=PATH_TUBE_LIMIT):
            return False, v, g, states, rel
        return True, v, g, states, rel

    # ---------- Ciclo ----------
    def update(self):
        if not self.have_odom or not self.ref_path or self.joy_override: return
        if REQUIRE_OBSTACLES_TO_START and not self.have_obstacles: return

        (_, idx_now, _) = self.find_closest_point_ordered(self.x, self.y)

        # progresso
        if idx_now > self.max_index_reached:
            self.max_index_reached = idx_now
        total_idx = max(1, len(self.ref_path) - 1)
        self.progress_ratio = float(self.max_index_reached) / float(total_idx)

        # obstáculos relevantes ao curto
        base_pts_short, _, _ = self.get_local_reference_base(idx_now, H=H_SHORT)
        relevant_obs_now = self.obstacles_relevant_for_points(base_pts_short, extra=PATH_FILTER_MARGIN)

        # plano curto default
        succ_s, v_s, g_s, states_s = self.run_optimization(H_SHORT, self.control_sequence_v, self.control_sequence_gamma)
        v_plan, g_plan, states_plan = v_s, g_s, states_s
        used_override = False

        # bloco proibido mesmo em cima → rejoin sequencial
        unsafe_block = self.find_unsafe_block(idx_now, relevant_obs_now, max_scan=80) if relevant_obs_now else None
        if unsafe_block is not None:
            _, last = unsafe_block
            first_safe = self.first_safe_after(last, relevant_obs_now, max_scan=80)
            success_any = False
            for k in range(REJOIN_CANDIDATES):
                cand = min(len(self.ref_path) - 1, first_safe + k * REJOIN_STEP)
                ok, v_l, g_l, states_l, rel_cand = self.plan_from_index(cand, self.control_sequence_v, self.control_sequence_gamma, H=H_LONG)
                if ok:
                    v_plan, g_plan, states_plan = v_l, g_l, states_l
                    self.active_obstacles = rel_cand
                    used_override = True
                    success_any = True
                    break
            if not success_any:
                self.publish_stop(); self.publish_markers(states_plan)
                self.throttled_log("STOP: sem trajetória livre após obstáculo (todas as tentativas falharam).")
                return
        else:
            # se o curto colide, tenta longo; caso contrário fica como está
            if self.states_hit_obstacle(states_s, relevant_obs_now, appear_extra=0.0):
                succ_l, v_l, g_l, states_l = self.run_optimization(H_LONG, v_s, g_s)
                if not self.states_hit_obstacle(states_l, relevant_obs_now, appear_extra=0.0) and not self.states_outside_tube(states_l):
                    v_plan, g_plan, states_plan = v_l, g_l, states_l
                else:
                    self.publish_stop(); self.publish_markers(states_l)
                    self.throttled_log("STOP: sem trajetória livre (curto e longo).")
                    return
            self.active_obstacles = relevant_obs_now

        # pós-obstáculo: viés de rejoin
        if used_override:
            self.post_obs_bias_count = POST_OBS_BIAS_STEPS
        else:
            if self.post_obs_bias_count > 0:
                self.post_obs_bias_count -= 1

        # receding horizon curto
        self.H = H_SHORT
        self.control_sequence_v = (v_plan + [v_plan[-1]])[:H_SHORT]
        self.control_sequence_gamma = (g_plan + [g_plan[-1]])[:H_SHORT]

        (ref_x, ref_y), idx, ref_heading = self.find_closest_point_ordered(self.x, self.y)
        cte = math.hypot(self.x - ref_x, self.y - ref_y)
        heading_diff = abs(self.normalize_angle(self.yaw - ref_heading))

        # comando base
        v_cmd = float(np.clip(self.control_sequence_v[0], V_MIN, V_MAX))
        gamma0 = float(self.control_sequence_gamma[0])

        # caps
        aligned_heading = heading_diff <= math.radians(ALIGN_HEADING_THRESH_DEG)
        aligned_cte = cte <= ALIGN_CTE_THRESH
        v_cap_align = V_MAX if (aligned_heading and aligned_cte) else V_MAX_NOT_ALIGNED

        max_gamma = max(abs(float(g)) for g in self.control_sequence_gamma[:3])
        tan_g = max(1e-3, abs(math.tan(max_gamma)))
        v_cap_omega = OMEGA_MAX * WHEEL_BASE / tan_g

        curv_ahead_max = 0.0
        if self.path_curvatures:
            j0 = idx; j1 = min(len(self.path_curvatures), idx + 12)
            curv_ahead_max = max(abs(c) for c in self.path_curvatures[j0:j1]) if j1 > j0 else 0.0
        eps = 1e-6
        v_cap_curv = V_MAX if curv_ahead_max < eps else max(V_MIN, min(V_MAX, math.sqrt(max(MAX_LAT_ACC/(abs(curv_ahead_max)+eps), 0.0))))

        # fim com progresso
        v_cap_end = V_MAX
        if not self.closed_loop:
            remain = self.remaining_distance(idx)
            if self.progress_ratio >= PROGRESS_SLOWDOWN_MIN and remain < END_SLOWDOWN_REMAIN:
                v_cap_end = max(0.0, min(V_MAX_NOT_ALIGNED, (remain / END_SLOWDOWN_REMAIN) * V_MAX_NOT_ALIGNED))
            dist_to_last = math.hypot(self.x - self.ref_path[-1][0], self.y - self.ref_path[-1][1])
            if (self.progress_ratio >= PROGRESS_STOP_MIN) and ((remain < END_STOP_REMAIN) or (dist_to_last < END_FORCE_STOP_RADIUS)):
                self.publish_stop(); self.publish_markers(states_plan)
                self.throttled_log(f"STOP: fim de trajetória (progress={self.progress_ratio:.2f})."); return

        # abrandar se plano passa muito perto dos obstáculos ativos
        danger_pred = self.states_hit_obstacle(states_plan, self.active_obstacles, appear_extra=OBS_APPEARS_THRESH_FOR_SLOW)
        v_cap_danger = 0.35 if danger_pred else V_MAX

        v_cmd = min(v_cmd, v_cap_align, v_cap_omega, v_cap_curv, v_cap_end, v_cap_danger)

        # limitar variação
        dv = v_cmd - self.prev_v_cmd
        max_up = ACCEL_LIMIT * dt; max_dn = DECEL_LIMIT * dt
        if dv > max_up: v_cmd = self.prev_v_cmd + max_up
        elif dv < -max_dn: v_cmd = self.prev_v_cmd - max_dn
        self.prev_v_cmd = v_cmd

        # hard-constraint: >=3000 RPM quando >0
        rpm_cmd = v_to_rpm(v_cmd)
        if rpm_cmd > 0.0 and rpm_cmd < RPM_CMD_MIN:
            rpm_cmd = RPM_CMD_MIN
        self.speed_pub.publish(Float64(data=rpm_cmd))

        gamma = -float(gamma0)
        servo_pos = float(np.clip(SERVO_CENTER + (gamma / GAMMA_MAX) * (SERVO_MAX - SERVO_CENTER),
                                  SERVO_MIN, SERVO_MAX))
        self.steer_pub.publish(Float64(data=servo_pos))

        self.publish_markers(states_plan)

        self.throttled_log(
            f"idx={idx} | prog={self.progress_ratio:.2f} | CTE={cte:.2f}m | head={math.degrees(heading_diff):.1f}° | "
            f"v={v_cmd:.2f} m/s | rpm={rpm_cmd:.0f} | active_obs={len(self.active_obstacles)} | rejoin={self.post_obs_bias_count}"
        )

    # ---------- Helpers ----------
    def normalize_angle(self, a): return (a + math.pi) % (2 * math.pi) - math.pi
    def remaining_distance(self, idx):
        if not self.path_cumdist: return float('inf')
        total = self.path_cumdist[-1]
        idx = int(max(0, min(idx, len(self.path_cumdist) - 1)))
        return max(0.0, total - self.path_cumdist[idx])
    def publish_stop(self):
        self.prev_v_cmd = 0.0
        self.speed_pub.publish(Float64(data=0.0))  # STOP = 0 RPM (única exceção)
        self.steer_pub.publish(Float64(data=SERVO_CENTER))
    def throttled_log(self, s):
        now = time.time()
        if (now - self._last_log_time) >= self._log_period:
            self._last_log_time = now; self.get_logger().info(s)

def main(args=None):
    rclpy.init(args=args)
    controller = MPCController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
