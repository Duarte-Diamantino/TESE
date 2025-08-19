# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use("TkAgg")  # TkAgg + Tkinter
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.collections import LineCollection
import heapq
from collections import deque
import math

# ---- ROS2 (opcional) ----
try:
    import rclpy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    HAVE_ROS2 = True
except Exception:
    HAVE_ROS2 = False

# ---------- ScrollableFrame ----------
class ScrollableFrame(ttk.Frame):
    def __init__(self, master, *args, **kwargs):
        super().__init__(master, *args, **kwargs)
        self.canvas = tk.Canvas(self, highlightthickness=0)
        self.vbar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.hbar = ttk.Scrollbar(self, orient="horizontal", command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=self.vbar.set, xscrollcommand=self.hbar.set)
        self.vbar.pack(side="right", fill="y")
        self.hbar.pack(side="bottom", fill="x")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.body = ttk.Frame(self.canvas)
        self.window = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind("<Configure>", self._on_body_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Shift-MouseWheel>", self._on_shift_mousewheel)
        self.canvas.bind("<Button-4>", lambda e: self.canvas.yview_scroll(-1, "units"))
        self.canvas.bind("<Button-5>", lambda e: self.canvas.yview_scroll( 1, "units"))
        self.canvas.bind("<Shift-Button-4>", lambda e: self.canvas.xview_scroll(-1, "units"))
        self.canvas.bind("<Shift-Button-5>", lambda e: self.canvas.xview_scroll( 1, "units"))

    def _on_body_configure(self, _):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self.window, width=event.width)

    def _on_mousewheel(self, event):
        delta = -1 * int(event.delta / 120) if event.delta else (1 if getattr(event, "num", 0) == 5 else -1)
        self.canvas.yview_scroll(delta, "units")

    def _on_shift_mousewheel(self, event):
        delta = -1 * int(event.delta / 120) if event.delta else (1 if getattr(event, "num", 0) == 5 else -1)
        self.canvas.xview_scroll(delta, "units")

# ------------- helpers -------------
def clamp(v, lo, hi): return max(lo, min(hi, v))
def ang_norm(a): return math.atan2(math.sin(a), math.cos(a))

# ------------- app -------------
class TrajApp(tk.Frame):
    def __init__(self, master=None, initial_squares=24):
        super().__init__(master)
        # grid & imagem
        self.grid_count = initial_squares
        self.grid_spacing = None
        self.image = None
        self.image_array = None

        # estado
        self.mode = None
        self.obstacle_points = []
        self.color_tolerance = 30

        # origem + orientação
        self.start_point = None
        self.start_theta = 0.0

        # escala (2 modos)
        self.scale_mode = tk.StringVar(value="sqm")  # "sqm": 1 m = N quadrículas; "mps": 1 quadrícula = S m
        self.squares_per_meter = 5.0
        self.meters_per_square = 0.20
        self.show_meter_grid = tk.BooleanVar(value=True)

        # veículo
        self.wheelbase = 0.324
        self.max_steering_angle = math.radians(30)
        self.max_speed = 2.0
        self.dt = 0.1
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steering_angle)

        # objetivos
        self.goals = []  # [x, y, raio_px]
        self.selected_goal = None
        self.goal_radius = 18

        # regras de objetivos
        self.goal_avoid_margin_m = 0.6
        self.goal_avoid_weight  = 0.6
        self.early_entry_band_m = 0.5

        # execução
        self.run_state = None
        self.anim_timer = None

        # racing-line
        self.mu = 0.9
        self.ax_max = 3.0
        self.brake_max = 5.0
        self.safety_margin_m = 0.30

        # ROS2
        self.ros_node = None
        self.ros_pub = None
        self.ros_inited = False
        self.ref_path_topic = "reference_path"
        self.frame_id = "map"
        self.origin_x_m = 0.0
        self.origin_y_m = 0.0

        # eixos
        self.axis_options = ["Direita", "Esquerda", "Cima", "Baixo"]
        self.axis_x_vec = (1.0, 0.0)     # X+ → direita
        self.axis_y_vec = (0.0, -1.0)    # Y+ → cima (y imagem cresce para baixo)
        self.pending_axis_x = "Direita"
        self.pending_axis_y = "Cima"

        self.anchor_origin_start = tk.BooleanVar(value=True)

        # desenho manual (pincel)
        self.draw_mode = False
        aelf = self  # (nada, só evita warnings de linters)
        self.draw_active_drag = False
        self.draw_points = []
        self.draw_sampling_px = 1.0
        self.draw_smooth_on_use = tk.BooleanVar(value=True)
        self.manual_path = None

        # colorbar/LC únicos
        self._raceline_cbar = None
        self._raceline_lc = None

        # medição
        self.measure_p1 = None
        self.measure_p2 = None
        self.last_measure = None  # (p1,p2,dpx,dm)

        self.create_ui()
        self.pack(fill=tk.BOTH, expand=True)

    # ---------- unidades ----------
    def ppm(self):
        if not self.grid_spacing:
            return 1.0
        if self.scale_mode.get() == "sqm":
            return float(self.grid_spacing) * float(self.squares_per_meter)
        else:
            mps = float(self.meters_per_square)
            if mps <= 0: mps = 1e-6
            return float(self.grid_spacing) / mps

    def m2px(self, m): return float(m) * self.ppm()
    def px2m(self, px):
        PPM = self.ppm()
        return float(px) / PPM if PPM else float(px)

    # ---------- UI ----------
    def create_ui(self):
        root = tk.Frame(self); root.pack(fill=tk.BOTH, expand=True)
        pw = ttk.Panedwindow(root, orient=tk.HORIZONTAL); pw.pack(fill=tk.BOTH, expand=True)
        left = ttk.Frame(pw, width=420); right = ttk.Frame(pw)
        pw.add(left, weight=0); pw.add(right, weight=1)

        tabs = ttk.Notebook(left); tabs.pack(fill=tk.BOTH, expand=True, padx=6, pady=(6,0))
        tab_map_sf = ScrollableFrame(tabs); tabs.add(tab_map_sf, text="Mapa & Grid");       tab_map = tab_map_sf.body
        tab_vehicle_sf = ScrollableFrame(tabs); tabs.add(tab_vehicle_sf, text="Veículo & Escala"); tab_vehicle = tab_vehicle_sf.body
        tab_goals_sf = ScrollableFrame(tabs); tabs.add(tab_goals_sf, text="Objetivos");     tab_goals = tab_goals_sf.body
        tab_run_sf = ScrollableFrame(tabs); tabs.add(tab_run_sf, text="Executar (H*A* + DWA + MPC)"); tab_run = tab_run_sf.body
        tab_draw_sf = ScrollableFrame(tabs); tabs.add(tab_draw_sf, text="Desenhar"); tab_draw = tab_draw_sf.body

        self.status_label = tk.Label(left, text="Status: Pronto", bg='lightgray', relief=tk.SUNKEN)
        self.status_label.pack(fill=tk.X, side=tk.BOTTOM)

        self.fig = Figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111); self.ax.axis('off')
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # --- tab mapa ---
        r=0
        ttk.Button(tab_map, text="Carregar Imagem", command=self.load_image).grid(row=r, column=0, padx=6, pady=6, sticky="w"); r+=1
        ttk.Label(tab_map, text="Resolução da Grid (linhas):").grid(row=r, column=0, sticky="w", padx=6)
        self.grid_slider = tk.Scale(tab_map, from_=15, to=120, orient=tk.HORIZONTAL, command=self.update_grid, length=260)
        self.grid_slider.set(self.grid_count); self.grid_slider.grid(row=r, column=1, sticky="w", padx=6, pady=6); r+=1

        axes_box = ttk.LabelFrame(tab_map, text="Orientação dos eixos (aplica ao validar)")
        axes_box.grid(row=r, column=0, columnspan=2, sticky="we", padx=6, pady=6); r+=1
        ttk.Label(axes_box, text="X+ →").grid(row=0, column=0, padx=(8,4), pady=6, sticky="w")
        self.axis_x_var = tk.StringVar(value=self.pending_axis_x)
        ttk.Combobox(axes_box, values=self.axis_options, textvariable=self.axis_x_var, state="readonly", width=10)\
            .grid(row=0, column=1, padx=4, pady=6, sticky="w")
        ttk.Label(axes_box, text="Y+ →").grid(row=0, column=2, padx=(12,4), pady=6, sticky="w")
        self.axis_y_var = tk.StringVar(value=self.pending_axis_y)
        ttk.Combobox(axes_box, values=self.axis_options, textvariable=self.axis_y_var, state="readonly", width=10)\
            .grid(row=0, column=3, padx=4, pady=6, sticky="w")
        ttk.Button(axes_box, text="✅ Validar Orientação", command=self.validate_axes)\
            .grid(row=0, column=4, padx=12, pady=6, sticky="w")

        # (REMOVIDO) Check duplicado "Mostrar grelha em metros (1 m)"

        ttk.Label(tab_map, text="Limites & Obstáculos:").grid(row=r, column=0, sticky="w", padx=6); r+=1
        ttk.Button(tab_map, text="Adicionar Limite (flood)", command=self.add_limit_mode).grid(row=r, column=0, padx=6, pady=6, sticky="w")
        ttk.Button(tab_map, text="Remover Limite", command=self.remove_limit_mode).grid(row=r, column=1, padx=6, pady=6, sticky="w"); r+=1
        ttk.Button(tab_map, text="Limpar Todos os Limites", command=self.clear_all_limits).grid(row=r, column=0, padx=6, pady=6, sticky="w"); r+=1

        ttk.Button(tab_map, text="📏 Medir 2 pontos", command=self.start_measure)\
            .grid(row=r, column=0, padx=6, pady=6, sticky="w"); r += 1

        # --- Escala (AGORA NA 1ª ABA, logo por baixo do Medir) ---
        scale = ttk.LabelFrame(tab_map, text="Escala de Comprimento (baseada na GRID)")
        scale.grid(row=r, column=0, columnspan=2, sticky="we", padx=6, pady=6); r+=1

        ttk.Radiobutton(scale, text="1 m = N quadrículas", value="sqm",
                        variable=self.scale_mode, command=self.update_scale)\
            .grid(row=0, column=0, columnspan=2, padx=6, pady=(6,2), sticky="w")
        ttk.Radiobutton(scale, text="1 quadrícula = S metros", value="mps",
                        variable=self.scale_mode, command=self.update_scale)\
            .grid(row=0, column=2, columnspan=2, padx=6, pady=(6,2), sticky="w")

        ttk.Label(scale, text="N quadrículas por 1 m:").grid(row=1, column=0, padx=(8,4), pady=4, sticky="w")
        self.sqm_var = tk.DoubleVar(value=self.squares_per_meter)
        tk.Spinbox(scale, from_=0.5, to=200, increment=0.5, width=7,
                   textvariable=self.sqm_var,
                   command=lambda: self._set_sqm(self.sqm_var.get()))\
            .grid(row=1, column=1, padx=4, pady=4, sticky="w")

        ttk.Label(scale, text="S metros por 1 quadrícula:").grid(row=2, column=0, padx=(8,4), pady=4, sticky="w")
        self.mps_var = tk.DoubleVar(value=self.meters_per_square)
        tk.Spinbox(scale, from_=0.01, to=50.0, increment=0.01, width=7,
                   textvariable=self.mps_var,
                   command=lambda: self._set_mps(self.mps_var.get()))\
            .grid(row=2, column=1, padx=4, pady=4, sticky="w")

        ttk.Checkbutton(scale, text="Mostrar grelha de 1 m",
                        variable=self.show_meter_grid, command=self.redraw)\
            .grid(row=3, column=0, columnspan=2, padx=6, pady=(6,6), sticky="w")

        self.lbl_scale_info = ttk.Label(scale, text="")
        self.lbl_scale_info.grid(row=3, column=2, columnspan=2, padx=6, pady=(6,6), sticky="e")

        for c in range(4):
            scale.columnconfigure(c, weight=1)

        # --- tab veículo/escala ---
        v=0
        ttk.Button(tab_vehicle, text="Definir Origem + Orientação", command=self.set_start_point).grid(row=v, column=0, padx=6, pady=6, sticky="w"); v+=1
        ttk.Button(tab_vehicle, text="Recolocar Origem", command=self.relocate_start).grid(row=v, column=0, padx=6, pady=6, sticky="w"); v+=1

        # (REMOVIDA) toda a secção "Escala de Comprimento (baseada na GRID)" que estava aqui

        params = ttk.LabelFrame(tab_vehicle, text="Modelo Bicicleta")
        params.grid(row=v, column=0, columnspan=2, sticky="we", padx=6, pady=6)
        self.lbl_wb = ttk.Label(params, text=f"Wheelbase: {self.wheelbase*1000:.0f} mm"); self.lbl_wb.grid(row=0, column=0, sticky="w", padx=6)
        self.lbl_sa = ttk.Label(params, text=f"Direção máx: {math.degrees(self.max_steering_angle):.0f}°"); self.lbl_sa.grid(row=1, column=0, sticky="w", padx=6)
        self.lbl_rt = ttk.Label(params, text=f"Raio mínimo: {self.min_turning_radius:.2f} m"); self.lbl_rt.grid(row=2, column=0, sticky="w", padx=6)
        ttk.Label(params, text="Velocidade máx (m/s)").grid(row=0, column=1, padx=6, sticky="w")
        self.speed_slider = tk.Scale(params, from_=0.5, to=6.0, resolution=0.1, orient=tk.HORIZONTAL,
                                     command=self.update_max_speed, length=220)
        self.speed_slider.set(self.max_speed); self.speed_slider.grid(row=0, column=2, padx=6)
        ttk.Label(params, text="Ângulo máx direção (°)").grid(row=1, column=1, padx=6, sticky="w")
        self.steering_slider = tk.Scale(params, from_=10, to=45, resolution=1, orient=tk.HORIZONTAL,
                                        command=self.update_max_steering, length=220)
        self.steering_slider.set(math.degrees(self.max_steering_angle)); self.steering_slider.grid(row=1, column=2, padx=6)

        # --- tab objetivos ---
        ttk.Button(tab_goals, text="Adicionar Objetivo", command=self.add_goal_mode).pack(side=tk.TOP, padx=8, pady=8)
        avoid_frame = ttk.LabelFrame(tab_goals, text="Evitar objetivos não-ativos")
        avoid_frame.pack(fill=tk.X, padx=8, pady=(0,8))
        ttk.Label(avoid_frame, text="Margem (m):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        self.avoid_margin_var = tk.DoubleVar(value=self.goal_avoid_margin_m)
        tk.Spinbox(avoid_frame, from_=0.0, to=3.0, increment=0.1,
                   textvariable=self.avoid_margin_var, width=5,
                   command=lambda: setattr(self,"goal_avoid_margin_m", float(self.avoid_margin_var.get()))
                   ).grid(row=0, column=1, padx=4, pady=4, sticky="w")
        ttk.Label(avoid_frame, text="Peso:").grid(row=0, column=2, sticky="w", padx=(12,6), pady=4)
        self.avoid_weight_var = tk.DoubleVar(value=self.goal_avoid_weight)
        tk.Spinbox(avoid_frame, from_=0.0, to=3.0, increment=0.1,
                   textvariable=self.avoid_weight_var, width=5,
                   command=lambda: setattr(self,"goal_avoid_weight", float(self.avoid_weight_var.get()))
                   ).grid(row=0, column=3, padx=4, pady=4, sticky="w")

        entry_frame = ttk.LabelFrame(tab_goals, text="Entrada no objetivo corrente")
        entry_frame.pack(fill=tk.X, padx=8, pady=(0,8))
        ttk.Label(entry_frame, text="Banda de chegada (m):").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        self.band_var = tk.DoubleVar(value=self.early_entry_band_m)
        tk.Spinbox(entry_frame, from_=0.0, to=2.0, increment=0.1,
                   textvariable=self.band_var, width=5,
                   command=lambda: setattr(self,"early_entry_band_m", float(self.band_var.get()))
                   ).grid(row=0, column=1, padx=4, pady=4, sticky="w")

        # --- tab executar ---
        run_top = ttk.Frame(tab_run); run_top.pack(fill=tk.X, padx=6, pady=6)
        ttk.Button(run_top, text="🟢 Comparar (Hybrid A* + DWA + MPC)", command=self.run_all)\
            .pack(side=tk.LEFT, padx=4)
        ttk.Button(run_top, text="⏹ Parar", command=self.stop_all)\
            .pack(side=tk.LEFT, padx=4)

        self.progress = ttk.Progressbar(tab_run, mode="determinate", maximum=100)
        self.progress.pack(fill=tk.X, padx=8, pady=(4,6))

        log_frame = ttk.Frame(tab_run); log_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self.log = tk.Text(log_frame, height=18, wrap="none")
        ybar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        xbar = ttk.Scrollbar(log_frame, orient="horizontal", command=self.log.xview)
        self.log.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        self.log.grid(row=0, column=0, sticky="nsew"); ybar.grid(row=0, column=1, sticky="ns"); xbar.grid(row=1, column=0, sticky="ew")
        log_frame.rowconfigure(0, weight=1); log_frame.columnconfigure(0, weight=1)

        export_frame = ttk.LabelFrame(tab_run, text="Publicar caminho (ROS2)")
        export_frame.pack(fill=tk.X, padx=6, pady=6)
        ttk.Label(export_frame, text="frame_id:").grid(row=0, column=0, padx=6, pady=4, sticky="w")
        self.frame_id_var = tk.StringVar(value=self.frame_id)
        ttk.Entry(export_frame, textvariable=self.frame_id_var, width=12).grid(row=0, column=1, padx=4, pady=4, sticky="w")

        ttk.Label(export_frame, text="offset X (m):").grid(row=0, column=2, padx=(12,6), pady=4, sticky="w")
        self.offx_var = tk.DoubleVar(value=self.origin_x_m)
        tk.Spinbox(export_frame, from_=-1000, to=1000, increment=0.1, width=8,
                   textvariable=self.offx_var,
                   command=lambda: setattr(self, "origin_x_m", float(self.offx_var.get()))
                   ).grid(row=0, column=3, padx=4, pady=4, sticky="w")

        ttk.Label(export_frame, text="offset Y (m):").grid(row=0, column=4, padx=(12,6), pady=4, sticky="w")
        self.offy_var = tk.DoubleVar(value=self.origin_y_m)
        tk.Spinbox(export_frame, from_=-1000, to=1000, increment=0.1, width=8,
                   textvariable=self.offy_var,
                   command=lambda: setattr(self, "origin_y_m", float(self.offy_var.get()))
                   ).grid(row=0, column=5, padx=4, pady=4, sticky="w")

        ttk.Checkbutton(export_frame, text="Ancorar origem no start (0,0)", variable=self.anchor_origin_start)\
            .grid(row=2, column=0, columnspan=6, padx=6, pady=(2,8), sticky="w")

        ttk.Button(export_frame, text="📤 Publicar Path (ROS2)",
                   command=self.publish_reference_path)\
            .grid(row=3, column=0, columnspan=8, padx=6, pady=8, sticky="we")
        for c in range(8): export_frame.columnconfigure(c, weight=1)

        # --- tab Desenhar (pincel) ---
        d = 0
        ttk.Label(tab_draw, text="Desenha no canvas à direita mantendo o botão esquerdo premido e a arrastar (pincel).").grid(row=d, column=0, columnspan=4, padx=8, pady=(8,4), sticky="w"); d+=1
        ttk.Button(tab_draw, text="✏️ Iniciar desenho", command=self.start_draw_mode)\
            .grid(row=d, column=0, padx=6, pady=6, sticky="w")
        ttk.Button(tab_draw, text="⏹ Parar", command=self.stop_draw_mode)\
            .grid(row=d, column=1, padx=6, pady=6, sticky="w")
        ttk.Button(tab_draw, text="🧹 Limpar", command=self.clear_draw)\
            .grid(row=d, column=2, padx=6, pady=6, sticky="w"); d+=1

        ttk.Label(tab_draw, text="Amostragem (px entre pontos):").grid(row=d, column=0, padx=6, pady=6, sticky="w")
        self.draw_sample_var = tk.DoubleVar(value=self.draw_sampling_px)
        tk.Spinbox(tab_draw, from_=0.5, to=10.0, increment=0.5, width=6,
                   textvariable=self.draw_sample_var,
                   command=lambda: setattr(self, "draw_sampling_px", float(self.draw_sample_var.get())))\
            .grid(row=d, column=1, padx=4, pady=6, sticky="w")

        ttk.Checkbutton(tab_draw, text="Suavizar quando usar (Chaikin leve)", variable=self.draw_smooth_on_use)\
            .grid(row=d, column=2, padx=6, pady=6, sticky="w"); d+=1

        ttk.Button(tab_draw, text="✅ Usar desenho como caminho", command=self.use_draw_as_path)\
            .grid(row=d, column=0, columnspan=2, padx=6, pady=10, sticky="we")
        ttk.Button(tab_draw, text="📤 Publicar desenho (ROS2)", command=self.publish_drawn_path)\
            .grid(row=d, column=2, columnspan=2, padx=6, pady=10, sticky="we"); d+=1
        for c in range(4): tab_draw.columnconfigure(c, weight=1)

        # eventos canvas
        self.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas.mpl_connect('button_release_event', self.on_mouse_release)

        # inicializa label escala
        self.update_scale()

    # ---------- UI helpers ----------
    def _set_sqm(self, val):
        try: self.squares_per_meter = float(val)
        except: return
        if self.scale_mode.get() == "sqm":
            self.update_scale()

    def _set_mps(self, val):
        try: self.meters_per_square = float(val)
        except: return
        if self.scale_mode.get() == "mps":
            self.update_scale()

    def update_scale(self):
        if self.scale_mode.get() == "sqm":
            try: self.squares_per_meter = float(self.sqm_var.get())
            except: pass
        else:
            try: self.meters_per_square = float(self.mps_var.get())
            except: pass
        ppm = self.ppm()
        px_per_sq = float(self.grid_spacing) if self.grid_spacing else 0.0
        if self.scale_mode.get() == "sqm":
            txt = f"1 m = {self.squares_per_meter:.2f} quad  |  {ppm:.1f} px/m  |  1 quad = {self.px2m(px_per_sq):.3f} m"
        else:
            txt = f"1 quad = {self.meters_per_square:.3f} m  |  {ppm:.1f} px/m"
        try: self.lbl_scale_info.config(text=txt)
        except: pass
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steering_angle)
        self.lbl_rt.config(text=f"Raio mínimo: {self.min_turning_radius:.2f} m")
        self.redraw()

    def update_max_speed(self, v): self.max_speed = float(v)
    def update_max_steering(self, v):
        self.max_steering_angle = math.radians(float(v))
        self.min_turning_radius = self.wheelbase / math.tan(self.max_steering_angle)
        self.lbl_rt.config(text=f"Raio mínimo: {self.min_turning_radius:.2f} m")

    def validate_axes(self):
        name_x = self.axis_x_var.get()
        name_y = self.axis_y_var.get()
        vec_map = {"Direita": ( 1.0,  0.0), "Esquerda":(-1.0,  0.0), "Cima":(0.0,-1.0), "Baixo":(0.0,1.0)}
        vx, vy = vec_map[name_x]; ux, uy = vec_map[name_y]
        if abs(vx*ux + vy*uy) != 0:
            messagebox.showerror("Orientação inválida",
                                 f"X+ = {name_x}, Y+ = {name_y} não são ortogonais.")
            return
        self.axis_x_vec = (vx, vy); self.axis_y_vec = (ux, uy)
        self.pending_axis_x = name_x; self.pending_axis_y = name_y
        self.log_msg(f"✅ Eixos aplicados: X+→{name_x}, Y+→{name_y}")

    def update_status(self, s):
        self.status_label.config(text=f"Status: {s}")
        self.update()

    def log_msg(self, s):
        self.log.config(state="normal"); self.log.insert("end", s+"\n"); self.log.see("end"); self.log.config(state="disabled")

    # ---------- física ----------
    def bicycle_step(self, x, y, th, v, steer, dt):
        dx = self.m2px(v*math.cos(th)*dt)
        dy = self.m2px(v*math.sin(th)*dt)
        x2, y2 = x + dx, y + dy
        th2 = ang_norm(th + (v / self.wheelbase) * math.tan(steer) * dt)
        return x2, y2, th2

    # ---------- colisão/visão ----------
    def is_valid(self, x, y):
        if not self.image: return True
        w,h = self.image.size
        if not (0 <= x < w and 0 <= y < h): return False
        margin = max(10, self.grid_spacing/3 if self.grid_spacing else 10)
        for ox,oy in self.obstacle_points:
            if abs(x-ox) < margin and abs(y-oy) < margin:
                if math.hypot(x-ox, y-oy) < margin: return False
        return True

    def los_clear(self, p1, p2):
        x1,y1 = p1; x2,y2 = p2
        d = math.hypot(x2-x1, y2-y1)
        if d == 0: return True
        step = max(2, int(d / max(2, (self.grid_spacing or 20)/2)))
        for i in range(step+1):
            t = i/step
            x = x1 + t*(x2-x1); y = y1 + t*(y2-y1)
            if not self.is_valid(x,y): return False
        return True

    # ---------- objectivo: regras ----------
    def goal_hit(self, x, y, goal):
        gx, gy, gr = goal
        extra = max(12, int(self.grid_spacing or 12))
        return (x-gx)**2 + (y-gy)**2 <= (gr + extra)**2

    def goal_avoid_penalty(self, x, y, current_idx):
        if not self.goals: return 0.0
        pen_px = 0.0
        margin_px = self.m2px(self.goal_avoid_margin_m)
        for j, (gx, gy, gr) in enumerate(self.goals):
            if j == current_idx: continue
            d = math.hypot(x - gx, y - gy)
            thresh = gr + margin_px
            if d < thresh: pen_px += (thresh - d)
        return self.px2m(pen_px) * float(self.goal_avoid_weight)

    def early_entry_forbidden(self, prev_xy, new_xy, goal):
        gx, gy, gr = goal
        d_prev = math.hypot(prev_xy[0]-gx, prev_xy[1]-gy)
        d_new  = math.hypot(new_xy[0]-gx,  new_xy[1]-gy)
        band = self.m2px(self.early_entry_band_m)
        return (d_new <= gr) and (d_prev > gr + band)

    # ---------- desenho ----------
    def redraw(self, also_paths=None, overlay=None):
        self.ax.clear()
        if self.image is not None:
            self.ax.imshow(self.image); w,h = self.image.size
            self.ax.set_xlim(0,w); self.ax.set_ylim(h,0)
        self.ax.axis('off')

        if self.grid_spacing and self.image is not None:
            w,h = self.image.size
            for x in np.arange(0,w,self.grid_spacing):
                self.ax.axvline(x=x, linestyle='--', linewidth=0.3, color='gray', alpha=0.5)
            for y in np.arange(0,h,self.grid_spacing):
                self.ax.axhline(y=y, linestyle='--', linewidth=0.3, color='gray', alpha=0.5)

        if self.show_meter_grid.get() and self.image is not None:
            w,h = self.image.size
            step = max(8.0, self.m2px(1.0))
            if step > 0:
                xs = np.arange(0, w+1, step)
                ys = np.arange(0, h+1, step)
                for x in xs:
                    self.ax.axvline(x=x, linestyle='-', linewidth=0.25, color='lightblue', alpha=0.7)
                for y in ys:
                    self.ax.axhline(y=y, linestyle='-', linewidth=0.25, color='lightblue', alpha=0.7)

        if self.start_point is not None:
            sx,sy = self.start_point
            self.ax.plot(sx,sy,'go',markersize=11, markeredgecolor='black', markeredgewidth=2)
            L = self.m2px(0.35)
            self.ax.arrow(sx,sy, L*math.cos(self.start_theta), L*math.sin(self.start_theta),
                          head_width=8, head_length=5, fc='red', ec='red', alpha=0.9)

        for px,py in self.obstacle_points:
            self.ax.plot(px,py,'ro',markersize=5,alpha=0.8)

        for i,(gx,gy,gr) in enumerate(self.goals):
            circ = matplotlib.patches.Circle((gx, gy), gr, color='green', alpha=0.55,
                                             edgecolor='darkgreen', linewidth=2)
            self.ax.add_patch(circ)
            self.ax.text(gx, gy, str(i+1), color='white', fontsize=12, ha='center', va='center', fontweight='bold')

        if self.image is not None and self.grid_spacing:
            w,h = self.image.size
            bar = self.m2px(1.0); y0 = h - 15; x0 = 15
            self.ax.plot([x0,x0+bar],[y0,y0],'k-',linewidth=4)
            self.ax.text(x0+bar/2,y0-8,"1 m",ha='center',va='bottom',fontsize=10,color='black',fontweight='bold',
                         bbox=dict(facecolor='white',alpha=0.7,edgecolor='none'))

        if also_paths:
            for col, path, lw, alpha in also_paths:
                if path and len(path) > 1:
                    xs = [p[0] for p in path]; ys=[p[1] for p in path]
                    self.ax.plot(xs, ys, color=col, linewidth=lw, alpha=alpha)

        if overlay:
            x,y,theta,color = overlay
            head = self.m2px(0.22); tail = self.m2px(0.12)
            l = theta + 2.6; r = theta - 2.6
            self.ax.fill([x+head*math.cos(theta), x-tail*math.cos(l), x-tail*math.cos(r)],
                         [y+head*math.sin(theta), y-tail*math.sin(l), y-tail*math.sin(r)],
                         color=color, alpha=0.95, edgecolor='black', linewidth=1.0)

        self._draw_overlay()

        if self.last_measure:
            (x1,y1), (x2,y2), dpx, dm = self.last_measure
            self.ax.plot([x1,x2],[y1,y2], color='red', linewidth=2.5, alpha=0.9)
            mx, my = (x1+x2)/2.0, (y1+y2)/2.0
            self.ax.text(mx, my, f"{dm:.3f} m", color='red', fontsize=10,
                         ha='center', va='bottom',
                         bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

        self.canvas.draw()

    # ---------- modos / interacção ----------
    def load_image(self):
        from tkinter import filedialog
        path = filedialog.askopenfilename(filetypes=[("Image files","*.png *.jpg *.jpeg *.bmp *.gif")])
        if not path: return
        try:
            self.image = Image.open(path); self.image_array = np.array(self.image)
            w,h = self.image.size; self.grid_spacing = h / self.grid_count
            self.obstacle_points.clear(); self.start_point=None; self.goals.clear()
            self.update_scale()
            self.redraw(); self.update_status("Imagem carregada")
        except Exception as e:
            messagebox.showerror("Erro", f"Erro ao carregar imagem: {e}")

    def update_grid(self, v):
        try:
            self.grid_count = int(v)
            if self.image:
                _,h = self.image.size; self.grid_spacing = h / self.grid_count
                self.update_scale()
        except Exception: pass

    def set_start_point(self):
        if self.start_point is None:
            self.mode = 'set_start'
            self.update_status("Clique para posição da origem; mova o rato para orientar; clique de novo para confirmar.")
        else:
            self.update_status("Clique em cima da origem e arraste para rodar; ou use 'Recolocar Origem'.")
            self.mode = None

    def relocate_start(self):
        self.mode = 'relocate_start'
        self.update_status("Clique em novo ponto para recolocar a origem; depois mova o rato para orientar e clique para confirmar.")

    def add_limit_mode(self):
        if not self.image:
            messagebox.showwarning("Aviso", "Carregue uma imagem primeiro."); return
        self.mode = 'add_limit'; self.update_status("Clique numa zona (flood) para marcar obstáculo.")

    def remove_limit_mode(self):
        self.mode = 'remove_limit'; self.update_status("Clique num ponto vermelho para remover obstáculo.")

    def clear_all_limits(self):
        self.obstacle_points.clear(); self.redraw()

    def add_goal_mode(self):
        self.mode = 'add_goal'; self.update_status("Clique para adicionar objetivo; arraste a borda para redimensionar.")

    def start_measure(self):
        self.measure_p1 = None; self.measure_p2 = None; self.last_measure = None
        self.mode = 'measure'
        self.update_status("Medição: clica no ponto 1 e depois no ponto 2.")

    def _finish_measure(self):
        if self.measure_p1 and self.measure_p2:
            x1,y1 = self.measure_p1; x2,y2 = self.measure_p2
            d_px = float(math.hypot(x2-x1, y2-y1))
            d_m  = float(self.px2m(d_px))
            self.last_measure = (self.measure_p1, self.measure_p2, d_px, d_m)
            self.log_msg(f"📏 Medição: d ≈ {d_px:.1f} px  |  {d_m:.3f} m")
        self.mode = None
        self.update_status("Medição concluída.")
        self.redraw()

    def on_mouse_press(self, e):
        if e.xdata is None or e.ydata is None: return

        if self.mode == 'measure':
            if self.measure_p1 is None:
                self.measure_p1 = (e.xdata, e.ydata)
                self.update_status("Medição: escolhe o ponto 2.")
            else:
                self.measure_p2 = (e.xdata, e.ydata)
                self._finish_measure()
            return

        if self.mode == 'draw_free':
            self.draw_active_drag = True
            self._append_draw_point(e.xdata, e.ydata)
            self.redraw()
            return

        if self.mode is None and self.start_point is not None:
            sx, sy = self.start_point
            if (e.xdata - sx)**2 + (e.ydata - sy)**2 <= (12**2):
                self.mode = 'rotate_start'
                self.update_status("A rodar a orientação inicial…")
                return

        if self.mode == 'set_start' or self.mode == 'relocate_start':
            self.start_point = (e.xdata, e.ydata)
            self.start_theta = 0.0
            self.mode = 'set_start_heading'
            self.redraw(); return

        if self.mode == 'set_start_heading':
            self.mode = None; self.update_status("Origem definida."); return

        elif self.mode == 'add_limit':
            if not self.grid_spacing: return
            cx = int(e.xdata // self.grid_spacing); cy = int(e.ydata // self.grid_spacing)
            pts = self.flood_fill_grid(cx, cy)
            for p in pts:
                if p not in self.obstacle_points: self.obstacle_points.append(p)
            self.redraw()

        elif self.mode == 'remove_limit':
            if not self.obstacle_points or not self.grid_spacing: return
            px,py = e.xdata, e.ydata
            for i,(cx,cy) in enumerate(self.obstacle_points):
                if abs(cx-px)<self.grid_spacing/2 and abs(cy-py)<self.grid_spacing/2:
                    self.obstacle_points.pop(i); break
            self.redraw()

        elif self.mode == 'add_goal':
            self.goals.append([e.xdata, e.ydata, self.goal_radius]); self.mode=None; self.redraw()

        else:
            for idx,(gx,gy,gr) in enumerate(self.goals):
                if (e.xdata-gx)**2 + (e.ydata-gy)**2 < gr**2:
                    self.selected_goal = idx; self.mode='resize_goal'; break

    def on_mouse_move(self, e):
        if e.xdata is None or e.ydata is None: return

        if self.mode == 'draw_free' and self.draw_active_drag:
            if self._append_draw_point(e.xdata, e.ydata):
                self.redraw()
            return

        if self.mode == 'set_start_heading' and self.start_point is not None:
            sx,sy = self.start_point
            self.start_theta = math.atan2(e.ydata - sy, e.xdata - sx); self.redraw()
        elif self.mode == 'rotate_start' and self.start_point is not None:
            sx,sy = self.start_point
            self.start_theta = math.atan2(e.ydata - sy, e.xdata - sx); self.redraw()
        if self.selected_goal is not None:
            gx,gy,gr = self.goals[self.selected_goal]
            self.goals[self.selected_goal][2] = max(6, int(np.hypot(e.xdata-gx, e.ydata-gy)))
            self.redraw()

    def on_mouse_release(self, e):
        if self.mode == 'draw_free':
            self.draw_active_drag = False
            return
        if self.mode == 'rotate_start':
            self.mode = None; self.update_status("Origem orientada.")
        self.selected_goal=None
        if self.mode == 'resize_goal': self.mode=None

    # ---------- flood fill ----------
    def flood_fill_grid(self, cell_x, cell_y):
        if self.image_array is None or self.grid_spacing is None: return []
        h,w = self.image_array.shape[:2]
        gw = int(w // self.grid_spacing); gh = int(h // self.grid_spacing)
        if not (0 <= cell_x < gw and 0 <= cell_y < gh): return []
        visited=set(); q=deque([(cell_x,cell_y)]); pts=[]
        px=int(cell_x*self.grid_spacing + self.grid_spacing//2)
        py=int(cell_y*self.grid_spacing + self.grid_spacing//2)
        if px<0 or px>=w or py<0 or py>=h: return []
        ref = self.image_array[py,px]
        while q:
            cx,cy=q.popleft()
            if (cx,cy) in visited or not (0<=cx<gw and 0<=cy<gh): continue
            visited.add((cx,cy))
            px=int(cx*self.grid_spacing + self.grid_spacing//2)
            py=int(cy*self.grid_spacing + self.grid_spacing//2)
            if px<0 or px>=w or py<0 or py>=h:
                continue
            col = self.image_array[py,px]
            ok = (np.all(np.abs(col.astype(int)-ref.astype(int))<=self.color_tolerance)
                  if len(self.image_array.shape)==3 else abs(int(col)-int(ref))<=self.color_tolerance)
            if ok:
                pts.append((px,py))
                for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),( -1,-1),( -1,1),(1,-1),(1,1)]:
                    q.append((cx+dx,cy+dy))
        return pts

    # =========================================================
    # =============== PIPELINE: H*A* + DWA + MPC ==============
    # =========================================================
    def stop_all(self):
        if self.anim_timer:
            try: self.after_cancel(self.anim_timer)
            except: pass
        self.anim_timer = None
        self.run_state = None
        self.update_status("Parado.")

    def run_all(self):
        if not self.image or not self.start_point or not self.goals:
            messagebox.showwarning("Aviso","Carrega imagem, define origem+orientação e adiciona objetivos.")
            return
        self.stop_all()
        self.log.delete("1.0","end")
        self.progress['value'] = 0
        self.run_state = dict(
            seg_index = 0,
            seg_count = len(self.goals),
            seg_start_pose = (self.start_point[0], self.start_point[1], self.start_theta),
            full_path = [],
            attempt = 0, max_attempts = 3, entry_pts = None, entry_pt = None,
            phase = "hybrid",
            ha_open = [], ha_counter = 0, ha_closed = set(),
            ha_came = {}, ha_g = {},
            ha_grid_res = max(20, self.grid_spacing or 30),
            ha_angle_res = math.pi/4,
            ha_current_best = None,
            ha_iter = 0, ha_max_iter = 4000,
            dwa_state = None, dwa_path = [], dwa_steps = 0, dwa_stuck = 0,
            mpc_iter = 0, mpc_max_iter = 60, mpc_best = None, mpc_seed = None,
            path_hybrid = None, path_dwa = None, path_mpc = None
        )
        self.manual_path = None
        self._start_segment()

    def _entry_points(self, gx, gy, gr, sx, sy, K=6):
        base_ang = math.atan2(sy - gy, sx - gx)
        pts=[]
        for i in range(K):
            ang = base_ang + (i - (K-1)/2.0) * (math.pi/(K+1))
            x = gx + (gr - self.m2px(0.1)) * math.cos(ang)
            y = gy + (gr - self.m2px(0.1)) * math.sin(ang)
            pts.append((x,y, ang))
        return sorted(pts, key=lambda p: abs(ang_norm(p[2]-base_ang)))

    def _start_segment(self):
        st = self.run_state
        if st["seg_index"] >= st["seg_count"]:
            if st["full_path"]:
                self.redraw(also_paths=[('blue', [(x,y) for (x,y,_,_) in st["full_path"]], 3, 0.8)])
                self._build_and_draw_raceline(st["full_path"])
                self.update_status("Concluído — todos os objetivos atingidos + linha de corrida feita.")
                self.progress['value'] = 100
            else:
                self.update_status("Sem caminho.")
            return

        if st["attempt"] == 0:
            gx, gy, gr = self.goals[st["seg_index"]]
            sx, sy, _ = st["seg_start_pose"]
            st["entry_pts"] = self._entry_points(gx, gy, gr, sx, sy, K=6)

        if st["attempt"] >= st["max_attempts"]:
            self.log_msg(f"[Seg {st['seg_index']+1}] Falhou após {st['max_attempts']} tentativas → a saltar alvo.")
            st["seg_index"] += 1
            self._start_segment()
            return

        idx = min(st["attempt"], len(st["entry_pts"])-1)
        st["entry_pt"] = (st["entry_pts"][idx][0], st["entry_pts"][idx][1])

        st["phase"] = "hybrid"
        st["path_hybrid"] = st["path_dwa"] = st["path_mpc"] = None
        st["ha_open"].clear(); st["ha_closed"].clear(); st["ha_came"].clear(); st["ha_g"].clear()
        st["ha_iter"] = 0; st["ha_current_best"] = None; st["ha_counter"] = 0
        st["ha_grid_res"] = max(16, (self.grid_spacing or 30) * (0.9 ** st["attempt"]))
        st["ha_max_iter"] = 4000 + 1500*st["attempt"]

        gx, gy, gr = self.goals[st["seg_index"]]
        st["cur_goal"] = (gx, gy, gr)
        sx, sy, sth = st["seg_start_pose"]

        st["ha_start"] = (sx, sy, sth)
        st["ha_goal"]  = (gx, gy)
        heapq.heappush(st["ha_open"], (0.0, st["ha_counter"], st["ha_start"])); st["ha_counter"] += 1
        st["ha_g"][st["ha_start"]] = 0.0

        self.log_msg(f"[Seg {st['seg_index']+1}] Tentativa {st['attempt']+1}: Hybrid A* → entrada dirigida.")
        self.update_status(f"Segmento {st['seg_index']+1} — tentativa {st['attempt']+1}: Hybrid A*")
        self._tick_hybrid()

    def _ha_disc(self, x,y,th, grid_res, ang_res):
        return (int(x / grid_res), int(y / grid_res), int((th + math.pi) / ang_res))

    def _gen_primitives(self, x,y,th, cur_goal):
        speeds = [0.6, 1.0, 1.6]
        steering = [-self.max_steering_angle, -self.max_steering_angle/2, 0,
                    self.max_steering_angle/2, self.max_steering_angle]
        prims = []
        for v in speeds:
            for sa in steering:
                cx,cy,ct = x,y,th
                traj = [(cx,cy,ct)]
                ok=True
                for _ in range(8):
                    px,py = cx,cy
                    cx,cy,ct = self.bicycle_step(cx,cy,ct,v,sa,self.dt)
                    if not self.is_valid(cx,cy): ok=False; break
                    if self.early_entry_forbidden((px,py), (cx,cy), cur_goal): ok=False; break
                    traj.append((cx,cy,ct))
                if ok and len(traj)>1:
                    dist_m = self.px2m( math.hypot(traj[-1][0]-x, traj[-1][1]-y) )
                    cost = dist_m + 0.3*abs(sa)
                    prims.append(dict(end=(cx,cy,ct), traj=traj, cost=cost))
        return prims

    def _ha_reconstruct(self, came, cur):
        path=[]
        s=cur
        while s in came:
            path.extend(reversed(came[s]['traj']))
            s=came[s]['parent']
        path.reverse()
        return path

    def _tick_hybrid(self):
        if not self.run_state or self.run_state["phase"] != "hybrid": return
        st = self.run_state
        grid_res = st["ha_grid_res"]; ang_res = st["ha_angle_res"]
        it = st["ha_iter"]; it += 1; st["ha_iter"] = it

        if not st["ha_open"] or it > st["ha_max_iter"]:
            self.log_msg("Hybrid A*: terminou (sem solução clara).")
            st["path_hybrid"] = st["ha_current_best"] or None
            self._start_dwa(); return

        _, _, cur = heapq.heappop(st["ha_open"])
        cd = self._ha_disc(*cur, grid_res, ang_res)
        if cd in st["ha_closed"]:
            self.anim_timer = self.after(1, self._tick_hybrid); return
        st["ha_closed"].add(cd)

        gx,gy,gr = st["cur_goal"]
        ex,ey = st["entry_pt"]
        dist_center = math.hypot(cur[0]-gx, cur[1]-gy)
        use_tx, use_ty = (ex,ey) if dist_center > gr + self.m2px(self.early_entry_band_m*0.8) else (gx,gy)

        if dist_center < grid_res*1.5:
            path = self._ha_reconstruct(st["ha_came"], cur)
            if not path:
                path = [(st["ha_start"][0], st["ha_start"][1], st["ha_start"][2]), (gx,gy,0.0)]
            st["path_hybrid"] = [(p[0],p[1], p[2], self.max_speed) for p in path]
            self.log_msg(f"Hybrid A*: solução (tent {st['attempt']+1}) com {len(path)} pontos em {it} it.")
            self.redraw(also_paths=[('orange', [(p[0],p[1]) for p in st["path_hybrid"]], 3, 0.9)])
            self._start_dwa(); return

        for prim in self._gen_primitives(*cur, cur_goal=st["cur_goal"]):
            nxt = prim['end']; nd = self._ha_disc(*nxt, grid_res, ang_res)
            if nd in st["ha_closed"]: continue
            ng = st["ha_g"][cur] + prim['cost']
            if nxt not in st["ha_g"] or ng < st["ha_g"][nxt]:
                st["ha_g"][nxt] = ng
                h = math.hypot(nxt[0]-use_tx, nxt[1]-use_ty)
                h += 0.4 * self.goal_avoid_penalty(nxt[0], nxt[1], st["seg_index"])
                f = ng + h
                heapq.heappush(st["ha_open"], (f, st["ha_counter"], nxt)); st["ha_counter"] += 1
                st["ha_came"][nxt] = dict(parent=cur, traj=prim['traj'])

        st["ha_current_best"] = self._ha_reconstruct(st["ha_came"], cur)
        show_best = [(p[0],p[1]) for p in st["ha_current_best"]] if st["ha_current_best"] else None
        self.redraw(also_paths=[('orange', show_best, 2, 0.9)] if show_best else None)
        self.update_status(f"Seg {st['seg_index']+1} tent {st['attempt']+1}: Hybrid A* — it {it}/{st['ha_max_iter']}")
        self.anim_timer = self.after(1, self._tick_hybrid)

    # ----------- DWA -----------
    def _start_dwa(self):
        if not self.run_state: return
        st = self.run_state
        st["phase"] = "dwa"
        sx, sy, sth = st["seg_start_pose"]
        st["dwa_state"] = dict(x=sx, y=sy, th=sth, v=self.max_speed*0.6)
        st["dwa_path"] = [(sx, sy, sth, st["dwa_state"]["v"])]
        st["dwa_steps"] = 0
        st["dwa_stuck"] = 0
        self.log_msg("DWA: a avançar para entrar na região (entrada dirigida, sem entrada precoce).")
        self.update_status(f"Seg {st['seg_index']+1} tent {st['attempt']+1}: DWA")
        self._tick_dwa()

    def _tick_dwa(self):
        if not self.run_state or self.run_state["phase"] != "dwa": return
        st = self.run_state
        gx,gy,gr = st["cur_goal"]
        ex,ey = st["entry_pt"]
        s = st["dwa_state"]
        dist_center = math.hypot(s["x"]-gx, s["y"]-gy)
        tx,ty = (ex,ey) if dist_center > gr + self.m2px(self.early_entry_band_m*0.8) else (gx,gy)

        if self.goal_hit(s["x"], s["y"], (gx,gy,gr)):
            st["path_dwa"] = st["dwa_path"][:]
            self.log_msg(f"DWA: terminou com {len(st['path_dwa'])} amostras (tent {st['attempt']+1}).")
            self._start_mpc_seed(); return

        if st["dwa_steps"] > 1600:
            self.log_msg("DWA: bloqueado por passos excessivos.")
            self._retry_segment(); return

        prev_dist = math.hypot(st["dwa_path"][-1][0]-tx, st["dwa_path"][-1][1]-ty)

        rich = (st["attempt"] >= 1)
        allow_reverse = (st["attempt"] >= 2)
        v = s["v"]; v_min = -0.5 if allow_reverse else 0.0; v_max=self.max_speed; acc=2.2 if rich else 2.0
        v_cands = np.linspace(clamp(v-acc*self.dt, v_min, v_max), clamp(v+acc*self.dt, v_min, v_max), 5 if rich else 4)
        steer_cands = np.linspace(-self.max_steering_angle, self.max_steering_angle, 13 if rich else 9)
        best=None
        for vv in v_cands:
            for ss in steer_cands:
                rx,ry,rth = s["x"], s["y"], s["th"]; ok=True; closest=1e9
                for _ in range(6 + (2 if rich else 0)):
                    px,py = rx,ry
                    rx,ry,rth = self.bicycle_step(rx,ry,rth,vv,ss,self.dt*0.5)
                    if not self.is_valid(rx,ry): ok=False; break
                    if self.early_entry_forbidden((px,py),(rx,ry),(gx,gy,gr)): ok=False; break
                    for (ox,oy) in self.obstacle_points:
                        d = math.hypot(rx-ox, ry-oy); closest = min(closest,d)
                if not ok: continue
                dist = math.hypot(rx-tx, ry-ty)
                cost = dist + 0.12*abs(ss) + 0.18*(v_max - max(0.0,vv))
                if closest < 25: cost += (25-closest)*0.9
                cost += self.goal_avoid_penalty(rx, ry, st["seg_index"])
                if best is None or cost < best[0]:
                    best=(cost, vv, ss)
        if best is None:
            self.log_msg("DWA: sem movimento viável → retry.")
            self._retry_segment(); return

        _, v_new, steer = best
        s["x"], s["y"], s["th"] = self.bicycle_step(s["x"], s["y"], s["th"], v_new, steer, self.dt)
        if self.early_entry_forbidden((st["dwa_path"][-1][0], st["dwa_path"][-1][1]),
                                      (s["x"],s["y"]), (gx,gy,gr)):
            s["x"], s["y"], s["th"] = self.bicycle_step(st["dwa_path"][-1][0], st["dwa_path"][-1][1],
                                                        st["dwa_path"][-1][2], 0.4, 0.0, self.dt)
        s["v"] = v_new
        st["dwa_path"].append((s["x"], s["y"], s["th"], s["v"]))
        st["dwa_steps"] += 1

        new_dist = math.hypot(s["x"]-tx, s["y"]-ty)
        if new_dist >= prev_dist - self.m2px(0.02):
            st["dwa_stuck"] += 1
        else:
            st["dwa_stuck"] = 0
        if st["dwa_stuck"] > 120:
            self.log_msg("DWA: sem progresso → retry com outra entrada.")
            self._retry_segment(); return

        also=[]
        if st.get("path_hybrid"):
            also.append(('orange', [(p[0],p[1]) for p in st["path_hybrid"]], 2, 0.6))
        also.append(('purple', [(p[0],p[1]) for p in st["dwa_path"]], 3, 0.9))
        self.redraw(also_paths=also, overlay=(s["x"], s["y"], s["th"], 'purple'))
        self.update_status(f"Seg {st['seg_index']+1} tent {st['attempt']+1}: DWA — passo {st['dwa_steps']}")
        self.anim_timer = self.after(16, self._tick_dwa)

    def _retry_segment(self):
        st = self.run_state
        st["attempt"] += 1
        self.log_msg(f"→ Retry {st['attempt']+1} / {st['max_attempts']}")
        self._start_segment()

    # ----------- MPC -----------
    def _start_mpc_seed(self):
        if not self.run_state: return
        st = self.run_state
        st["phase"] = "mpc"
        c_h = self._traj_cost(st.get("path_hybrid"))
        c_d = self._traj_cost(st.get("path_dwa"))
        seed = st.get("path_dwa") if (st.get("path_dwa") and c_d <= c_h) else st.get("path_hybrid")
        st["mpc_seed"] = seed; st["mpc_best"] = seed; st["mpc_best_cost"] = self._traj_cost(seed)
        st["mpc_iter"] = 0
        self.log_msg(f"MPC: seed custo {st['mpc_best_cost']:.2f}. A otimizar…")
        self.update_status(f"Seg {st['seg_index']+1} tent {st['attempt']+1}: MPC")
        self._tick_mpc()

    def _traj_cost(self, path):
        if not path or len(path)<2: return 1e9
        st = self.run_state
        dist=0.0; smooth=0.0; obs_pen=0.0; early_pen=0.0
        gx=gy=gr=None
        if st and "cur_goal" in st and st["cur_goal"]:
            gx,gy,gr = st["cur_goal"]
        allow_tail = 8
        for i in range(1,len(path)):
            x1,y1=path[i-1][0],path[i-1][1]; x2,y2=path[i][0],path[i][1]
            dist += self.px2m(math.hypot(x2-x1,y2-y1))
            if i>=2:
                v1=(x1-path[i-2][0], y1-path[i-2][1]); v2=(x2-x1, y2-y1)
                a1=math.atan2(v1[1],v1[0]); a2=math.atan2(v2[1],v2[0])
                smooth += abs(ang_norm(a2-a1))
            for (ox,oy) in self.obstacle_points[::max(1,len(self.obstacle_points)//80 or 1)]:
                d = math.hypot(x2-ox, y2-oy)
                if d < 28: obs_pen += (28-d)*0.06
            cur_idx = st["seg_index"] if st else 0
            obs_pen += self.goal_avoid_penalty(x2, y2, cur_idx)
            if gx is not None and i < len(path)-allow_tail:
                if self.early_entry_forbidden((x1,y1),(x2,y2),(gx,gy,gr)):
                    early_pen += 5.0
        return dist + 0.2*smooth + obs_pen + early_pen

    def _curvature(self, pts):
        k = np.zeros(len(pts))
        if len(pts) < 3: return k
        for i in range(1, len(pts)-1):
            x1,y1 = pts[i-1]; x2,y2=pts[i]; x3,y3=pts[i+1]
            a = np.hypot(x2-x1, y2-y1)
            b = np.hypot(x3-x2, y3-y2)
            c = np.hypot(x3-x1, y3-y1)
            if a*b*c == 0: k[i]=0; continue
            s = (a+b+c)/2.0
            area = max(1e-9, np.sqrt(max(0.0, s*(s-a)*(s-b)*(s-c))))
            R = (a*b*c)/(4.0*area)
            k[i] = 0.0 if R==0 else 1.0/self.px2m(R)
        k[0]=k[1]; k[-1]=k[-2]
        return k

    def _estimate_time(self, path):
        if not path or len(path)<2: return 1e9
        pts = [(x,y) for (x,y,_,_) in path]
        ds = [self.px2m(np.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])) for i in range(len(pts)-1)]
        ds.append(ds[-1] if ds else 0.1)
        k = self._curvature(pts)
        ay_max = self.mu * 9.81
        v_k = [min(self.max_speed, np.sqrt(ay_max/max(1e-6,abs(k[i])))) for i in range(len(pts))]
        v = [0.0]*len(pts); v[0] = min(v_k[0], self.max_speed)
        for i in range(len(pts)-1):
            v_hat = np.sqrt(max(0.0, v[i]**2 + 2*self.ax_max*ds[i]))
            v[i+1] = min(v_hat, v_k[i+1], self.max_speed)
        for i in range(len(pts)-2, -1, -1):
            v_hat = np.sqrt(max(0.0, v[i+1]**2 + 2*self.brake_max*ds[i]))
            v[i] = min(v[i], v_hat, v_k[i], self.max_speed)
        return sum(ds[i]/max(0.2, v[i]) for i in range(len(pts)))

    def _smooth_path(self, path, iters=2):
        if not path or len(path) < 3: return path[:]
        pts = [(x,y) for (x,y,_,_) in path]
        for _ in range(iters):
            new = [pts[0]]
            for i in range(len(pts)-1):
                p = np.array(pts[i]); q = np.array(pts[i+1])
                new.append(tuple(0.75*p + 0.25*q))
                new.append(tuple(0.25*p + 0.75*q))
            new.append(pts[-1])
            pts = new
        out=[]
        for i,(x,y) in enumerate(pts):
            if i==0: th = math.atan2(pts[1][1]-y, pts[1][0]-x)
            else: th = math.atan2(y-pts[i-1][1], x-pts[i-1][0])
            out.append((x,y,th,self.max_speed))
        return out

    def _nearest_obstacle_dist(self, x, y):
        if not self.obstacle_points: return 1e9
        step = max(1, len(self.obstacle_points)//300)
        dmin = 1e9
        for (ox,oy) in self.obstacle_points[::step]:
            d = math.hypot(x-ox, y-oy)
            if d < dmin: dmin = d
        return self.px2m(dmin)

    def _raceline_profile(self, path):
        if not path or len(path) < 2: return None
        pts = [(x,y) for (x,y,_,_) in path]
        ds = [self.px2m(np.hypot(pts[i+1][0]-pts[i][0], pts[i+1][1]-pts[i][1])) for i in range(len(pts)-1)]
        ds.append(ds[-1] if ds else 0.1)
        k = self._curvature(pts)
        ay_max = self.mu * 9.81
        v_k = np.array([min(self.max_speed, np.sqrt(ay_max/max(1e-6,abs(k[i])))) for i in range(len(pts))])
        v_obs = []
        for (x,y) in pts:
            d = max(0.01, self._nearest_obstacle_dist(x,y) - self.safety_margin_m)
            factor = clamp(d/(d+0.7), 0.35, 1.0)
            v_obs.append(self.max_speed * factor)
        v_lim = np.minimum(v_k, np.array(v_obs))

        v = [0.0]*len(pts); v[0] = min(v_lim[0], self.max_speed)
        for i in range(len(pts)-1):
            v_hat = np.sqrt(max(0.0, v[i]**2 + 2*self.ax_max*ds[i]))
            v[i+1] = min(v_hat, v_lim[i+1], self.max_speed)
        for i in range(len(pts)-2, -1, -1):
            v_hat = np.sqrt(max(0.0, v[i+1]**2 + 2*self.brake_max*ds[i]))
            v[i] = min(v[i], v_hat, v_lim[i], self.max_speed)

        total_time = sum(ds[i]/max(0.2, v[i]) for i in range(len(pts)))
        avg_v = (sum(ds[:-1]) / max(1e-6, total_time))
        return dict(points=pts, speeds=v, ds=ds, time=total_time, vavg=avg_v)

    def _draw_raceline(self, raceline):
        pts = raceline["points"]; v = raceline["speeds"]
        segs = [[pts[i], pts[i+1]] for i in range(len(pts)-1)]
        lc = LineCollection(segs, array=np.array(v[:-1]), cmap='plasma', linewidths=4)
        if self._raceline_lc is not None:
            try: self._raceline_lc.remove()
            except: pass
            self._raceline_lc = None
        self.ax.add_collection(lc)
        self._raceline_lc = lc
        if self._raceline_cbar is not None:
            try: self._raceline_cbar.remove()
            except: pass
            self._raceline_cbar = None
        self._raceline_cbar = self.fig.colorbar(lc, ax=self.ax, fraction=0.03, pad=0.01)
        self._raceline_cbar.set_label('Velocidade [m/s]')
        self.canvas.draw()

    def _build_and_draw_raceline(self, full_path):
        smooth = self._smooth_path(full_path, iters=2)
        race = self._raceline_profile(smooth)
        if race is None:
            self.log_msg("Linha de corrida: falhou a gerar perfil.")
            return
        self.redraw()
        self._draw_raceline(race)
        self.log_msg(f"Race line: tempo {race['time']:.2f} s | v̄ = {race['vavg']:.2f} m/s")

    def _tick_mpc(self):
        if not self.run_state or self.run_state["phase"] != "mpc": return
        st = self.run_state
        it = st["mpc_iter"]; it += 1; st["mpc_iter"] = it
        if it > st["mpc_max_iter"]:
            st["path_mpc"] = st["mpc_best"]
            self._finish_segment_and_continue(); return

        base = st["mpc_best"]
        if not base or len(base) < 3:
            st["path_mpc"] = base
            self._finish_segment_and_continue(); return

        N = 40
        best_local = None; best_c = 1e9
        for _ in range(N):
            cand = []
            for i,p in enumerate(base):
                x,y,th,v = p
                if 0 < i < len(base)-1:
                    x += np.random.normal(0, 3.0)
                    y += np.random.normal(0, 3.0)
                cand.append((x,y,th,v))
            ok=True
            for i in range(1,len(cand)):
                if not self.los_clear((cand[i-1][0],cand[i-1][1]), (cand[i][0],cand[i][1])):
                    ok=False; break
            if not ok: continue
            c = self._traj_cost(cand)
            if c < best_c:
                best_c = c; best_local = cand

        if best_local is not None and best_c < st["mpc_best_cost"]:
            st["mpc_best"] = best_local
            st["mpc_best_cost"] = best_c

        also = []
        if st.get("mpc_seed"):
            also.append(('lightgray', [(p[0],p[1]) for p in st["mpc_seed"]], 2, 0.6))
        if st.get("path_hybrid"):
            also.append(('orange', [(p[0],p[1]) for p in st["path_hybrid"]], 1.5, 0.5))
        if st.get("path_dwa"):
            also.append(('purple', [(p[0],p[1]) for p in st["path_dwa"]], 1.5, 0.5))
        also.append(('magenta', [(p[0],p[1]) for p in st["mpc_best"]], 3, 0.9))
        self.redraw(also_paths=also)
        self.update_status(f"Seg {st['seg_index']+1} tent {st['attempt']+1}: MPC — it {it}/{st['mpc_max_iter']}")
        self.anim_timer = self.after(1, self._tick_mpc)

    def _finish_segment_and_continue(self):
        st = self.run_state
        gx, gy, gr = st["cur_goal"]

        def enters(path):
            return bool(path) and self.goal_hit(path[-1][0], path[-1][1], (gx,gy,gr))

        cands = []
        if enters(st.get("path_hybrid")): cands.append(("Hybrid A*", st["path_hybrid"]))
        if enters(st.get("path_dwa")):    cands.append(("DWA",       st["path_dwa"]))
        if enters(st.get("path_mpc")):    cands.append(("MPC",       st["path_mpc"]))

        if not cands:
            self.log_msg(f"[Seg {st['seg_index']+1}] Nenhum caminho entrou no objetivo → nova tentativa.")
            self._retry_segment(); return

        ranked = []
        for name, p in cands:
            t = self._estimate_time(p)
            c = self._traj_cost(p)
            ranked.append((t, c, name, p))
        ranked.sort(key=lambda z: (z[0], z[1]))
        t_best, c_best, name, best = ranked[0]
        self.log_msg(f"[Seg {st['seg_index']+1}] Melhor: {name}  (tempo {t_best:.2f}s | custo {c_best:.2f})")

        if not st["full_path"]:
            st["full_path"].extend(best)
        else:
            st["full_path"].extend(best[1:])
        xb,yb,thb,_ = best[-1]
        st["seg_start_pose"] = (xb,yb,thb)

        st["attempt"] = 0
        st["seg_index"] += 1
        self.redraw(also_paths=[('blue', [(x,y) for (x,y,_,_) in st["full_path"]], 3, 0.9)])
        self._start_segment()

    # ===================== DESENHO MANUAL (pincel) =====================
    def start_draw_mode(self):
        self.mode = 'draw_free'
        self.draw_mode = True
        self.draw_active_drag = False
        self.update_status("Desenho à mão — arrasta com o rato.")
        self.log_msg("Desenho: modo ativo.")

    def stop_draw_mode(self):
        self.draw_mode = False
        if self.mode == 'draw_free': self.mode = None
        self.draw_active_drag = False
        self.update_status("Desenho parado.")
        self.log_msg("Desenho: parado.")

    def clear_draw(self):
        self.draw_points.clear()
        self.manual_path = None
        self.redraw()
        self.log_msg("Desenho: limpo.")

    def _append_draw_point(self, x, y):
        if x is None or y is None: return False
        if not self.draw_points:
            self.draw_points.append((x,y)); return True
        lx, ly = self.draw_points[-1]
        if math.hypot(x - lx, y - ly) >= float(self.draw_sampling_px):
            self.draw_points.append((x,y)); return True
        return False

    def _smooth_polyline(self, pts, iters=2):
        if len(pts) < 3: return pts[:]
        cur = pts[:]
        for _ in range(iters):
            new = [cur[0]]
            for i in range(len(cur)-1):
                p = np.array(cur[i]); q = np.array(cur[i+1])
                new.append(tuple(0.75*p + 0.25*q))
                new.append(tuple(0.25*p + 0.75*q))
            new.append(cur[-1])
            cur = new
        return cur

    def _polyline_to_path(self, pts_px):
        if len(pts_px) < 2: return None
        if bool(self.draw_smooth_on_use.get()):
            pts_px = self._smooth_polyline(pts_px, iters=2)
        out = []
        for i, (x, y) in enumerate(pts_px):
            if i == 0:
                nx, ny = pts_px[i+1]; th = math.atan2(ny - y, nx - x)
            else:
                px, py = pts_px[i-1]; th = math.atan2(y - py, x - px)
            out.append((float(x), float(y), float(th), float(self.max_speed)))
        return out

    def use_draw_as_path(self):
        if len(self.draw_points) < 2:
            messagebox.showwarning("Desenho", "Desenha primeiro (mínimo dois pontos)."); return
        path = self._polyline_to_path(self.draw_points)
        if not path:
            messagebox.showwarning("Desenho", "Não consegui converter o desenho em caminho."); return
        self.manual_path = path
        self.run_state = dict(full_path=path)
        self.redraw(also_paths=[('cyan', [(x,y) for (x,y,_,_) in path], 3, 0.95)])
        total_m = sum(self.px2m(np.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])) for i in range(1,len(path)))
        self.log_msg(f"Desenho: {len(path)} pts | comprimento ~ {total_m:.2f} m")

    def publish_drawn_path(self):
        if len(self.draw_points) < 2:
            messagebox.showwarning("Publicar desenho", "O desenho ainda não tem pontos suficientes."); return
        self.manual_path = self._polyline_to_path(self.draw_points)
        if not self.manual_path:
            messagebox.showwarning("Publicar desenho", "Falha ao converter desenho em caminho."); return
        self.publish_reference_path()

    def _draw_overlay(self):
        if self.draw_points and len(self.draw_points) > 1:
            xs = [p[0] for p in self.draw_points]
            ys = [p[1] for p in self.draw_points]
            self.ax.plot(xs, ys, linewidth=3.0, alpha=0.95, color='cyan')

    # ===================== ROS2 helpers & publish =====================
    def _yaw_to_quat(self, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)

    def _px_to_world(self, x_px, y_px):
        vx, vy = self.axis_x_vec; ux, uy = self.axis_y_vec
        Xp = vx * x_px + vy * y_px
        Yp = ux * x_px + uy * y_px
        xm = self.px2m(Xp) + float(self.origin_x_m)
        ym = self.px2m(Yp) + float(self.origin_y_m)
        return xm, ym

    def _ensure_ros2(self):
        if not HAVE_ROS2:
            messagebox.showerror("ROS2", "ROS2 não está disponível.")
            return False
        if not self.ros_inited:
            try:
                rclpy.init(args=None)
                self.ros_node = rclpy.create_node("traj_gui_publisher")
                self.ros_pub = self.ros_node.create_publisher(Path, self.ref_path_topic, 10)
                self.ros_inited = True
                self.log_msg(f"ROS2: publisher criado em '{self.ref_path_topic}'.")
            except Exception as e:
                messagebox.showerror("ROS2", f"Falha a iniciar ROS2: {e}")
                self.ros_node = None; self.ros_pub = None; self.ros_inited = False
                return False
        return True

    def _build_nav_path(self, path_pts):
        if not path_pts or len(path_pts) < 2: return None
        msg = Path()
        frame = self.frame_id_var.get().strip() or "map"
        msg.header.frame_id = frame
        if self.ros_node is not None:
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        poses = []
        for (x, y, th, _) in path_pts:
            px, py = self._px_to_world(x, y)
            qx, qy, qz, qw = self._yaw_to_quat(th)
            ps = PoseStamped()
            ps.header.frame_id = frame
            if self.ros_node is not None:
                ps.header.stamp = self.ros_node.get_clock().now().to_msg()
            ps.pose.position.x = float(px); ps.pose.position.y = float(py); ps.pose.position.z = 0.0
            ps.pose.orientation.x = qx; ps.pose.orientation.y = qy; ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
            poses.append(ps)
        msg.poses = poses
        return msg

    def _choose_best_available_path(self):
        if self.manual_path and len(self.manual_path) >= 2: return self.manual_path
        st = self.run_state or {}
        if st.get("full_path"): return st["full_path"]
        for key in ("path_mpc", "path_dwa", "path_hybrid"):
            if st.get(key): return st[key]
        return None

    def publish_reference_path(self):
        path_pts = self._choose_best_available_path()
        if not path_pts:
            messagebox.showwarning("Publicar Path", "Não há caminho para publicar ainda."); return
        self.frame_id = self.frame_id_var.get().strip() or "map"
        if bool(self.anchor_origin_start.get()):
            x0, y0, _, _ = path_pts[0]
            vx, vy = self.axis_x_vec; ux, uy = self.axis_y_vec
            Xp0 = vx * x0 + vy * y0
            Yp0 = ux * x0 + uy * y0
            self.origin_x_m = - self.px2m(Xp0)
            self.origin_y_m = - self.px2m(Yp0)
            self.offx_var.set(self.origin_x_m); self.offy_var.set(self.origin_y_m)
            self.log_msg(f"Origin @ start: offsets = ({self.origin_x_m:.3f}, {self.origin_y_m:.3f}) m")
        else:
            self.origin_x_m = float(self.offx_var.get())
            self.origin_y_m = float(self.offy_var.get())

        if not self._ensure_ros2(): return
        try:
            msg = self._build_nav_path(path_pts)
            if msg is None:
                messagebox.showwarning("Publicar Path", "Caminho inválido/curto."); return
            x0, y0, _, _ = path_pts[0]
            wx0, wy0 = self._px_to_world(x0, y0)
            self.log_msg(f"Debug origem: primeiro ponto GUI ({x0:.1f},{y0:.1f}) → mundo ({wx0:.3f},{wy0:.3f}) m")
            self.ros_pub.publish(msg)
            self.log_msg(f"ROS2: publicado {len(msg.poses)} poses em '{self.ref_path_topic}' (frame_id='{msg.header.frame_id}').")
            self.update_status(f"Publicado em {self.ref_path_topic}")
        except Exception as e:
            messagebox.showerror("Publicar Path", f"Erro ao publicar: {e}")

# ----------------------- main -----------------------
if __name__ == "__main__":
    root = tk.Tk()
    root.title("Planner: Hybrid A* + DWA + MPC + Race Line + Desenhar (pincel)")
    root.geometry("1550x950")
    app = TrajApp(master=root)

    def on_close():
        try: app.stop_all()
        except: pass
        try:
            if app._raceline_cbar is not None:
                try: app._raceline_cbar.remove()
                except: pass
                app._raceline_cbar = None
            if app._raceline_lc is not None:
                try: app._raceline_lc.remove()
                except: pass
                app._raceline_lc = None
        except: pass
        try:
            if getattr(app, "ros_inited", False) and app.ros_node is not None:
                app.ros_node.destroy_node()
                rclpy.shutdown()
        except: pass
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    app.mainloop()
