#!/usr/bin/env python3
"""
visao_odom_localizacao.py
──────────────────────────────────────────────────────────────────────────────
Localização por Visão Computacional + Odometria ROS 1 (TurtleBot3 / Noetic)

Uso:
  python3 visao_odom_localizacao.py
  q = sair | s = salvar frame | a = zerar ref | r = reiniciar traj | m = quadrado
"""

from __future__ import annotations

import os
import cv2
import numpy as np
import math
import time
import sys
import pickle
import importlib
import threading
from typing import Any, Optional, Tuple
from itertools import combinations
from collections import deque

os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

# ── Modo headless (sem display) ───────────────────────────────────────────────
# Detecta automaticamente se não há DISPLAY (SSH sem X11) ou se HEADLESS=true no .env
def _is_headless() -> bool:
    if os.getenv("HEADLESS", "").strip().lower() in ("1", "true", "yes", "y", "on"):
        return True
    display = os.getenv("DISPLAY", "").strip()
    if not display:
        return True
    return False

HEADLESS = _is_headless()

# ── ROS 1 (Noetic) ───────────────────────────────────────────────────────────
import rospy
from geometry_msgs.msg import Twist as TwistMsg
from nav_msgs.msg import Odometry

ROS2_AVAILABLE = False  # mantido para compatibilidade com variáveis existentes


def _load_env_file(path: str = ".env") -> None:
    if not os.path.exists(path):
        return
    try:
        with open(path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.split("=", 1)
                key = key.strip()
                if not key or key in os.environ:
                    continue
                os.environ[key] = value.strip().strip('"').strip("'")
    except OSError as e:
        print(f"[AVISO] Falha ao ler '{path}': {e}")


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name)
    if v is None:
        return default
    return v.strip().lower() in ("1", "true", "yes", "y", "on", "sim")


def _env_float(name: str, default: float) -> float:
    v = os.getenv(name)
    if v is None:
        return default
    try:
        return float(v)
    except ValueError:
        print(f"[AVISO] Valor inválido em {name}='{v}', usando default {default}.")
        return default


def _env_int(name: str, default: int) -> int:
    v = os.getenv(name)
    if v is None:
        return default
    try:
        return int(v)
    except ValueError:
        print(f"[AVISO] Valor inválido em {name}='{v}', usando default {default}.")
        return default


def _env_first(names: tuple[str, ...], default: str = "") -> str:
    for n in names:
        val = os.getenv(n)
        if val:
            return val
    return default


def _patch_numpy_pickle_compat() -> None:
    if "numpy._core" not in sys.modules:
        sys.modules["numpy._core"] = np.core  # type: ignore[attr-defined]
    if "numpy._core.multiarray" not in sys.modules:
        sys.modules["numpy._core.multiarray"] = np.core.multiarray  # type: ignore[attr-defined]


_load_env_file()

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                          CONFIGURAÇÕES                                  ║
# ╚══════════════════════════════════════════════════════════════════════════╝

RTSP_URL = os.getenv(
    "RTSP_URL",
    "rtsp://admin:nupedee7@192.168.1.4:554"
    "/cam/realmonitor?channel=1&subtype=1&proto=Onvif",
)

# ── RTSP / estabilidade ────────────────────────────────────────────────────
RTSP_TRANSPORT = os.getenv("RTSP_TRANSPORT", "tcp").strip().lower()
RTSP_BUFFER_FRAMES = _env_int("RTSP_BUFFER_FRAMES", 4)
RTSP_OPEN_TIMEOUT_MS = _env_int("RTSP_OPEN_TIMEOUT_MS", 7000)
RTSP_READ_TIMEOUT_MS = _env_int("RTSP_READ_TIMEOUT_MS", 7000)
RTSP_FFMPEG_BUFFER_SIZE = _env_int("RTSP_FFMPEG_BUFFER_SIZE", 1048576)
RTSP_FFMPEG_MAX_DELAY_MS = _env_int("RTSP_FFMPEG_MAX_DELAY_MS", 500)
RTSP_FFMPEG_STIMEOUT_MS = _env_int("RTSP_FFMPEG_STIMEOUT_MS", 7000)
RTSP_FFMPEG_REORDER_QUEUE_SIZE = _env_int("RTSP_FFMPEG_REORDER_QUEUE_SIZE", 0)
FFMPEG_DISCARD_CORRUPT = _env_bool("FFMPEG_DISCARD_CORRUPT", True)
FFMPEG_LOW_DELAY = _env_bool("FFMPEG_LOW_DELAY", False)

CALIBRACAO_PKL = os.getenv("CALIBRACAO_PKL", "calibracao_camera.pkl")
# HOMOGRAPHY_PKL vazio ("") = desabilitado; nao usa fallback de nome de arquivo
_hpkl_raw = os.getenv("HOMOGRAPHY_PKL", "")
HOMOGRAPHY_PKL = _hpkl_raw.strip()

LOWER_GREEN = np.array([35, 50, 50])
UPPER_GREEN = np.array([90, 255, 255])

N_POINTS = 2450
LEMNISCATA_SCALE = 80

# ── Conversão pixel → cm ─────────────────────────────────────────────────────
DIST_MARCADORES_CM = _env_float("DIST_MARCADORES_CM", 15.0)
ESCALA_EMA_ALPHA = _env_float("ESCALA_EMA_ALPHA", 0.15)

# ── Filtros de visão ────────────────────────────────────────────────────────
MAX_SALTO_VISAO_CM = _env_float("MAX_SALTO_VISAO_CM", 20.0)
MARKER_REAR_HINT = os.getenv("MARKER_REAR_HINT", "largest_area").strip().lower()
# Ajustes de eixo (caso a câmera esteja invertida/rotacionada)
VIS_FLIP_X = _env_bool("VIS_FLIP_X", False)
VIS_FLIP_Y = _env_bool("VIS_FLIP_Y", False)

# ── ROS 1 ────────────────────────────────────────────────────────────────────
ROS_ODOM_TOPIC = os.getenv("ROS_ODOM_TOPIC", os.getenv("ROS1_ODOM_TOPIC", "/odom"))
ROS_CMD_VEL_TOPIC = os.getenv("ROS_CMD_VEL_TOPIC", os.getenv("ROS1_CMD_VEL_TOPIC", "/cmd_vel"))

# ── Controle de movimento (trajetória quadrada) ─────────────────────────────
SQUARE_TRAJ_ENABLE = _env_bool("SQUARE_TRAJ_ENABLE", False)
SQUARE_TRAJ_AUTO_START = _env_bool("SQUARE_TRAJ_AUTO_START", False)
SQUARE_SIDE_CM = _env_float("SQUARE_SIDE_CM", 40.0)
SQUARE_LINEAR_MPS = _env_float("SQUARE_LINEAR_MPS", 0.10)
# SQUARE_FORWARD_S é derivado de SQUARE_SIDE_CM/SQUARE_LINEAR_MPS quando não definido no .env
SQUARE_FORWARD_S = _env_float("SQUARE_FORWARD_S", SQUARE_SIDE_CM / (SQUARE_LINEAR_MPS * 100.0))
SQUARE_LEFT_TURN_S = _env_float("SQUARE_LEFT_TURN_S", 4)
SQUARE_TURN_ANGLE_DEG = _env_float("SQUARE_TURN_ANGLE_DEG", 90.0)
SQUARE_NUM_SIDES = _env_int("SQUARE_NUM_SIDES", 4)
SQUARE_START_DELAY_S = _env_float("SQUARE_START_DELAY_S", 1.5)
# Heading-hold PID (0 = desabilita correção angular nas retas)
SQUARE_HEADING_KP = _env_float("SQUARE_HEADING_KP", 0.5)
SQUARE_HEADING_MAX_FRAC = _env_float("SQUARE_HEADING_MAX_FRAC", 0.10)
# Odometria deve estar fresca para mover (<=0 desativa a checagem)
SQUARE_ODOM_MAX_AGE_S = _env_float("SQUARE_ODOM_MAX_AGE_S", 0.5)
# Mede avanço da reta pela projeção no heading inicial
SQUARE_FORWARD_PROJ = _env_bool("SQUARE_FORWARD_PROJ", True)
# Usa visão (homografia) para encerrar retas/curvas do quadrado
SQUARE_USE_VISION = _env_bool("SQUARE_USE_VISION", False)
SQUARE_VISION_MAX_AGE_S = _env_float("SQUARE_VISION_MAX_AGE_S", 0.25)

# ── SSH ─────────────────────────────────────────────────────────────────────
SSH_ROBO_HOST = os.getenv("ROBOT_SSH_HOST", "192.168.1.3")
SSH_ROBO_USER = os.getenv("ROBOT_SSH_USER", "turtlebot")
SSH_ROBO_PASS = _env_first(("ROBOT_SSH_PASS", "TURTLEBOT_PASS"), "")

# ── Misc ─────────────────────────────────────────────────────────────────────
SHOW_MASK = _env_bool("SHOW_MASK", True)
SAVE_CSV = _env_bool("SAVE_CSV", False)
CSV_PATH = os.getenv("CSV_PATH", "pose_log.csv")
PLOT_TRAJETORIA = _env_bool("PLOT_TRAJETORIA", True)
PLOT_SHOW = _env_bool("PLOT_SHOW", False)
PLOT_SAVE_PATH = os.getenv("PLOT_SAVE_PATH", "trajetoria_xy_theta.png")
MAX_HISTORICO = _env_int("MAX_HISTORICO", 500)
AREA_MINIMA_CENTRO = _env_int("AREA_MINIMA_CENTRO", 60)
AREA_MINIMA_BORDA = _env_int("AREA_MINIMA_BORDA", 20)
AREA_MINIMA_FALLBACK = _env_int("AREA_MINIMA_FALLBACK", 12)
MAX_SALTO_RASTRO_PX = _env_int("MAX_SALTO_RASTRO_PX", 120)

# ── Limiar de RMSE para aceitar o alinhamento vis→odom ───────────────────────
# Se RMSE > threshold, usa visão em bruto (sem transformar) para evitar
# alinhamentos ruins com poucos pontos comuns.
ALIGN_RMSE_MAX_CM = _env_float("ALIGN_RMSE_MAX_CM", 15.0)
ALIGN_MIN_POINTS = _env_int("ALIGN_MIN_POINTS", 10)

# ── Fusão visão↔odom e homografia ───────────────────────────────────────────
VIS_CORRECAO_BETA = _env_float("VIS_CORRECAO_BETA", 0.25)
VIS_CORRECAO_MAX_GAP_CM = _env_float("VIS_CORRECAO_MAX_GAP_CM", 12.0)
# Rotação do referencial da câmera em relação ao mundo da odometria (graus).
# Medir no gráfico offline (campo "rot" do alinhamento vis→odom) e colocar aqui.
# Ex: rot=-12.3° → VIS_FRAME_ROT_DEG=-12.3
VIS_FRAME_ROT_DEG = _env_float("VIS_FRAME_ROT_DEG", 0.0)
# Fator de escala visão→odom (s do alinhamento). Ex: s=1.110 → VIS_FRAME_SCALE=1.110
VIS_FRAME_SCALE = _env_float("VIS_FRAME_SCALE", 1.0)
# Desativa correção de visão durante curvas (|dyaw/dt| > limiar em graus/frame)
VIS_CORRECAO_PAUSA_CURVA_DEG = _env_float("VIS_CORRECAO_PAUSA_CURVA_DEG", 5.0)
HOMOGRAPHY_MAX_REL_ERR = _env_float("HOMOGRAPHY_MAX_REL_ERR", 0.55)
# Se True, usa homografia ignorando o erro relativo máximo.
# Se False, desabilita a homografia completamente (usa px→cm).
HOMOGRAPHY_FORCE = _env_bool("HOMOGRAPHY_FORCE", False)

# ── ArUco Ground Truth (plano do chão) ─────────────────────────────────────
ARUCO_GT_ENABLE = _env_bool("ARUCO_GT_ENABLE", False)
ARUCO_DICT_NAME = os.getenv("ARUCO_DICT", "DICT_4X4_50")
ARUCO_TAG_SIZE_CM = _env_float("ARUCO_TAG_SIZE_CM", 5.0)
ARUCO_WORLD_MARKERS = os.getenv("ARUCO_WORLD_MARKERS", "").strip()
ARUCO_MIN_MARKERS = _env_int("ARUCO_MIN_MARKERS", 2)
ARUCO_RANSAC_REPROJ_ERR = _env_float("ARUCO_RANSAC_REPROJ_ERR", 3.0)

historico_traseiro: list = []
historico_dianteiro: list = []
historico_centro_img: list = []


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       ESTADO DE ODOMETRIA                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

class OdomState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.stamp = 0.0
        self.received = False

        self._ref_x = 0.0
        self._ref_y = 0.0
        self._ref_yaw = 0.0
        self.ref_set = False

        self.vis_ref_x_cm: Optional[float] = None
        self.vis_ref_y_cm: Optional[float] = None

    def update(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self.x, self.y, self.yaw = x, y, yaw
            self.stamp = time.monotonic()
            self.received = True

    def read(self) -> Tuple[float, float, float, float, bool]:
        with self._lock:
            return self.x, self.y, self.yaw, self.stamp, self.received

    def set_reference(
        self,
        vis_x_cm: Optional[float] = None,
        vis_y_cm: Optional[float] = None,
    ) -> None:
        with self._lock:
            self._ref_x = self.x
            self._ref_y = self.y
            self._ref_yaw = self.yaw
            self.ref_set = True
            # FIX: só atualiza vis_ref se a visão for válida
            if vis_x_cm is not None and np.isfinite(vis_x_cm):
                self.vis_ref_x_cm = vis_x_cm
            if vis_y_cm is not None and np.isfinite(vis_y_cm):
                self.vis_ref_y_cm = vis_y_cm

    def get_relative(self) -> Tuple[float, float, float]:
        with self._lock:
            if not self.ref_set:
                return 0.0, 0.0, 0.0
            return (
                self.x - self._ref_x,
                self.y - self._ref_y,
                _wrap(self.yaw - self._ref_yaw),
            )


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                        NÓ ROS 1 DE ODOMETRIA                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class OdomNode:
    def __init__(self, state: OdomState, topic: str = ROS_ODOM_TOPIC) -> None:
        self._state = state
        self._square_controller: Optional[SquareTrajectoryController] = None
        self._sub = rospy.Subscriber(topic, Odometry, self._cb, queue_size=10)
        self._cmd_pub = rospy.Publisher(ROS_CMD_VEL_TOPIC, TwistMsg, queue_size=10)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._motion_tick)
        try:
            rospy.loginfo(f"Subscrito a '{topic}' (ROS 1)")
        except Exception:
            print(f"[INFO] Subscrito a '{topic}' (ROS 1)")

    def set_square_controller(self, controller: Optional["SquareTrajectoryController"]) -> None:
        self._square_controller = controller

    def _cb(self, msg: Any) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._state.update(p.x, p.y, _quat_to_yaw(q.x, q.y, q.z, q.w))

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        msg: Any = TwistMsg()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        try:
            self._cmd_pub.publish(msg)
        except Exception:
            return

    def stop_robot(self) -> None:
        self.publish_cmd(0.0, 0.0)

    def _motion_tick(self, _event: Optional[Any] = None) -> None:
        controller = self._square_controller
        if controller is None:
            return
        ox, oy, oyaw, stamp, oreceived = self._state.read()
        if oreceived:
            controller.set_odom(ox, oy, oyaw, stamp)
        cmd_v, cmd_w, _phase = controller.step(time.monotonic())
        self.publish_cmd(cmd_v, cmd_w)


def _spin_ros_node() -> None:
    try:
        rospy.spin()
    except (KeyboardInterrupt, Exception):
        pass


class SquareTrajectoryController:
    """Controlador de trajetória quadrada com feedback de odometria.

    Usa distância percorrida (odom) para encerrar cada reta e ângulo girado
    (odom yaw) para encerrar cada curva — eliminando erros de tempo/velocidade.
    """

    # Tolerâncias de conclusão de fase
    DIST_TOL_CM = 2.0    # considera reta concluída dentro de ±2 cm do alvo
    ANGLE_TOL_DEG = 2.0  # considera curva concluída dentro de ±2° do alvo

    # Limites de tempo por fase (segurança caso odom falhe)
    MAX_FORWARD_FACTOR = 2.5   # espera no máximo forward_s * fator
    MAX_TURN_FACTOR = 2.5

    def __init__(
        self,
        forward_s: float,
        turn_left_s: float,
        linear_mps: float,
        turn_angle_deg: float,
        num_sides: int,
        start_delay_s: float = 0.0,
        heading_kp: float = 0.5,
        heading_max_frac: float = 0.10,
        odom_max_age_s: float = 0.5,
        use_forward_projection: bool = True,
        use_vision: bool = False,
        vision_max_age_s: float = 0.25,
        side_cm: Optional[float] = None,
    ) -> None:
        self.linear_mps = max(0.02, abs(linear_mps))
        self.turn_angle_deg = max(1.0, abs(turn_angle_deg))
        self.num_sides = max(1, int(num_sides))
        self.start_delay_s = max(0.0, start_delay_s)
        self.heading_kp = max(0.0, heading_kp)
        self.heading_max_frac = max(0.0, min(1.0, heading_max_frac))
        self.odom_max_age_s = max(0.0, odom_max_age_s)
        self.use_forward_projection = bool(use_forward_projection)
        self.use_vision = bool(use_vision)
        self.vision_max_age_s = max(0.0, vision_max_age_s)

        # distância alvo por reta em cm — side_cm tem prioridade sobre linear_mps*forward_s
        if side_cm is not None and side_cm > 0.0:
            self._target_dist_cm = float(side_cm)
            self.forward_s = max(0.1, self._target_dist_cm / (self.linear_mps * 100.0))
        else:
            self.forward_s = max(0.1, abs(forward_s))
            self._target_dist_cm = self.linear_mps * self.forward_s * 100.0

        self.turn_left_s = max(0.1, abs(turn_left_s))
        self.angular_radps = math.radians(self.turn_angle_deg) / self.turn_left_s

        self._active = False
        self._finished = False
        self._start_t = 0.0

        # estado interno por fase
        self._phase = "IDLE"      # ATRASO | FORWARD | TURN | DONE
        self._side = 0            # 0-indexed
        self._phase_start_t = 0.0
        self._phase_start_x = 0.0
        self._phase_start_y = 0.0
        self._phase_start_yaw = 0.0

        # visão (homografia) para controle do quadrado
        self._vis_lock = threading.Lock()
        self._vis_x_cm: Optional[float] = None
        self._vis_y_cm: Optional[float] = None
        self._vis_yaw = 0.0
        self._vis_ok = False
        self._vis_stamp: Optional[float] = None
        self._phase_start_vis_x: Optional[float] = None
        self._phase_start_vis_y: Optional[float] = None
        self._phase_start_vis_yaw: Optional[float] = None

        # última leitura de odometria passada pelo chamador
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_ok = False
        self._odom_stamp: Optional[float] = None

        # pausa por odometria stale
        self._pause_start_t: Optional[float] = None
        self._waiting_odom = False

    @property
    def active(self) -> bool:
        return self._active

    @property
    def phase_label(self) -> str:
        if self._finished:
            return "DONE"
        if not self._active:
            return "IDLE"
        if self._waiting_odom:
            return "WAIT_ODOM"
        if self._phase == "ATRASO":
            return "ATRASO"
        side_label = f"{self._side + 1}/{self.num_sides}"
        if self._phase == "FORWARD":
            return f"L{side_label}"
        if self._phase == "TURN":
            return f"G{side_label}"
        return self._phase

    def set_odom(self, x_m: float, y_m: float, yaw_rad: float, stamp_s: Optional[float] = None) -> None:
        """Atualiza leitura de odometria. Chamar antes de step() a cada tick."""
        self._odom_x = x_m
        self._odom_y = y_m
        self._odom_yaw = yaw_rad
        self._odom_ok = True
        self._odom_stamp = time.monotonic() if stamp_s is None else stamp_s

    def set_vision(
        self,
        x_cm: Optional[float],
        y_cm: Optional[float],
        yaw_rad: Optional[float],
        stamp_s: Optional[float] = None,
        valid: bool = True,
    ) -> None:
        now = time.monotonic() if stamp_s is None else stamp_s
        with self._vis_lock:
            if valid and x_cm is not None and y_cm is not None and np.isfinite(x_cm) and np.isfinite(y_cm):
                self._vis_x_cm = float(x_cm)
                self._vis_y_cm = float(y_cm)
                if yaw_rad is not None and np.isfinite(yaw_rad):
                    self._vis_yaw = float(yaw_rad)
                self._vis_ok = True
                self._vis_stamp = now
            else:
                self._vis_ok = False

    def _vision_fresh(self, now_s: float) -> bool:
        if not self.use_vision:
            return False
        with self._vis_lock:
            if not self._vis_ok or self._vis_stamp is None:
                return False
            return (now_s - self._vis_stamp) <= self.vision_max_age_s

    def start(self, now_s: Optional[float] = None) -> None:
        now = time.monotonic() if now_s is None else now_s
        self._active = True
        self._finished = False
        self._start_t = now
        self._phase = "ATRASO"
        self._side = 0
        self._pause_start_t = None
        self._waiting_odom = False

    def stop(self) -> None:
        self._active = False
        self._phase = "IDLE"
        self._pause_start_t = None
        self._waiting_odom = False

    def toggle(self, now_s: Optional[float] = None) -> bool:
        if self._active:
            self.stop()
            return False
        self.start(now_s)
        return True

    def _begin_forward(self, now_s: float) -> None:
        self._phase = "FORWARD"
        self._phase_start_t = now_s
        self._phase_start_x = self._odom_x
        self._phase_start_y = self._odom_y
        self._phase_start_yaw = self._odom_yaw  # referência para heading-hold
        if self._vision_fresh(now_s):
            with self._vis_lock:
                self._phase_start_vis_x = self._vis_x_cm
                self._phase_start_vis_y = self._vis_y_cm
                self._phase_start_vis_yaw = self._vis_yaw
        else:
            self._phase_start_vis_x = None
            self._phase_start_vis_y = None
            self._phase_start_vis_yaw = None

    def _begin_turn(self, now_s: float) -> None:
        self._phase = "TURN"
        self._phase_start_t = now_s
        self._phase_start_yaw = self._odom_yaw
        if self._vision_fresh(now_s):
            with self._vis_lock:
                self._phase_start_vis_yaw = self._vis_yaw
        else:
            self._phase_start_vis_yaw = None

    def step(self, now_s: float) -> Tuple[float, float, str]:
        if self._finished:
            self._waiting_odom = False
            return 0.0, 0.0, "DONE"
        if not self._active:
            self._waiting_odom = False
            return 0.0, 0.0, "IDLE"

        side_label = f"{self._side + 1}/{self.num_sides}"

        odom_fresh = self._odom_ok
        if self.odom_max_age_s > 0.0:
            odom_fresh = (
                odom_fresh
                and self._odom_stamp is not None
                and (now_s - self._odom_stamp) <= self.odom_max_age_s
            )

        if not odom_fresh:
            self._waiting_odom = True
            if self._pause_start_t is None:
                self._pause_start_t = now_s
            return 0.0, 0.0, "WAIT_ODOM"

        if self._pause_start_t is not None:
            pause_dt = now_s - self._pause_start_t
            self._start_t += pause_dt
            self._phase_start_t += pause_dt
            self._pause_start_t = None
        self._waiting_odom = False

        # ── Atraso inicial ─────────────────────────────────────────────────────
        if self._phase == "ATRASO":
            if now_s - self._start_t < self.start_delay_s:
                return 0.0, 0.0, "ATRASO"
            self._begin_forward(now_s)
            return self.linear_mps, 0.0, f"L{side_label}"

        # ── Fase de reta ───────────────────────────────────────────────────────
        if self._phase == "FORWARD":
            vis_fresh = self._vision_fresh(now_s)
            dist_cm: float
            if (
                vis_fresh
                and self._phase_start_vis_x is not None
                and self._phase_start_vis_y is not None
            ):
                with self._vis_lock:
                    vis_dx = (self._vis_x_cm or 0.0) - self._phase_start_vis_x
                    vis_dy = (self._vis_y_cm or 0.0) - self._phase_start_vis_y
                    vis_yaw = self._vis_yaw
                if self.use_forward_projection:
                    heading = self._phase_start_vis_yaw if self._phase_start_vis_yaw is not None else self._phase_start_yaw
                    forward_cm = vis_dx * math.cos(heading) + vis_dy * math.sin(heading)
                    dist_cm = max(0.0, forward_cm)
                else:
                    dist_cm = math.hypot(vis_dx, vis_dy)
            else:
                dx_m = self._odom_x - self._phase_start_x
                dy_m = self._odom_y - self._phase_start_y
                if self.use_forward_projection:
                    forward_cm = (
                        dx_m * math.cos(self._phase_start_yaw)
                        + dy_m * math.sin(self._phase_start_yaw)
                    ) * 100.0
                    dist_cm = max(0.0, forward_cm)
                else:
                    dist_cm = math.hypot(dx_m, dy_m) * 100.0
            elapsed = now_s - self._phase_start_t
            max_t = self.forward_s * self.MAX_FORWARD_FACTOR

            done = (
                (dist_cm >= self._target_dist_cm - self.DIST_TOL_CM)
                or elapsed >= max_t
            )
            if done:
                self._begin_turn(now_s)
                return 0.0, self.angular_radps, f"G{side_label}"

            # Heading-hold: corrige deriva angular durante a reta com P-controller
            if self.heading_kp > 0.0 and self.heading_max_frac > 0.0:
                if vis_fresh and self._phase_start_vis_yaw is not None:
                    with self._vis_lock:
                        heading_err = _wrap(self._phase_start_vis_yaw - self._vis_yaw)
                else:
                    heading_err = _wrap(self._phase_start_yaw - self._odom_yaw)
                max_ang = self.angular_radps * self.heading_max_frac
                ang_corr = max(-max_ang, min(max_ang, self.heading_kp * heading_err))
            else:
                ang_corr = 0.0
            return self.linear_mps, ang_corr, f"L{side_label}"

        # ── Fase de curva ──────────────────────────────────────────────────────
        if self._phase == "TURN":
            vis_fresh = self._vision_fresh(now_s)
            if vis_fresh and self._phase_start_vis_yaw is not None:
                with self._vis_lock:
                    turned_deg = math.degrees(abs(_wrap(self._vis_yaw - self._phase_start_vis_yaw)))
            else:
                turned_deg = math.degrees(abs(_wrap(self._odom_yaw - self._phase_start_yaw)))
            elapsed = now_s - self._phase_start_t
            max_t = self.turn_left_s * self.MAX_TURN_FACTOR

            done = (
                (turned_deg >= self.turn_angle_deg - self.ANGLE_TOL_DEG)
                or elapsed >= max_t
            )
            if done:
                self._side += 1
                if self._side >= self.num_sides:
                    self._active = False
                    self._finished = True
                    return 0.0, 0.0, "DONE"
                self._begin_forward(now_s)
                return self.linear_mps, 0.0, f"L{self._side + 1}/{self.num_sides}"
            return 0.0, self.angular_radps, f"G{side_label}"

        return 0.0, 0.0, "IDLE"


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                    FUNÇÕES DE COORDENADAS                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def img_to_cart(p: tuple, h: int) -> Tuple[float, float]:
    x, y = p
    return float(x), float((h - 1) - y)


def cart_to_img(p: tuple, h: int) -> Tuple[int, int]:
    x, y = p
    return int(round(x)), int(round((h - 1) - y))


def cart_to_cm(
    x_cart: float,
    y_cart: float,
    cx_ref: float,
    cy_ref: float,
    cm_per_px: float,
) -> Tuple[float, float]:
    return (x_cart - cx_ref) * cm_per_px, (y_cart - cy_ref) * cm_per_px


def _apply_vis_flip(x: float, y: float) -> Tuple[float, float]:
    if VIS_FLIP_X:
        x = -x
    if VIS_FLIP_Y:
        y = -y
    return x, y


def ema_angle(prev: float, new: float, alpha: float) -> float:
    diff = _wrap(new - prev)
    return _wrap(prev + alpha * diff)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                     HOMOGRAFIA OPCIONAL                                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def carregar_homografia(path: str) -> Optional[np.ndarray]:
    if not os.path.exists(path):
        return None
    try:
        if path.lower().endswith(".npz"):
            data = np.load(path, allow_pickle=True)
            H = None
            for key in ("H", "homography"):
                if key in data:
                    H = data[key]
                    break
            if H is None:
                print(f"[AVISO] Homografia em '{path}' sem chave H/homography.")
                return None
        else:
            _patch_numpy_pickle_compat()
            with open(path, "rb") as f:
                data = pickle.load(f)
            if isinstance(data, dict):
                H = data.get("H", data.get("homography", None))
            else:
                H = data

        H = np.asarray(H, dtype=np.float32)
        if H.shape != (3, 3):
            print(f"[AVISO] Homografia inválida em '{path}' (shape={H.shape}).")
            return None
        print(f"[INFO] Homografia carregada de '{path}'.")
        return H
    except Exception as e:
        print(f"[AVISO] Falha ao carregar homografia '{path}': {e}")
        return None


def img_pt_to_metric_cm(
    x_img: float,
    y_img: float,
    h: int,
    cx_ref: float,
    cy_ref: float,
    cm_per_px: float,
    H: Optional[np.ndarray] = None,
) -> Tuple[float, float]:
    if H is not None:
        pts = np.array([[[float(x_img), float(y_img)]]], dtype=np.float32)
        mapped = cv2.perspectiveTransform(pts, H)[0, 0]
        return float(mapped[0]), float(mapped[1])

    x_cart, y_cart = img_to_cart((x_img, y_img), h)
    return cart_to_cm(x_cart, y_cart, cx_ref, cy_ref, cm_per_px)


def _parse_aruco_world_markers(spec: str) -> dict[int, Tuple[float, float, float]]:
    """
    Formato: "id:x,y[,deg];id:x,y[,deg]" (cm, graus).
    Ex: "0:0,0;1:120,0;2:120,80;3:0,80".
    """
    if not spec:
        return {}
    out: dict[int, Tuple[float, float, float]] = {}
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk or ":" not in chunk:
            continue
        id_str, coords = chunk.split(":", 1)
        try:
            marker_id = int(id_str.strip())
        except ValueError:
            continue
        parts = [p.strip() for p in coords.split(",") if p.strip()]
        if len(parts) < 2:
            continue
        try:
            x_cm = float(parts[0])
            y_cm = float(parts[1])
            rot_deg = float(parts[2]) if len(parts) >= 3 else 0.0
        except ValueError:
            continue
        out[marker_id] = (x_cm, y_cm, rot_deg)
    return out


def _get_aruco_dict(name: str):
    if not hasattr(cv2, "aruco"):
        return None
    aruco = cv2.aruco
    if hasattr(aruco, name):
        return aruco.getPredefinedDictionary(getattr(aruco, name))
    return None


def _create_aruco_detector(aruco_dict):
    if aruco_dict is None or not hasattr(cv2, "aruco"):
        return None, None
    aruco = cv2.aruco
    try:
        params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, params)
        return detector, params
    except AttributeError:
        create_fn = getattr(aruco, "DetectorParameters_create", None)
        if create_fn is None:
            return None, None
        params = create_fn()
        return None, params


def _aruco_marker_world_corners(
    cx_cm: float,
    cy_cm: float,
    size_cm: float,
    rot_deg: float = 0.0,
) -> np.ndarray:
    half = 0.5 * size_cm
    pts = np.array(
        [
            [-half,  half],
            [ half,  half],
            [ half, -half],
            [-half, -half],
        ],
        dtype=np.float32,
    )
    if rot_deg != 0.0:
        ang = math.radians(rot_deg)
        c, s = math.cos(ang), math.sin(ang)
        R = np.array([[c, -s], [s, c]], dtype=np.float32)
        pts = (pts @ R.T).astype(np.float32)
    pts[:, 0] += float(cx_cm)
    pts[:, 1] += float(cy_cm)
    return pts


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                      CARREGAR CALIBRAÇÃO                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def carregar_calibracao(path: str):
    if not os.path.exists(path):
        print(f"[AVISO] Calibração não encontrada: '{path}' — SEM undistort.")
        return None, None

    _patch_numpy_pickle_compat()

    with open(path, "rb") as f:
        d = pickle.load(f)

    def _get(keys):
        for k in keys:
            if k in d and d[k] is not None:
                return d[k]
        return None

    mtx = _get(("camera_matrix", "mtx", "K"))
    dist = _get(("dist_coeffs", "dist", "D"))

    if mtx is None or dist is None:
        print(f"[AVISO] Chaves não reconhecidas: {list(d.keys())} — SEM undistort.")
        return None, None

    rms = d.get("rms", None)
    if rms is not None:
        print(f"[INFO] Calibração carregada de '{path}'  RMS={float(rms):.4f}")
    else:
        print(f"[INFO] Calibração carregada de '{path}'")
    return mtx, dist


def construir_mapa_undistort(mtx, dist, shape_hw):
    h, w = shape_hw
    nm, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha=0, newImgSize=(w, h))
    m1, m2 = cv2.initUndistortRectifyMap(mtx, dist, None, nm, (w, h), cv2.CV_16SC2)
    return m1, m2, nm


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                    GERAÇÃO DA LEMNISCATA                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def gerar_lemniscata(n: int, escala: int, cx: int, cy: int):
    t = np.linspace(-np.pi, np.pi, n)
    denom = 1 + np.sin(t) ** 2
    x = escala * (4 * np.cos(t) / denom) + cx
    y = escala * (4 * np.cos(t) * np.sin(t) / denom) + cy
    return x.astype(np.float32), y.astype(np.float32)


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                  DETECÇÃO DOS MARCADORES VERDES                         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

CIRCULARITY_MIN = 0.30

def detectar_candidatos(mask: np.ndarray, area_minima: int = 200):
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidatos = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area <= area_minima:
            continue
        perimeter = cv2.arcLength(c, True)
        if perimeter < 1.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < CIRCULARITY_MIN:
            continue
        M = cv2.moments(c)
        if M["m00"] == 0:
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        candidatos.append(
            {"contour": c, "centroid": (cx, cy), "area": float(area), "circularity": circularity}
        )
    candidatos.sort(key=lambda d: d["area"], reverse=True)
    return candidatos


def _dist_px(p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
    return math.hypot(float(p1[0] - p2[0]), float(p1[1] - p2[1]))


def escolher_marcadores(
    candidatos: list[dict],
    prev_rear: Optional[Tuple[int, int]],
    prev_front: Optional[Tuple[int, int]],
) -> Optional[Tuple[dict, dict]]:
    if len(candidatos) < 2:
        return None

    pool = candidatos[:6]
    melhor = None
    melhor_score = float("inf")

    for i, j in combinations(range(len(pool)), 2):
        a = pool[i]
        b = pool[j]

        if prev_rear is not None and prev_front is not None:
            score_ab = _dist_px(a["centroid"], prev_rear) + _dist_px(b["centroid"], prev_front)
            score_ba = _dist_px(b["centroid"], prev_rear) + _dist_px(a["centroid"], prev_front)
            if score_ba < score_ab:
                rear, front = b, a
                score = score_ba
            else:
                rear, front = a, b
                score = score_ab
        else:
            if MARKER_REAR_HINT == "lower_y":
                rear, front = (a, b) if a["centroid"][1] >= b["centroid"][1] else (b, a)
            elif MARKER_REAR_HINT == "upper_y":
                rear, front = (a, b) if a["centroid"][1] <= b["centroid"][1] else (b, a)
            else:
                rear, front = (a, b) if a["area"] >= b["area"] else (b, a)
            score = -_dist_px(rear["centroid"], front["centroid"])

        if score < melhor_score:
            melhor = (rear, front)
            melhor_score = score

    return melhor


def area_minima_dinamica(x: int, largura: int) -> int:
    if largura <= 1:
        return AREA_MINIMA_CENTRO
    xc = max(0, min(int(x), largura - 1))
    d = abs((xc / (largura - 1)) - 0.5) * 2.0
    return int(round(AREA_MINIMA_CENTRO - (AREA_MINIMA_CENTRO - AREA_MINIMA_BORDA) * d))


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                       ABERTURA DO STREAM RTSP                           ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _build_ffmpeg_capture_options() -> Optional[str]:
    existing = os.getenv("OPENCV_FFMPEG_CAPTURE_OPTIONS")
    if existing:
        return existing

    opts = []
    transport = RTSP_TRANSPORT.strip().lower()
    if transport and transport not in ("auto", "default", "none"):
        opts.append(f"rtsp_transport;{transport}")
    if RTSP_FFMPEG_BUFFER_SIZE > 0:
        opts.append(f"buffer_size;{RTSP_FFMPEG_BUFFER_SIZE}")
    if RTSP_FFMPEG_MAX_DELAY_MS > 0:
        opts.append(f"max_delay;{int(RTSP_FFMPEG_MAX_DELAY_MS * 1000)}")
    if RTSP_FFMPEG_STIMEOUT_MS > 0:
        opts.append(f"stimeout;{int(RTSP_FFMPEG_STIMEOUT_MS * 1000)}")
    if RTSP_FFMPEG_REORDER_QUEUE_SIZE > 0:
        opts.append(f"reorder_queue_size;{RTSP_FFMPEG_REORDER_QUEUE_SIZE}")
    if FFMPEG_DISCARD_CORRUPT:
        opts.append("fflags;discardcorrupt")
    if FFMPEG_LOW_DELAY:
        opts.append("flags;low_delay")

    if not opts:
        return None

    opt_str = "|".join(opts)
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = opt_str
    return opt_str

def abrir_rtsp(url: str) -> cv2.VideoCapture:
    ffmpeg_opts = _build_ffmpeg_capture_options()
    if ffmpeg_opts:
        print(f"[INFO] RTSP/FFmpeg options: {ffmpeg_opts}")

    cap = cv2.VideoCapture()
    if RTSP_OPEN_TIMEOUT_MS > 0 and hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, RTSP_OPEN_TIMEOUT_MS)
    if RTSP_READ_TIMEOUT_MS > 0 and hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, RTSP_READ_TIMEOUT_MS)

    cap.open(url, cv2.CAP_FFMPEG)
    if RTSP_BUFFER_FRAMES > 0:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, RTSP_BUFFER_FRAMES)
    if not cap.isOpened():
        print(f"[ERRO] Não foi possível abrir: {url}")
        sys.exit(1)
    return cap


def _estimate_similarity_transform(src: np.ndarray, dst: np.ndarray) -> Tuple[float, np.ndarray, np.ndarray, float]:
    if src.shape[0] < 3 or dst.shape[0] < 3:
        return 1.0, np.eye(2), np.zeros(2), float("inf")

    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)

    src_c = src - src_mean
    dst_c = dst - dst_mean

    var_src = float(np.sum(src_c**2))
    if var_src < 1e-12:
        return 1.0, np.eye(2), dst_mean - src_mean, float("inf")

    H = src_c.T @ dst_c
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    scale = float(np.sum(S) / var_src)
    t = dst_mean - scale * (R @ src_mean)

    pred = (scale * (src @ R.T)) + t
    rmse = float(np.sqrt(np.mean(np.sum((pred - dst) ** 2, axis=1))))
    return scale, R, t, rmse


def _vis_to_world_frame(dx_cm: float, dy_cm: float) -> Tuple[float, float]:
    """Converte deslocamento de visão (frame câmera) para frame do mundo (odom).

    Aplica rotação VIS_FRAME_ROT_DEG e divisão por VIS_FRAME_SCALE.
    Calibrar com os valores do gráfico de alinhamento vis→odom (rot, s).
    """
    if VIS_FRAME_ROT_DEG == 0.0 and VIS_FRAME_SCALE == 1.0:
        return dx_cm, dy_cm
    rot = math.radians(VIS_FRAME_ROT_DEG)
    c, s = math.cos(rot), math.sin(rot)
    # Rotaciona e corrige escala
    scale = VIS_FRAME_SCALE if VIS_FRAME_SCALE > 0 else 1.0
    wx = (c * dx_cm - s * dy_cm) / scale
    wy = (s * dx_cm + c * dy_cm) / scale
    return wx, wy


def _apply_similarity_transform(points: np.ndarray, scale: float, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (scale * (points @ R.T)) + t


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║               GROUND TRUTH DA TRAJETÓRIA QUADRADA                       ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def gerar_ground_truth_quadrado(
    forward_s: float,
    turn_s: float,
    linear_mps: float,
    turn_angle_deg: float,
    num_sides: int,
    start_delay_s: float,
    t_ref_rel: float,
    sq_start_t_rel: float,
    odom_ref_x_cm: float,
    odom_ref_y_cm: float,
    odom_ref_yaw_deg: float,
    t_total: float,
    dt: float = 0.05,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Simula a trajetória teórica do controlador de quadrado.

    Retorna arrays (t_gt, x_gt_cm, y_gt_cm) no referencial centrado em 'a'.

    Parâmetros:
      t_ref_rel      : timestamp relativo (desde t0) em que 'a' foi pressionado.
      sq_start_t_rel : timestamp relativo em que o controlador foi iniciado.
      odom_ref_{x,y}_cm : posição da odometria quando 'a' foi pressionado (cm).
      odom_ref_yaw_deg  : yaw da odometria quando 'a' foi pressionado (graus).
      t_total        : duração total da gravação (s) — para extender além do quadrado.
    """
    angular_radps = math.radians(turn_angle_deg) / turn_s
    cycle = forward_s + turn_s
    total_sq = num_sides * cycle + start_delay_s

    # Simula desde sq_start até t_total
    t_end = max(t_total, total_sq + 2.0)
    ts = np.arange(0.0, t_end, dt)

    # Integra posição/yaw da pose inicial do controlador em coordenadas do mundo
    # (depois alinhamos ao referencial 'a')
    x_sim = [0.0]
    y_sim = [0.0]
    yaw_sim = [0.0]  # heading inicial = 0 (alinhamos depois)

    for i in range(1, len(ts)):
        elapsed = ts[i]
        if elapsed < start_delay_s:
            v, w = 0.0, 0.0
        else:
            eff = elapsed - start_delay_s
            if eff >= num_sides * cycle:
                v, w = 0.0, 0.0
            else:
                side_idx = int(eff // cycle)
                in_cycle = eff - side_idx * cycle
                if in_cycle < forward_s:
                    v, w = linear_mps * 100.0, 0.0  # cm/s
                else:
                    v, w = 0.0, angular_radps

        new_yaw = yaw_sim[-1] + w * dt
        new_x = x_sim[-1] + v * math.cos(yaw_sim[-1]) * dt
        new_y = y_sim[-1] + v * math.sin(yaw_sim[-1]) * dt
        x_sim.append(new_x)
        y_sim.append(new_y)
        yaw_sim.append(new_yaw)

    x_sim_arr = np.array(x_sim)
    y_sim_arr = np.array(y_sim)

    # Tempo absoluto relativo ao início da gravação
    t_abs = ts + sq_start_t_rel  # timestamps relativos à gravação

    # Encontra estado do simulador no momento em que 'a' foi pressionado
    t_diff_from_sq = t_ref_rel - sq_start_t_rel
    if t_diff_from_sq < 0:
        t_diff_from_sq = 0.0
    ref_idx_sim = int(round(t_diff_from_sq / dt))
    ref_idx_sim = max(0, min(ref_idx_sim, len(x_sim_arr) - 1))

    # Translada para que a posição no instante 'a' seja (0,0) no referencial do gráfico
    ref_x_sim = x_sim_arr[ref_idx_sim]
    ref_y_sim = y_sim_arr[ref_idx_sim]

    # Rotaciona pela diferença entre o yaw teórico e o yaw real da odometria em 'a'
    yaw_ref_sim = yaw_sim[ref_idx_sim]
    yaw_ref_real = math.radians(odom_ref_yaw_deg)
    rot_angle = yaw_ref_real - yaw_ref_sim

    cos_r = math.cos(rot_angle)
    sin_r = math.sin(rot_angle)

    dx = x_sim_arr - ref_x_sim
    dy = y_sim_arr - ref_y_sim

    x_rot = cos_r * dx - sin_r * dy
    y_rot = sin_r * dx + cos_r * dy

    # Aplica offset da odometria real em 'a' (que já está centrada em 0 no gráfico)
    # odom_ref já é (0,0) pois o gráfico é centrado em 'a'

    return t_abs, x_rot, y_rot


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                        GERAÇÃO DE GRÁFICOS                              ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def gerar_graficos_trajetoria(
    t_s: list[float],
    vis_dx_cm: list[float],
    vis_dy_cm: list[float],
    vis_theta_deg: list[float],
    odom_dx_cm: list[float],
    odom_dy_cm: list[float],
    odom_dyaw_deg: list[float],
    gamma_deg: list[float],
    vis_abs_x_cm: list[float],
    vis_abs_y_cm: list[float],
    odom_abs_x_cm: list[float],
    odom_abs_y_cm: list[float],
    save_path: str,
    vis_raw_abs_x_cm: Optional[list[float]] = None,
    vis_raw_abs_y_cm: Optional[list[float]] = None,
    show_plot: bool = False,
    ref_t_s: Optional[float] = None,
    # Ref odom yaw (para info)
    odom_ref_yaw_deg: Optional[float] = None,
    # Ground truth (ArUco)
    gt_aruco_t_s: Optional[np.ndarray] = None,
    gt_aruco_x_cm: Optional[np.ndarray] = None,
    gt_aruco_y_cm: Optional[np.ndarray] = None,
    # Ground truth teórico (controlador)
    gt_teo_t_s: Optional[np.ndarray] = None,
    gt_teo_x_cm: Optional[np.ndarray] = None,
    gt_teo_y_cm: Optional[np.ndarray] = None,
) -> None:
    if len(t_s) < 2:
        print("[AVISO] Pontos insuficientes para gerar gráfico de trajetória.")
        return

    try:
        matplotlib = importlib.import_module("matplotlib")
        if not show_plot:
            matplotlib.use("Agg")
        plt = importlib.import_module("matplotlib.pyplot")
    except Exception as e:
        print(f"[AVISO] Matplotlib indisponível, gráfico não gerado: {e}")
        return

    t = np.asarray(t_s, dtype=float)

    vdx_rel = np.asarray(vis_dx_cm, dtype=float)
    vdy_rel = np.asarray(vis_dy_cm, dtype=float)
    vt = np.asarray(vis_theta_deg, dtype=float)

    odx_rel = np.asarray(odom_dx_cm, dtype=float)
    ody_rel = np.asarray(odom_dy_cm, dtype=float)
    oyaw = np.asarray(odom_dyaw_deg, dtype=float)
    g = np.asarray(gamma_deg, dtype=float)

    gt_teo_mask_t: Optional[np.ndarray] = None
    gt_teo_t_plot: Optional[np.ndarray] = None
    if gt_teo_t_s is not None and ref_t_s is not None:
        gt_teo_mask_t = (gt_teo_t_s >= (ref_t_s - 1.0))
        if np.any(gt_teo_mask_t):
            gt_teo_t_plot = gt_teo_t_s[gt_teo_mask_t]

    gt_aruco_t: Optional[np.ndarray] = None
    gt_aruco_x: Optional[np.ndarray] = None
    gt_aruco_y: Optional[np.ndarray] = None
    if gt_aruco_x_cm is not None and gt_aruco_y_cm is not None:
        gt_aruco_x = np.asarray(gt_aruco_x_cm, dtype=float)
        gt_aruco_y = np.asarray(gt_aruco_y_cm, dtype=float)
        gt_aruco_t = np.asarray(gt_aruco_t_s, dtype=float) if gt_aruco_t_s is not None else t
        if gt_aruco_t.shape != gt_aruco_x.shape or gt_aruco_t.shape != gt_aruco_y.shape:
            gt_aruco_t = None
            gt_aruco_x = None
            gt_aruco_y = None

    vis_abs = np.column_stack([np.asarray(vis_abs_x_cm, dtype=float), np.asarray(vis_abs_y_cm, dtype=float)])
    vis_raw_abs = None
    if vis_raw_abs_x_cm is not None and vis_raw_abs_y_cm is not None:
        vis_raw_abs = np.column_stack([
            np.asarray(vis_raw_abs_x_cm, dtype=float),
            np.asarray(vis_raw_abs_y_cm, dtype=float),
        ])
    odom_abs = np.column_stack([np.asarray(odom_abs_x_cm, dtype=float), np.asarray(odom_abs_y_cm, dtype=float)])

    vis_align = vis_raw_abs if vis_raw_abs is not None else vis_abs
    finite_both = (
        np.isfinite(vis_align[:, 0]) & np.isfinite(vis_align[:, 1]) &
        np.isfinite(odom_abs[:, 0]) & np.isfinite(odom_abs[:, 1])
    )
    n_valid = int(np.count_nonzero(finite_both))

    vis_abs_aligned = vis_abs.copy()
    vis_raw_abs_aligned = vis_raw_abs.copy() if vis_raw_abs is not None else None
    transform_info = None

    # ── Alinhamento vis→odom apenas com pontos suficientes e RMSE aceitável ──
    if n_valid >= ALIGN_MIN_POINTS:
        s, R, tv, rmse = _estimate_similarity_transform(vis_align[finite_both], odom_abs[finite_both])
        rot_deg = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        if rmse <= ALIGN_RMSE_MAX_CM:
            vis_abs_aligned = _apply_similarity_transform(vis_abs, s, R, tv)
            if vis_raw_abs_aligned is not None:
                vis_raw_abs_aligned = _apply_similarity_transform(vis_raw_abs_aligned, s, R, tv)
            transform_info = (s, rot_deg, tv[0], tv[1], rmse)
            print(
                f"[INFO] Alinhamento vis→odom aceito: s={s:.3f}, rot={rot_deg:+.1f}°, "
                f"tx={tv[0]:+.1f} cm, ty={tv[1]:+.1f} cm, rmse={rmse:.2f} cm"
            )
        else:
            print(
                f"[AVISO] Alinhamento rejeitado (RMSE={rmse:.2f} cm > {ALIGN_RMSE_MAX_CM:.1f} cm threshold) — "
                f"usando visão sem transformar. Pontos válidos: {n_valid}."
            )
            print(
                f"[INFO] Diagnóstico vis→odom: s={s:.3f}, rot={rot_deg:+.1f}°, "
                f"tx={tv[0]:+.1f} cm, ty={tv[1]:+.1f} cm"
            )
            print(
                "[DICA] Se a visão estiver rotacionada/escala diferente, experimente ajustar "
                "VIS_FRAME_ROT_DEG e VIS_FRAME_SCALE com base nos valores acima."
            )
    else:
        print(
            f"[AVISO] Poucos pontos vis/odom coincidentes ({n_valid} < {ALIGN_MIN_POINTS}) — "
            "sem alinhamento. Verifique detecção dos marcadores."
        )

    # ── Centra trajetórias na referência 'a' ─────────────────────────────────
    vis_abs_plot = vis_abs_aligned.copy()
    vis_raw_abs_plot = vis_raw_abs_aligned.copy() if vis_raw_abs_aligned is not None else None
    odom_abs_plot = odom_abs.copy()

    ref_idx: Optional[int] = None
    if ref_t_s is not None and np.isfinite(ref_t_s) and len(t) > 0:
        ref_idx = int(np.argmin(np.abs(t - ref_t_s)))
        if 0 <= ref_idx < len(t):
            if np.isfinite(vis_abs_plot[ref_idx, 0]) and np.isfinite(vis_abs_plot[ref_idx, 1]):
                vis_abs_plot -= vis_abs_plot[ref_idx]
            if vis_raw_abs_plot is not None:
                if np.isfinite(vis_raw_abs_plot[ref_idx, 0]) and np.isfinite(vis_raw_abs_plot[ref_idx, 1]):
                    vis_raw_abs_plot -= vis_raw_abs_plot[ref_idx]
            if np.isfinite(odom_abs_plot[ref_idx, 0]) and np.isfinite(odom_abs_plot[ref_idx, 1]):
                odom_abs_plot -= odom_abs_plot[ref_idx]

    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle("Localização: Visão vs Odometria vs Ground Truth", fontsize=12, fontweight="bold")

    # ── Linha vertical da referência 'a' ─────────────────────────────────────
    if ref_t_s is not None and np.isfinite(ref_t_s):
        for ax in (axs[0, 0], axs[0, 1], axs[1, 0]):
            ax.axvline(ref_t_s, color="gray", linestyle="--", lw=1.0, alpha=0.85)
        axs[0, 0].annotate(
            "ref 'a'",
            xy=(ref_t_s, 0.98),
            xycoords=("data", "axes fraction"),
            xytext=(4, 0),
            textcoords="offset points",
            fontsize=8,
            color="gray",
            rotation=90,
            va="top",
        )

    # ── Δx(t) ────────────────────────────────────────────────────────────────
    # Use the fused estimate (vis+odom) as the primary vision series — it
    # correctly propagates position when raw markers aren't detected (HOLD/NONE).
    # Raw vision is shown as a faint dotted line when it actually moves.
    fused_x = vis_abs[:, 0].copy()
    fused_y = vis_abs[:, 1].copy()
    if ref_idx is not None and 0 <= ref_idx < len(fused_x) and np.isfinite(fused_x[ref_idx]):
        fused_x -= fused_x[ref_idx]
        fused_y -= fused_y[ref_idx]

    axs[0, 0].plot(t, fused_x, color="tab:red", lw=1.5, label="vis+odom Δx (cm)", zorder=3)
    if np.any(np.isfinite(vdx_rel) & (vdx_rel != 0.0)):
        axs[0, 0].plot(t, vdx_rel, color="tab:red", lw=0.8, linestyle=":",
                       alpha=0.5, label="vis bruta Δx (cm)", zorder=2)
    axs[0, 0].plot(t, odx_rel, color="tab:orange", lw=1.2, label="odom Δx (cm)", zorder=2)
    if gt_aruco_x is not None and gt_aruco_t is not None:
        gt_ax = gt_aruco_x.copy()
        if ref_idx is not None and 0 <= ref_idx < len(gt_ax) and np.isfinite(gt_ax[ref_idx]):
            gt_ax -= gt_ax[ref_idx]
        gt_ok = np.isfinite(gt_ax) & np.isfinite(gt_aruco_t)
        if np.any(gt_ok):
            axs[0, 0].plot(
                gt_aruco_t[gt_ok], gt_ax[gt_ok],
                color="tab:green", lw=1.4,
                label="GT ArUco Δx (cm)", zorder=1,
            )
    if gt_teo_x_cm is not None and gt_teo_mask_t is not None and gt_teo_t_plot is not None and np.any(gt_teo_mask_t):
        gt_dx = gt_teo_x_cm[gt_teo_mask_t] - gt_teo_x_cm[gt_teo_mask_t][0]
        axs[0, 0].plot(
            gt_teo_t_plot, gt_dx,
            color="tab:green", lw=1.1, linestyle="--",
            label="GT teórico Δx (cm)", zorder=1,
        )
    axs[0, 0].set_title("Δx(t)")
    axs[0, 0].set_xlabel("tempo (s)")
    axs[0, 0].set_ylabel("cm")
    axs[0, 0].grid(True, alpha=0.3)
    axs[0, 0].legend(loc="best", fontsize=8)

    # ── Δy(t) ────────────────────────────────────────────────────────────────
    axs[0, 1].plot(t, fused_y, color="tab:blue", lw=1.5, label="vis+odom Δy (cm)", zorder=3)
    if np.any(np.isfinite(vdy_rel) & (vdy_rel != 0.0)):
        axs[0, 1].plot(t, vdy_rel, color="tab:blue", lw=0.8, linestyle=":",
                       alpha=0.5, label="vis bruta Δy (cm)", zorder=2)
    axs[0, 1].plot(t, ody_rel, color="tab:cyan", lw=1.2, label="odom Δy (cm)", zorder=2)
    if gt_aruco_y is not None and gt_aruco_t is not None:
        gt_ay = gt_aruco_y.copy()
        if ref_idx is not None and 0 <= ref_idx < len(gt_ay) and np.isfinite(gt_ay[ref_idx]):
            gt_ay -= gt_ay[ref_idx]
        gt_ok = np.isfinite(gt_ay) & np.isfinite(gt_aruco_t)
        if np.any(gt_ok):
            axs[0, 1].plot(
                gt_aruco_t[gt_ok], gt_ay[gt_ok],
                color="tab:green", lw=1.4,
                label="GT ArUco Δy (cm)", zorder=1,
            )
    if gt_teo_y_cm is not None and gt_teo_mask_t is not None and gt_teo_t_plot is not None and np.any(gt_teo_mask_t):
        gt_dy = gt_teo_y_cm[gt_teo_mask_t] - gt_teo_y_cm[gt_teo_mask_t][0]
        axs[0, 1].plot(
            gt_teo_t_plot, gt_dy,
            color="tab:green", lw=1.1, linestyle="--",
            label="GT teórico Δy (cm)", zorder=1,
        )
    axs[0, 1].set_title("Δy(t)")
    axs[0, 1].set_xlabel("tempo (s)")
    axs[0, 1].set_ylabel("cm")
    axs[0, 1].grid(True, alpha=0.3)
    axs[0, 1].legend(loc="best", fontsize=8)

    # ── θ(t) / Δyaw(t) / γ(t) ───────────────────────────────────────────────
    axs[1, 0].plot(t, vt, color="tab:green", lw=1.5, label="theta visão (°)", zorder=3)
    axs[1, 0].plot(t, oyaw, color="tab:brown", lw=1.2, label="Δyaw odom (°)", zorder=2)
    axs[1, 0].plot(t, g, color="tab:purple", lw=1.0, linestyle=":", label="gamma (°)", zorder=1)
    axs[1, 0].set_title("θ(t), Δyaw(t), γ(t)")
    axs[1, 0].set_xlabel("tempo (s)")
    axs[1, 0].set_ylabel("graus")
    axs[1, 0].grid(True, alpha=0.3)
    axs[1, 0].legend(loc="best", fontsize=8)

    # ── XY absoluto ──────────────────────────────────────────────────────────
    ax_xy = axs[1, 1]

    vis_ok = np.isfinite(vis_abs_plot[:, 0]) & np.isfinite(vis_abs_plot[:, 1])
    odom_ok = np.isfinite(odom_abs_plot[:, 0]) & np.isfinite(odom_abs_plot[:, 1])

    if np.any(vis_ok):
        ax_xy.plot(
            vis_abs_plot[vis_ok, 0], vis_abs_plot[vis_ok, 1],
            color="tab:red", lw=1.8, label="visão", zorder=3,
        )

    if np.any(odom_ok):
        ax_xy.plot(
            odom_abs_plot[odom_ok, 0], odom_abs_plot[odom_ok, 1],
            color="tab:orange", lw=1.5, label="odometria", zorder=2,
        )

    # Ground truth ArUco no XY
    if gt_aruco_x is not None and gt_aruco_y is not None and gt_aruco_t is not None:
        gt_ax = gt_aruco_x.copy()
        gt_ay = gt_aruco_y.copy()
        if ref_idx is not None and 0 <= ref_idx < len(gt_ax):
            if np.isfinite(gt_ax[ref_idx]) and np.isfinite(gt_ay[ref_idx]):
                gt_ax -= gt_ax[ref_idx]
                gt_ay -= gt_ay[ref_idx]
        gt_ok = np.isfinite(gt_ax) & np.isfinite(gt_ay)
        if np.any(gt_ok):
            ax_xy.plot(
                gt_ax[gt_ok], gt_ay[gt_ok],
                color="tab:green", lw=2.0, label="GT ArUco", zorder=1,
            )

    # Ground truth teórico no XY
    if gt_teo_t_s is not None and gt_teo_x_cm is not None and gt_teo_y_cm is not None:
        gt_mask_xy = (gt_teo_t_s >= 0) & (gt_teo_t_s <= (t[-1] + 2.0))
        if np.any(gt_mask_xy):
            ax_xy.plot(
                gt_teo_x_cm[gt_mask_xy], gt_teo_y_cm[gt_mask_xy],
                color="tab:green", lw=1.4, linestyle="--",
                label="GT teórico", zorder=1,
            )

    # Marca a referência 'a' como origem
    if ref_idx is not None and 0 <= ref_idx < len(t):
        ax_xy.scatter([0.0], [0.0], marker="*", s=200, color="black", zorder=8, label="ref 'a'")
        ax_xy.annotate("ref 'a'", xy=(0.0, 0.0), xytext=(6, 6),
                       textcoords="offset points", fontsize=8, color="black")

    ax_xy.set_title("Trajetória XY  [centrada na referência 'a']")
    ax_xy.set_xlabel("Δx (cm)")
    ax_xy.set_ylabel("Δy (cm)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")
    ax_xy.axhline(0, color="gray", lw=0.5)
    ax_xy.axvline(0, color="gray", lw=0.5)
    ax_xy.legend(loc="best", fontsize=8)

    # ── Info box ─────────────────────────────────────────────────────────────
    align_txt = (
        f"Alinhamento vis→odom: {transform_info[4]:.1f} cm RMSE, rot={transform_info[1]:+.1f}°, s={transform_info[0]:.3f}"
        if transform_info
        else f"Sem alinhamento (pontos válidos={n_valid})"
    )
    gt_notes = []
    if gt_aruco_x is not None and gt_aruco_y is not None:
        gt_notes.append("GT ArUco: homografia dos marcadores no chão")
    if gt_teo_x_cm is not None and gt_teo_y_cm is not None:
        gt_notes.append("GT teórico: integração do controlador quadrado")
    gt_note_txt = "  |  ".join(gt_notes) if gt_notes else "GT: indisponível"
    note = f"{align_txt}  |  {gt_note_txt}"
    fig.text(0.5, 0.01, note, ha="center", fontsize=7, color="gray")
    fig.tight_layout(rect=[0, 0.04, 1, 0.97])

    out_dir = os.path.dirname(save_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(save_path, dpi=150)
    print(f"[INFO] Gráfico salvo em: {save_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)


def _trim(lst: list) -> None:
    while len(lst) > MAX_HISTORICO:
        lst.pop(0)


def _draw_trail(img: np.ndarray, historico: list, bgr_channel: int) -> None:
    n = len(historico)
    for i in range(1, n):
        p0, p1 = historico[i - 1], historico[i]
        if p0 is None or p1 is None:
            continue
        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > MAX_SALTO_RASTRO_PX:
            continue
        alpha = i / n
        intensity = int(80 + 175 * alpha)
        color = [0, 0, 0]
        color[bgr_channel] = intensity
        cv2.line(img, p0, p1, tuple(color), 1)


def _draw_center_trail(img: np.ndarray, historico: list[Optional[Tuple[int, int]]]) -> None:
    n = len(historico)
    for i in range(1, n):
        p0, p1 = historico[i - 1], historico[i]
        if p0 is None or p1 is None:
            continue
        if math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > MAX_SALTO_RASTRO_PX:
            continue
        alpha = i / n
        intensity = int(60 + 195 * alpha)
        cv2.line(img, p0, p1, (0, intensity, 255), 2)


def main() -> None:
    print(f"[INFO] Execução no notebook | robô SSH alvo: {SSH_ROBO_USER}@{SSH_ROBO_HOST}")
    if not SSH_ROBO_PASS:
        print("[AVISO] Senha SSH não definida em .env (ROBOT_SSH_PASS/TURTLEBOT_PASS).")

    # ── Calibração / Homografia ────────────────────────────────────────────
    mtx, dist = carregar_calibracao(CALIBRACAO_PKL)
    usar_cal = mtx is not None
    H = carregar_homografia(HOMOGRAPHY_PKL)

    # ── ArUco GT ───────────────────────────────────────────────────────────
    aruco_world = _parse_aruco_world_markers(ARUCO_WORLD_MARKERS)
    aruco_gt_enabled = ARUCO_GT_ENABLE
    aruco_dict = None
    aruco_detector = None
    aruco_params = None
    aruco_H: Optional[np.ndarray] = None

    if aruco_gt_enabled:
        if not hasattr(cv2, "aruco"):
            print("[AVISO] OpenCV sem módulo aruco. Instale opencv-contrib-python para usar GT ArUco.")
            aruco_gt_enabled = False
        elif ARUCO_TAG_SIZE_CM <= 0.0:
            print("[AVISO] ARUCO_TAG_SIZE_CM inválido (<=0) — GT ArUco desativado.")
            aruco_gt_enabled = False
        else:
            aruco_dict = _get_aruco_dict(ARUCO_DICT_NAME)
            if aruco_dict is None:
                print(f"[AVISO] Dicionário ArUco inválido: {ARUCO_DICT_NAME}")
                aruco_gt_enabled = False
            elif not aruco_world:
                print("[AVISO] ARUCO_WORLD_MARKERS vazio — GT ArUco desativado.")
                aruco_gt_enabled = False
            else:
                aruco_detector, aruco_params = _create_aruco_detector(aruco_dict)
                print(
                    f"[INFO] ArUco GT habilitado: dict={ARUCO_DICT_NAME}, size={ARUCO_TAG_SIZE_CM:.1f}cm, "
                    f"markers={len(aruco_world)}"
                )

    # ── ROS 1 / Odometria ────────────────────────────────────────────────────
    odom_state = OdomState()
    odom_node = None
    ros_thread = None
    square_controller: Optional[SquareTrajectoryController] = None
    sq_status = "SQ:OFF"
    sq_start_t_rel: Optional[float] = None   # ← timestamp relativo ao t0 do início do controlador

    rospy.init_node("visao_odom_listener", anonymous=True)
    odom_node = OdomNode(odom_state)
    ros_thread = threading.Thread(
        target=_spin_ros_node, daemon=True, name="ros1_spin"
    )
    ros_thread.start()
    print("[INFO] Thread ROS 1 iniciada.")

    if SQUARE_TRAJ_ENABLE:
        square_controller = SquareTrajectoryController(
            forward_s=SQUARE_FORWARD_S,
            turn_left_s=SQUARE_LEFT_TURN_S,
            linear_mps=SQUARE_LINEAR_MPS,
            turn_angle_deg=SQUARE_TURN_ANGLE_DEG,
            num_sides=SQUARE_NUM_SIDES,
            start_delay_s=SQUARE_START_DELAY_S,
            heading_kp=SQUARE_HEADING_KP,
            heading_max_frac=SQUARE_HEADING_MAX_FRAC,
            odom_max_age_s=SQUARE_ODOM_MAX_AGE_S,
            use_forward_projection=SQUARE_FORWARD_PROJ,
            use_vision=SQUARE_USE_VISION,
            vision_max_age_s=SQUARE_VISION_MAX_AGE_S,
            side_cm=SQUARE_SIDE_CM,
        )
        if not SQUARE_TRAJ_AUTO_START:
            print("[INFO] Trajetória quadrada habilitada (tecla 'm' para iniciar/parar).")
        odom_node.set_square_controller(square_controller)

    # ── Câmera ───────────────────────────────────────────────────────────────
    print("[INFO] Conectando ao stream RTSP...")
    cap = abrir_rtsp(RTSP_URL)

    for _ in range(15):
        cap.grab()
    cap.grab()
    ret, frame0 = cap.retrieve()
    if not ret:
        print("[ERRO] Frame inicial inválido.")
        cap.release()
        sys.exit(1)

    h, w = frame0.shape[:2]
    print(f"[INFO] Resolução: {w}×{h}")

    t0 = time.monotonic()

    if square_controller is not None and SQUARE_TRAJ_AUTO_START:
        now = time.monotonic()
        square_controller.start(now)
        sq_start_t_rel = now - t0
        print(
            f"[INFO] Trajetória quadrada AUTO iniciada "
            f"(frente={SQUARE_FORWARD_S:.1f}s, esquerda={SQUARE_LEFT_TURN_S:.1f}s, "
            f"lados={SQUARE_NUM_SIDES}, v={SQUARE_LINEAR_MPS:.2f}m/s, "
            f"ang={SQUARE_TURN_ANGLE_DEG:.1f}°)."
        )

    # Centro da imagem em coordenadas cartesianas
    cx_cart_ref = float(w) / 2.0
    cy_cart_ref = float(h - 1) / 2.0

    # ── Mapas de undistort ───────────────────────────────────────────────────
    map1 = map2 = None
    if usar_cal:
        map1, map2, _ = construir_mapa_undistort(mtx, dist, (h, w))
        print("[INFO] Mapas undistort prontos.")

    # ── Lemniscata ───────────────────────────────────────────────────────────
    x_g_img, y_g_img = gerar_lemniscata(N_POINTS, LEMNISCATA_SCALE, w // 2, h // 2)
    x_g_cart = x_g_img.copy()
    y_g_cart = (h - 1) - y_g_img

    print(f"[INFO] Lemniscata: {N_POINTS} pts | escala={LEMNISCATA_SCALE} px")
    print(f"[INFO] DIST_MARCADORES_CM = {DIST_MARCADORES_CM:.1f} cm — ajuste se necessário.")
    if H is not None:
        print("[INFO] Homografia ativa: a visão pode ficar bem mais estável.")
    if HEADLESS:
        print("[INFO] Modo headless ativo (sem janelas). Use Ctrl+C para sair.")
    print("[INFO] Teclas: q=sair | s=salvar | a=resincronizar opcional | r=reiniciar traj | m=quadrado")
    print(f"[INFO] Gráfico final: {'ON' if PLOT_TRAJETORIA else 'OFF'} | arquivo: {PLOT_SAVE_PATH}")
    print(f"[INFO] Ground truth: alinhamento RMSE máx={ALIGN_RMSE_MAX_CM:.1f} cm, mín pontos={ALIGN_MIN_POINTS}")
    print(
        f"[INFO] Fusão vis↔odom: beta={VIS_CORRECAO_BETA:.2f}, "
        f"gap_max={VIS_CORRECAO_MAX_GAP_CM:.1f}cm, H_rel_err_max={HOMOGRAPHY_MAX_REL_ERR:.2f}"
    )
    print(f"[INFO] Homografia: {'FORCE' if HOMOGRAPHY_FORCE else 'AUTO'}")
    if SQUARE_TRAJ_ENABLE:
        print(
            f"[INFO] Quadrado: use_visao={'ON' if SQUARE_USE_VISION else 'OFF'} | "
            f"vis_max_age={SQUARE_VISION_MAX_AGE_S:.2f}s"
        )

    # ── CSV ──────────────────────────────────────────────────────────────────
    csv_file = None
    if SAVE_CSV:
        csv_file = open(CSV_PATH, "w", encoding="utf-8")
        csv_file.write(
            "t_s,"
            "rear_px_x,rear_px_y,front_px_x,front_px_y,"
            "rear_cm_x,rear_cm_y,front_cm_x,front_cm_y,"
            "center_cm_x,center_cm_y,"
            "vis_dx_cm,vis_dy_cm,"
            "theta_deg,theta_goal_deg,gamma_deg,"
            "dist_ref_cm,ref_cm_x,ref_cm_y,"
            "cm_per_px,"
            "odom_abs_x_m,odom_abs_y_m,odom_abs_yaw_deg,"
            "odom_rel_dx_m,odom_rel_dy_m,odom_rel_dyaw_deg,"
            "odom_yaw_minus_theta_deg,"
            "vis_disp_cm,odom_disp_cm,erro_disp_cm\n"
        )

    # ── Estado do loop ───────────────────────────────────────────────────────
    l = 0
    ultimo_x_det = w // 2
    escala_janela: deque[float] = deque(maxlen=45)
    titulo = "Localizacao (undistorted)" if usar_cal else "Localizacao"

    vis_ref_x_cm: Optional[float] = None
    vis_ref_y_cm: Optional[float] = None
    ref_t_rel: Optional[float] = None
    ref_auto_set = False
    # FIX: guardamos pose da odom no instante 'a' para o ground truth
    odom_ref_yaw_deg_gt: float = 0.0

    serie_t_s: list[float] = []
    serie_vis_dx_cm: list[float] = []
    serie_vis_dy_cm: list[float] = []
    serie_vis_theta: list[float] = []
    serie_odom_dx_cm: list[float] = []
    serie_odom_dy_cm: list[float] = []
    serie_odom_dyaw: list[float] = []
    serie_gamma: list[float] = []
    serie_vis_abs_x_cm: list[float] = []
    serie_vis_abs_y_cm: list[float] = []
    serie_vis_raw_abs_x_cm: list[float] = []
    serie_vis_raw_abs_y_cm: list[float] = []
    serie_odom_abs_x_cm: list[float] = []
    serie_odom_abs_y_cm: list[float] = []
    serie_gt_aruco_x_cm: list[float] = []
    serie_gt_aruco_y_cm: list[float] = []

    prev_rear_img: Optional[Tuple[int, int]] = None
    prev_front_img: Optional[Tuple[int, int]] = None
    prev_center_cm: Optional[Tuple[float, float]] = None
    prev_theta: Optional[float] = None
    hold_frames: int = 0              # frames consecutivos em HOLD/NONE
    prev_odom_abs_x_cm: Optional[float] = None   # odom absoluta no frame anterior
    prev_odom_abs_y_cm: Optional[float] = None

    # Trajetória contínua (usa odometria como preditor e visão como correção)
    traj_est_x_cm: Optional[float] = None
    traj_est_y_cm: Optional[float] = None
    prev_odom_rel_x_cm: Optional[float] = None
    prev_odom_rel_y_cm: Optional[float] = None
    prev_odyaw: Optional[float] = None  # yaw relativo anterior (detecção de curva)

    # ── Objetos reutilizáveis ────────────────────────────────────────────────
    clahe_v = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    kernel_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    while True:
    # ── Sinal de parada via arquivo ──────────────────────────────────────
        if os.path.exists("/tmp/stop_cv_robot"):
            os.remove("/tmp/stop_cv_robot")
            break
    # ── Captura ──────────────────────────────────────────────────────────
        cap.grab()
        ret, img_raw = cap.retrieve()
        if not ret:
            time.sleep(0.03)
            continue

        # ── Undistort ────────────────────────────────────────────────────────
        if usar_cal and map1 is not None and map2 is not None:
            img = cv2.remap(img_raw, map1, map2, cv2.INTER_LINEAR)
        else:
            img = img_raw.copy()

        # ── Status do controlador ────────────────────────────────────────────
        if square_controller is not None:
            sq_status = f"SQ:{square_controller.phase_label}"
        else:
            sq_status = "SQ:OFF"

        # ── ArUco GT (homografia do chão) ──────────────────────────────────
        aruco_markers_used = 0
        if aruco_gt_enabled and aruco_dict is not None and aruco_world:
            if aruco_detector is not None:
                corners, ids, _ = aruco_detector.detectMarkers(img)
            else:
                detect_fn = getattr(cv2.aruco, "detectMarkers", None)
                if detect_fn is None:
                    corners, ids = [], None
                else:
                    corners, ids, _ = detect_fn(img, aruco_dict, parameters=aruco_params)

            aruco_img_pts = []
            aruco_world_pts = []

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten()
                for i, marker_id in enumerate(ids_flat):
                    if marker_id not in aruco_world:
                        continue
                    aruco_markers_used += 1
                    cx_cm, cy_cm, rot_deg = aruco_world[int(marker_id)]
                    img_corners = corners[i].reshape(4, 2).astype(np.float32)
                    world_corners = _aruco_marker_world_corners(cx_cm, cy_cm, ARUCO_TAG_SIZE_CM, rot_deg)
                    aruco_img_pts.append(img_corners)
                    aruco_world_pts.append(world_corners)

            if aruco_markers_used >= ARUCO_MIN_MARKERS and len(aruco_img_pts) >= 1:
                img_pts = np.vstack(aruco_img_pts).astype(np.float32)
                world_pts = np.vstack(aruco_world_pts).astype(np.float32)
                if img_pts.shape[0] >= 4:
                    H_aruco, _mask = cv2.findHomography(
                        img_pts,
                        world_pts,
                        cv2.RANSAC,
                        ARUCO_RANSAC_REPROJ_ERR,
                    )
                    if H_aruco is not None and H_aruco.shape == (3, 3):
                        aruco_H = H_aruco

        # ── Segmentação HSV ──────────────────────────────────────────────────
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        v_ch = clahe_v.apply(v_ch)
        hsv_eq = cv2.merge([h_ch, s_ch, v_ch])
        mask = cv2.inRange(hsv_eq, LOWER_GREEN, UPPER_GREEN)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

        # ── Detecção dos marcadores ──────────────────────────────────────────
        area_min = area_minima_dinamica(ultimo_x_det, w)
        candidatos = detectar_candidatos(mask, area_min)
        if len(candidatos) < 2:
            candidatos = detectar_candidatos(mask, AREA_MINIMA_FALLBACK)

        par = escolher_marcadores(candidatos, prev_rear_img, prev_front_img)

        if par is None:
            historico_traseiro.append(None)
            historico_dianteiro.append(None)
            historico_centro_img.append(None)
            _trim(historico_traseiro)
            _trim(historico_dianteiro)
            _trim(historico_centro_img)
            label = ("" if usar_cal else "SEM CAL | ") + "MARCADORES NAO DETECTADOS"
            cv2.putText(img, label, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            if not HEADLESS:
                cv2.imshow(titulo, img)
                if SHOW_MASK:
                    cv2.imshow("Mascara HSV", mask)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    break
            continue

        rear, front = par
        (cX, cY) = rear["centroid"]
        (cx, cy) = front["centroid"]
        cnt_l = rear["contour"]
        cnt_s = front["contour"]
        ultimo_x_det = cX

        prev_rear_img = (cX, cY)
        prev_front_img = (cx, cy)

        # ── Escala px → cm ───────────────────────────────────────────────────
        dist_marc_px = math.hypot(float(cX - cx), float(cY - cy))
        if dist_marc_px > 1.0:
            if len(escala_janela) >= 5:
                mediana_atual = float(np.median(list(escala_janela)))
                if abs(dist_marc_px - mediana_atual) / mediana_atual < 0.30:
                    escala_janela.append(dist_marc_px)
            else:
                escala_janela.append(dist_marc_px)

            if len(escala_janela) > 0:
                escala_mediana_px = float(np.median(list(escala_janela)))
                cm_per_px = DIST_MARCADORES_CM / escala_mediana_px
            else:
                cm_per_px = None
        else:
            cm_per_px = None

        # Centro do robô em imagem
        center_x_img = 0.5 * (cX + cx)
        center_y_img = 0.5 * (cY + cy)

        # ── Ground truth por ArUco (se disponível) ─────────────────────────
        gt_aruco_x_cm = gt_aruco_y_cm = float("nan")
        if aruco_gt_enabled and aruco_H is not None:
            pt = cv2.perspectiveTransform(
                np.array([[[float(center_x_img), float(center_y_img)]]], dtype=np.float32),
                aruco_H,
            )[0, 0]
            gt_aruco_x_cm = float(pt[0])
            gt_aruco_y_cm = float(pt[1])

        # ── Coordenadas métricas dos marcadores/centro ──────────────────────
        # 1) Tenta homografia, mas valida pelo erro relativo da distância entre
        #    marcadores (deve ser próximo de DIST_MARCADORES_CM).
        # 2) Se H estiver inconsistente, cai para pixel->cm.
        metric_mode = "NONE"
        h_rel_err = float("nan")

        cX_h_cm = cY_h_cm = cx_h_cm = cy_h_cm = center_h_x_cm = center_h_y_cm = float("nan")
        use_h = False

        # HOMOGRAPHY_FORCE=True  → usa H ignorando h_rel_err (aceita qualquer erro relativo)
        # HOMOGRAPHY_FORCE=False → homografia completamente desabilitada, mesmo se H carregado
        if H is not None and HOMOGRAPHY_FORCE:
            cX_h_cm, cY_h_cm = img_pt_to_metric_cm(cX, cY, h, cx_cart_ref, cy_cart_ref, cm_per_px or 1.0, H=H)
            cx_h_cm, cy_h_cm = img_pt_to_metric_cm(cx, cy, h, cx_cart_ref, cy_cart_ref, cm_per_px or 1.0, H=H)
            center_h_x_cm, center_h_y_cm = img_pt_to_metric_cm(
                center_x_img, center_y_img, h, cx_cart_ref, cy_cart_ref, cm_per_px or 1.0, H=H
            )
            dist_h_cm = math.hypot(cx_h_cm - cX_h_cm, cy_h_cm - cY_h_cm)
            if np.isfinite(dist_h_cm) and DIST_MARCADORES_CM > 1e-6:
                h_rel_err = abs(dist_h_cm - DIST_MARCADORES_CM) / DIST_MARCADORES_CM
            else:
                h_rel_err = float("inf")
            use_h = np.isfinite(h_rel_err)
        # Se HOMOGRAPHY_FORCE=False: use_h permanece False, ignora H mesmo que carregado

        if use_h:
            cX_cm, cY_cm = _apply_vis_flip(cX_h_cm, cY_h_cm)
            cx_cm, cy_cm = _apply_vis_flip(cx_h_cm, cy_h_cm)
            center_x_cm, center_y_cm = _apply_vis_flip(center_h_x_cm, center_h_y_cm)
            metric_mode = "H"
        elif cm_per_px is not None:
            cX_cart, cY_cart = img_to_cart((cX, cY), h)
            cx_cart, cy_cart = img_to_cart((cx, cy), h)
            center_x_cart, center_y_cart = img_to_cart((center_x_img, center_y_img), h)

            cX_cm, cY_cm = cart_to_cm(cX_cart, cY_cart, cx_cart_ref, cy_cart_ref, cm_per_px)
            cx_cm, cy_cm = cart_to_cm(cx_cart, cy_cart, cx_cart_ref, cy_cart_ref, cm_per_px)
            center_x_cm, center_y_cm = cart_to_cm(
                center_x_cart, center_y_cart, cx_cart_ref, cy_cart_ref, cm_per_px
            )
            cX_cm, cY_cm = _apply_vis_flip(cX_cm, cY_cm)
            cx_cm, cy_cm = _apply_vis_flip(cx_cm, cy_cm)
            center_x_cm, center_y_cm = _apply_vis_flip(center_x_cm, center_y_cm)
            metric_mode = "PX"
        else:
            cX_cm = cY_cm = cx_cm = cy_cm = float("nan")
            center_x_cm = center_y_cm = float("nan")

        # ── Filtro de salto ─────────────────────────────────────────────────
        # Durante HOLD, avança a referência pelo delta da odom para evitar
        # "sticky HOLD": sem isso, qualquer nova detecção ≥ MAX_SALTO é rejeitada
        # porque prev_center_cm ficou preso enquanto o robô se movia.
        _ox, _oy, _oyaw_j, _ostamp_j, _orec_j = odom_state.read()
        _ox_cm_j = _ox * 100.0
        _oy_cm_j = _oy * 100.0
        if prev_center_cm is not None and _orec_j and prev_odom_abs_x_cm is not None:
            odom_delta_x = _ox_cm_j - prev_odom_abs_x_cm
            odom_delta_y = _oy_cm_j - prev_odom_abs_y_cm  # type: ignore[operator]
            prev_center_cm = (
                prev_center_cm[0] + odom_delta_x,
                prev_center_cm[1] + odom_delta_y,
            )

        if np.isfinite(center_x_cm) and np.isfinite(center_y_cm):
            if prev_center_cm is not None and hold_frames <= 3:
                jump = math.hypot(center_x_cm - prev_center_cm[0], center_y_cm - prev_center_cm[1])
                if jump > MAX_SALTO_VISAO_CM:
                    center_x_cm = center_y_cm = float("nan")
            # hold_frames > 3: aceita a detecção sem filtrar (re-âncora após oclusão)

        if np.isfinite(center_x_cm) and np.isfinite(center_y_cm):
            prev_center_cm = (center_x_cm, center_y_cm)
            hold_frames = 0
        else:
            hold_frames += 1

        if _orec_j:
            prev_odom_abs_x_cm = _ox_cm_j
            prev_odom_abs_y_cm = _oy_cm_j

        # Fonte do centro para métricas de visão:
        # LIVE = frame atual válido | HOLD = última visão válida | NONE = indisponível
        if np.isfinite(center_x_cm) and np.isfinite(center_y_cm):
            vis_center_x_cm, vis_center_y_cm = center_x_cm, center_y_cm
            vis_center_status = "LIVE"
        elif prev_center_cm is not None:
            vis_center_x_cm, vis_center_y_cm = prev_center_cm
            vis_center_status = "HOLD"
        else:
            vis_center_x_cm = vis_center_y_cm = float("nan")
            vis_center_status = "NONE"

        # ── Orientação θ ─────────────────────────────────────────────────────
        if np.isfinite(cX_cm) and np.isfinite(cY_cm) and np.isfinite(cx_cm) and np.isfinite(cy_cm):
            teta_raw = math.atan2(cy_cm - cY_cm, cx_cm - cX_cm)
        elif prev_theta is not None:
            teta_raw = prev_theta
        else:
            teta_raw = 0.0

        if prev_theta is None:
            teta = teta_raw
        else:
            teta = ema_angle(prev_theta, teta_raw, 0.35)
        prev_theta = teta

        # ── Atualiza visão no controlador do quadrado ─────────────────────
        if square_controller is not None:
            vis_for_ctrl = (
                metric_mode != "NONE"
                and vis_center_status == "LIVE"
                and np.isfinite(center_x_cm)
                and np.isfinite(center_y_cm)
            )
            if vis_for_ctrl:
                square_controller.set_vision(
                    center_x_cm,
                    center_y_cm,
                    teta,
                    time.monotonic(),
                    valid=True,
                )
            else:
                square_controller.set_vision(None, None, None, time.monotonic(), valid=False)

        # ── Ponto de referência na lemniscata ───────────────────────────────
        xref_img = int(round(x_g_img[l]))
        yref_img = int(round(y_g_img[l]))
        xref_cart = float(x_g_cart[l])
        yref_cart = float(y_g_cart[l])

        if cm_per_px:
            xref_cm, yref_cm = cart_to_cm(xref_cart, yref_cart, cx_cart_ref, cy_cart_ref, cm_per_px)
        else:
            xref_cm = yref_cm = float("nan")

        # ── Ângulo até a referência ──────────────────────────────────────────
        if np.isfinite(center_x_cm) and np.isfinite(center_y_cm) and np.isfinite(xref_cm):
            teta_g = math.atan2(yref_cm - center_y_cm, xref_cm - center_x_cm)
        else:
            teta_g = float("nan")

        gama = _wrap(teta - teta_g) if np.isfinite(teta_g) else float("nan")

        dist_ref_cm = (
            math.hypot(xref_cm - center_x_cm, yref_cm - center_y_cm)
            if np.isfinite(center_x_cm) and np.isfinite(xref_cm)
            else float("nan")
        )

        l = 0 if l >= N_POINTS - 1 else l + 1

        # ── Leitura da odometria ────────────────────────────────────────────
        ox, oy, oyaw, otime, oreceived = odom_state.read()
        odx_m, ody_m, odyaw = odom_state.get_relative()
        odom_age = (time.monotonic() - otime) if oreceived else float("inf")

        ox_cm = ox * 100.0
        oy_cm = oy * 100.0
        odx_cm = odx_m * 100.0
        ody_cm = ody_m * 100.0

        # ── Auto-referência ────────────────────────────────────────────────
        # 1) zera a odometria automaticamente quando ela estiver disponível;
        # 2) fixa a referência de visão no primeiro centro estável.
        if not odom_state.ref_set and oreceived and odom_age < 0.5:
            odom_state.set_reference()
            ref_t_rel = time.monotonic() - t0
            odom_ref_yaw_deg_gt = math.degrees(oyaw)
            print(
                f"[INFO] Referência odom auto-definida — "
                f"odom({ox:.3f}m, {oy:.3f}m, {math.degrees(oyaw):.1f}°)"
            )

        if (
            vis_ref_x_cm is None
            and np.isfinite(vis_center_x_cm)
            and np.isfinite(vis_center_y_cm)
        ):
            vis_ref_x_cm = float(vis_center_x_cm)
            vis_ref_y_cm = float(vis_center_y_cm)
            if ref_t_rel is None:
                ref_t_rel = time.monotonic() - t0
            print(
                f"[INFO] Referência visual auto-definida — "
                f"centro({vis_ref_x_cm:.1f}cm, {vis_ref_y_cm:.1f}cm)"
            )

        # ── Deslocamentos relativos de visão (desde ref visual) ────────────
        if vis_ref_x_cm is not None and vis_ref_y_cm is not None and np.isfinite(vis_center_x_cm):
            vis_ref_x = float(vis_ref_x_cm)
            vis_ref_y = float(vis_ref_y_cm)
            vis_dx_cm = vis_center_x_cm - vis_ref_x
            vis_dy_cm = vis_center_y_cm - vis_ref_y
        else:
            vis_dx_cm = vis_dy_cm = float("nan")

        # ── Trajetória contínua estimada (visão corrigindo odometria) ───────
        if odom_state.ref_set and prev_odom_rel_x_cm is None and prev_odom_rel_y_cm is None:
            prev_odom_rel_x_cm = odx_cm
            prev_odom_rel_y_cm = ody_cm
            if traj_est_x_cm is None or traj_est_y_cm is None:
                traj_est_x_cm = 0.0
                traj_est_y_cm = 0.0

        if odom_state.ref_set and prev_odom_rel_x_cm is not None and prev_odom_rel_y_cm is not None:
            odom_step_x_cm = odx_cm - prev_odom_rel_x_cm
            odom_step_y_cm = ody_cm - prev_odom_rel_y_cm

            pred_x_cm = (traj_est_x_cm if traj_est_x_cm is not None else 0.0) + odom_step_x_cm
            pred_y_cm = (traj_est_y_cm if traj_est_y_cm is not None else 0.0) + odom_step_y_cm

            vis_rel_ok = (
                vis_ref_x_cm is not None
                and vis_ref_y_cm is not None
                and np.isfinite(vis_center_x_cm)
                and np.isfinite(vis_center_y_cm)
                and vis_center_status == "LIVE"
            )

            # Detecta curva: variação do yaw odom maior que limiar → pausa visão
            dyaw_deg = abs(math.degrees(_wrap(odyaw - (prev_odyaw if prev_odyaw is not None else odyaw))))
            turning = dyaw_deg > VIS_CORRECAO_PAUSA_CURVA_DEG

            if vis_rel_ok and not turning:
                vis_ref_x = float(vis_ref_x_cm) if vis_ref_x_cm is not None else 0.0
                vis_ref_y = float(vis_ref_y_cm) if vis_ref_y_cm is not None else 0.0
                vis_rel_x_raw = vis_center_x_cm - vis_ref_x
                vis_rel_y_raw = vis_center_y_cm - vis_ref_y
                # Corrige rotação e escala do referencial da câmera → mundo
                vis_rel_x_cm, vis_rel_y_cm = _vis_to_world_frame(vis_rel_x_raw, vis_rel_y_raw)
                gap_cm = math.hypot(vis_rel_x_cm - pred_x_cm, vis_rel_y_cm - pred_y_cm)

                if gap_cm <= VIS_CORRECAO_MAX_GAP_CM:
                    beta = VIS_CORRECAO_BETA
                    traj_est_x_cm = beta * vis_rel_x_cm + (1.0 - beta) * pred_x_cm
                    traj_est_y_cm = beta * vis_rel_y_cm + (1.0 - beta) * pred_y_cm
                else:
                    traj_est_x_cm = pred_x_cm
                    traj_est_y_cm = pred_y_cm
            else:
                traj_est_x_cm = pred_x_cm
                traj_est_y_cm = pred_y_cm

            prev_odom_rel_x_cm = odx_cm
            prev_odom_rel_y_cm = ody_cm

        prev_odyaw = odyaw

        # ── Quando visão bruta é NaN, usa estimativa fusionada como fallback ──
        # Isso garante que vis_dx_cm sempre reflete a melhor estimativa de posição
        # (visão LIVE > fusão vis+odom > NaN), evitando a série congelada em 0.
        if not np.isfinite(vis_dx_cm) and traj_est_x_cm is not None:
            vis_dx_cm = traj_est_x_cm
            vis_dy_cm = traj_est_y_cm if traj_est_y_cm is not None else float("nan")

        # ── Erro de deslocamento visão vs odom ──────────────────────────────
        vis_disp_cm = odom_disp_cm = erro_disp_cm = float("nan")
        erro_disp_msg = ""

        if metric_mode == "NONE":
            erro_disp_msg = "sem métrica de visão (escala/H)"
        elif not odom_state.ref_set:
            erro_disp_msg = "aguardando referência automática"
        elif not oreceived:
            erro_disp_msg = "sem odometria ROS (/odom)"
        elif odom_age >= 0.5:
            erro_disp_msg = f"odometria antiga ({odom_age:.1f}s)"
        elif vis_ref_x_cm is None or vis_ref_y_cm is None:
            erro_disp_msg = "aguardando visão estável para referência"
        elif not (np.isfinite(vis_dx_cm) and np.isfinite(vis_dy_cm)):
            erro_disp_msg = "visão instável/NaN (requer nova referência)"
        else:
            vis_disp_cm = math.hypot(vis_dx_cm, vis_dy_cm)
            odom_disp_cm = math.hypot(odx_cm, ody_cm)
            erro_disp_cm = abs(vis_disp_cm - odom_disp_cm)

        odom_yaw_vs_teta = math.degrees(_wrap(oyaw - teta)) if oreceived else float("nan")

        # ── Séries temporais ─────────────────────────────────────────────────
        t_rel = time.monotonic() - t0
        serie_t_s.append(t_rel)
        serie_vis_dx_cm.append(vis_dx_cm)
        serie_vis_dy_cm.append(vis_dy_cm)
        serie_vis_theta.append(math.degrees(teta))
        ref_ok = oreceived and odom_state.ref_set
        serie_odom_dx_cm.append(odx_cm if ref_ok else float("nan"))
        serie_odom_dy_cm.append(ody_cm if ref_ok else float("nan"))
        serie_odom_dyaw.append(math.degrees(odyaw) if ref_ok else float("nan"))
        serie_gamma.append(math.degrees(gama) if np.isfinite(gama) else float("nan"))
        serie_vis_abs_x_cm.append(traj_est_x_cm if traj_est_x_cm is not None else float("nan"))
        serie_vis_abs_y_cm.append(traj_est_y_cm if traj_est_y_cm is not None else float("nan"))
        serie_vis_raw_abs_x_cm.append(center_x_cm if np.isfinite(center_x_cm) else float("nan"))
        serie_vis_raw_abs_y_cm.append(center_y_cm if np.isfinite(center_y_cm) else float("nan"))
        serie_odom_abs_x_cm.append(ox_cm)
        serie_odom_abs_y_cm.append(oy_cm)
        serie_gt_aruco_x_cm.append(gt_aruco_x_cm)
        serie_gt_aruco_y_cm.append(gt_aruco_y_cm)

        # ── CSV ─────────────────────────────────────────────────────────────
        if SAVE_CSV and csv_file:
            csv_file.write(
                f"{t_rel:.3f},"
                f"{cX},{cY},{cx},{cy},"
                f"{cX_cm:.2f},{cY_cm:.2f},{cx_cm:.2f},{cy_cm:.2f},"
                f"{center_x_cm:.2f},{center_y_cm:.2f},"
                f"{vis_dx_cm:.2f},{vis_dy_cm:.2f},"
                f"{math.degrees(teta):.3f},{math.degrees(teta_g):.3f},{math.degrees(gama):.3f},"
                f"{dist_ref_cm:.2f},{xref_cm:.2f},{yref_cm:.2f},"
                f"{cm_per_px if cm_per_px is not None else float('nan'):.6f},"
                f"{ox:.4f},{oy:.4f},{math.degrees(oyaw):.3f},"
                f"{odx_m:.4f},{ody_m:.4f},{math.degrees(odyaw):.3f},"
                f"{odom_yaw_vs_teta:.3f},"
                f"{vis_disp_cm:.2f},{odom_disp_cm:.2f},{erro_disp_cm:.2f}\n"
            )

        # ── Visualização ────────────────────────────────────────────────────
        for i in range(0, N_POINTS - 10, 10):
            cv2.line(
                img,
                (int(round(x_g_img[i])), int(round(y_g_img[i]))),
                (int(round(x_g_img[i + 10])), int(round(y_g_img[i + 10]))),
                (180, 180, 0), 1,
            )

        cv2.circle(img, (xref_img, yref_img), 10, (0, 255, 0), 2)

        _draw_trail(img, historico_traseiro, 2)
        _draw_trail(img, historico_dianteiro, 0)

        if np.isfinite(vis_center_x_cm) and np.isfinite(vis_center_y_cm):
            historico_centro_img.append((int(round(center_x_img)), int(round(center_y_img))))
        else:
            historico_centro_img.append(None)
        _trim(historico_centro_img)
        _draw_center_trail(img, historico_centro_img)

        cv2.drawContours(img, [cnt_l], -1, (0, 0, 255), 2)
        cv2.drawContours(img, [cnt_s], -1, (255, 0, 0), 2)
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)
        cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)

        center_img = (int(round(center_x_img)), int(round(center_y_img)))
        if np.isfinite(center_x_cm) and np.isfinite(center_y_cm):
            cv2.circle(img, center_img, 6, (0, 255, 255), -1)
            cv2.putText(
                img, "centro", (center_img[0] + 6, center_img[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA,
            )

        cv2.arrowedLine(img, (cX, cY), (cx, cy), (0, 255, 255), 2, tipLength=0.2)
        cv2.line(img, (cX, cY), (xref_img, yref_img), (0, 200, 100), 1)
        cv2.line(img, (cx, cy), (xref_img, yref_img), (50, 200, 50), 1)

        # ── HUD ──────────────────────────────────────────────────────────────
        cal_lbl = "CAL:OK" if usar_cal else "CAL:OFF"
        sc_lbl = f"{cm_per_px:.4f}cm/px" if cm_per_px else "escala:—"
        if metric_mode == "H":
            met_lbl = f"M:H({h_rel_err*100:.0f}%)"
        elif metric_mode == "PX":
            met_lbl = "M:PX"
        else:
            met_lbl = "M:—"
        ros_lbl = (
            "ROS:OK" if (oreceived and odom_age < 0.5) else
            "ROS:OLD" if oreceived else
            "ROS:—"
        )
        ref_lbl = "REF:OK" if odom_state.ref_set else "REF:—  (auto)"
        aruco_lbl = ""
        if aruco_gt_enabled:
            aruco_lbl = f" | ARUCO:{aruco_markers_used} {'H:OK' if aruco_H is not None else 'H:—'}"
        if vis_center_status == "LIVE":
            center_line = f"Centro    cm({center_x_cm:+7.1f},{center_y_cm:+7.1f})"
        elif vis_center_status == "HOLD":
            center_line = f"Centro*   cm({vis_center_x_cm:+7.1f},{vis_center_y_cm:+7.1f}) [HOLD]"
        else:
            center_line = "Centro    cm(   nan,   nan)"

        hud = [
            (f"[{cal_lbl} | {ros_lbl} | {ref_lbl} | {sc_lbl} | {met_lbl}{aruco_lbl} | {sq_status}]", (200, 200, 200)),
            (f"Traseiro  px({cX:4d},{cY:4d})  cm({cX_cm:+7.1f},{cY_cm:+7.1f})", (0, 0, 255)),
            (f"Dianteiro px({cx:4d},{cy:4d})  cm({cx_cm:+7.1f},{cy_cm:+7.1f})", (255, 0, 0)),
            (center_line, (0, 255, 255)),
            (f"Ref.traj  px({xref_img:4d},{yref_img:4d}) cm({xref_cm:+7.1f},{yref_cm:+7.1f})", (0, 200, 0)),
            (
                f"theta:{math.degrees(teta):+.1f}  teta_g:{math.degrees(teta_g):+.1f}  gamma:{math.degrees(gama):+.1f} deg"
                if np.isfinite(teta_g)
                else f"theta:{math.degrees(teta):+.1f}  teta_g:—  gamma:—",
                (0, 230, 230),
            ),
            (f"dist_ref: {dist_ref_cm:.1f} cm", (0, 255, 255)),
            (
                f"vis Δ   dx:{vis_dx_cm:+6.1f}cm  dy:{vis_dy_cm:+6.1f}cm"
                if not math.isnan(vis_dx_cm)
                else "vis Δ: aguardando referência automática",
                (255, 100, 100),
            ),
            (f"ODOM abs  x:{ox_cm:+7.1f}cm  y:{oy_cm:+7.1f}cm  yaw:{math.degrees(oyaw):+.1f}°",
             (255, 180, 0)),
            (f"ODOM Δ    dx:{odx_cm:+6.1f}cm  dy:{ody_cm:+6.1f}cm  dyaw:{math.degrees(odyaw):+.1f}°",
             (255, 210, 80)),
            (f"Odom yaw - visão theta: {odom_yaw_vs_teta:+.1f}°  (diff orientação)",
             (255, 230, 120)),
            (
                f"Disp vis:{vis_disp_cm:.1f}cm  odom:{odom_disp_cm:.1f}cm  erro:|{erro_disp_cm:.1f}|cm"
                if not math.isnan(erro_disp_cm)
                else f"Erro disp: {erro_disp_msg}",
                (180, 100, 255),
            ),
            (f"traj {l}/{N_POINTS} | hist {len(historico_centro_img)}/{MAX_HISTORICO}",
             (150, 150, 150)),
        ]

        for i, (txt, cor) in enumerate(hud):
            cv2.putText(
                img, txt, (10, 26 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, cor, 1, cv2.LINE_AA,
            )

        if not HEADLESS:
            cv2.imshow(titulo, img)
            if SHOW_MASK:
                cv2.imshow("Mascara HSV", mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                fname = f"frame_{int(time.monotonic())}.jpg"
                cv2.imwrite(fname, img)
                print(f"[INFO] Frame salvo: {fname}")
            elif key == ord("a"):
                # Re-sync robusto: usa LIVE; se não houver LIVE, usa HOLD (última válida)
                vis_live = np.isfinite(center_x_cm) and np.isfinite(center_y_cm)
                prev_center_for_ref = prev_center_cm
                vis_hold = (not vis_live) and (prev_center_for_ref is not None)
                if vis_live:
                    vis_ref_candidate_x = float(center_x_cm)
                    vis_ref_candidate_y = float(center_y_cm)
                    vis_ref_src = "LIVE"
                elif vis_hold and prev_center_for_ref is not None:
                    vis_ref_candidate_x = float(prev_center_for_ref[0])
                    vis_ref_candidate_y = float(prev_center_for_ref[1])
                    vis_ref_src = "HOLD"
                else:
                    vis_ref_candidate_x = None
                    vis_ref_candidate_y = None
                    vis_ref_src = "NONE"

                odom_state.set_reference(
                    vis_x_cm=vis_ref_candidate_x,
                    vis_y_cm=vis_ref_candidate_y,
                )
                if vis_ref_candidate_x is not None and vis_ref_candidate_y is not None:
                    vis_ref_x_cm = vis_ref_candidate_x
                    vis_ref_y_cm = vis_ref_candidate_y
                    print(
                        f"[INFO] Referência zerada (visão+odom) — "
                        f"odom({ox:.3f}m, {oy:.3f}m, {math.degrees(oyaw):.1f}°) | "
                        f"visão {vis_ref_src} centro({vis_ref_candidate_x:.1f}cm, {vis_ref_candidate_y:.1f}cm)"
                    )
                else:
                    print(
                        f"[INFO] Referência odom zerada (visão indisponível — vis_ref PRESERVADA) — "
                        f"odom({ox:.3f}m, {oy:.3f}m, {math.degrees(oyaw):.1f}°)"
                    )
                ref_t_rel = t_rel
                odom_ref_yaw_deg_gt = math.degrees(oyaw)
                traj_est_x_cm = 0.0
                traj_est_y_cm = 0.0
                prev_odom_rel_x_cm = odx_cm
                prev_odom_rel_y_cm = ody_cm

            elif key == ord("r"):
                l = 0
                historico_traseiro.clear()
                historico_dianteiro.clear()
                historico_centro_img.clear()
                prev_rear_img = None
                prev_front_img = None
                prev_center_cm = None
                prev_theta = None
                vis_ref_x_cm = None
                vis_ref_y_cm = None
                traj_est_x_cm = None
                traj_est_y_cm = None
                prev_odom_rel_x_cm = None
                prev_odom_rel_y_cm = None
                odom_state.ref_set = False
                print("[INFO] Trajetória reiniciada.")
            elif key == ord("m"):
                if square_controller is None or odom_node is None:
                    print("[AVISO] Trajetória quadrada indisponível (requer odometria ativa).")
                else:
                    started = square_controller.toggle(time.monotonic())
                    if started:
                        sq_start_t_rel = time.monotonic() - t0
                        print(f"[INFO] Trajetória quadrada iniciada (t_rel={sq_start_t_rel:.2f}s).")
                    else:
                        odom_node.stop_robot()
                        print("[INFO] Trajetória quadrada parada.")

    # ── Finalização ───────────────────────────────────────────────────────────
    if odom_node is not None:
        odom_node.stop_robot()

    cap.release()
    cv2.destroyAllWindows()

    if csv_file:
        csv_file.close()
        print(f"[INFO] CSV salvo em: {CSV_PATH}")

    if PLOT_TRAJETORIA:
        # ── Ground truth ─────────────────────────────────────────────────────
        gt_teo_t = gt_teo_x = gt_teo_y = None
        if (
            SQUARE_TRAJ_ENABLE
            and sq_start_t_rel is not None
            and ref_t_rel is not None
            and len(serie_t_s) > 0
        ):
            try:
                t_total_rec = serie_t_s[-1]
                gt_teo_t, gt_teo_x, gt_teo_y = gerar_ground_truth_quadrado(
                    forward_s=SQUARE_FORWARD_S,
                    turn_s=SQUARE_LEFT_TURN_S,
                    linear_mps=SQUARE_LINEAR_MPS,
                    turn_angle_deg=SQUARE_TURN_ANGLE_DEG,
                    num_sides=SQUARE_NUM_SIDES,
                    start_delay_s=SQUARE_START_DELAY_S,
                    t_ref_rel=ref_t_rel,
                    sq_start_t_rel=sq_start_t_rel,
                    odom_ref_x_cm=0.0,   # centrado em 'a' → origem
                    odom_ref_y_cm=0.0,
                    odom_ref_yaw_deg=odom_ref_yaw_deg_gt,
                    t_total=t_total_rec,
                )
                print(f"[INFO] Ground truth teórico gerado: {len(gt_teo_t)} pontos.")
            except Exception as e:
                print(f"[AVISO] Falha ao gerar ground truth: {e}")

        gt_aruco_t = gt_aruco_x = gt_aruco_y = None
        if np.any(np.isfinite(serie_gt_aruco_x_cm)) and np.any(np.isfinite(serie_gt_aruco_y_cm)):
            gt_aruco_t = np.asarray(serie_t_s, dtype=float)
            gt_aruco_x = np.asarray(serie_gt_aruco_x_cm, dtype=float)
            gt_aruco_y = np.asarray(serie_gt_aruco_y_cm, dtype=float)

        gerar_graficos_trajetoria(
            t_s=serie_t_s,
            vis_dx_cm=serie_vis_dx_cm,
            vis_dy_cm=serie_vis_dy_cm,
            vis_theta_deg=serie_vis_theta,
            odom_dx_cm=serie_odom_dx_cm,
            odom_dy_cm=serie_odom_dy_cm,
            odom_dyaw_deg=serie_odom_dyaw,
            gamma_deg=serie_gamma,
            vis_abs_x_cm=serie_vis_abs_x_cm,
            vis_abs_y_cm=serie_vis_abs_y_cm,
            vis_raw_abs_x_cm=serie_vis_raw_abs_x_cm,
            vis_raw_abs_y_cm=serie_vis_raw_abs_y_cm,
            odom_abs_x_cm=serie_odom_abs_x_cm,
            odom_abs_y_cm=serie_odom_abs_y_cm,
            save_path=PLOT_SAVE_PATH,
            show_plot=PLOT_SHOW,
            ref_t_s=ref_t_rel,
            odom_ref_yaw_deg=odom_ref_yaw_deg_gt,
            gt_aruco_t_s=gt_aruco_t,
            gt_aruco_x_cm=gt_aruco_x,
            gt_aruco_y_cm=gt_aruco_y,
            gt_teo_t_s=gt_teo_t,
            gt_teo_x_cm=gt_teo_x,
            gt_teo_y_cm=gt_teo_y,
        )

    if ros_thread is not None:
        ros_thread.join(timeout=3.0)

    print("[INFO] Encerrado.")


if __name__ == "__main__":
    main()
