#!/usr/bin/env python3
"""
ball_viewer.py  —  GTK3: RGB (izq) + Depth/Cloud (der), detección pelota naranja.

Ejecutar:
  PYTHONNOUSERSITE=1 /usr/bin/python3 examples/ball_viewer.py [IP]

Controles:
  Botón "Depth / 3D"  → alterna entre mapa de profundidad y nube 3D rotable
  Drag en panel der   → rota la nube 3D (en modo 3D)
  Click en panel der  → muestra coordenadas XYZ (en modo Depth)
  Botón "Guardar"     → guarda PNG en captures/
"""
import os, site, sys, threading, time, json, subprocess
from collections import deque
from pathlib import Path

if not os.environ.get("SCEPTER_ALLOW_USER_SITE"):
    sys.path = [p for p in sys.path if p != site.getusersitepackages()]

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib, GdkPixbuf, Gdk

import cv2
import numpy as np

_HERE = Path(__file__).resolve()
ROOT  = _HERE.parents[1]
sys.path.insert(0, str(_HERE.parent))
from picker.camera import ScepterCamera


# ═══════════════════════════════════════════════════════════════════════
# UTILIDADES DE IMAGEN
# ═══════════════════════════════════════════════════════════════════════

def bgr_to_pixbuf(bgr: np.ndarray) -> GdkPixbuf.Pixbuf:
    """
    Convierte ndarray BGR → GdkPixbuf RGB.
    Guarda referencia a los bytes en el pixbuf para evitar GC prematuro.
    """
    rgb  = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    # IMPORTANTE: tobytes() crea bytes que deben sobrevivir hasta que
    # Cairo termine de renderizar. Lo adjuntamos al pixbuf (_keep).
    data = rgb.tobytes()
    pb   = GdkPixbuf.Pixbuf.new_from_data(
        data, GdkPixbuf.Colorspace.RGB, False, 8, w, h, w * 3
    )
    pb._keep = data          # evita que el GC libere los bytes
    return pb


def fit_pixbuf(pb: GdkPixbuf.Pixbuf, wa: int, ha: int) -> GdkPixbuf.Pixbuf:
    """Escala pixbuf para llenar (wa × ha) sin recortar."""
    return pb.scale_simple(wa, ha, GdkPixbuf.InterpType.NEAREST)


# ── Colormap de profundidad (EMA range, buffer pre-allocado) ─────────────────

class DepthColorizer:
    _ALPHA = 0.06

    def __init__(self):
        self._near = None
        self._far  = None
        self._buf  = None

    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        valid = depth_mm[(depth_mm > 0) & (depth_mm < 65535)]
        if valid.size > 500:
            p2, p98 = float(np.percentile(valid, 2)), float(np.percentile(valid, 98))
            if self._near is None:
                self._near, self._far = p2, p98
            else:
                a = self._ALPHA
                self._near = self._near * (1 - a) + p2  * a
                self._far  = self._far  * (1 - a) + p98 * a

        if self._near is None:
            return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)

        span = max(self._far - self._near, 50.0)
        if self._buf is None or self._buf.shape != depth_mm.shape:
            self._buf = np.empty(depth_mm.shape, dtype=np.uint8)

        cv2.convertScaleAbs(
            depth_mm.astype(np.float32), self._buf,
            alpha=255.0 / span,
            beta=-self._near * 255.0 / span,
        )
        self._buf[(depth_mm == 0) | (depth_mm >= 65535)] = 0
        return cv2.applyColorMap(self._buf, cv2.COLORMAP_TURBO)


# ── Renderizador de nube 3D (numpy, dentro del panel) ───────────────────────

class CloudRenderer:
    """
    Proyección ortográfica del point cloud.
    - Drag   → rotar (azimuth / elevation)
    - Scroll → zoom
    - Color  → temperatura por distancia al sensor (rojo=cerca, azul=lejos)
    - Puntos → dibujados en bloques 3×3 px para mayor densidad visual
    """
    MAX_PTS = 50_000

    def __init__(self, w=800, h=600):
        self.W, self.H   = w, h
        self.azimuth     = 30.0
        self.elevation   = 25.0
        self._scale_base = None   # escala automática (primer frame)
        self._zoom       = 1.0

    # ── Zoom ────────────────────────────────────────────────────────────────
    def zoom(self, factor: float):
        self._zoom = float(np.clip(self._zoom * factor, 0.1, 20.0))

    def reset_scale(self):
        self._scale_base = None
        self._zoom       = 1.0

    # ── Render ──────────────────────────────────────────────────────────────
    def render(self, xyz_mm: np.ndarray) -> np.ndarray:
        img  = np.full((self.H, self.W, 3), 18, dtype=np.uint8)

        mask = np.isfinite(xyz_mm[:, :, 2]) & (xyz_mm[:, :, 2] > 50) & (xyz_mm[:, :, 2] < 4000)
        pts  = xyz_mm[mask]
        if len(pts) == 0:
            cv2.putText(img, "Sin puntos 3D", (220, 300),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (120, 120, 120), 2)
            return img

        # Submuestrear uniformemente
        step = max(1, len(pts) // self.MAX_PTS)
        pts  = pts[::step]

        # ── Rotación ────────────────────────────────────────────────────────
        az  = np.radians(self.azimuth)
        el  = np.radians(self.elevation)
        Raz = np.array([[ np.cos(az), np.sin(az), 0],
                        [-np.sin(az), np.cos(az), 0],
                        [0, 0, 1]], dtype=np.float32)
        Rel = np.array([[1, 0, 0],
                        [0,  np.cos(el), np.sin(el)],
                        [0, -np.sin(el), np.cos(el)]], dtype=np.float32)
        rot = ((Rel @ Raz) @ pts.T).T          # Nx3

        # ── Escala ──────────────────────────────────────────────────────────
        if self._scale_base is None:
            span = max(rot[:, 0].ptp(), rot[:, 1].ptp(), 1.0)
            self._scale_base = min(self.W, self.H) * 0.65 / span

        scale = self._scale_base * self._zoom
        sx = (rot[:, 0] * scale + self.W / 2).astype(np.int32)
        sy = (-rot[:, 1] * scale + self.H / 2).astype(np.int32)

        # ── Color por distancia original (Z del sensor) ──────────────────────
        # pts[:,2] = profundidad real en mm. Cerca → rojo, lejos → azul.
        z_real = pts[:, 2]
        z_near, z_far = z_real.min(), z_real.max()
        # Invertimos: valor alto = cerca = rojo en JET
        zn = (255 - ((z_real - z_near) / max(z_far - z_near, 1) * 255)).astype(np.uint8)
        colors = cv2.applyColorMap(zn.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

        # ── Painter's: de lejos a cerca ──────────────────────────────────────
        order = np.argsort(rot[:, 2])[::-1]
        sx, sy, colors = sx[order], sy[order], colors[order]

        # ── Dibujar bloques 3×3 px para mayor densidad ───────────────────────
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy = sy + dy
                xx = sx + dx
                ok = (xx >= 0) & (xx < self.W) & (yy >= 0) & (yy < self.H)
                img[yy[ok], xx[ok]] = colors[ok]

        # ── Barra de temperatura (leyenda) ───────────────────────────────────
        bar_x, bar_y, bar_w, bar_h = self.W - 28, 40, 14, 160
        bar = np.arange(255, -1, -255 / bar_h, dtype=np.uint8)[:bar_h]
        bar_rgb = cv2.applyColorMap(bar.reshape(-1, 1), cv2.COLORMAP_JET).reshape(bar_h, 1, 3)
        img[bar_y:bar_y+bar_h, bar_x:bar_x+bar_w] = np.repeat(bar_rgb, bar_w, axis=1)
        cv2.rectangle(img, (bar_x-1, bar_y-1), (bar_x+bar_w, bar_y+bar_h), (180,180,180), 1)
        cv2.putText(img, f"{z_near:.0f}", (bar_x - 6, bar_y + bar_h + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        cv2.putText(img, f"{z_far:.0f}mm", (bar_x - 6, bar_y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        cv2.putText(img, "lejos", (bar_x - 4, bar_y + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 100, 255), 1)
        cv2.putText(img, "cerca", (bar_x - 4, bar_y + bar_h - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (100, 100, 255), 1)

        # ── HUD ─────────────────────────────────────────────────────────────
        cv2.putText(img, f"az={self.azimuth:.0f}  el={self.elevation:.0f}  zoom={self._zoom:.1f}x",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1)
        cv2.putText(img, "drag=rotar  scroll=zoom  pts=" + str(len(pts)),
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 130, 130), 1)
        return img


# ═══════════════════════════════════════════════════════════════════════
# DETECCIÓN + TRACKER
# ═══════════════════════════════════════════════════════════════════════

def _patch_xyz(xyz_mm, cy, cx, half=5):
    h, w = xyz_mm.shape[:2]
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    patch = xyz_mm[y0:y1, x0:x1]
    ok = patch[np.isfinite(patch[:, :, 2]) & (patch[:, :, 2] > 0)]
    return np.median(ok, axis=0) if len(ok) else None


def _edge_xyz(xyz_mm, cy, cx, h, w, s=8):
    for d in range(s):
        for dy, dx in [(0,0),(0,d),(d,0),(0,-d),(-d,0)]:
            ny, nx = cy+dy, cx+dx
            if 0 <= ny < h and 0 <= nx < w:
                v = xyz_mm[ny, nx]
                if np.isfinite(v[2]) and v[2] > 0:
                    return v
    return None


def _size_3d(xyz_mm, cy, cx, r, h, w):
    L = _edge_xyz(xyz_mm, cy, max(0, cx-r),   h, w)
    R = _edge_xyz(xyz_mm, cy, min(w-1, cx+r), h, w)
    T = _edge_xyz(xyz_mm, max(0, cy-r),   cx, h, w)
    B = _edge_xyz(xyz_mm, min(h-1, cy+r), cx, h, w)
    wm = float(np.linalg.norm(R-L)) if (L is not None and R is not None) else None
    hm = float(np.linalg.norm(B-T)) if (T is not None and B is not None) else None
    return wm, hm


def detect_orange(bgr, xyz_mm):
    h, w = bgr.shape[:2]
    small = cv2.resize(bgr, (w//2, h//2), interpolation=cv2.INTER_NEAREST)
    hsv   = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    m1    = cv2.inRange(hsv, (2, 100, 60), (20, 255, 255))
    m2    = cv2.inRange(hsv, (20,  80, 60), (30, 255, 255))
    mask  = cv2.bitwise_or(m1, m2)
    k     = np.ones((5, 5), np.uint8)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    mask  = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 800:
        return None
    (cx, cy), radius = cv2.minEnclosingCircle(c)
    cx, cy, radius = int(cx), int(cy), int(radius)
    xyz    = _patch_xyz(xyz_mm, cy, cx)
    wm, hm = _size_3d(xyz_mm, cy, cx, radius, h, w)
    return dict(cx=cx, cy=cy, radius=radius, xyz=xyz, width_mm=wm, height_mm=hm)


class BallTracker:
    N, MISS = 10, 8

    def __init__(self):
        self._buf  = deque(maxlen=self.N)
        self._miss = 0

    def update(self, det):
        if det:
            self._buf.append(det); self._miss = 0
        else:
            self._miss += 1
            if self._miss > self.MISS:
                self._buf.clear()

    @property
    def result(self):
        if not self._buf:
            return None
        xyzs = [d["xyz"] for d in self._buf if d["xyz"] is not None]
        wms  = [d["width_mm"]  for d in self._buf if d["width_mm"]  is not None]
        hms  = [d["height_mm"] for d in self._buf if d["height_mm"] is not None]
        return dict(
            cx        = int(np.mean([d["cx"]     for d in self._buf])),
            cy        = int(np.mean([d["cy"]     for d in self._buf])),
            radius    = int(np.mean([d["radius"] for d in self._buf])),
            xyz       = np.mean(xyzs, axis=0) if xyzs else None,
            width_mm  = float(np.mean(wms)) if wms else None,
            height_mm = float(np.mean(hms)) if hms else None,
        )


# ═══════════════════════════════════════════════════════════════════════
# DIBUJO DE OVERLAYS
# ═══════════════════════════════════════════════════════════════════════

def draw_ball_rgb(img, ball):
    if not ball:
        return
    cx, cy, r = ball["cx"], ball["cy"], ball["radius"]
    wm, hm = ball["width_mm"], ball["height_mm"]
    xyz    = ball["xyz"]

    cv2.circle(img, (cx, cy), r, (0, 165, 255), 2)
    cv2.circle(img, (cx, cy), 4, (0, 255, 255), -1)
    cv2.line(img, (cx-14, cy), (cx+14, cy), (0, 255, 255), 1)
    cv2.line(img, (cx, cy-14), (cx, cy+14), (0, 255, 255), 1)

    cv2.arrowedLine(img, (cx-r, cy-2), (cx+r, cy-2), (0, 180, 255), 1, tipLength=0.08)
    cv2.arrowedLine(img, (cx+r, cy-2), (cx-r, cy-2), (0, 180, 255), 1, tipLength=0.08)
    cv2.arrowedLine(img, (cx+2, cy-r), (cx+2, cy+r), (0, 180, 255), 1, tipLength=0.08)
    cv2.arrowedLine(img, (cx+2, cy+r), (cx+2, cy-r), (0, 180, 255), 1, tipLength=0.08)

    y0 = max(cy - r - 36, 52)
    if xyz is not None:
        cv2.putText(img, f"x={xyz[0]:.0f} y={xyz[1]:.0f} z={xyz[2]:.0f} mm",
                    (cx-100, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 1)
    w_s = f"{wm:.0f}" if wm else "---"
    h_s = f"{hm:.0f}" if hm else "---"
    cv2.putText(img, f"ancho={w_s}mm  alto={h_s}mm",
                (cx-80, y0+18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 255, 180), 1)
    if wm:
        cv2.putText(img, f"{wm:.0f}mm", (cx-18, cy-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)
    if hm:
        cv2.putText(img, f"{hm:.0f}mm", (cx+7,  cy+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 255), 1)


def draw_ball_depth(img, ball):
    if not ball:
        return
    cx, cy, r = ball["cx"], ball["cy"], ball["radius"]
    cv2.circle(img, (cx, cy), r, (255, 255, 255), 2)
    cv2.circle(img, (cx, cy), 4, (255, 255, 255), -1)
    xyz = ball["xyz"]
    if xyz is not None:
        cv2.putText(img, f"z={xyz[2]:.0f}mm",
                    (cx-35, cy-r-8), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1)
    wm, hm = ball["width_mm"], ball["height_mm"]
    w_s = f"{wm:.0f}" if wm else "---"
    h_s = f"{hm:.0f}" if hm else "---"
    cv2.putText(img, f"W:{w_s} H:{h_s}mm",
                (cx-50, cy-r+10), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 255, 200), 1)


# ═══════════════════════════════════════════════════════════════════════
# VENTANA PRINCIPAL
# ═══════════════════════════════════════════════════════════════════════

class BallViewerWindow(Gtk.Window):

    MODE_DEPTH = "depth"
    MODE_CLOUD = "cloud"

    def __init__(self, ip):
        super().__init__(title="Ball Viewer  —  RGB + Depth 3D")
        self.ip = ip
        self.set_default_size(1640, 680)
        self.connect("destroy", self._quit)

        # ── Estado compartido (protegido por lock) ──────────────────────────
        self._lock        = threading.Lock()
        self._rgb_frame   = None     # BGR uint8 (600,800,3)
        self._right_frame = None     # BGR uint8 — depth colormap o cloud render
        self._ball        = None
        self._fps         = 0
        self._xyz_mm      = None     # último xyz_mm para click/hover

        # ── Modo del panel derecho ──────────────────────────────────────────
        self._mode       = self.MODE_DEPTH
        self._colorizer  = DepthColorizer()
        self._cloud      = CloudRenderer()
        self._tracker    = BallTracker()

        # ── Drag para nube 3D ───────────────────────────────────────────────
        self._drag       = None     # (x, y, az0, el0) al inicio del drag

        # ── Click en Depth (panel der) ──────────────────────────────────────
        self._depth_click_px  = None
        self._depth_click_xyz = None

        # ── Click en RGB (panel izq) → verificar alineación con depth ───────
        self._rgb_click_px  = None   # (px, py) en imagen 800×600
        self._rgb_click_bgr = None   # color BGR en ese pixel
        self._rgb_click_xyz = None   # xyz del mismo pixel en depth map

        self._running    = True

        # ── Construir UI ────────────────────────────────────────────────────
        vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(vbox)

        vbox.pack_start(self._build_toolbar(), False, False, 0)

        # Dos drawing areas lado a lado
        hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=4)
        hbox.set_margin_start(4); hbox.set_margin_end(4)
        hbox.set_margin_top(4);   hbox.set_margin_bottom(4)
        vbox.pack_start(hbox, True, True, 0)

        # Panel izquierdo: RGB (con click para alineación)
        self._da_rgb = Gtk.DrawingArea()
        self._da_rgb.set_size_request(800, 600)
        self._da_rgb.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self._da_rgb.connect("draw",               self._draw_rgb)
        self._da_rgb.connect("button-press-event", self._on_rgb_click)
        hbox.pack_start(self._da_rgb, True, True, 0)

        # Panel derecho: Depth / Cloud
        self._da_right = Gtk.DrawingArea()
        self._da_right.set_size_request(800, 600)
        self._da_right.add_events(
            Gdk.EventMask.BUTTON_PRESS_MASK   |
            Gdk.EventMask.BUTTON_RELEASE_MASK |
            Gdk.EventMask.POINTER_MOTION_MASK |
            Gdk.EventMask.BUTTON_MOTION_MASK  |
            Gdk.EventMask.SCROLL_MASK
        )
        self._da_right.connect("draw",                 self._draw_right)
        self._da_right.connect("button-press-event",   self._on_right_press)
        self._da_right.connect("button-release-event", self._on_right_release)
        self._da_right.connect("motion-notify-event",  self._on_right_motion)
        self._da_right.connect("scroll-event",         self._on_scroll)
        hbox.pack_start(self._da_right, True, True, 0)

        # Barra de estado
        self._lbl_status = Gtk.Label(label="Conectando a cámara...")
        self._lbl_status.set_xalign(0)
        self._lbl_status.set_margin_start(8)
        self._lbl_status.set_margin_bottom(2)
        vbox.pack_start(self._lbl_status, False, False, 0)

        self._lbl_coords = Gtk.Label(label="")
        self._lbl_coords.set_xalign(0)
        self._lbl_coords.set_margin_start(8)
        self._lbl_coords.set_margin_bottom(6)
        vbox.pack_start(self._lbl_coords, False, False, 0)

        self.show_all()

        # Timer GTK para refrescar UI a 30fps (más estable que idle_add desde thread)
        GLib.timeout_add(33, self._tick)

        # Hilo de cámara
        threading.Thread(target=self._camera_loop, daemon=True).start()

    # ── Toolbar ─────────────────────────────────────────────────────────────

    def _build_toolbar(self):
        bar = Gtk.Toolbar()
        bar.get_style_context().add_class(Gtk.STYLE_CLASS_PRIMARY_TOOLBAR)

        self._btn_mode = Gtk.ToolButton()
        self._btn_mode.set_label("Cambiar a 3D")
        self._btn_mode.set_icon_name("applications-graphics")
        self._btn_mode.set_is_important(True)
        self._btn_mode.connect("clicked", self._toggle_mode)
        bar.insert(self._btn_mode, 0)

        bar.insert(Gtk.SeparatorToolItem(), 1)

        self._btn_sim = Gtk.ToolButton()
        self._btn_sim.set_label("Simular Pick")
        self._btn_sim.set_icon_name("media-playback-start")
        self._btn_sim.set_is_important(True)
        self._btn_sim.set_sensitive(False)   # activar solo cuando hay pelota
        self._btn_sim.connect("clicked", self._on_simulate)
        bar.insert(self._btn_sim, 2)

        bar.insert(Gtk.SeparatorToolItem(), 3)

        btn_save = Gtk.ToolButton()
        btn_save.set_label("Guardar")
        btn_save.set_icon_name("document-save")
        btn_save.set_is_important(True)
        btn_save.connect("clicked", self._on_save)
        bar.insert(btn_save, 4)

        bar.insert(Gtk.SeparatorToolItem(), 5)

        btn_quit = Gtk.ToolButton()
        btn_quit.set_label("Salir")
        btn_quit.set_icon_name("application-exit")
        btn_quit.set_is_important(True)
        btn_quit.connect("clicked", lambda *_: self._quit())
        bar.insert(btn_quit, 6)

        return bar

    # ── Toggle Depth / Cloud ─────────────────────────────────────────────────

    def _toggle_mode(self, *_):
        if self._mode == self.MODE_DEPTH:
            self._mode = self.MODE_CLOUD
            self._cloud.reset_scale()
            self._btn_mode.set_label("Cambiar a Depth")
        else:
            self._mode = self.MODE_DEPTH
            self._btn_mode.set_label("Cambiar a 3D")
        self._click_px = None   # limpiar click anterior

    # ── Draw callbacks ───────────────────────────────────────────────────────

    def _draw_rgb(self, widget, cr):
        with self._lock:
            frame    = self._rgb_frame
            ball     = self._ball
            fps      = self._fps
            clk_px   = self._rgb_click_px
            clk_xyz  = self._rgb_click_xyz
            clk_bgr  = self._rgb_click_bgr
        if frame is None:
            return False

        img = frame.copy()
        draw_ball_rgb(img, ball)

        # Crosshair del punto clickeado en RGB
        if clk_px is not None:
            px, py = clk_px
            cv2.drawMarker(img, (px, py), (255, 80, 0), cv2.MARKER_CROSS, 26, 2)
            # Cuadrado de color muestreado
            if clk_bgr is not None:
                b, g, r = int(clk_bgr[0]), int(clk_bgr[1]), int(clk_bgr[2])
                cv2.rectangle(img, (px+14, py-14), (px+30, py+2), (b,g,r), -1)
                cv2.rectangle(img, (px+14, py-14), (px+30, py+2), (255,255,255), 1)

        cv2.putText(img, f"RGB  {fps}fps  [click=alinear]", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        pb = bgr_to_pixbuf(img)
        wa = widget.get_allocated_width()
        ha = widget.get_allocated_height()
        Gdk.cairo_set_source_pixbuf(cr, fit_pixbuf(pb, wa, ha), 0, 0)
        cr.paint()
        return False

    def _draw_right(self, widget, cr):
        with self._lock:
            frame    = self._right_frame
            ball     = self._ball
            mode     = self._mode
            dclk_px  = self._depth_click_px
            dclk_xyz = self._depth_click_xyz
            rclk_px  = self._rgb_click_px     # mismo pixel, alineado
            rclk_xyz = self._rgb_click_xyz
        if frame is None:
            return False

        img = frame.copy()

        if mode == self.MODE_DEPTH:
            draw_ball_depth(img, ball)
            cv2.putText(img, "Depth  [click=XYZ]", (10, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Punto clickeado en depth
            if dclk_px is not None:
                px, py = dclk_px
                cv2.drawMarker(img, (px, py), (0, 255, 0), cv2.MARKER_CROSS, 22, 2)
                if dclk_xyz is not None:
                    lbl = f"x={dclk_xyz[0]:.0f} y={dclk_xyz[1]:.0f} z={dclk_xyz[2]:.0f}mm"
                    (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
                    bx = min(px+10, img.shape[1]-tw-6); by = max(py-10, th+6)
                    cv2.rectangle(img, (bx-2, by-th-2), (bx+tw+2, by+2), (0,0,0), -1)
                    cv2.putText(img, lbl, (bx, by), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0,255,0), 1)

            # Crosshair del click RGB → mismo pixel en depth (alineación)
            if rclk_px is not None:
                px, py = rclk_px
                cv2.drawMarker(img, (px, py), (255, 80, 0), cv2.MARKER_TILTED_CROSS, 26, 2)
                if rclk_xyz is not None:
                    lbl = f"[RGB→] z={rclk_xyz[2]:.0f}mm"
                    cv2.putText(img, lbl, (px+14, py-6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 120, 0), 1)
        else:
            cv2.putText(img, "3D Cloud  [drag=rotar  scroll=zoom]", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        pb = bgr_to_pixbuf(img)
        wa = widget.get_allocated_width()
        ha = widget.get_allocated_height()
        Gdk.cairo_set_source_pixbuf(cr, fit_pixbuf(pb, wa, ha), 0, 0)
        cr.paint()
        return False

    # ── Utilidad coordenadas ─────────────────────────────────────────────────

    def _widget_to_img(self, widget, wx, wy, iw=800, ih=600):
        wa = widget.get_allocated_width()
        ha = widget.get_allocated_height()
        return int(wx * iw / wa), int(wy * ih / ha)

    def _xyz_at(self, px, py):
        with self._lock:
            xm = self._xyz_mm
        if xm is None:
            return None
        h, w = xm.shape[:2]
        if 0 <= py < h and 0 <= px < w:
            v = xm[py, px]
            if np.isfinite(v[2]) and v[2] > 0:
                return v.copy()
        return None

    # ── Click en RGB → alineación depth ─────────────────────────────────────

    def _on_rgb_click(self, widget, event):
        if event.button != 1:
            return
        px, py = self._widget_to_img(widget, event.x, event.y)
        xyz = self._xyz_at(px, py)
        with self._lock:
            bgr_val = self._rgb_frame[py, px].copy() if self._rgb_frame is not None else None
            self._rgb_click_px  = (px, py)
            self._rgb_click_bgr = bgr_val
            self._rgb_click_xyz = xyz

        b, g, r = (int(bgr_val[i]) for i in range(3)) if bgr_val is not None else (0,0,0)
        if xyz is not None:
            self._lbl_coords.set_markup(
                f"<b>RGB click</b> ({px},{py})  BGR=({b},{g},{r})  "
                f"→  Depth x=<b>{xyz[0]:.1f}</b>  y=<b>{xyz[1]:.1f}</b>  z=<b>{xyz[2]:.1f}</b> mm  "
                f"<span foreground='#80ff80'>✓ alineado</span>"
                if xyz is not None else
                f"<b>RGB click</b> ({px},{py})  BGR=({b},{g},{r})  → sin depth en ese pixel"
            )
        else:
            self._lbl_coords.set_text(
                f"RGB click ({px},{py})  BGR=({b},{g},{r})  → sin depth en ese pixel"
            )

    # ── Mouse en panel derecho ───────────────────────────────────────────────

    def _on_right_press(self, widget, event):
        if event.button != 1:
            return
        if self._mode == self.MODE_CLOUD:
            self._drag = (event.x, event.y,
                          self._cloud.azimuth, self._cloud.elevation)
        else:
            px, py = self._widget_to_img(widget, event.x, event.y)
            xyz    = self._xyz_at(px, py)
            with self._lock:
                self._depth_click_px  = (px, py)
                self._depth_click_xyz = xyz
            if xyz is not None:
                self._lbl_coords.set_markup(
                    f"<b>Depth click</b> ({px},{py})  →  "
                    f"x=<b>{xyz[0]:.1f}</b>  y=<b>{xyz[1]:.1f}</b>  z=<b>{xyz[2]:.1f}</b> mm"
                )
            else:
                self._lbl_coords.set_text(f"Depth click ({px},{py}) → sin dato 3D")

    def _on_right_release(self, widget, event):
        self._drag = None

    def _on_right_motion(self, widget, event):
        if self._mode == self.MODE_CLOUD and self._drag is not None:
            dx = event.x - self._drag[0]
            dy = event.y - self._drag[1]
            self._cloud.azimuth   = self._drag[2] + dx * 0.4
            self._cloud.elevation = float(np.clip(self._drag[3] - dy * 0.4, -89, 89))
        elif self._mode == self.MODE_DEPTH:
            px, py = self._widget_to_img(widget, event.x, event.y)
            xyz    = self._xyz_at(px, py)
            if xyz is not None:
                self._lbl_coords.set_markup(
                    f"<small>Hover ({px},{py})  →  "
                    f"x={xyz[0]:.1f}  y={xyz[1]:.1f}  z={xyz[2]:.1f} mm</small>"
                )

    def _on_scroll(self, widget, event):
        if self._mode != self.MODE_CLOUD:
            return
        if event.direction == Gdk.ScrollDirection.UP:
            self._cloud.zoom(1.12)
        elif event.direction == Gdk.ScrollDirection.DOWN:
            self._cloud.zoom(1 / 1.12)
        # Smooth scroll (trackpad)
        elif event.direction == Gdk.ScrollDirection.SMOOTH:
            _, dy = event.get_scroll_deltas()
            if dy < 0:
                self._cloud.zoom(1.06)
            elif dy > 0:
                self._cloud.zoom(1 / 1.06)

    # ── Timer GTK (hilo principal) ───────────────────────────────────────────

    def _tick(self):
        """Refresca ambos panels a 30fps desde el hilo GTK."""
        self._da_rgb.queue_draw()
        self._da_right.queue_draw()
        return self._running   # False detiene el timer

    # ── Botón Simular Pick → PyBullet ────────────────────────────────────────

    def _on_simulate(self, *_):
        with self._lock:
            ball = self._ball

        if not ball or ball["xyz"] is None:
            self._lbl_status.set_text("Sin pelota detectada — apunta la cámara a la pelota")
            return

        # Guardar datos de la pelota en JSON
        out = ROOT / "captures"
        out.mkdir(exist_ok=True)
        data = {
            "xyz_mm":    ball["xyz"].tolist(),
            "width_mm":  ball["width_mm"],
            "height_mm": ball["height_mm"],
        }
        json_path = out / "ball_latest.json"
        json_path.write_text(json.dumps(data, indent=2))
        print(f"[sim] Guardado: {json_path}")
        print(f"[sim] xyz_mm={data['xyz_mm']}  w={data['width_mm']}  h={data['height_mm']}")

        # Lanzar ball_sim.py con python3 normal (PyBullet en ~/.local).
        # IMPORTANTE: eliminar PYTHONNOUSERSITE heredado del proceso padre,
        # de lo contrario el hijo tampoco encuentra ~/.local/lib/pybullet.
        sim_script = ROOT / "examples" / "ball_sim.py"
        child_env  = os.environ.copy()
        child_env.pop("PYTHONNOUSERSITE", None)
        proc = subprocess.Popen(
            ["/usr/bin/python3", str(sim_script)],
            env=child_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self._lbl_status.set_markup(
            f"<b>Simulando en PyBullet</b>  |  "
            f"xyz=({data['xyz_mm'][0]:.0f}, {data['xyz_mm'][1]:.0f}, {data['xyz_mm'][2]:.0f}) mm  |  "
            f"diámetro={data['width_mm']:.0f}mm"
        )

        # Hilo para capturar output de PyBullet sin bloquear la UI
        def _watch():
            for line in proc.stdout:
                print(f"[bullet] {line}", end="")
            proc.wait()
            GLib.idle_add(
                self._lbl_status.set_text,
                f"PyBullet terminó (código {proc.returncode})"
            )

        threading.Thread(target=_watch, daemon=True).start()

    # ── Botón guardar ────────────────────────────────────────────────────────

    def _on_save(self, *_):
        with self._lock:
            rgb_s   = self._rgb_frame.copy()   if self._rgb_frame   is not None else None
            right_s = self._right_frame.copy() if self._right_frame is not None else None
        if rgb_s is None:
            return
        ts  = int(time.time())
        out = ROOT / "captures"
        out.mkdir(exist_ok=True)
        cv2.imwrite(str(out / f"ball_{ts}_rgb.png"),   rgb_s)
        if right_s is not None:
            suffix = "cloud" if self._mode == self.MODE_CLOUD else "depth"
            cv2.imwrite(str(out / f"ball_{ts}_{suffix}.png"), right_s)
        self._lbl_status.set_text(f"Guardado → captures/ball_{ts}_*.png")

    # ── Salir ────────────────────────────────────────────────────────────────

    def _quit(self, *_):
        self._running = False
        Gtk.main_quit()

    # ── Hilo de cámara ───────────────────────────────────────────────────────

    def _camera_loop(self):
        fps_count = 0
        fps_time  = time.time()

        try:
            with ScepterCamera(self.ip) as cam:
                while self._running:
                    bgr, depth_mm, xyz_mm = cam.get_frames()
                    if bgr is None or depth_mm is None or xyz_mm is None:
                        continue

                    fps_count += 1
                    now = time.time()
                    if now - fps_time >= 1.0:
                        fps = fps_count
                        fps_count = 0
                        fps_time  = now
                        with self._lock:
                            self._fps = fps

                    # Detección
                    raw  = detect_orange(bgr, xyz_mm)
                    self._tracker.update(raw)
                    ball = self._tracker.result

                    # Panel derecho según modo
                    mode = self._mode
                    if mode == self.MODE_DEPTH:
                        right = self._colorizer(depth_mm)
                    else:
                        right = self._cloud.render(xyz_mm)

                    with self._lock:
                        self._rgb_frame   = bgr
                        self._right_frame = right
                        self._ball        = ball
                        self._xyz_mm      = xyz_mm

                    # Actualizar status desde GTK thread
                    GLib.idle_add(self._update_status, ball)

        except Exception as e:
            GLib.idle_add(self._lbl_status.set_text, f"Error cámara: {e}")

    def _update_status(self, ball):
        with self._lock:
            fps = self._fps

        has_3d = ball is not None and ball.get("xyz") is not None
        self._btn_sim.set_sensitive(has_3d)

        if ball:
            xyz = ball["xyz"]
            wm  = ball["width_mm"]
            hm  = ball["height_mm"]
            w_s = f"{wm:.0f}" if wm else "---"
            h_s = f"{hm:.0f}" if hm else "---"
            if xyz is not None:
                self._lbl_status.set_markup(
                    f"<b>PELOTA</b>  "
                    f"x=<b>{xyz[0]:.0f}</b>  y=<b>{xyz[1]:.0f}</b>  z=<b>{xyz[2]:.0f}</b> mm  |  "
                    f"ancho=<b>{w_s}</b>mm  alto=<b>{h_s}</b>mm  |  "
                    f"{fps}fps  —  <i>Botón 'Simular Pick' disponible</i>"
                )
            else:
                self._lbl_status.set_text(
                    f"Pelota detectada (sin depth)  ancho={w_s} alto={h_s}mm  {fps}fps"
                )
        else:
            self._lbl_status.set_text(f"Buscando pelota naranja...  {fps}fps")
        return False


# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.101"
    BallViewerWindow(ip)
    Gtk.main()


if __name__ == "__main__":
    main()
