#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BanaPick — Live operator panel (GTK3 / PyGObject)
Industrial robotic banana-hand smart picking platform.

Target: Jetson Orin NX touchscreen, 1920x1080.

Run:
    sudo apt install python3-gi gir1.2-gtk-3.0   # Debian/Ubuntu/L4T
    python3 banapick.py
    # F11 toggles fullscreen, Esc quits.

This recreates the Live tab: left sidebar (7 tabs), top bar, 3-column main
area (camera feed / cell-flow stepper / box fill) and a bottom metrics bar.
Widgets are styled with GTK3 CSS; the camera scene, success gauge, throughput
sparkline and box schematic are custom-drawn with Cairo.
"""

import math
import os
import site
import sys
import threading
import time
from pathlib import Path

if not os.environ.get("SCEPTER_ALLOW_USER_SITE"):
    sys.path = [p for p in sys.path if p != site.getusersitepackages()]

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gdk, GLib, Pango  # noqa: E402
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = ROOT / "examples"
sys.path.insert(0, str(EXAMPLES))

try:
    from ball_viewer import (  # noqa: E402
        CloudRenderer,
        DepthColorizer,
        ScepterCamera,
        bgr_to_pixbuf,
        detect_orange,
        draw_ball_depth,
        draw_ball_rgb,
        fit_pixbuf,
    )
    CAMERA_TOOLS_ERROR = None
except Exception as exc:  # pragma: no cover - depends on Jetson camera stack
    CloudRenderer = None
    DepthColorizer = None
    ScepterCamera = None
    bgr_to_pixbuf = None
    detect_orange = None
    draw_ball_depth = None
    draw_ball_rgb = None
    fit_pixbuf = None
    CAMERA_TOOLS_ERROR = exc

# ---------------------------------------------------------------------------
# Palette (matches the HTML spec)
# ---------------------------------------------------------------------------
TEAL       = (0x1D/255, 0x9E/255, 0x75/255)
TEAL_BR    = (0x3B/255, 0xD8/255, 0x9C/255)
AMBER      = (0xBA/255, 0x75/255, 0x17/255)
AMBER_BR   = (0xE0/255, 0xA3/255, 0x3D/255)
RED        = (0xDC/255, 0x26/255, 0x26/255)
ORANGE     = (0xF0/255, 0x8C/255, 0x2E/255)
GRID       = (0x2C/255, 0x37/255, 0x42/255)
BANANA     = (0xC2/255, 0xCB/255, 0x46/255)
WHITE      = (1, 1, 1)
DIM        = (0x6B/255, 0x75/255, 0x85/255)

CSS = b"""
* { font-family: "Inter", "Cantarell", sans-serif; }

.app-bg        { background-color: #0A0D12; }

/* ---- sidebar ---- */
.sidebar       { background-color: #0E1218; border-right: 1px solid #1C232D; }
.logo-box      { background-color: #13241E; border: 1px solid #1D9E75;
                 border-radius: 8px; }
.navbtn        { background: none; border: 1px solid transparent;
                 border-radius: 8px; padding: 8px 0; }
.navbtn:hover  { background-color: #141A21; }
.navbtn.active { background-color: #13241E; border: 1px solid #1D9E75; }
.nav-label     { color: #6B7585; font-size: 11px; font-weight: bold; }
.navbtn.active .nav-label { color: #3BD89C; }

/* ---- top bar ---- */
.topbar        { background-color: #0E1218; border-bottom: 1px solid #1C232D; }
.logo          { color: #E6EAF0; font-size: 22px; font-weight: bold; }
.logo-accent   { color: #1D9E75; font-size: 22px; font-weight: bold; }
.cell-id       { color: #6B7585; font-size: 11px; font-weight: bold; }
.pill-run      { background-color: #10271E; border: 1px solid #1D9E75;
                 border-radius: 8px; padding: 6px 12px;
                 color: #3BD89C; font-size: 13px; font-weight: bold; }
.dot-run       { background-color: #22C58B; border-radius: 8px; }
.pill-stop     { background-color: #2A1011; border: 1px solid #DC2626;
                 border-radius: 8px; padding: 6px 12px;
                 color: #F0726A; font-size: 13px; font-weight: bold; }
.dot-stop      { background-color: #DC2626; border-radius: 8px; }
.time-pill     { background-color: #13191F; border: 1px solid #232C37;
                 border-radius: 8px; padding: 6px 12px;
                 color: #C7CFDA; font-size: 15px; font-weight: bold;
                 font-family: monospace; }
.estop         { background-color: #DC2626; border: 1px solid #F0594E;
                 border-radius: 8px; padding: 8px 18px;
                 color: #ffffff; font-size: 14px; font-weight: bold; }
.estop:hover   { background-color: #E83b30; }
.reconnect     { background-color: #10271E; border: 1px solid #1D9E75;
                 border-radius: 8px; padding: 8px 14px;
                 color: #3BD89C; font-size: 14px; font-weight: bold; }
.reconnect:hover { background-color: #143326; }

/* ---- cards ---- */
.card          { background-color: #0E1218; border: 1px solid #1C232D;
                 border-radius: 8px; box-shadow: 0 10px 24px rgba(0,0,0,.22); }
.cam-card      { background-color: #0C0F14; border: 1px solid #1C232D;
                 border-radius: 8px; box-shadow: 0 10px 24px rgba(0,0,0,.22); }
.section-title { color: #C7CFDA; font-size: 14px; font-weight: bold; }
.muted         { color: #6B7585; font-size: 11px; font-weight: bold;
                 font-family: monospace; }

/* ---- stepper ---- */
.step-title    { color: #C7CFDA; font-size: 14px; font-weight: bold; }
.step-title.active { color: #ffffff; }
.step-title.alarm  { color: #E0A33D; }
.step-title.idle   { color: #8A94A3; }
.step-sub      { color: #6B7585; font-size: 11px; }
.dot           { border-radius: 8px; font-size: 14px; font-weight: bold; }
.dot-done      { background-color: #10271E; border: 1px solid #1D9E75;
                 color: #3BD89C; }
.dot-active    { background-color: #1D9E75; border: 1px solid #3BD89C;
                 color: #06120D; }
.dot-alarm     { background-color: #2A1F0C; border: 1px solid #BA7517;
                 color: #E0A33D; }
.dot-idle      { background-color: #13191F; border: 1px solid #2E3946;
                 color: #6B7585; }
.tag           { border-radius: 5px; font-size: 10px; font-weight: bold;
                 padding: 1px 7px; }
.tag-done      { background-color: #10271E; color: #3BD89C; }
.tag-active    { background-color: #1D9E75; color: #06120D; }
.tag-alarm     { background-color: #2A1F0C; color: #E0A33D; }
.current-panel { background-color: #10271E; border: 1px solid #1D9E75;
                 border-radius: 8px; padding: 12px; }
.current-label { color: #3BD89C; font-size: 11px; font-weight: bold; }
.current-step  { color: #ffffff; font-size: 18px; font-weight: bold; }
.current-time  { color: #C7CFDA; font-size: 12px; font-weight: bold;
                 font-family: monospace; }

/* ---- buttons ---- */
.btn           { background-color: #13191F; border: 1px solid #2E3946;
                 border-radius: 8px; padding: 12px;
                 color: #C7CFDA; font-size: 14px; font-weight: bold; }
.btn:hover     { background-color: #182029; }
.btn-rgb       { background-color: #10271E; border: 1px solid #1D9E75;
                 border-radius: 8px; padding: 11px;
                 color: #3BD89C; font-size: 13px; font-weight: bold; }

/* ---- box fill ---- */
.box-frame     { background-color: #0C0F14; border: 1px solid #232C37;
                 border-radius: 8px; }
.fill-num      { color: #ffffff; font-size: 28px; font-weight: bold;
                 font-family: monospace; }
.fill-den      { color: #6B7585; font-size: 18px; font-weight: bold;
                 font-family: monospace; }
.alert-amber   { background-color: #2A1F0C; border: 1px solid #BA7517;
                 border-radius: 8px; padding: 10px 14px; }
.alert-title   { color: #E0A33D; font-size: 13px; font-weight: bold; }
.alert-sub     { color: #9A8650; font-size: 11px; }

/* ---- metrics ---- */
.metric-label  { color: #9AA4B2; font-size: 12px; font-weight: bold; }
.metric-value  { color: #ffffff; font-size: 32px; font-weight: bold;
                 font-family: monospace; }
.metric-unit   { color: #6B7585; font-size: 13px; font-weight: bold; }
.up            { color: #3BD89C; font-size: 12px; font-weight: bold; }
.badge-reject  { background-color: #2A1011; border: 1px solid #DC2626;
                 border-radius: 8px; padding: 4px 11px;
                 color: #F0726A; font-size: 11px; font-weight: bold; }
.mini-label    { color: #6B7585; font-size: 11px; font-weight: bold; }
.mini-val      { color: #C7CFDA; font-size: 16px; font-weight: bold;
                 font-family: monospace; }
.page-title    { color: #E6EAF0; font-size: 24px; font-weight: bold; }
.page-sub      { color: #8A94A3; font-size: 13px; }
.row-label     { color: #9AA4B2; font-size: 12px; font-weight: bold; }
.row-value     { color: #E6EAF0; font-size: 15px; font-weight: bold;
                 font-family: monospace; }
.status-ok     { color: #3BD89C; font-size: 13px; font-weight: bold; }
.status-warn   { color: #E0A33D; font-size: 13px; font-weight: bold; }
.camera-note   { color: #8A94A3; font-size: 12px; }
"""


# ---------------------------------------------------------------------------
# Cairo helpers
# ---------------------------------------------------------------------------
def rounded_rect(cr, x, y, w, h, r):
    cr.new_sub_path()
    cr.arc(x + w - r, y + r, r, -math.pi / 2, 0)
    cr.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
    cr.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
    cr.arc(x + r, y + r, r, math.pi, 1.5 * math.pi)
    cr.close_path()


def banana_glyph(cr, cx, cy, s, color):
    """Draw a small stylised banana hand."""
    cr.save()
    cr.translate(cx, cy)
    cr.scale(s, s)
    cr.set_source_rgb(*color)
    cr.set_line_width(0.22)
    cr.set_line_cap(1)  # round
    for off in (-0.18, 0.0, 0.18):
        cr.new_path()
        cr.move_to(-0.45 + off, -0.5)
        cr.curve_to(-0.1 + off, -0.55, 0.35 + off, -0.1, 0.4 + off, 0.55)
        cr.stroke()
    cr.restore()


# ---------------------------------------------------------------------------
# Custom drawing areas
# ---------------------------------------------------------------------------
class CameraView(Gtk.DrawingArea):
    def __init__(self):
        super().__init__()
        self.frame = None
        self.message = "Waiting for camera stream..."
        self.connect("draw", self.on_draw)

    def set_frame(self, frame):
        self.frame = frame
        self.queue_draw()

    def set_message(self, message):
        self.message = message
        self.frame = None
        self.queue_draw()

    def on_draw(self, _w, cr):
        a = self.get_allocation()
        W, H = a.width, a.height

        if self.frame is not None and bgr_to_pixbuf is not None:
            pb = bgr_to_pixbuf(self.frame)
            Gdk.cairo_set_source_pixbuf(cr, fit_pixbuf(pb, W, H), 0, 0)
            cr.paint()
            return False

        # background radial
        grad = __import__("cairo").RadialGradient(W * .5, H * .42, 0,
                                                  W * .5, H * .42, W * .75)
        grad.add_color_stop_rgb(0, 0x1A/255, 0x26/255, 0x30/255)
        grad.add_color_stop_rgb(.6, 0x10/255, 0x17/255, 0x1E/255)
        grad.add_color_stop_rgb(1, 0x08/255, 0x0B/255, 0x0F/255)
        cr.set_source(grad)
        cr.rectangle(0, 0, W, H)
        cr.fill()

        if self.message:
            cr.select_font_face("sans-serif", 0, 1)
            cr.set_font_size(18)
            cr.set_source_rgb(0x8A/255, 0x94/255, 0xA3/255)
            ext = cr.text_extents(self.message)
            cr.move_to(max(16, W / 2 - ext.width / 2), max(40, H * .12))
            cr.show_text(self.message[:80])

        sx, sy = W / 960.0, H / 620.0

        def P(x, y):
            return x * sx, y * sy

        # conveyor
        cr.set_source_rgb(0x20/255, 0x29/255, 0x30/255)
        cr.move_to(*P(120, 620)); cr.line_to(*P(380, 300))
        cr.line_to(*P(580, 300)); cr.line_to(*P(860, 620))
        cr.close_path(); cr.fill()
        cr.set_source_rgb(*GRID); cr.set_line_width(1.5)
        for ln in [(180, 540, 420, 320), (780, 540, 540, 320),
                   (250, 470, 710, 470), (300, 400, 660, 400)]:
            cr.move_to(*P(ln[0], ln[1])); cr.line_to(*P(ln[2], ln[3])); cr.stroke()

        # banana hands
        banana_glyph(cr, *P(415, 360), 70 * sx, BANANA)
        banana_glyph(cr, *P(515, 455), 60 * sx, BANANA)

        # detection 1
        self._detection(cr, P, 320, 300, 190, 180, (344, 304), (470, 468),
                        "HAND 0.96")
        # detection 2
        self._detection(cr, P, 452, 392, 150, 148, (486, 398), (566, 528),
                        "HAND 0.88")

    def _detection(self, cr, P, bx, by, bw, bh, crown, tip, label):
        x, y = P(bx, by)
        x2, _ = P(bx + bw, by)
        _, y2 = P(bx, by + bh)
        w, h = x2 - x, y2 - y
        # box
        cr.set_source_rgba(*TEAL, 0.06)
        rounded_rect(cr, x, y, w, h, 6); cr.fill()
        cr.set_source_rgb(*TEAL); cr.set_line_width(2.4)
        rounded_rect(cr, x, y, w, h, 6); cr.stroke()
        # keypoint line
        kx, ky = P(*crown); tx, ty = P(*tip)
        cr.set_source_rgba(1, 1, 1, .5); cr.set_line_width(1.4)
        cr.set_dash([4, 3]); cr.move_to(kx, ky); cr.line_to(tx, ty)
        cr.stroke(); cr.set_dash([])
        # crown (teal)
        cr.set_source_rgb(*TEAL); cr.arc(kx, ky, 7, 0, 2*math.pi); cr.fill()
        cr.set_source_rgb(0x08/255, 0x11/255, 0x0D/255); cr.set_line_width(2)
        cr.arc(kx, ky, 7, 0, 2*math.pi); cr.stroke()
        # tip (orange)
        cr.set_source_rgb(*ORANGE); cr.arc(tx, ty, 7, 0, 2*math.pi); cr.fill()
        cr.set_source_rgb(0x1A/255, 0x0E/255, 0x04/255); cr.set_line_width(2)
        cr.arc(tx, ty, 7, 0, 2*math.pi); cr.stroke()
        # confidence badge
        bgx, bgy = x, y - 22
        cr.set_source_rgb(0x0B/255, 0x1F/255, 0x18/255)
        rounded_rect(cr, bgx, bgy, 92, 19, 4); cr.fill()
        cr.set_source_rgb(*TEAL); cr.set_line_width(1)
        rounded_rect(cr, bgx, bgy, 92, 19, 4); cr.stroke()
        cr.select_font_face("monospace", 0, 1)
        cr.set_font_size(12); cr.set_source_rgb(*TEAL_BR)
        cr.move_to(bgx + 8, bgy + 14); cr.show_text(label)


class GaugeView(Gtk.DrawingArea):
    def __init__(self, value):
        super().__init__()
        self.value = value
        self.set_size_request(82, 82)
        self.connect("draw", self.on_draw)

    def on_draw(self, _w, cr):
        a = self.get_allocation()
        cx, cy = a.width / 2, a.height / 2
        r = min(cx, cy) - 6
        cr.set_line_width(9); cr.set_line_cap(1)
        cr.set_source_rgb(0x1A/255, 0x22/255, 0x2B/255)
        cr.arc(cx, cy, r, 0, 2 * math.pi); cr.stroke()
        cr.set_source_rgb(*TEAL)
        start = -math.pi / 2
        cr.arc(cx, cy, r, start, start + 2 * math.pi * self.value / 100.0)
        cr.stroke()
        cr.select_font_face("monospace", 0, 1); cr.set_font_size(16)
        cr.set_source_rgb(*WHITE)
        txt = "%d%%" % round(self.value)
        ext = cr.text_extents(txt)
        cr.move_to(cx - ext.width / 2, cy + ext.height / 2)
        cr.show_text(txt)


class SparkView(Gtk.DrawingArea):
    PTS = [34, 30, 32, 22, 26, 18, 24, 12, 16, 8, 14]

    def __init__(self):
        super().__init__()
        self.set_size_request(150, 46)
        self.connect("draw", self.on_draw)

    def on_draw(self, _w, cr):
        a = self.get_allocation()
        W, H = a.width, a.height
        n = len(self.PTS)
        xs = [W * i / (n - 1) for i in range(n)]
        ys = [H * (p / 46.0) for p in self.PTS]
        # fill
        cr.move_to(xs[0], ys[0])
        for i in range(1, n):
            cr.line_to(xs[i], ys[i])
        cr.line_to(xs[-1], H); cr.line_to(xs[0], H); cr.close_path()
        cr.set_source_rgba(*TEAL, 0.22); cr.fill()
        # line
        cr.set_source_rgb(*TEAL); cr.set_line_width(2); cr.set_line_join(1)
        cr.move_to(xs[0], ys[0])
        for i in range(1, n):
            cr.line_to(xs[i], ys[i])
        cr.stroke()
        cr.set_source_rgb(*TEAL_BR)
        cr.arc(xs[-2], ys[-2], 3, 0, 2 * math.pi); cr.fill()


class BoxView(Gtk.DrawingArea):
    def __init__(self, filled, total, cols=4, rows=3):
        super().__init__()
        self.filled, self.total = filled, total
        self.cols, self.rows = cols, rows
        self.set_size_request(0, 220)
        self.connect("draw", self.on_draw)

    def on_draw(self, _w, cr):
        a = self.get_allocation()
        W, H = a.width, a.height
        gap = 9
        cw = (W - gap * (self.cols - 1)) / self.cols
        ch = (H - gap * (self.rows - 1)) / self.rows
        i = 0
        for rr in range(self.rows):
            for cc in range(self.cols):
                x = cc * (cw + gap)
                y = rr * (ch + gap)
                fill = i < self.filled
                if fill:
                    cr.set_source_rgb(0x10/255, 0x27/255, 0x1E/255)
                else:
                    cr.set_source_rgb(0x11/255, 0x16/255, 0x1C/255)
                rounded_rect(cr, x, y, cw, ch, 8); cr.fill()
                cr.set_line_width(1)
                cr.set_source_rgb(*TEAL) if fill else cr.set_source_rgb(
                    0x23/255, 0x2C/255, 0x37/255)
                rounded_rect(cr, x, y, cw, ch, 8); cr.stroke()
                if fill:
                    banana_glyph(cr, x + cw/2, y + ch/2, min(cw, ch)*0.5, TEAL)
                i += 1


# ---------------------------------------------------------------------------
# Small widget builders
# ---------------------------------------------------------------------------
def lbl(text, css=None, xalign=0.0):
    w = Gtk.Label(label=text, xalign=xalign)
    if css:
        w.get_style_context().add_class(css)
    return w


def icon(name, px, css="nav-icon"):
    img = Gtk.Image.new_from_icon_name(name, Gtk.IconSize.DIALOG)
    img.set_pixel_size(px)
    return img


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------
class BanaPickLiveWindow(Gtk.Window):
    TABS = [
        ("Live",       "camera-video-symbolic"),
        ("Cameras",    "camera-photo-symbolic"),
        ("Calibration", "find-location-symbolic"),
        ("Detection",  "applications-science-symbolic"),
        ("Robot",      "applications-engineering-symbolic"),
        ("Routing",    "network-workgroup-symbolic"),
        ("Log / DB",   "drive-harddisk-symbolic"),
    ]
    STEPS = [
        ("Detect hand",          "YOLO-pose · 0.96 conf",  "done"),
        ("Verify stem",          "orientation: STEM DOWN", "alarm"),
        ("Grasp by stem",        "closing gripper · 18N",  "active"),
        ("Verify grasp",         "load-cell confirm",      "idle"),
        ("Rotate + Inspect",     "360 vision sweep",       "idle"),
        ("Classify destination", "grade A / B / reject",   "idle"),
    ]

    def __init__(self, camera_ip="192.168.1.101"):
        super().__init__(title="BanaPick — Live")
        self.camera_ip = camera_ip
        self._running = True
        self._camera_running = True
        self._camera_thread = None
        self._camera_generation = 0
        self._depth_mode = "depth"
        self._nav_buttons = []
        self._colorizer = DepthColorizer() if DepthColorizer else None
        self._cloud = CloudRenderer() if CloudRenderer else None
        self._rgb_view = None
        self._depth_view = None
        self._cloud_view = None
        self._camera_stack = None
        self._rgb_hdr = None
        self._depth_hdr = None
        self._cloud_hdr = None
        self._depth_button = None
        self._point_scale = None
        self._point_label = None
        self._cloud_drag = None
        self._coord_label = None
        self._model_status_label = None
        self._part_status_label = None
        self._latest_xyz_mm = None
        self._latest_rgb = None

        self.set_default_size(1920, 1080)
        self.get_style_context().add_class("app-bg")
        self.connect("destroy", self._quit)
        self.connect("key-press-event", self._keys)

        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        self.add(root)
        root.pack_start(self._sidebar(), False, False, 0)

        main = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        root.pack_start(main, True, True, 0)
        main.pack_start(self._topbar(), False, False, 0)
        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        self.stack.set_transition_duration(180)
        self.stack.add_named(self._live_page(), "Live")
        for name, ic in self.TABS[1:]:
            self.stack.add_named(self._placeholder_page(name, ic), name)
        main.pack_start(self.stack, True, True, 0)

        GLib.timeout_add_seconds(1, self._tick)
        self._start_camera()

    # -- sidebar -----------------------------------------------------------
    def _sidebar(self):
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        box.get_style_context().add_class("sidebar")
        box.set_size_request(108, -1)
        box.set_border_width(14)

        logo = Gtk.Box()
        logo.get_style_context().add_class("logo-box")
        logo.set_size_request(48, 48)
        logo.set_halign(Gtk.Align.CENTER)
        lg = icon("appointment-soon-symbolic", 26)
        logo.set_center_widget(lg)
        box.pack_start(logo, False, False, 0)
        box.pack_start(Gtk.Box(), False, False, 8)

        for i, (name, ic) in enumerate(self.TABS):
            btn = Gtk.Button()
            btn.get_style_context().add_class("navbtn")
            if i == 0:
                btn.get_style_context().add_class("active")
            btn.connect("clicked", self._switch_tab, i, name)
            inner = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
            inner.pack_start(icon(ic, 24), False, False, 0)
            inner.pack_start(lbl(name, "nav-label", 0.5), False, False, 0)
            btn.add(inner)
            self._nav_buttons.append(btn)
            box.pack_start(btn, False, False, 3)
        return box

    # -- top bar -----------------------------------------------------------
    def _topbar(self):
        bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=16)
        bar.get_style_context().add_class("topbar")
        bar.set_size_request(-1, 68)
        bar.set_border_width(14)

        logo = Gtk.Box(spacing=8)
        logo.set_valign(Gtk.Align.CENTER)
        logo.pack_start(lbl("Bana", "logo"), False, False, 0)
        l2 = lbl("Pick", "logo-accent"); logo.pack_start(l2, False, False, 0)
        logo.pack_start(lbl("CELL A-03", "cell-id"), False, False, 6)
        bar.pack_start(logo, False, False, 6)

        self.status_pill = Gtk.Box(spacing=8)
        self.status_pill.get_style_context().add_class("pill-run")
        self.status_pill.set_valign(Gtk.Align.CENTER)
        self.status_dot = Gtk.Box()
        self.status_dot.get_style_context().add_class("dot-run")
        self.status_dot.set_size_request(9, 9)
        self.status_dot.set_valign(Gtk.Align.CENTER)
        self.status_text = lbl("RUNNING")
        self.status_pill.pack_start(self.status_dot, False, False, 0)
        self.status_pill.pack_start(self.status_text, False, False, 0)
        bar.pack_start(self.status_pill, False, False, 0)

        bar.pack_start(Gtk.Box(), True, True, 0)  # spacer

        self.clock = lbl(time.strftime("%H:%M:%S"), "time-pill")
        self.clock.set_valign(Gtk.Align.CENTER)
        bar.pack_start(self.clock, False, False, 0)

        estop = Gtk.Button(label="⏻  E-STOP")
        estop.get_style_context().add_class("estop")
        estop.set_valign(Gtk.Align.CENTER)
        estop.connect("clicked", self._emergency_stop)
        bar.pack_start(estop, False, False, 0)

        reconnect = Gtk.Button(label="↻  Reconnect")
        reconnect.get_style_context().add_class("reconnect")
        reconnect.set_valign(Gtk.Align.CENTER)
        reconnect.connect("clicked", self._reconnect_camera)
        bar.pack_start(reconnect, False, False, 0)
        return bar

    # -- pages -------------------------------------------------------------
    def _live_page(self):
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        page.pack_start(self._content(), True, True, 0)
        page.pack_start(self._metrics(), False, False, 0)
        return page

    def _placeholder_page(self, name, icon_name):
        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=18)
        page.set_border_width(18)

        header = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=14)
        header.pack_start(icon(icon_name, 34), False, False, 0)
        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        title_box.pack_start(lbl(name, "page-title"), False, False, 0)
        title_box.pack_start(lbl("Operator controls", "page-sub"), False, False, 0)
        header.pack_start(title_box, False, False, 0)
        page.pack_start(header, False, False, 0)

        grid = Gtk.Grid(column_spacing=18, row_spacing=18)
        grid.set_column_homogeneous(True)
        page.pack_start(grid, True, True, 0)

        for idx, card in enumerate(self._tab_cards(name)):
            grid.attach(card, idx % 2, idx // 2, 1, 1)
        return page

    def _tab_cards(self, name):
        data = {
            "Cameras": [
                ("PRIMARY CAMERA", [("Model", "Scepter ToF"), ("IP", self.camera_ip), ("Stream", "RGB-D")], "ONLINE"),
                ("DEPTH PIPELINE", [("Alignment", "Color sensor"), ("Point cloud", "Enabled"), ("FPS target", "30")], "READY"),
                ("CAPTURE", [("RGB", "800x600"), ("Depth", "mm"), ("XYZ", "matrix")], "ARMED"),
            ],
            "Calibration": [
                ("HAND-EYE", [("Profile", "CELL A-03"), ("Matrix", "Loaded"), ("Residual", "1.8 mm")], "READY"),
                ("DEPTH CHECK", [("Plane fit", "10 mm"), ("Intrinsics", "TOF sensor"), ("Offset", "50 mm")], "OK"),
                ("WORKSPACE", [("Min Z", "50 mm"), ("Max Z", "4000 mm"), ("Frame", "camera")], "LOCKED"),
            ],
            "Detection": [
                ("AI MODEL", [("Detector", "banana-hand pose"), ("Crown/tip", "2 keypoints"), ("Confidence", "0.88")], "RUNNING"),
                ("SEGMENTATION", [("Plane", "RANSAC"), ("Clusters", "DBSCAN"), ("Rejects", "3 today")], "ACTIVE"),
                ("QUALITY", [("Stem up", "green"), ("Stem down", "amber"), ("Bruising", "watch")], "WATCH"),
            ],
            "Robot": [
                ("ROBOT LINK", [("Mode", "TCP"), ("Host", "127.0.0.1"), ("Port", "5005")], "READY"),
                ("GRIPPER", [("Force", "18 N"), ("State", "open"), ("Cycle", "#4,182")], "IDLE"),
                ("SAFETY", [("E-stop", "armed"), ("Zone", "clear"), ("Manual", "enabled")], "OK"),
            ],
            "Routing": [
                ("DESTINATIONS", [("Grade A", "Box 1"), ("Grade B", "Box 2"), ("Reject", "Bin R")], "READY"),
                ("CURRENT ROUTE", [("Class", "grade A"), ("Cavity", "8 / 12"), ("Next", "Cavity 9")], "ACTIVE"),
                ("CONVEYOR", [("Lane", "L1"), ("Speed", "0.42 m/s"), ("Queue", "3 hands")], "RUNNING"),
            ],
            "Log / DB": [
                ("SHIFT LOG", [("Cycles", "4,182"), ("Throughput", "12/min"), ("Success", "94.2%")], "SYNCED"),
                ("EVENTS", [("Warnings", "1"), ("Rejects", "3"), ("Last", "Stem down")], "LIVE"),
                ("DATABASE", [("Backend", "local"), ("Writes", "enabled"), ("Retention", "30 days")], "OK"),
            ],
        }
        return [self._info_card(*spec) for spec in data.get(name, [])]

    def _info_card(self, title, rows, status):
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        card.get_style_context().add_class("card")
        card.set_border_width(18)
        head = Gtk.Box()
        head.pack_start(lbl(title, "section-title"), False, False, 0)
        head.pack_start(Gtk.Box(), True, True, 0)
        head.pack_start(lbl(status, "status-ok" if status in ("OK", "READY", "ONLINE", "RUNNING", "SYNCED", "ACTIVE") else "status-warn"), False, False, 0)
        card.pack_start(head, False, False, 0)
        for key, value in rows:
            row = Gtk.Box()
            row.pack_start(lbl(key, "row-label"), False, False, 0)
            row.pack_start(Gtk.Box(), True, True, 0)
            row.pack_start(lbl(value, "row-value"), False, False, 0)
            card.pack_start(row, False, False, 0)
        return card

    def _switch_tab(self, _button, index, name):
        for i, btn in enumerate(self._nav_buttons):
            ctx = btn.get_style_context()
            if i == index:
                ctx.add_class("active")
            else:
                ctx.remove_class("active")
        self.stack.set_visible_child_name(name)

    # -- main content ------------------------------------------------------
    def _content(self):
        wrap = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=18)
        wrap.set_border_width(18)
        wrap.pack_start(self._camera_col(), True, True, 0)
        wrap.pack_start(self._center_col(), False, False, 0)
        wrap.pack_start(self._box_col(), False, False, 0)
        return wrap

    def _center_col(self):
        col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        col.set_size_request(372, -1)
        col.pack_start(self._model_card(), False, False, 0)
        col.pack_start(self._stepper_col(), True, True, 0)
        return col

    def _model_card(self):
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        card.get_style_context().add_class("card")
        card.set_border_width(14)

        head = Gtk.Box()
        head.pack_start(lbl("DETECTION MODEL", "section-title"), False, False, 0)
        head.pack_start(Gtk.Box(), True, True, 0)
        head.pack_start(lbl("ACTIVE", "status-ok"), False, False, 0)
        card.pack_start(head, False, False, 0)

        self._model_status_label = lbl("banana-hand-pose-v1", "row-value")
        self._part_status_label = lbl("target: crown point + tip point + stem orientation", "camera-note")
        card.pack_start(self._model_status_label, False, False, 0)
        card.pack_start(self._part_status_label, False, False, 0)
        return card

    def _camera_col(self):
        col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14)

        cam = Gtk.Box()
        cam.get_style_context().add_class("cam-card")
        self._camera_stack = Gtk.Stack()
        self._camera_stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self._camera_stack.set_transition_duration(160)

        views = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        views.set_border_width(10)

        self._rgb_view = CameraView()
        self._rgb_view.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self._rgb_view.connect("button-press-event", self._on_rgb_click)
        self._rgb_hdr = lbl(f"● RGB · {self.camera_ip} · CONNECTING", "muted")
        views.pack_start(
            self._camera_panel("PICK CAM RGB", self._rgb_hdr, self._rgb_view, show_stem=True),
            True, True, 0
        )

        self._depth_view = CameraView()
        self._depth_view.add_events(Gdk.EventMask.BUTTON_PRESS_MASK)
        self._depth_view.connect("button-press-event", self._on_depth_click)
        self._depth_hdr = lbl("● DEPTH · CLICK=XYZ/HEIGHT", "muted")
        views.pack_start(
            self._camera_panel("DEPTH / 3D", self._depth_hdr, self._depth_view),
            True, True, 0
        )

        self._camera_stack.add_named(views, "split")

        cloud_page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        cloud_page.set_border_width(10)
        self._cloud_view = CameraView()
        self._cloud_view.add_events(
            Gdk.EventMask.BUTTON_PRESS_MASK |
            Gdk.EventMask.BUTTON_RELEASE_MASK |
            Gdk.EventMask.POINTER_MOTION_MASK |
            Gdk.EventMask.BUTTON_MOTION_MASK |
            Gdk.EventMask.SCROLL_MASK
        )
        self._cloud_view.connect("button-press-event", self._on_cloud_press)
        self._cloud_view.connect("button-release-event", self._on_cloud_release)
        self._cloud_view.connect("motion-notify-event", self._on_cloud_motion)
        self._cloud_view.connect("scroll-event", self._on_cloud_scroll)
        self._cloud_hdr = lbl("● 3D CLOUD · DRAG=ROTATE · SCROLL=ZOOM", "muted")
        cloud_page.pack_start(
            self._camera_panel("3D POINT CLOUD", self._cloud_hdr, self._cloud_view),
            True, True, 0
        )
        self._camera_stack.add_named(cloud_page, "cloud")
        self._camera_stack.set_visible_child_name("split")

        cam.pack_start(self._camera_stack, True, True, 0)
        col.pack_start(cam, True, True, 0)

        controls = Gtk.Box(spacing=10)
        depth = Gtk.Button(label="⬡  Open 3D Large")
        depth.get_style_context().add_class("btn")
        depth.connect("clicked", self._cycle_depth_mode)
        self._depth_button = depth
        controls.pack_start(depth, False, False, 0)

        self._point_label = lbl("Points 300k", "camera-note")
        controls.pack_start(self._point_label, False, False, 0)
        self._point_scale = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 30, 500, 10)
        self._point_scale.set_value(300)
        self._point_scale.set_size_request(170, -1)
        self._point_scale.set_draw_value(False)
        self._point_scale.connect("value-changed", self._on_point_budget_changed)
        controls.pack_start(self._point_scale, False, False, 0)

        self._coord_label = lbl(
            "Click en RGB o Depth: X/Y/Z en frame de cámara. Altura = eje Y respecto al centro óptico; Z = distancia desde el sensor ToF.",
            "camera-note",
        )
        self._coord_label.set_line_wrap(True)
        controls.pack_start(self._coord_label, True, True, 0)
        col.pack_start(controls, False, False, 0)
        return col

    def _camera_panel(self, title, header_label, view, show_stem=False):
        panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        panel.set_size_request(360, 320)
        title_row = Gtk.Box()
        title_row.pack_start(lbl(title, "section-title"), False, False, 0)
        title_row.pack_start(Gtk.Box(), True, True, 0)
        title_row.pack_start(header_label, False, False, 0)
        panel.pack_start(title_row, False, False, 0)

        overlay = Gtk.Overlay()
        overlay.add(view)
        if show_stem:
            stems = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            stems.set_halign(Gtk.Align.END)
            stems.set_valign(Gtk.Align.START)
            stems.set_margin_top(10)
            stems.set_margin_end(10)
            stems.pack_start(lbl(" STEM UP ", "pill-run"), False, False, 0)
            dn = lbl(" STEM DOWN ⚠ ", "alert-title")
            dnb = Gtk.Box()
            dnb.get_style_context().add_class("alert-amber")
            dnb.add(dn)
            stems.pack_start(dnb, False, False, 0)
            overlay.add_overlay(stems)
        panel.pack_start(overlay, True, True, 0)
        return panel

    def _stepper_col(self):
        col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        col.get_style_context().add_class("card")
        col.set_size_request(372, -1)
        col.set_border_width(18)

        head = Gtk.Box()
        head.pack_start(lbl("CELL FLOW", "section-title"), False, False, 0)
        head.pack_start(Gtk.Box(), True, True, 0)
        head.pack_start(lbl("CYCLE #4,182", "muted"), False, False, 0)
        col.pack_start(head, False, False, 0)
        col.pack_start(Gtk.Box(), False, False, 9)

        for i, (title, sub, state) in enumerate(self.STEPS):
            row = Gtk.Box(spacing=14)
            # rail
            rail = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            dot_text = "✓" if state == "done" else ("!" if state == "alarm" else str(i + 1))
            dot = Gtk.Label(label=dot_text, xalign=0.5, yalign=0.5)
            dot.get_style_context().add_class("dot")
            dot.get_style_context().add_class("dot-" + ("idle" if state ==
                                              "idle" else state))
            dot.set_size_request(36, 36)
            rail.pack_start(dot, False, False, 0)
            if i < len(self.STEPS) - 1:
                line = Gtk.Box(); line.set_size_request(2, -1)
                col_line = "#1D9E75" if state == "done" else ("#BA7517" if state == "alarm" else "#1A222B")
                _tint(line, col_line)
                line.set_halign(Gtk.Align.CENTER)
                rail.pack_start(line, True, True, 4)
            row.pack_start(rail, False, False, 0)
            # content
            content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            titlerow = Gtk.Box(spacing=8)
            tcls = "active" if state == "active" else (
                "idle" if state == "idle" else ("alarm" if state == "alarm" else ""))
            tl = lbl(title, "step-title")
            if tcls:
                tl.get_style_context().add_class(tcls)
            titlerow.pack_start(tl, False, False, 0)
            if state in ("done", "active", "alarm"):
                tag = lbl(state.upper(), "tag")
                tag.get_style_context().add_class("tag-" + state)
                tag.set_valign(Gtk.Align.CENTER)
                titlerow.pack_start(tag, False, False, 0)
            content.pack_start(titlerow, False, False, 0)
            content.pack_start(lbl(sub, "step-sub"), False, False, 0)
            content.set_margin_bottom(6)
            row.pack_start(content, True, True, 0)
            col.pack_start(row, True, True, 0)

        # current step panel
        cur = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        cur.get_style_context().add_class("current-panel")
        crow = Gtk.Box()
        crow.pack_start(lbl("CURRENT STEP", "current-label"), False, False, 0)
        crow.pack_start(Gtk.Box(), True, True, 0)
        crow.pack_start(lbl("⏱ 00:01.8", "current-time"), False, False, 0)
        cur.pack_start(crow, False, False, 0)
        cur.pack_start(lbl("Grasp by stem", "current-step"), False, False, 0)
        col.pack_start(cur, False, False, 0)
        col.pack_start(Gtk.Box(), False, False, 6)

        manual = Gtk.Button(label="✋  Manual Pick")
        manual.get_style_context().add_class("btn")
        col.pack_start(manual, False, False, 0)
        return col

    def _box_col(self):
        col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        col.get_style_context().add_class("card")
        col.set_size_request(340, -1)
        col.set_border_width(18)

        head = Gtk.Box()
        head.pack_start(lbl("BOX FILL", "section-title"), False, False, 0)
        head.pack_start(Gtk.Box(), True, True, 0)
        head.pack_start(lbl("SKU CAV-XL", "muted"), False, False, 0)
        col.pack_start(head, False, False, 0)
        col.pack_start(Gtk.Box(), False, False, 8)

        frame = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        frame.get_style_context().add_class("box-frame")
        frame.set_border_width(14)
        sub = Gtk.Box()
        sub.pack_start(lbl("TOP-DOWN · 4 × 3", "mini-label"), False, False, 0)
        sub.pack_start(Gtk.Box(), True, True, 0)
        sub.pack_start(lbl("FILLING", "up"), False, False, 0)
        frame.pack_start(sub, False, False, 0)
        frame.pack_start(BoxView(8, 12), True, True, 0)
        col.pack_start(frame, False, False, 0)
        col.pack_start(Gtk.Box(), False, False, 8)

        countrow = Gtk.Box()
        countrow.pack_start(lbl("Filled", "metric-unit"), False, False, 0)
        countrow.pack_start(Gtk.Box(), True, True, 0)
        cnt = Gtk.Box(spacing=0)
        cnt.pack_start(lbl("8", "fill-num"), False, False, 0)
        cnt.pack_start(lbl(" / 12", "fill-den"), False, False, 0)
        countrow.pack_start(cnt, False, False, 0)
        col.pack_start(countrow, False, False, 0)

        prog = Gtk.ProgressBar(); prog.set_fraction(8 / 12)
        prog.set_margin_top(8)
        col.pack_start(prog, False, False, 0)

        col.pack_start(Gtk.Box(), True, True, 0)  # spacer

        alert = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        alert.get_style_context().add_class("alert-amber")
        alert.pack_start(lbl("⚠  BOX NEARLY FULL", "alert-title"),
                         False, False, 0)
        alert.pack_start(lbl("4 cavities remaining · ready soon", "alert-sub"),
                         False, False, 0)
        col.pack_start(alert, False, False, 0)
        col.pack_start(Gtk.Box(), False, False, 6)

        change = Gtk.Button(label="⬓  Change Box")
        change.get_style_context().add_class("btn")
        col.pack_start(change, False, False, 0)
        return col

    # -- bottom metrics ----------------------------------------------------
    def _metrics(self):
        bar = Gtk.Box(spacing=18, homogeneous=True)
        bar.set_margin_start(18); bar.set_margin_end(18)
        bar.set_margin_bottom(18)
        bar.set_size_request(-1, 156)

        # throughput
        c1 = self._card_shell("THROUGHPUT")
        body1 = Gtk.Box()
        val = Gtk.Box(spacing=6)
        val.set_valign(Gtk.Align.END)
        val.pack_start(lbl("12", "metric-value"), False, False, 0)
        val.pack_start(lbl("hands/min", "metric-unit"), False, False, 0)
        body1.pack_start(val, False, False, 0)
        body1.pack_start(Gtk.Box(), True, True, 0)
        spark = SparkView(); spark.set_valign(Gtk.Align.END)
        body1.pack_start(spark, False, False, 0)
        c1.pack_start(body1, True, True, 0)
        bar.pack_start(self._wrap_card(c1), True, True, 0)

        # grasp success
        c2 = self._card_shell("GRASP SUCCESS RATE")
        body2 = Gtk.Box(spacing=18)
        body2.set_valign(Gtk.Align.CENTER)
        body2.pack_start(GaugeView(94.2), False, False, 0)
        txt = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        txt.set_valign(Gtk.Align.CENTER)
        txt.pack_start(lbl("94.2%", "metric-value"), False, False, 0)
        txt.pack_start(lbl("▲ +1.4% vs shift avg", "up"), False, False, 0)
        body2.pack_start(txt, False, False, 0)
        c2.pack_start(body2, True, True, 0)
        bar.pack_start(self._wrap_card(c2), True, True, 0)

        # rejections
        c3 = self._card_shell("REJECTIONS TODAY")
        body3 = Gtk.Box()
        body3.set_valign(Gtk.Align.END)
        left = Gtk.Box(spacing=12)
        left.pack_start(lbl("3", "metric-value"), False, False, 0)
        badge = lbl(" ● REJECTS ", "badge-reject")
        badge.set_valign(Gtk.Align.CENTER)
        left.pack_start(badge, False, False, 0)
        body3.pack_start(left, False, False, 0)
        body3.pack_start(Gtk.Box(), True, True, 0)
        for name, n in (("Bruised", "2"), ("No stem", "1")):
            mini = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            mini.pack_start(lbl(name, "mini-label", 1.0), False, False, 0)
            mini.pack_start(lbl(n, "mini-val", 1.0), False, False, 0)
            body3.pack_start(mini, False, False, 8)
        c3.pack_start(body3, True, True, 0)
        bar.pack_start(self._wrap_card(c3), True, True, 0)
        return bar

    def _card_shell(self, title):
        col = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        head = Gtk.Box(spacing=8)
        head.pack_start(lbl(title, "metric-label"), False, False, 0)
        col.pack_start(head, False, False, 0)
        return col

    def _wrap_card(self, inner):
        card = Gtk.Box()
        card.get_style_context().add_class("card")
        card.set_border_width(16)
        card.add(inner)
        return card

    # -- live camera -------------------------------------------------------
    def _start_camera(self):
        if CAMERA_TOOLS_ERROR is not None or ScepterCamera is None:
            self._camera_error(f"Camera stack unavailable: {CAMERA_TOOLS_ERROR}")
            return

        if self._camera_thread is not None and self._camera_thread.is_alive():
            return

        self._camera_generation += 1
        generation = self._camera_generation
        self._camera_running = True
        self._set_run_state(True)
        self._camera_thread = threading.Thread(
            target=self._camera_loop,
            args=(generation,),
            daemon=True,
        )
        self._camera_thread.start()

    def _camera_loop(self, generation):
        fps_count = 0
        fps_time = time.time()
        try:
            with ScepterCamera(self.camera_ip) as cam:
                GLib.idle_add(self._camera_status, generation, "CONNECTED", 0)
                while self._camera_running and generation == self._camera_generation:
                    bgr, depth_mm, xyz_mm = cam.get_frames()
                    if bgr is None or depth_mm is None or xyz_mm is None:
                        continue

                    fps_count += 1
                    now = time.time()
                    fps = 0
                    if now - fps_time >= 1.0:
                        fps = fps_count
                        fps_count = 0
                        fps_time = now

                    mode = self._depth_mode
                    ball = detect_orange(bgr, xyz_mm) if detect_orange else None

                    rgb_frame = bgr.copy()
                    if draw_ball_rgb:
                        draw_ball_rgb(rgb_frame, ball)

                    if mode == "cloud" and self._cloud is not None:
                        depth_frame = self._cloud.render(xyz_mm)
                    elif self._colorizer is not None:
                        depth_frame = self._colorizer(depth_mm)
                        if draw_ball_depth:
                            draw_ball_depth(depth_frame, ball)
                    else:
                        depth_frame = bgr.copy()

                    self._latest_rgb = bgr
                    self._latest_xyz_mm = xyz_mm
                    GLib.idle_add(self._camera_frame, generation, rgb_frame, depth_frame, mode, fps)
        except Exception as exc:
            GLib.idle_add(self._camera_error, f"Camera error: {exc}", generation)

    def _camera_frame(self, generation, rgb_frame, depth_frame, mode, fps):
        if generation != self._camera_generation:
            return False
        if mode == "cloud" and self._camera_stack is not None:
            self._camera_stack.set_visible_child_name("cloud")
        elif self._camera_stack is not None:
            self._camera_stack.set_visible_child_name("split")

        if mode != "cloud" and self._rgb_view is not None:
            self._rgb_view.set_frame(rgb_frame)
        if mode != "cloud" and self._depth_view is not None:
            self._depth_view.set_frame(depth_frame)
        if mode == "cloud" and self._cloud_view is not None:
            self._cloud_view.set_frame(depth_frame)

        if self._rgb_hdr is not None:
            fps_text = f"{fps} FPS" if fps else "LIVE"
            self._rgb_hdr.set_text(f"● RGB · {fps_text} · {self.camera_ip}")
        if self._depth_hdr is not None:
            self._depth_hdr.set_text("● DEPTH · CLICK=XYZ/HEIGHT")
        if self._cloud_hdr is not None:
            fps_text = f"{fps} FPS" if fps else "LIVE"
            self._cloud_hdr.set_text(f"● 3D CLOUD · {fps_text} · DRAG=ROTATE · SCROLL=ZOOM")
        return False

    def _camera_status(self, generation, text, fps):
        if generation != self._camera_generation:
            return False
        if self._rgb_hdr is not None:
            self._rgb_hdr.set_text(f"● RGB · {text} · {fps} FPS")
        if self._depth_hdr is not None:
            self._depth_hdr.set_text("● DEPTH · CLICK=XYZ/HEIGHT")
        if self._cloud_hdr is not None:
            self._cloud_hdr.set_text("● 3D CLOUD · READY")
        return False

    def _camera_error(self, message, generation=None):
        if generation is not None and generation != self._camera_generation:
            return False
        if self._rgb_hdr is not None:
            self._rgb_hdr.set_text("● RGB · OFFLINE")
        if self._depth_hdr is not None:
            self._depth_hdr.set_text("● DEPTH · OFFLINE")
        if self._cloud_hdr is not None:
            self._cloud_hdr.set_text("● 3D CLOUD · OFFLINE")
        if self._rgb_view is not None:
            self._rgb_view.set_message(message)
        if self._depth_view is not None:
            self._depth_view.set_message("Depth unavailable until camera reconnects.")
        if self._cloud_view is not None:
            self._cloud_view.set_message("3D cloud unavailable until camera reconnects.")
        return False

    def _cycle_depth_mode(self, *_):
        if self._depth_mode == "depth":
            self._depth_mode = "cloud"
            if self._cloud is not None:
                self._cloud.reset_scale()
            if self._depth_button is not None:
                self._depth_button.set_label("⬡  Back to RGB + Depth")
            if self._camera_stack is not None:
                self._camera_stack.set_visible_child_name("cloud")
            if self._coord_label is not None:
                self._coord_label.set_text("3D grande activo: arrastra para rotar, rueda para zoom, ajusta Points para densidad.")
        else:
            self._depth_mode = "depth"
            if self._depth_button is not None:
                self._depth_button.set_label("⬡  Open 3D Large")
            if self._camera_stack is not None:
                self._camera_stack.set_visible_child_name("split")
            if self._coord_label is not None:
                self._coord_label.set_text("Click en RGB o Depth: X/Y/Z en frame de cámara. Z = distancia desde sensor ToF.")

    def _on_cloud_press(self, _widget, event):
        if event.button != 1 or self._cloud is None:
            return False
        self._cloud_drag = (event.x, event.y, self._cloud.azimuth, self._cloud.elevation)
        return False

    def _on_cloud_release(self, *_):
        self._cloud_drag = None
        return False

    def _on_cloud_motion(self, _widget, event):
        if self._cloud is None or self._cloud_drag is None:
            return False
        dx = event.x - self._cloud_drag[0]
        dy = event.y - self._cloud_drag[1]
        self._cloud.azimuth = self._cloud_drag[2] + dx * 0.4
        self._cloud.elevation = float(np.clip(self._cloud_drag[3] - dy * 0.4, -89, 89))
        self._render_cloud_from_latest()
        return False

    def _on_cloud_scroll(self, _widget, event):
        if self._cloud is None:
            return False
        if event.direction == Gdk.ScrollDirection.UP:
            self._cloud.zoom(1.12)
        elif event.direction == Gdk.ScrollDirection.DOWN:
            self._cloud.zoom(1 / 1.12)
        elif event.direction == Gdk.ScrollDirection.SMOOTH:
            _, dy = event.get_scroll_deltas()
            if dy < 0:
                self._cloud.zoom(1.06)
            elif dy > 0:
                self._cloud.zoom(1 / 1.06)
        self._render_cloud_from_latest()
        return False

    def _on_point_budget_changed(self, scale):
        points_k = int(scale.get_value())
        if self._point_label is not None:
            self._point_label.set_text(f"Points {points_k}k")
        if self._cloud is not None:
            self._cloud.MAX_PTS = points_k * 1000
            self._cloud.reset_scale()
            self._render_cloud_from_latest()

    def _render_cloud_from_latest(self):
        if self._cloud is None or self._cloud_view is None or self._latest_xyz_mm is None:
            return
        self._cloud_view.set_frame(self._cloud.render(self._latest_xyz_mm))

    def _widget_to_img(self, widget, wx, wy):
        wa = max(1, widget.get_allocated_width())
        ha = max(1, widget.get_allocated_height())
        with np.errstate(invalid="ignore"):
            xyz = self._latest_xyz_mm
        if xyz is None:
            return None
        h, w = xyz.shape[:2]
        px = int(wx * w / wa)
        py = int(wy * h / ha)
        return max(0, min(w - 1, px)), max(0, min(h - 1, py))

    def _xyz_at(self, px, py):
        xyz = self._latest_xyz_mm
        if xyz is None:
            return None
        h, w = xyz.shape[:2]
        if 0 <= py < h and 0 <= px < w:
            point = xyz[py, px]
            if np.isfinite(point[2]) and point[2] > 0:
                return point.copy()
        return None

    def _on_rgb_click(self, widget, event):
        if event.button != 1:
            return False
        return self._handle_camera_click(widget, event, "RGB")

    def _on_depth_click(self, widget, event):
        if event.button != 1:
            return False
        if self._depth_mode == "cloud":
            self._coord_label.set_text("En 3D Cloud el click no corresponde 1:1 a pixel depth. Cambia a Depth para medir altura.")
            return False
        return self._handle_camera_click(widget, event, "Depth")

    def _handle_camera_click(self, widget, event, source):
        pixel = self._widget_to_img(widget, event.x, event.y)
        if pixel is None:
            self._coord_label.set_text("Sin matriz XYZ todavía. Espera a que llegue un frame RGB-D.")
            return False

        px, py = pixel
        xyz = self._xyz_at(px, py)
        if xyz is None:
            self._coord_label.set_text(f"{source} click ({px},{py}): sin dato 3D válido en ese pixel.")
            return False

        x, y, z = (float(v) for v in xyz)
        self._coord_label.set_text(
            f"{source} click ({px},{py}) · X={x:.1f} mm lateral · "
            f"Y={y:.1f} mm altura respecto al centro óptico · "
            f"Z={z:.1f} mm distancia desde el sensor ToF"
        )
        return False

    def _set_run_state(self, running):
        if not hasattr(self, "status_text"):
            return
        if running:
            self.status_text.set_text("RUNNING")
            self.status_pill.get_style_context().remove_class("pill-stop")
            self.status_pill.get_style_context().add_class("pill-run")
            self.status_dot.get_style_context().remove_class("dot-stop")
            self.status_dot.get_style_context().add_class("dot-run")
        else:
            self.status_text.set_text("STOPPED")
            self.status_pill.get_style_context().remove_class("pill-run")
            self.status_pill.get_style_context().add_class("pill-stop")
            self.status_dot.get_style_context().remove_class("dot-run")
            self.status_dot.get_style_context().add_class("dot-stop")

    # -- misc --------------------------------------------------------------
    def _tick(self):
        self.clock.set_text(time.strftime("%H:%M:%S"))
        return self._running

    def _emergency_stop(self, *_):
        self._camera_running = False
        self._camera_generation += 1
        self._set_run_state(False)
        if self._rgb_view is not None:
            self._rgb_view.set_message("E-STOP activo. Pulsa Reconnect para reiniciar cámara y algoritmo.")
        if self._depth_view is not None:
            self._depth_view.set_message("Stream detenido por E-STOP.")
        if self._cloud_view is not None:
            self._cloud_view.set_message("3D detenido por E-STOP. Pulsa Reconnect.")
        if self._coord_label is not None:
            self._coord_label.set_text("E-STOP detuvo el algoritmo. Pulsa Reconnect para volver a conectar.")

    def _reconnect_camera(self, *_):
        self._camera_running = False
        self._camera_generation += 1
        self._latest_xyz_mm = None
        self._latest_rgb = None
        self._colorizer = DepthColorizer() if DepthColorizer else None
        self._cloud = CloudRenderer() if CloudRenderer else None
        if self._cloud is not None and self._point_scale is not None:
            self._cloud.MAX_PTS = int(self._point_scale.get_value()) * 1000
        self._set_run_state(True)
        if self._rgb_view is not None:
            self._rgb_view.set_message("Reconnecting RGB stream...")
        if self._depth_view is not None:
            self._depth_view.set_message("Reconnecting Depth stream...")
        if self._cloud_view is not None:
            self._cloud_view.set_message("Reconnecting 3D cloud...")
        if self._coord_label is not None:
            self._coord_label.set_text("Reconectando. Espera el siguiente frame RGB-D antes de medir altura.")
        GLib.timeout_add(500, self._restart_camera)

    def _restart_camera(self):
        self._camera_thread = None
        self._start_camera()
        return False

    def _quit(self, *_):
        self._running = False
        self._camera_running = False
        self._camera_generation += 1
        Gtk.main_quit()

    def _keys(self, _w, ev):
        kv = ev.keyval
        if kv == Gdk.KEY_Escape:
            self._quit()
        elif kv == Gdk.KEY_F11:
            if self._is_full():
                self.unfullscreen()
            else:
                self.fullscreen()

    def _is_full(self):
        win = self.get_window()
        return bool(win and win.get_state() & Gdk.WindowState.FULLSCREEN)


def _tint(widget, hexcol):
    p = Gtk.CssProvider()
    p.load_from_data(("* { background-color: %s; }" % hexcol).encode())
    widget.get_style_context().add_provider(
        p, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)


def main():
    camera_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.101"
    prov = Gtk.CssProvider()
    prov.load_from_data(CSS)
    Gtk.StyleContext.add_provider_for_screen(
        Gdk.Screen.get_default(), prov,
        Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    win = BanaPickLiveWindow(camera_ip)
    win.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
