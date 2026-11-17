#!/usr/bin/env python3
"""
sim_dashboard.py - live flight-test dashboard for ArduPilot SITL (or a real vehicle). "Sahel night" theme:
instrument rail on the left, ground track with range rings in the centre, eight charts on the right.

While you fly it shows status, key numbers and live charts. When you stop it (close the window,
Ctrl+C, or --duration runs out) it saves a CSV of every sample, a PNG report in the same design,
and a text summary to --outdir.

Usage (with SITL running):
    python3 sim_dashboard.py                      # connects to SITL's spare port tcp:5762
    python3 sim_dashboard.py --pid                # desired vs actual rates (sets GCS_PID_MASK 7)
    python3 sim_dashboard.py --name hover_test    # label the saved files
    python3 sim_dashboard.py --title VIRMAYA      # name shown in the header
    python3 sim_dashboard.py --connect udpin:0.0.0.0:14551   # listen for MAVProxy output instead
    python3 sim_dashboard.py --report ~/sim_results/X.csv    # rebuild report from a saved CSV

Needs: pymavlink, matplotlib (both in the ArduPilot venv).
"""
import argparse
import csv
import logging
import math
import os
import textwrap
import threading
import time
from collections import deque
from datetime import datetime

from pymavlink import mavutil
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.text import Annotation
from matplotlib.ticker import FuncFormatter, MaxNLocator

logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)

# --------------------------------------------------------------------------------------------
# Design tokens: "Sahel night". Warm dark surfaces, field-instrument layout.
# Series colours are a 5-slot dark categorical palette validated with the dataviz checker
# (lightness band, chroma, CVD and normal-vision separation, 3:1 on the surface), used in fixed
# order so colour follows the entity. Status colours are reserved for state and always paired
# with a symbol + word. Text never wears a series colour.
# --------------------------------------------------------------------------------------------
T = {
    "page": "#100f0c", "surface": "#191814", "chip": "#24221c", "rail": "#15140f",
    "ink": "#f2efe6", "ink2": "#c9c3b2", "muted": "#8f8977",
    "grid": "#2e2b23", "base": "#3b382e", "border": (1, 0.95, 0.8, 0.07),
}
SERIES = ["#22a38a", "#c27e14", "#4f8fe0", "#d2577a", "#9085e9"]   # teal, ochre, sky, rose, violet
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
MONO = ["Ubuntu Mono", "DejaVu Sans Mono", "monospace"]

plt.rcParams.update({
    "toolbar": "None",
    "font.family": "sans-serif",
    "font.sans-serif": ["Ubuntu", "Inter", "Cantarell", "Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
    "figure.facecolor": T["page"], "savefig.facecolor": T["page"],
    "axes.facecolor": T["surface"], "axes.edgecolor": T["surface"],
    "axes.labelcolor": T["muted"], "text.color": T["ink"],
    "xtick.color": T["muted"], "ytick.color": T["muted"],
    "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "xtick.major.size": 0, "ytick.major.size": 0, "xtick.major.pad": 4, "ytick.major.pad": 4,
    "grid.color": T["grid"], "grid.linewidth": 0.8, "grid.linestyle": (0, (1, 2.5)),
    "lines.linewidth": 1.6, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
    "legend.frameon": False, "legend.fontsize": 7.2, "legend.labelcolor": T["ink2"],
})

SCALARS = [
    "alt", "climb", "roll", "pitch", "yaw", "yawrate", "nav_roll", "nav_pitch",
    "rdes", "rach", "pdes", "pach", "ydes", "yach",
    "airspeed", "groundspeed", "throttle", "heading",
    "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8",
    "volt", "curr", "batt_pct", "x", "y", "lat", "lon",
]

# chart key: (title, unit, [(data key, legend label, series slot, line style)], signed?)
CHARTS = {
    "alt":   ("Altitude", "m above home", [("alt", "altitude", 0, "-")], False),
    "spd":   ("Speed", "m/s", [("airspeed", "air", 0, "-"), ("groundspeed", "ground", 1, "-")], False),
    "climb": ("Climb rate", "m/s", [("climb", "climb", 0, "-")], True),
    "att":   ("Attitude", "deg", [("roll", "roll", 0, "-"), ("nav_roll", None, 0, "--"),
                                  ("pitch", "pitch", 1, "-"), ("nav_pitch", None, 1, "--")], True),
    "mot":   ("Motors", "PWM", [("s5", "M1", 0, "-"), ("s6", "M2", 1, "-"), ("s7", "M3", 2, "-"),
                                       ("s8", "M4", 3, "-"), ("s3", "push", 4, "-")], False),
    "yaw":   ("Yaw rate", "deg/s", [("yach", "actual", 0, "-"), ("ydes", "desired", 0, "--")], True),
    "cur":   ("Battery current", "A", [("curr", "current", 0, "-")], False),
    "volt":  ("Battery voltage", "V", [("volt", "voltage", 0, "-")], False),
}
CHART_GRID = [["alt", "spd"], ["climb", "att"], ["mot", "yaw"], ["cur", "volt"]]
SINGLE_ENDPOINT = {"alt", "climb", "cur", "volt"}     # charts that get a ringed endpoint dot
LOG_LINES = 9


# --------------------------------------------------------------------------------------------
# Telemetry state
# --------------------------------------------------------------------------------------------
class State:
    """Latest values plus rolling history, filled by the MAVLink reader thread."""

    def __init__(self, window_s):
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.window_s = window_s
        self.hist = {k: deque() for k in SCALARS}
        self.track = deque(maxlen=50000)
        self.latest = {k: math.nan for k in SCALARS}
        self.mode = "-"
        self.armed = False
        self.msgs = deque(maxlen=LOG_LINES)
        self.mode_changes = deque(maxlen=200)       # (t, mode)
        self.origin = None
        self.connected = False
        self.last_rx = 0.0
        self.have_pid = False
        self.energy_wh = 0.0
        self.charge_mah = 0.0
        self._last_batt_t = None
        self.dist_m = 0.0
        self._last_xy = None
        self.armed_time = 0.0
        self._last_arm_t = None

    def now(self):
        return time.time() - self.t0

    def put(self, key, val):
        t = self.now()
        self.latest[key] = val
        h = self.hist[key]
        h.append((t, val))
        while h and t - h[0][0] > self.window_s:
            h.popleft()


def reader(master, st, stop):
    while not stop.is_set():
        msg = master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        mt = msg.get_type()
        if mt == "BAD_DATA":
            continue
        with st.lock:
            st.last_rx = time.time()
            st.connected = True
            if mt == "HEARTBEAT":
                if msg.type == mavutil.mavlink.MAV_TYPE_GCS or \
                        msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    continue
                mode = mavutil.mode_string_v10(msg)
                if mode != st.mode:
                    st.mode_changes.append((st.now(), mode))
                st.mode = mode
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                t = st.now()
                if armed and st._last_arm_t is not None:
                    st.armed_time += t - st._last_arm_t
                st._last_arm_t = t if armed else None
                st.armed = armed
            elif mt == "ATTITUDE":
                st.put("roll", math.degrees(msg.roll))
                st.put("pitch", math.degrees(msg.pitch))
                st.put("yaw", math.degrees(msg.yaw) % 360)
                st.put("yawrate", math.degrees(msg.yawspeed))
            elif mt == "NAV_CONTROLLER_OUTPUT":
                st.put("nav_roll", msg.nav_roll)
                st.put("nav_pitch", msg.nav_pitch)
            elif mt == "PID_TUNING":
                st.have_pid = True
                if msg.axis == 1:
                    st.put("rdes", msg.desired); st.put("rach", msg.achieved)
                elif msg.axis == 2:
                    st.put("pdes", msg.desired); st.put("pach", msg.achieved)
                elif msg.axis == 3:
                    st.put("ydes", msg.desired); st.put("yach", msg.achieved)
            elif mt == "VFR_HUD":
                st.put("airspeed", msg.airspeed)
                st.put("groundspeed", msg.groundspeed)
                st.put("throttle", msg.throttle)
                st.put("heading", msg.heading)
                st.put("climb", msg.climb)
            elif mt == "GLOBAL_POSITION_INT":
                lat, lon = msg.lat / 1e7, msg.lon / 1e7
                st.put("alt", msg.relative_alt / 1000.0)
                if lat == 0 and lon == 0:
                    continue
                if st.origin is not None and st._last_xy is not None:
                    # SITL moved (restart / new home far away): start the track again from here
                    jx = (lon - st.origin[1]) * math.cos(math.radians(st.origin[0])) * 111320.0 - st._last_xy[0]
                    jy = (lat - st.origin[0]) * 110540.0 - st._last_xy[1]
                    if math.hypot(jx, jy) > 5000:
                        st.origin, st._last_xy = None, None
                        st.track.clear()
                if st.origin is None:
                    st.origin = (lat, lon)
                x = (lon - st.origin[1]) * math.cos(math.radians(st.origin[0])) * 111320.0
                y = (lat - st.origin[0]) * 110540.0
                st.put("x", x); st.put("y", y); st.put("lat", lat); st.put("lon", lon)
                st.track.append((x, y))
                if st._last_xy is not None and st.armed:
                    st.dist_m += math.hypot(x - st._last_xy[0], y - st._last_xy[1])
                st._last_xy = (x, y)
            elif mt == "SERVO_OUTPUT_RAW":
                for i in range(1, 9):
                    st.put(f"s{i}", getattr(msg, f"servo{i}_raw"))
            elif mt == "SYS_STATUS":
                v = msg.voltage_battery / 1000.0
                a = msg.current_battery / 100.0 if msg.current_battery >= 0 else math.nan
                st.put("volt", v); st.put("curr", a)
                if msg.battery_remaining >= 0:
                    st.put("batt_pct", msg.battery_remaining)
                t = st.now()
                if st._last_batt_t is not None and not math.isnan(a):
                    dt = t - st._last_batt_t
                    st.energy_wh += v * a * dt / 3600.0
                    st.charge_mah += a * dt / 3.6
                st._last_batt_t = t
            elif mt == "STATUSTEXT":
                st.msgs.append((st.now(), msg.text))


# --------------------------------------------------------------------------------------------
# Formatting helpers
# --------------------------------------------------------------------------------------------
def isnum(v):
    return v is not None and not (isinstance(v, float) and math.isnan(v))


def fmt(v, f="{:.1f}"):
    return f.format(v) if isnum(v) else "–"


def mmss(sec):
    sec = max(0, int(sec))
    if sec >= 3600:
        return f"{sec // 3600:d}:{sec % 3600 // 60:02d}:{sec % 60:02d}"
    return f"{sec // 60:d}:{sec % 60:02d}"


def unwrap_heading(t, v):
    """Break the heading line where it wraps 359 -> 0 so it doesn't draw a vertical stroke."""
    tt, vv = [], []
    for i, (a, b) in enumerate(zip(t, v)):
        if i and isnum(b) and isnum(vv[-1]) and abs(b - vv[-1]) > 180:
            tt.append(a); vv.append(math.nan)
        tt.append(a); vv.append(b)
    return tt, vv


def time_ticks(xmin, xmax, target=5):
    span = max(1.0, xmax - xmin)
    for step in (5, 10, 15, 30, 60, 120, 300, 600, 900, 1200, 1800, 3600, 7200):
        if span / step <= target:
            break
    first = math.ceil(xmin / step) * step
    ticks = [first + i * step for i in range(int((xmax - first) // step) + 1)]
    return [t for t in ticks if xmax - t > 0.07 * span]     # keep labels off the card edge


def battery_status(pct):
    if not isnum(pct):
        return None
    if pct >= 50:
        return "good", "ok"
    if pct >= 25:
        return "warning", "low"
    return "critical", "critical"


# --------------------------------------------------------------------------------------------
# Dashboard figure. Layout (px at dpi 100):
#   header strip      wordmark left, status chips right
#   left rail         instrument readouts, one per row, values in a fixed tabular column
#   centre            ground track with range rings (big), events below it
#   right             eight time-series charts, 2 columns x 4 rows
# --------------------------------------------------------------------------------------------
class Dashboard:
    W, H = 1600, 900
    PAD, GAP, TOP = 16, 10, 70
    RAIL_W, MAP_W = 300, 560
    MAP_H = 540

    def __init__(self, title="SITL", subtitle=""):
        self.fig = plt.figure(figsize=(self.W / 100, self.H / 100), dpi=100)
        try:
            self.fig.canvas.manager.set_window_title(f"{title} · flight lab")
        except AttributeError:
            pass
        self.ax, self.L, self.tiles, self.markers = {}, {}, {}, {}
        self._header(title, subtitle)
        self._rail()
        self._map_and_events()
        self._charts()

    # geometry helpers (px -> figure fraction, origin top-left for readability)
    def fx(self, px):
        return px / self.W

    def fy(self, px_from_top):
        return 1 - px_from_top / self.H

    def card(self, x, y, w, h, color=None, radius=0.008):
        p = FancyBboxPatch((self.fx(x), self.fy(y + h)), self.fx(w), h / self.H,
                           boxstyle=f"round,pad=0,rounding_size={radius}", transform=self.fig.transFigure,
                           facecolor=color or T["surface"], edgecolor=T["border"], linewidth=0.8, zorder=0)
        self.fig.patches.append(p)
        return p

    def hline(self, x0, x1, y, color=None, lw=0.8):
        self.fig.add_artist(plt.Line2D([self.fx(x0), self.fx(x1)], [self.fy(y), self.fy(y)],
                                       transform=self.fig.transFigure, color=color or T["grid"], linewidth=lw, zorder=1))

    def text(self, x, y, s, **kw):
        kw.setdefault("color", T["ink"])
        kw.setdefault("zorder", 3)
        return self.fig.text(self.fx(x), self.fy(y), s, **kw)

    def after(self, anchor, s, gap_pt=5, **kw):
        """Text that sits right after `anchor` (measured when drawn, so it never collides)."""
        kw.setdefault("color", T["ink"])
        a = Annotation(s, xy=(1, 0), xycoords=anchor, xytext=(gap_pt, 0), textcoords="offset points",
                       va="bottom", ha="left", zorder=3, **kw)
        self.fig.add_artist(a)
        return a

    def section(self, x, y, label, unit=""):
        """Small upper-case section label with an optional unit after it."""
        t = self.text(x, y, label.upper(), fontsize=8, fontweight="bold", color=T["ink2"], va="bottom")
        u = self.after(t, unit, 6, fontsize=7.5, color=T["muted"]) if unit is not None else None
        return t, u

    # ---- header -----------------------------------------------------------------------------
    def _header(self, title, subtitle):
        self.card(self.PAD, 14, 6, 34, color=T["ink2"], radius=0.002)              # wordmark bar
        tt = self.text(self.PAD + 16, 38, title.upper(), fontsize=17, fontweight="bold", va="baseline")
        self.after(tt, "flight lab", 8, fontsize=11, color=T["ink2"])
        self.h_sub = self.text(self.PAD + 16, 54, subtitle, fontsize=8, color=T["muted"], va="baseline")
        # status chips, right-aligned: mode, motors, telemetry, clock
        self.h = {}
        x = self.W - self.PAD
        for k, w, label in (("clock", 112, "since start"), ("link", 128, "telemetry"),
                            ("armed", 168, "motors"), ("mode", 150, "flight mode")):
            x -= w
            self.card(x, 12, w, 44, color=T["chip"], radius=0.006)
            if k == "clock":
                self.h[k] = self.text(x + 14, 33, "", fontsize=14, family=MONO, va="baseline")
            else:
                self.h[k + "_dot"] = self.text(x + 12, 33, "", fontsize=11, va="baseline")
                self.h[k] = self.after(self.h[k + "_dot"], "", 4, fontsize=11.5, fontweight="bold")
            self.text(x + 14, 48, label, fontsize=7.2, color=T["muted"], va="baseline")
            x -= 8
        self.hline(self.PAD, self.W - self.PAD, 62, color=T["base"])

    def set_status(self, key, word, status=None, symbol="●"):
        self.h[key].set_text(word)
        self.h[key + "_dot"].set_text(symbol)
        self.h[key + "_dot"].set_color(STATUS[status] if status else T["muted"])

    # ---- left rail: instrument readouts ------------------------------------------------------
    TILE_KEYS = ["alt", "air", "gnd", "climb", "thr", "hdg", "batt", "time"]

    def _rail(self):
        x, y0, w = self.PAD, self.TOP, self.RAIL_W
        h = self.H - self.PAD - y0
        self.card(x, y0, w, h, color=T["rail"])
        self.section(x + 16, y0 + 26, "Instruments", None)
        top = y0 + 40
        rh = (y0 + h - 10 - top) / len(self.TILE_KEYS)
        vcol = x + w - 56                                  # right edge of values (tabular column)
        for i, k in enumerate(self.TILE_KEYS):
            ry = top + i * rh
            if i:
                self.hline(x + 16, x + w - 16, ry)
            t = {"label": self.text(x + 16, ry + 26, "", fontsize=8.5, color=T["muted"], va="baseline"),
                 "value": self.text(vcol, ry + rh * 0.60, "", fontsize=26, family=MONO, ha="right", va="baseline"),
                 "unit": self.text(vcol + 6, ry + rh * 0.60, "", fontsize=10, color=T["ink2"], va="baseline"),
                 "sub_dot": self.text(x + 16, ry + rh - 11, "", fontsize=8, va="baseline")}
            t["sub"] = self.after(t["sub_dot"], "", 2, fontsize=8, color=T["muted"])
            self.tiles[k] = t
        # battery meter under the battery readout: track + fill
        by = top + self.TILE_KEYS.index("batt") * rh + rh - 29
        self.meter_box = (x + 16, by, w - 32)
        self.meter_track = self.card(x + 16, by, w - 32, 5, color=T["base"], radius=0.002)
        self.meter = self.card(x + 16, by, 0.001, 5, color=T["muted"], radius=0.002)

    def set_tile(self, k, label, value, unit="", sub="", sub_status=None):
        t = self.tiles[k]
        t["label"].set_text(label)
        t["value"].set_text(value)
        t["unit"].set_text(unit)
        t["sub_dot"].set_text("●" if sub_status else "")
        if sub_status:
            t["sub_dot"].set_color(STATUS[sub_status])
        t["sub"].set_text(sub)

    def set_meter(self, pct, status=None):
        x, y, w = self.meter_box
        if not isnum(pct):
            self.meter.set_visible(False); self.meter_track.set_visible(False)
            return
        self.meter.set_visible(True); self.meter_track.set_visible(True)
        self.meter.set_width(self.fx(max(0.001, w * min(100, max(0, pct)) / 100)))
        self.meter.set_facecolor(STATUS[status] if status else T["ink2"])

    # ---- centre: map + events ------------------------------------------------------------------
    def _map_and_events(self):
        x = self.PAD + self.RAIL_W + self.GAP
        y, w, h = self.TOP, self.MAP_W, self.MAP_H
        self.card(x, y, w, h)
        self.section(x + 16, y + 26, "Ground track", "m from takeoff")
        self.map_scale = self.text(x + w - 16, y + 26, "", fontsize=7.5, color=T["muted"], ha="right", va="bottom")
        ax = self.fig.add_axes([self.fx(x + 12), self.fy(y + h - 12), self.fx(w - 24), (h - 12 - 40) / self.H])
        ax.set_zorder(2)
        self.ax["trk"] = ax
        self._track(ax)
        ey = y + h + self.GAP
        self._log(x, ey, w, self.H - self.PAD - ey)

    def _track(self, ax):
        from matplotlib.patches import Circle
        for s_ in ax.spines.values():
            s_.set_visible(False)
        ax.set_xticks([]); ax.set_yticks([])
        ax.axhline(0, color=T["grid"], linewidth=0.8, zorder=1)
        ax.axvline(0, color=T["grid"], linewidth=0.8, zorder=1)
        self.rings, self.ring_lbl = [], []
        for i in range(4):
            c = Circle((0, 0), 1, fill=False, edgecolor=T["base"], linewidth=0.9, linestyle=(0, (1, 2.5)), zorder=1)
            ax.add_patch(c)
            self.rings.append(c)
            self.ring_lbl.append(ax.text(0, 0, "", fontsize=7, color=T["muted"], ha="left", va="bottom", zorder=2,
                                         clip_on=True))
        self.north = ax.text(0.5, 0.995, "N", transform=ax.transAxes, ha="center", va="top", fontsize=8,
                             fontweight="bold", color=T["ink2"], zorder=5)
        self.L["trk"], = ax.plot([], [], color=SERIES[0], linewidth=1.8, zorder=3)
        self.L["home"], = ax.plot([0], [0], "s", markersize=7, color=T["ink2"], markeredgecolor=T["surface"],
                                  markeredgewidth=2, zorder=4)
        self.L["home_lbl"] = ax.annotate("home", xy=(0, 0), xytext=(0, -11), textcoords="offset points",
                                         ha="center", va="top", color=T["muted"], fontsize=7.5, zorder=4)
        self.L["pos"], = ax.plot([], [], "o", markersize=10, color=T["ink"], markeredgecolor=SERIES[0],
                                 markeredgewidth=2.5, zorder=6)
        self.L["hdg_arrow"], = ax.plot([], [], color=T["ink"], linewidth=1.6, zorder=5)

    def _log(self, x, y, w, h):
        self.card(x, y, w, h)
        _, self.log_sub = self.section(x + 16, y + 26, "Events", "ArduPilot messages")
        self.log_t, self.log_m = [], []
        n = LOG_LINES
        lh = (h - 44) / n
        for i in range(n):
            yy = y + 46 + i * lh
            self.log_t.append(self.text(x + 58, yy, "", fontsize=7.8, family=MONO, color=T["muted"], ha="right", va="baseline"))
            self.log_m.append(self.text(x + 66, yy, "", fontsize=7.8, color=T["ink2"], va="baseline"))
        self.log_wrap = int((w - 110) / 5.6)

    def set_log(self, entries):
        """entries: newest first, list of (time label, message)."""
        for i in range(LOG_LINES):
            if i < len(entries):
                tl, m = entries[i]
                m = m if len(m) <= self.log_wrap else m[: self.log_wrap - 1] + "…"
                self.log_t[i].set_text(tl); self.log_m[i].set_text(m)
                self.log_m[i].set_color(T["ink"] if i == 0 else T["ink2"])
            else:
                self.log_t[i].set_text(""); self.log_m[i].set_text("")

    # ---- right: charts ---------------------------------------------------------------------------
    def _charts(self):
        x0 = self.PAD + self.RAIL_W + self.GAP + self.MAP_W + self.GAP
        top, bottom = self.TOP, self.H - self.PAD
        cols, rows = 2, len(CHART_GRID)
        cw = (self.W - self.PAD - x0 - self.GAP * (cols - 1)) / cols
        rh = (bottom - top - self.GAP * (rows - 1)) / rows
        for r, row in enumerate(CHART_GRID):
            for c, key in enumerate(row):
                x, y = x0 + c * (cw + self.GAP), top + r * (rh + self.GAP)
                self.card(x, y, cw, rh)
                title, unit = CHARTS[key][:2]
                self.section(x + 14, y + 22, title, unit)
                ax = self.fig.add_axes([self.fx(x + 40), self.fy(y + rh - 22), self.fx(cw - 52), (rh - 22 - 34) / self.H])
                ax.set_zorder(2)
                self.ax[key] = ax
                for s_ in ax.spines.values():
                    s_.set_visible(False)
                ax.grid(True, axis="y")
                ax.set_axisbelow(True)
                ax.yaxis.set_major_locator(MaxNLocator(4))
                self._timeseries(key, ax)

    def _timeseries(self, key, ax):
        title, unit, series, signed = CHARTS[key]
        if signed:
            ax.axhline(0, color=T["base"], linewidth=0.9, zorder=1)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: mmss(v)))
        handles = []
        for dk, label, slot, ls in series:
            dashed = ls == "--"
            ln, = ax.plot([], [], color=SERIES[slot], linewidth=1.1 if dashed else 1.6,
                          linestyle=(0, (3, 2)) if dashed else "-", alpha=0.75 if dashed else 1.0, zorder=3 if not dashed else 2)
            self.L[dk] = ln
            if label:
                handles.append((ln, label))
        if key == "att":                       # one key for "target" instead of two coloured dashed entries
            tgt, = ax.plot([], [], color=T["ink2"], linewidth=1.1, linestyle=(0, (3, 2)))
            handles.append((tgt, "target"))
        if len(handles) >= 2:
            ax.legend([h for h, _ in handles], [l for _, l in handles], loc="lower right",
                      bbox_to_anchor=(1.0, 1.0), ncol=len(handles), handlelength=1.0, columnspacing=0.7,
                      borderaxespad=0.45, handletextpad=0.35)
        if key in SINGLE_ENDPOINT:
            dot, = ax.plot([], [], "o", markersize=6, color=SERIES[0], markeredgecolor=T["surface"],
                           markeredgewidth=2, zorder=4)
            self.L[key + "_end"] = dot
        if key == "yaw":
            self.yaw_note = ax.text(0.99, 0.04, "", transform=ax.transAxes, ha="right", va="bottom",
                                    fontsize=7, color=T["muted"], zorder=5)

    # ---- shared updates ---------------------------------------------------------------------
    def set_series(self, key, t, v):
        if key not in self.L:
            return
        self.L[key].set_data(t, v)

    def set_endpoints(self):
        for key in SINGLE_ENDPOINT:
            dk = CHARTS[key][2][0][0]
            xs, ys = self.L[dk].get_data()
            pts = [(a, b) for a, b in zip(xs, ys) if isnum(b)]
            self.L[key + "_end"].set_data([pts[-1][0]], [pts[-1][1]]) if pts else self.L[key + "_end"].set_data([], [])

    def set_track(self, xs, ys, heading_deg):
        self.L["trk"].set_data(xs, ys)
        ax = self.ax["trk"]
        pos = ax.get_position()
        r = (pos.height * self.H) / (pos.width * self.W)          # box height / width, so 1 m looks the same both ways
        if xs:
            self.L["pos"].set_data([xs[-1]], [ys[-1]])
            x0, x1, y0, y1 = min(xs + [0]), max(xs + [0]), min(ys + [0]), max(ys + [0])
        else:
            x0 = x1 = y0 = y1 = 0.0
        hx = max(30.0, (x1 - x0) / 2 * 1.15, (y1 - y0) / 2 * 1.15 / r)
        hy = hx * r
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.set_xlim(cx - hx, cx + hx); ax.set_ylim(cy - hy, cy + hy)
        # range rings around home at a round spacing, about four across the view
        reach = max(abs(cx) + hx, abs(cy) + hy)
        step = next((s_ for s_ in (10, 20, 25, 50, 100, 200, 250, 500, 1000, 2000, 2500, 5000, 10000)
                     if s_ >= max(hx, hy) / 3.5), 20000)
        a = math.radians(40)
        for i, (c, lbl) in enumerate(zip(self.rings, self.ring_lbl)):
            rad = step * (i + 1)
            c.set_radius(rad)
            lx, ly = rad * math.cos(a), rad * math.sin(a)
            inside = cx - hx < lx < cx + hx * 0.9 and cy - hy < ly < cy + hy * 0.92
            c.set_visible(rad < reach * 1.5)
            lbl.set_position((lx, ly))
            lbl.set_text((f"{rad/1000:g} km" if rad >= 1000 else f"{rad:g} m") if inside else "")
        self.map_scale.set_text(f"rings every {step/1000:g} km" if step >= 1000 else f"rings every {step:g} m")
        if xs and isnum(heading_deg):
            L_ = hx * 0.12
            ex, ey = xs[-1] + L_ * math.sin(math.radians(heading_deg)), ys[-1] + L_ * math.cos(math.radians(heading_deg))
            self.L["hdg_arrow"].set_data([xs[-1], ex], [ys[-1], ey])

    def set_mode_markers(self, changes, xmin, label_axes="alt"):
        """Hairline at each flight-mode change, mode name on the altitude chart."""
        keep = set()
        for t, mode in changes:
            if t < xmin:
                continue
            keep.add(t)
            if t in self.markers:
                continue
            arts = []
            for k, ax in self.ax.items():
                if k == "trk":
                    continue
                arts.append(ax.axvline(t, color=T["base"], linewidth=0.9, zorder=1))
            ax = self.ax[label_axes]
            lbl = ax.text(t, 0.97, f" {mode}", transform=ax.get_xaxis_transform(), fontsize=6.8,
                          color=T["ink2"], va="top", ha="left", zorder=5,
                          bbox=dict(boxstyle="square,pad=0.2", facecolor=T["surface"], edgecolor="none"))
            arts.append(lbl)
            self.markers[t] = arts
        for t in list(self.markers):
            if t not in keep:
                for a in self.markers.pop(t):
                    a.remove()

    def rescale(self, xmin, xmax):
        for k, ax in self.ax.items():
            if k == "trk":
                continue
            ax.set_xlim(xmin, xmax)
            ax.set_xticks(time_ticks(xmin, xmax, target=4))
            ax.relim(); ax.autoscale_view(scalex=False)
            lo, hi = ax.get_ylim()
            if hi - lo < 1e-6:
                ax.set_ylim(lo - 1, hi + 1)
        # mode names on the altitude chart: keep a label only if it has room before the next one
        ax = self.ax["alt"]
        px_per_s = ax.get_position().width * self.W / max(1e-6, xmax - xmin)
        ts = sorted(self.markers)
        for i, t in enumerate(ts):
            lbl = self.markers[t][-1]
            room = ((ts[i + 1] if i + 1 < len(ts) else xmax) - t) * px_per_s
            lbl.set_visible(room >= 7.0 * (len(lbl.get_text()) + 1))


def compass(deg):
    return "N NE E SE S SW W NW".split()[int(((deg % 360) + 22.5) // 45) % 8] if isnum(deg) else ""



# --------------------------------------------------------------------------------------------
# Live update
# --------------------------------------------------------------------------------------------
def series(st, key):
    h = st.hist[key]
    if not h:
        return [], []
    t, v = zip(*h)
    return list(t), list(v)


def update_live(db, st, window_s):
    with st.lock:
        now = st.now()
        for key, (_, _, ser, _) in CHARTS.items():
            for dk, *_ in ser:
                if key == "yaw" and not st.have_pid:
                    continue
                db.set_series(dk, *series(st, dk))
        if not st.have_pid:
            db.set_series("yach", *series(st, "yawrate"))
            db.L["ydes"].set_data([], [])
            db.yaw_note.set_text("actual only · run with --pid for desired")
        else:
            db.yaw_note.set_text("")
        db.set_endpoints()
        xs, ys = ([p[0] for p in st.track], [p[1] for p in st.track]) if st.track else ([], [])
        db.set_track(xs, ys, st.latest["yaw"])
        lt = st.latest
        stale = (time.time() - st.last_rx) > 3 if st.connected else True
        db.set_status("link", "NO DATA" if stale else "LIVE", "critical" if stale else "good", "✕" if stale else "●")
        db.set_status("armed", "ARMED" if st.armed else "DISARMED", "warning" if st.armed else None,
                      "●" if st.armed else "○")
        db.set_status("mode", st.mode, None, "◆")
        db.h["clock"].set_text(mmss(now))
        db.set_tile("alt", "Altitude", fmt(lt["alt"]), "m", "above home")
        db.set_tile("air", "Airspeed", fmt(lt["airspeed"]), "m/s")
        db.set_tile("gnd", "Groundspeed", fmt(lt["groundspeed"]), "m/s")
        db.set_tile("climb", "Climb rate", fmt(lt["climb"]), "m/s")
        db.set_tile("thr", "Throttle", fmt(lt["throttle"], "{:.0f}"), "%")
        db.set_tile("hdg", "Heading", fmt(lt["heading"], "{:.0f}") + ("°" if isnum(lt["heading"]) else ""), "", compass(lt["heading"]))
        bs = battery_status(lt["batt_pct"])
        db.set_tile("batt", "Battery", fmt(lt["volt"], "{:.2f}"), "V",
                    f"{fmt(lt['batt_pct'], '{:.0f}')} % {bs[1] if bs else ''} · {fmt(lt['curr'])} A",
                    bs[0] if bs else None)
        db.set_meter(lt["batt_pct"], bs[0] if bs else None)
        db.set_tile("time", "Flight time", mmss(st.armed_time), "",
                    f"{st.dist_m/1000:.2f} km flown · {st.charge_mah:.0f} mAh")
        db.set_log([(mmss(t), m) for t, m in reversed(st.msgs)])
        changes = list(st.mode_changes)
    xmin = max(0.0, now - window_s)
    db.set_mode_markers(changes, xmin)
    db.rescale(xmin, max(now, xmin + 1))


# --------------------------------------------------------------------------------------------
# CSV log, summary and report
# --------------------------------------------------------------------------------------------
def csv_logger(st, path, stop, rate_hz=10):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "mode", "armed"] + SCALARS)
        while not stop.is_set():
            with st.lock:
                if st.connected:
                    w.writerow([f"{st.now():.2f}", st.mode, int(st.armed)] + [st.latest[k] for k in SCALARS])
            time.sleep(1.0 / rate_hz)


def load_rows(csv_path):
    rows = []
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            for k in r:
                if k != "mode":
                    try:
                        r[k] = float(r[k])
                    except (ValueError, TypeError):
                        r[k] = math.nan
            rows.append(r)
    return rows


def flight_window(rows, pad=5.0):
    armed_t = [r["t"] for r in rows if r["armed"] == 1]
    if not armed_t:
        return rows
    a, b = armed_t[0] - pad, armed_t[-1] + pad
    return [r for r in rows if a <= r["t"] <= b]


def flight_stats(rows):
    ar = [r for r in rows if r["armed"] == 1]

    def col(k):
        return [r[k] for r in ar if isnum(r[k])]

    armed_s = dist = wh = mah = 0.0
    for p, q in zip(ar, ar[1:]):
        dt = q["t"] - p["t"]
        if dt <= 0 or dt > 1.0:
            continue
        armed_s += dt
        if isnum(q["x"]) and isnum(p["x"]):
            step = math.hypot(q["x"] - p["x"], q["y"] - p["y"])
            if step < 100 * dt:              # ignore SITL position jumps (restart / moved home)
                dist += step
        if isnum(q["curr"]) and isnum(q["volt"]):
            wh += q["volt"] * q["curr"] * dt / 3600.0
            mah += q["curr"] * dt / 3.6
    modes = []
    for r in ar:
        if not modes or modes[-1] != r["mode"]:
            modes.append(r["mode"])
    yerr = [abs(r["ydes"] - r["yach"]) for r in ar if isnum(r["ydes"]) and isnum(r["yach"])]
    mx = lambda v: max(v) if v else math.nan
    mn = lambda v: min(v) if v else math.nan
    avg = lambda v: sum(v) / len(v) if v else math.nan
    return dict(armed_s=armed_s, dist=dist, wh=wh, mah=mah, modes=modes,
                alt_max=mx(col("alt")), alt_min=mn(col("alt")), as_max=mx(col("airspeed")),
                gs_max=mx(col("groundspeed")), climb_max=mx(col("climb")), sink_max=mn(col("climb")),
                thr_mean=avg(col("throttle")), roll_max=mx([abs(v) for v in col("roll")]),
                pitch_max=mx([abs(v) for v in col("pitch")]), yerr_mean=avg(yerr), yerr_max=mx(yerr),
                v_min=mn(col("volt")))


def summary_text(s, title):
    return "\n".join([
        f"Flight test summary: {title}",
        "modes flown (armed)  " + "\n                     ".join(textwrap.wrap(" > ".join(s["modes"]) or "-", 52)),
        f"armed time           {s['armed_s']:.0f} s",
        f"distance             {s['dist']:.0f} m",
        f"altitude             min {fmt(s['alt_min'])}, max {fmt(s['alt_max'])} m above home",
        f"max groundspeed      {fmt(s['gs_max'])} m/s",
        f"max airspeed         {fmt(s['as_max'])} m/s",
        f"mean throttle        {fmt(s['thr_mean'], '{:.0f}')} %",
        f"max |roll| |pitch|   {fmt(s['roll_max'])} / {fmt(s['pitch_max'])} deg",
        f"yaw rate error       mean {fmt(s['yerr_mean'])}, max {fmt(s['yerr_max'])} deg/s"
        + ("" if isnum(s["yerr_mean"]) else "  (run with --pid)"),
        f"charge used          {s['mah']:.0f} mAh   (SITL battery model, not calibrated)",
        f"energy used          {s['wh']:.2f} Wh   (SITL battery model, not calibrated)",
    ])


def track_from_latlon(rows):
    """Local x/y (m) from lat/lon, origin = first fix of this flight. Restarts if SITL jumped > 5 km."""
    xs, ys, o = [], [], None
    for r in rows:
        lat, lon = r.get("lat", math.nan), r.get("lon", math.nan)
        if not (isnum(lat) and isnum(lon)) or (lat == 0 and lon == 0):
            continue
        if o is None:
            o = (lat, lon)
        x = (lon - o[1]) * math.cos(math.radians(o[0])) * 111320.0
        y = (lat - o[0]) * 110540.0
        if xs and math.hypot(x - xs[-1], y - ys[-1]) > 5000:
            o, xs, ys = (lat, lon), [], []
            x = y = 0.0
        xs.append(x); ys.append(y)
    return xs, ys


def report_png(rows, png_path, title, name):
    rows = flight_window(rows)
    s = flight_stats(rows)
    db = Dashboard(title, f"{name} · report")
    t = [r["t"] for r in rows]
    have_pid = any(isnum(r["ydes"]) for r in rows)
    for key, (_, _, ser, _) in CHARTS.items():
        for dk, *_ in ser:
            db.set_series(dk, t, [r[dk] for r in rows])
    if not have_pid:
        db.set_series("yach", t, [r["yawrate"] for r in rows])
        db.L["ydes"].set_data([], [])
        db.yaw_note.set_text("actual only · flown without --pid")
    db.set_endpoints()
    xs, ys = track_from_latlon(rows)
    last_yaw = next((r["yaw"] for r in reversed(rows) if isnum(r["yaw"])), math.nan)
    db.set_track(xs, ys, last_yaw)
    db.set_status("link", "REPORT", None, "■")
    db.set_status("armed", f"{mmss(s['armed_s'])} armed", None, "○")
    db.set_status("mode", s["modes"][-1] if s["modes"] else "-", None, "◆")
    db.h["clock"].set_text(mmss(t[-1] - t[0]) if t else "0:00")
    db.set_tile("alt", "Max altitude", fmt(s["alt_max"]), "m", f"min {fmt(s['alt_min'])} m")
    db.set_tile("air", "Max airspeed", fmt(s["as_max"]), "m/s")
    db.set_tile("gnd", "Max groundspeed", fmt(s["gs_max"]), "m/s")
    db.set_tile("climb", "Max climb", fmt(s["climb_max"]), "m/s", f"max sink {fmt(s['sink_max'])} m/s")
    db.set_tile("thr", "Mean throttle", fmt(s["thr_mean"], "{:.0f}"), "%")
    db.set_tile("hdg", "Max bank", fmt(s["roll_max"], "{:.0f}") + "°", "", f"max pitch {fmt(s['pitch_max'], '{:.0f}')}°")
    db.set_tile("batt", "Energy used", f"{s['wh']:.0f}" if s["wh"] >= 100 else f"{s['wh']:.1f}", "Wh",
                f"{s['mah']:.0f} mAh · SITL model, not calibrated")
    db.set_meter(math.nan)
    db.set_tile("time", "Flight time", mmss(s["armed_s"]), "", f"{s['dist']/1000:.2f} km flown")
    wrapped = textwrap.wrap(" > ".join(s["modes"]) or "-", db.log_wrap - 6) or ["-"]
    lines = [("modes" if i == 0 else "", ln) for i, ln in enumerate(wrapped)] + [
             ("yaw error", f"mean {fmt(s['yerr_mean'])}, max {fmt(s['yerr_max'])} °/s" if have_pid else "not recorded"),
             ("min volt", f"{fmt(s['v_min'], '{:.2f}')} V"),
             ("battery", "SITL model, not calibrated")]
    db.log_sub.set_text("flight notes")
    db.set_log(lines)
    for i, tl in enumerate(db.log_t):
        tl.set_position((tl.get_position()[0] + db.fx(30), tl.get_position()[1]))
    for i, m in enumerate(db.log_m):
        m.set_position((m.get_position()[0] + db.fx(30), m.get_position()[1]))
    changes, prev = [], None
    for r in rows:
        if r["armed"] == 1 and r["mode"] != prev:
            changes.append((r["t"], r["mode"]))
        prev = r["mode"] if r["armed"] == 1 else None
    if t:
        db.set_mode_markers(changes, t[0])
        db.rescale(t[0], t[-1])
    db.fig.savefig(png_path, dpi=110)
    plt.close(db.fig)
    return s


def make_report(csv_path, title="SITL"):
    base = csv_path[:-4] if csv_path.endswith(".csv") else csv_path
    name = os.path.basename(base)
    rows = load_rows(csv_path)
    s = report_png(rows, base + ".png", title, name)
    text = summary_text(s, name)
    with open(base + "_summary.txt", "w") as fh:
        fh.write(text + "\n")
    return text, base


# --------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762", help="MAVLink connection string (default: SITL spare port)")
    ap.add_argument("--window", type=float, default=90, help="seconds of history shown")
    ap.add_argument("--rate", type=int, default=10, help="requested telemetry rate (Hz)")
    ap.add_argument("--pid", action="store_true", help="set GCS_PID_MASK=7 to stream desired vs actual rates")
    ap.add_argument("--name", default="run", help="label for the saved files")
    ap.add_argument("--title", default="SITL", help="name shown in the header, e.g. VIRMAYA")
    ap.add_argument("--outdir", default=os.path.expanduser("~/sim_results"))
    ap.add_argument("--duration", type=float, default=0, help="stop automatically after N seconds (0 = never)")
    ap.add_argument("--report", metavar="CSV", help="rebuild the PNG and summary from a saved CSV, then exit")
    args = ap.parse_args()

    if args.report:
        text, base = make_report(args.report, args.title)
        print(text + f"\n\nSaved: {base}.png\n       {base}_summary.txt")
        return

    os.makedirs(args.outdir, exist_ok=True)
    stamp = f"{datetime.now():%Y%m%d_%H%M%S}_{args.name}"
    base = os.path.join(args.outdir, stamp)

    print(f"Connecting on {args.connect} ...")
    master = mavutil.mavlink_connection(args.connect, source_system=254, source_component=191)
    hb = master.wait_heartbeat(timeout=30)
    if hb is None:
        print("No heartbeat after 30 s. Is SITL running? Check --connect.")
        return
    print(f"Connected to system {master.target_system}")
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)
    if args.pid:
        master.mav.param_set_send(master.target_system, master.target_component, b"GCS_PID_MASK", 7,
                                  mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    st = State(args.window)
    stop = threading.Event()
    threads = [threading.Thread(target=reader, args=(master, st, stop), daemon=True),
               threading.Thread(target=csv_logger, args=(st, base + ".csv", stop), daemon=True)]
    for t in threads:
        t.start()

    db = Dashboard(args.title, f"{args.name} · {datetime.now():%d %b %Y %H:%M} · {args.connect}")
    interactive = matplotlib.get_backend().lower() not in ("agg", "pdf", "svg", "ps", "cairo")
    if interactive:
        plt.ion(); plt.show(block=False)
    try:
        while True:
            update_live(db, st, args.window)
            if interactive:
                if not plt.fignum_exists(db.fig.number):
                    break
                db.fig.canvas.draw_idle()
                plt.pause(0.25)
            else:
                time.sleep(0.25)
            if args.duration and st.now() >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=2)
        if os.environ.get("SIM_DASHBOARD_SNAPSHOT"):
            db.fig.savefig(os.environ["SIM_DASHBOARD_SNAPSHOT"], dpi=100)
        plt.close(db.fig)
        text, base = make_report(base + ".csv", args.title)
        print("\n" + text)
        print(f"\nSaved: {base}.csv\n       {base}.png\n       {base}_summary.txt")


if __name__ == "__main__":
    main()
