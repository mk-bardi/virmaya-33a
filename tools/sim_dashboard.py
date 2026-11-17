#!/usr/bin/env python3
"""
sim_dashboard.py - live flight-test dashboard for ArduPilot SITL (or a real vehicle). Dark theme.

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
# Design tokens (dark mode). Series colours are the validated dark categorical slots 1-5, used
# in fixed order (colour follows the entity). Status colours are reserved for state and always
# paired with a symbol + word. Text never wears a series colour.
# --------------------------------------------------------------------------------------------
T = {
    "page": "#0d0d0d", "surface": "#1a1a19", "chip": "#242422",
    "ink": "#f0efec", "ink2": "#c3c2b7", "muted": "#898781",
    "grid": "#2c2c2a", "base": "#383835", "border": (1, 1, 1, 0.10),
}
SERIES = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181"]   # blue, orange, aqua, yellow, magenta
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}

plt.rcParams.update({
    "toolbar": "None",
    "font.family": "sans-serif",
    "font.sans-serif": ["Inter", "Ubuntu", "Cantarell", "Segoe UI", "Helvetica", "Arial", "DejaVu Sans"],
    "figure.facecolor": T["page"], "savefig.facecolor": T["page"],
    "axes.facecolor": T["surface"], "axes.edgecolor": T["surface"],
    "axes.labelcolor": T["muted"], "text.color": T["ink"],
    "xtick.color": T["muted"], "ytick.color": T["muted"],
    "xtick.labelsize": 7.5, "ytick.labelsize": 7.5,
    "xtick.major.size": 0, "ytick.major.size": 0, "xtick.major.pad": 4, "ytick.major.pad": 4,
    "grid.color": T["grid"], "grid.linewidth": 0.7, "grid.linestyle": "-",
    "lines.linewidth": 1.5, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
    "legend.frameon": False, "legend.fontsize": 7.5, "legend.labelcolor": T["ink2"],
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
    "climb": ("Climb rate", "m/s", [("climb", "climb", 0, "-")], True),
    "spd":   ("Speed", "m/s", [("airspeed", "airspeed", 0, "-"), ("groundspeed", "groundspeed", 1, "-")], False),
    "att":   ("Attitude", "deg", [("roll", "roll", 0, "-"), ("nav_roll", None, 0, "--"),
                                  ("pitch", "pitch", 1, "-"), ("nav_pitch", None, 1, "--")], True),
    "yaw":   ("Yaw rate", "deg/s", [("yach", "actual", 0, "-"), ("ydes", "desired", 0, "--")], True),
    "mot":   ("Motors", "PWM", [("s5", "M1", 0, "-"), ("s6", "M2", 1, "-"), ("s7", "M3", 2, "-"),
                                       ("s8", "M4", 3, "-"), ("s3", "pusher", 4, "-")], False),
    "cur":   ("Battery current", "A", [("curr", "current", 0, "-")], False),
    "volt":  ("Battery voltage", "V", [("volt", "voltage", 0, "-")], False),
    "hdg":   ("Heading", "deg", [("heading", "heading", 0, "-")], False),
}
GRID = [["alt", "climb", "spd", "trk"],
        ["att", "yaw", "mot", "trk"],
        ["cur", "volt", "hdg", "log"]]
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
# Dashboard figure
# --------------------------------------------------------------------------------------------
class Dashboard:
    W, H = 1600, 900          # px at dpi 100

    def __init__(self, title="SITL", subtitle=""):
        self.fig = plt.figure(figsize=(self.W / 100, self.H / 100), dpi=100)
        try:
            self.fig.canvas.manager.set_window_title(f"{title} · flight dashboard")
        except AttributeError:
            pass
        self.ax, self.L, self.tiles, self.markers = {}, {}, {}, {}
        self._header(title, subtitle)
        self._tiles()
        self._charts()

    # geometry helpers (px -> figure fraction, origin top-left for readability)
    def fx(self, px):
        return px / self.W

    def fy(self, px_from_top):
        return 1 - px_from_top / self.H

    def card(self, x, y, w, h):
        """Rounded card, x/y/w/h in px from the top-left."""
        p = FancyBboxPatch((self.fx(x), self.fy(y + h)), self.fx(w), h / self.H,
                           boxstyle="round,pad=0,rounding_size=0.006", transform=self.fig.transFigure,
                           facecolor=T["surface"], edgecolor=T["border"], linewidth=0.8, zorder=0)
        self.fig.patches.append(p)

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

    # ---- header -----------------------------------------------------------------------------
    def _header(self, title, subtitle):
        tt = self.text(20, 30, title.upper(), fontsize=14, fontweight="bold", va="bottom")
        self.after(tt, "flight test", 6, fontsize=11, color=T["ink2"])
        self.h_sub = self.text(20, 48, subtitle, fontsize=8.5, color=T["muted"], va="baseline")
        # right side: status items, laid out right-to-left at fixed slots
        self.h = {}
        slots = {"link": 1430, "armed": 1270, "mode": 1100, "clock": 1530}
        for k, x in slots.items():
            if k == "clock":
                self.h[k] = self.text(x + 50, 30, "", fontsize=12, color=T["ink"], ha="right", va="bottom")
                self.text(x + 50, 47, "time since start", fontsize=7.5, color=T["muted"], ha="right", va="baseline")
                continue
            self.h[k + "_dot"] = self.text(x, 30, "", fontsize=11, va="bottom")
            self.h[k] = self.after(self.h[k + "_dot"], "", 4, fontsize=11, fontweight="bold")
            self.text(x + 16, 47, {"link": "telemetry", "armed": "motors", "mode": "flight mode"}[k],
                      fontsize=7.5, color=T["muted"], va="baseline")

    def set_status(self, key, word, status=None, symbol="●"):
        self.h[key].set_text(word)
        self.h[key + "_dot"].set_text(symbol)
        self.h[key + "_dot"].set_color(STATUS[status] if status else T["muted"])

    # ---- stat tiles -------------------------------------------------------------------------
    TILE_KEYS = ["alt", "air", "gnd", "climb", "thr", "batt", "time"]

    def _tiles(self):
        x0, y0, gap, h = 16, 62, 10, 92
        w = (self.W - 2 * x0 - gap * (len(self.TILE_KEYS) - 1)) / len(self.TILE_KEYS)
        for i, k in enumerate(self.TILE_KEYS):
            x = x0 + i * (w + gap)
            self.card(x, y0, w, h)
            t = {"label": self.text(x + 14, y0 + 22, "", fontsize=8.5, color=T["muted"], va="baseline"),
                 "value": self.text(x + 14, y0 + 64, "", fontsize=24, color=T["ink"], va="bottom"),
                 "sub_dot": self.text(x + 14, y0 + 84, "", fontsize=8, va="bottom"),
                 "x": x}
            t["unit"] = self.after(t["value"], "", 3, fontsize=10, color=T["ink2"])
            t["sub"] = self.after(t["sub_dot"], "", 2, fontsize=8, color=T["muted"])
            self.tiles[k] = t

    def set_tile(self, k, label, value, unit="", sub="", sub_status=None):
        t = self.tiles[k]
        t["label"].set_text(label)
        t["value"].set_text(value)
        t["unit"].set_text(unit)
        t["sub_dot"].set_text("●" if sub_status else "")
        if sub_status:
            t["sub_dot"].set_color(STATUS[sub_status])
        t["sub"].set_text(sub)

    # ---- charts -----------------------------------------------------------------------------
    def _charts(self):
        x0, top, gap = 16, 166, 10
        bottom = self.H - 14
        cols, rows = 4, 3
        cw = (self.W - 2 * x0 - gap * (cols - 1)) / cols
        rh = (bottom - top - gap * (rows - 1)) / rows
        placed = set()
        for r in range(rows):
            for c in range(cols):
                key = GRID[r][c]
                if key in placed:
                    continue
                placed.add(key)
                span = sum(1 for rr in range(rows) if GRID[rr][c] == key)
                x, y = x0 + c * (cw + gap), top + r * (rh + gap)
                h = rh * span + gap * (span - 1)
                self.card(x, y, cw, h)
                if key == "log":
                    self._log(x, y, cw, h)
                    continue
                ax = self.fig.add_axes([self.fx(x + 46), self.fy(y + h - 24), self.fx(cw - 60), (h - 24 - 38) / self.H])
                ax.set_zorder(2)
                self.ax[key] = ax
                title, unit = ("Ground track", "m from takeoff") if key == "trk" else CHARTS[key][:2]
                tt = self.text(x + 14, y + 26, title, fontsize=9.5, fontweight="bold", va="bottom")
                self.after(tt, unit, 5, fontsize=8, color=T["muted"])
                self._style_axes(ax)
                if key == "trk":
                    self._track(ax)
                else:
                    self._timeseries(key, ax, x, y, cw)

    def _style_axes(self, ax):
        for s in ax.spines.values():
            s.set_visible(False)
        ax.grid(True, axis="y")
        ax.set_axisbelow(True)
        ax.yaxis.set_major_locator(MaxNLocator(4))
        ax.xaxis.set_major_locator(MaxNLocator(5, integer=True))

    def _timeseries(self, key, ax, x, y, cw):
        title, unit, series, signed = CHARTS[key]
        if signed:
            ax.axhline(0, color=T["base"], linewidth=0.9, zorder=1)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: mmss(v)))
        handles = []
        for dk, label, slot, ls in series:
            dashed = ls == "--"
            ln, = ax.plot([], [], color=SERIES[slot], linewidth=1.1 if dashed else 1.5,
                          linestyle=(0, (3, 2)) if dashed else "-", alpha=0.75 if dashed else 1.0, zorder=3 if not dashed else 2)
            self.L[dk] = ln
            if label:
                handles.append((ln, label))
        if key == "att":                       # one key for "target" instead of two coloured dashed entries
            tgt, = ax.plot([], [], color=T["ink2"], linewidth=1.1, linestyle=(0, (3, 2)))
            handles.append((tgt, "target"))
        if len(handles) >= 2:
            ax.legend([h for h, _ in handles], [l for _, l in handles], loc="lower right",
                      bbox_to_anchor=(1.0, 1.0), ncol=len(handles), handlelength=1.2, columnspacing=0.9,
                      borderaxespad=0.6, handletextpad=0.5)
        if key in SINGLE_ENDPOINT:
            dot, = ax.plot([], [], "o", markersize=6, color=SERIES[0], markeredgecolor=T["surface"],
                           markeredgewidth=2, zorder=4)
            self.L[key + "_end"] = dot
        if key == "yaw":
            self.yaw_note = ax.text(1.0, 1.0, "", transform=ax.transAxes, ha="right", va="bottom",
                                    fontsize=7.5, color=T["muted"])
        if key == "hdg":
            ax.set_ylim(0, 360)
            ax.set_yticks([0, 90, 180, 270, 360])
            ax.set_yticklabels(["N", "E", "S", "W", "N"])

    def _track(self, ax):
        ax.grid(True, axis="both")
        ax.xaxis.set_major_locator(MaxNLocator(5))
        self.L["trk"], = ax.plot([], [], color=SERIES[0], linewidth=1.5, zorder=3)
        self.L["home"], = ax.plot([0], [0], "s", markersize=7, color=T["ink2"], markeredgecolor=T["surface"],
                                  markeredgewidth=2, zorder=4)
        self.L["home_lbl"] = ax.annotate("home", xy=(0, 0), xytext=(0, -11), textcoords="offset points",
                                         ha="center", va="top", color=T["muted"], fontsize=7.5, zorder=4)
        self.L["pos"], = ax.plot([], [], "o", markersize=9, color=T["ink"], markeredgecolor=SERIES[0],
                                 markeredgewidth=2.5, zorder=6)
        self.L["hdg_arrow"], = ax.plot([], [], color=T["ink"], linewidth=1.5, zorder=5)

    def _log(self, x, y, w, h):
        tt = self.text(x + 14, y + 26, "Events", fontsize=9.5, fontweight="bold", va="bottom")
        self.log_sub = self.after(tt, "ArduPilot messages", 5, fontsize=8, color=T["muted"])
        self.log_t, self.log_m = [], []
        lh = (h - 44) / LOG_LINES
        for i in range(LOG_LINES):
            yy = y + 44 + i * lh
            self.log_t.append(self.text(x + 52, yy, "", fontsize=7.8, color=T["muted"], ha="right", va="baseline"))
            self.log_m.append(self.text(x + 60, yy, "", fontsize=7.8, color=T["ink2"], va="baseline"))
        self.log_wrap = int((w - 74) / 5.0)

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

    # ---- shared updates ---------------------------------------------------------------------
    def set_series(self, key, t, v):
        if key == "heading":
            t, v = unwrap_heading(t, v)
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
        if not xs:
            pos = ax.get_position(); r = (pos.height * self.H) / (pos.width * self.W)
            ax.set_xlim(-25, 25); ax.set_ylim(-25 * r, 25 * r)
            return
        self.L["pos"].set_data([xs[-1]], [ys[-1]])
        x0, x1, y0, y1 = min(xs + [0]), max(xs + [0]), min(ys + [0]), max(ys + [0])
        pos = ax.get_position()
        r = (pos.height * self.H) / (pos.width * self.W)          # box height / width, so 1 m looks the same both ways
        hx = max(25.0, (x1 - x0) / 2 * 1.12, (y1 - y0) / 2 * 1.12 / r)
        hy = hx * r
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.set_xlim(cx - hx, cx + hx); ax.set_ylim(cy - hy, cy + hy)
        half = hx
        if isnum(heading_deg):
            L_ = half * 0.14
            hx, hy = xs[-1] + L_ * math.sin(math.radians(heading_deg)), ys[-1] + L_ * math.cos(math.radians(heading_deg))
            self.L["hdg_arrow"].set_data([xs[-1], hx], [ys[-1], hy])

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
            arts.append(ax.text(t, 0.97, f" {mode}", transform=ax.get_xaxis_transform(), fontsize=7,
                                color=T["ink2"], va="top", ha="left", zorder=5,
                                bbox=dict(boxstyle="square,pad=0.2", facecolor=T["surface"], edgecolor="none")))
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
            ax.set_xticks(time_ticks(xmin, xmax))
            if k != "hdg":
                ax.relim(); ax.autoscale_view(scalex=False)
                lo, hi = ax.get_ylim()
                if hi - lo < 1e-6:
                    ax.set_ylim(lo - 1, hi + 1)


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
        db.set_tile("alt", "Altitude", fmt(lt["alt"]), "m")
        db.set_tile("air", "Airspeed", fmt(lt["airspeed"]), "m/s")
        db.set_tile("gnd", "Groundspeed", fmt(lt["groundspeed"]), "m/s")
        db.set_tile("climb", "Climb rate", fmt(lt["climb"]), "m/s")
        db.set_tile("thr", "Throttle", fmt(lt["throttle"], "{:.0f}"), "%")
        bs = battery_status(lt["batt_pct"])
        db.set_tile("batt", "Battery", fmt(lt["volt"], "{:.2f}"), "V",
                    f"{fmt(lt['batt_pct'], '{:.0f}')} % {bs[1] if bs else ''} · {fmt(lt['curr'])} A",
                    bs[0] if bs else None)
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
    db.set_tile("batt", "Energy used", f"{s['wh']:.0f}" if s["wh"] >= 100 else f"{s['wh']:.1f}", "Wh", f"{s['mah']:.0f} mAh · SITL model")
    db.set_tile("time", "Flight time", mmss(s["armed_s"]), "", f"{s['dist']/1000:.2f} km flown")
    lines = [("modes", " > ".join(s["modes"]) or "-"),
             ("roll/pitch", f"max {fmt(s['roll_max'])}° / {fmt(s['pitch_max'])}°"),
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
