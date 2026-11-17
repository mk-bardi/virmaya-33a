#!/usr/bin/env python3
"""
sim_dashboard.py - research telemetry dashboard for the Virmaya-33A in ArduPilot SITL (or the real aircraft).
Light "research notebook" design: figure-style panels (a)-(l) with commanded vs actual attitude, lift-motor
saturation, power, a flight-mode timeline, and a run-metrics table (tracking error RMS, motor saturation,
hover output). Saved reports come as PNG and PDF, ready for a paper.

While you fly it shows status, key numbers and live charts. When you stop it (close the window,
Ctrl+C, or --duration runs out) it saves a CSV of every sample, a PNG report in the same design,
a PDF copy and a text summary (with the research metrics) to --outdir.

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
# Design tokens: "research notebook". Light paper page, figure-style panels labelled (a)-(l),
# serif type, so a saved report drops straight into a paper or thesis.
# Series colours are the validated light categorical slots 1-5, used in fixed order. Three of
# them sit under 3:1 on white, so every multi-series panel carries a legend (relief rule).
# Commanded values are always grey dashed, so "actual vs command" reads the same in every panel.
# Status colours are reserved for state and always paired with a symbol + word.
# --------------------------------------------------------------------------------------------
T = {
    "page": "#fbfaf7", "band_a": "#ecebe5", "band_b": "#f4f3ee",
    "ink": "#1d1c1a", "ink2": "#52514e", "muted": "#7d7b72",
    "grid": "#e6e4dc", "spine": "#a3a197", "rule": "#1d1c1a",
}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]   # blue, orange, aqua, yellow, magenta
COMMAND = T["ink2"]
STATUS = {"good": "#0ca30c", "warning": "#fab219", "serious": "#ec835a", "critical": "#d03b3b"}
MOTOR_PWM_MIN, MOTOR_PWM_MAX = 1100, 1900       # Q_M_PWM_MIN / Q_M_PWM_MAX in params/virmaya.parm

plt.rcParams.update({
    "toolbar": "None",
    "font.family": "serif",
    "font.serif": ["STIX Two Text", "STIXGeneral", "Charter", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "figure.facecolor": T["page"], "savefig.facecolor": T["page"],
    "axes.facecolor": T["page"], "axes.edgecolor": T["spine"], "axes.linewidth": 0.8,
    "axes.labelcolor": T["ink2"], "text.color": T["ink"],
    "xtick.color": T["ink2"], "ytick.color": T["ink2"],
    "xtick.labelsize": 8, "ytick.labelsize": 8,
    "xtick.direction": "in", "ytick.direction": "in",
    "xtick.major.size": 3, "ytick.major.size": 3, "xtick.major.width": 0.7, "ytick.major.width": 0.7,
    "xtick.major.pad": 3, "ytick.major.pad": 3,
    "grid.color": T["grid"], "grid.linewidth": 0.6, "grid.linestyle": "-",
    "lines.linewidth": 1.3, "lines.solid_capstyle": "round", "lines.solid_joinstyle": "round",
    "legend.frameon": True, "legend.framealpha": 0.9, "legend.edgecolor": "none",
    "legend.facecolor": T["page"], "legend.fontsize": 7.5, "legend.labelcolor": T["ink2"],
})

SCALARS = [
    "alt", "climb", "roll", "pitch", "yaw", "yawrate", "nav_roll", "nav_pitch",
    "rdes", "rach", "pdes", "pach", "ydes", "yach",
    "airspeed", "groundspeed", "throttle", "heading",
    "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8",
    "volt", "curr", "batt_pct", "x", "y", "lat", "lon",
]

# panel key: (title, unit, [(data key, legend label, series slot or None = command, ...)], signed?)
PANELS = {
    "alt":   ("Altitude", "m", [("alt", "altitude", 0)], False),
    "spd":   ("Speed", "m/s", [("airspeed", "airspeed", 0), ("groundspeed", "groundspeed", 1)], False),
    "push":  ("Pusher output", "PWM, µs", [("s3", "pusher", 0)], False),
    "roll":  ("Roll tracking", "deg", [("roll", "actual", 0), ("nav_roll", "command", None)], True),
    "pitch": ("Pitch tracking", "deg", [("pitch", "actual", 0), ("nav_pitch", "command", None)], True),
    "yaw":   ("Yaw-rate tracking", "deg/s", [("yach", "actual", 0), ("ydes", "command", None)], True),
    "mot":   ("Lift motors", "PWM, µs", [("s5", "M1", 0), ("s6", "M2", 1), ("s7", "M3", 2), ("s8", "M4", 3)], False),
    "volt":  ("Battery voltage", "V", [("volt", "voltage", 0)], False),
    "cur":   ("Battery current", "A", [("curr", "current", 0)], False),
}
PANEL_GRID = [["alt", "spd", "push"], ["roll", "pitch", "yaw"], ["mot", "volt", "cur"]]
LOG_LINES = 5
METRIC_ROWS = 10


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
        self.rows = deque(maxlen=int(window_s * 10) + 600)   # 10 Hz snapshots for the live metrics
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
# Research metrics (same code for the live window and the saved report)
# --------------------------------------------------------------------------------------------
def research_metrics(rows):
    """Tracking errors, motor saturation and power figures from armed samples (dicts with SCALARS keys)."""
    ar = [r for r in rows if r.get("armed") == 1]
    rms = lambda v: math.sqrt(sum(x * x for x in v) / len(v)) if v else math.nan
    err = lambda a, b: [r[a] - r[b] for r in ar if isnum(r.get(a)) and isnum(r.get(b))]
    lift = [[r[k] for k in ("s5", "s6", "s7", "s8")] for r in ar
            if all(isnum(r.get(k)) for k in ("s5", "s6", "s7", "s8"))]
    active = [m for m in lift if sum(m) / 4 > MOTOR_PWM_MIN + 50]
    sat = [max(m) >= MOTOR_PWM_MAX - 20 for m in active]
    hover = [(sum(m) / 4 - MOTOR_PWM_MIN) / (MOTOR_PWM_MAX - MOTOR_PWM_MIN)
             for r, m in zip([r for r in ar if all(isnum(r.get(k)) for k in ("s5", "s6", "s7", "s8"))], lift)
             if sum(m) / 4 > MOTOR_PWM_MIN + 50 and isnum(r.get("climb")) and abs(r["climb"]) < 0.3
             and isnum(r.get("groundspeed")) and r["groundspeed"] < 3]
    col = lambda k: [r[k] for r in ar if isnum(r.get(k))]
    return dict(
        roll_rms=rms(err("roll", "nav_roll")), pitch_rms=rms(err("pitch", "nav_pitch")),
        yaw_rms=rms(err("yach", "ydes")),
        sat_pct=100 * sum(sat) / len(sat) if sat else math.nan,
        spread=sum(max(m) - min(m) for m in active) / len(active) if active else math.nan,
        hover=sum(hover) / len(hover) if hover else math.nan,
        v_min=min(col("volt")) if col("volt") else math.nan,
        i_max=max(col("curr")) if col("curr") else math.nan,
    )


def metric_rows(m, wh, armed_s, dist_m):
    return [
        ("Roll error, RMS", f"{fmt(m['roll_rms'])}°"),
        ("Pitch error, RMS", f"{fmt(m['pitch_rms'])}°"),
        ("Yaw-rate error, RMS", f"{fmt(m['yaw_rms'], '{:.2f}')} °/s" if isnum(m["yaw_rms"]) else "needs --pid"),
        ("Lift motors at max", f"{fmt(m['sat_pct'])} % of VTOL time"),
        ("Lift motor spread, mean", f"{fmt(m['spread'], '{:.0f}')} µs"),
        ("Hover motor output, mean", f"{fmt(m['hover'], '{:.2f}')} of range"),
        ("Minimum voltage", f"{fmt(m['v_min'], '{:.2f}')} V"),
        ("Maximum current", f"{fmt(m['i_max'])} A"),
        ("Energy used", f"{wh:.1f} Wh (SITL model)"),
        ("Armed time · distance", f"{mmss(armed_s)} · {dist_m/1000:.2f} km"),
    ]


# --------------------------------------------------------------------------------------------
# Dashboard figure. Layout (px at dpi 100):
#   header            title and run line left, status right, full-width rule under it
#   mode timeline     coloured bands per flight mode across the plotting width
#   left 3 x 3        figure panels (a)-(i): flight, tracking, power
#   right column      (j) ground track, (k) run metrics, (l) events
# --------------------------------------------------------------------------------------------
class Dashboard:
    W, H = 1600, 900
    PAD = 16
    LEFT_X1 = 1150                 # right edge of the 3 x 3 panel block
    RIGHT_X0 = 1196                # left edge of the right column

    def __init__(self, title="SITL", subtitle=""):
        self.fig = plt.figure(figsize=(self.W / 100, self.H / 100), dpi=100)
        try:
            self.fig.canvas.manager.set_window_title(f"{title} · research telemetry")
        except AttributeError:
            pass
        self.ax, self.L, self.mode_changes, self._mode_arts = {}, {}, [], []
        self._header(title, subtitle)
        self._panels()
        self._band()
        self._right()

    def fx(self, px):
        return px / self.W

    def fy(self, px_from_top):
        return 1 - px_from_top / self.H

    def rect(self, x, y, w, h):
        """axes rect from px (top-left origin)."""
        return [self.fx(x), self.fy(y + h), self.fx(w), h / self.H]

    def text(self, x, y, s, **kw):
        kw.setdefault("color", T["ink"])
        kw.setdefault("zorder", 3)
        return self.fig.text(self.fx(x), self.fy(y), s, **kw)

    def after(self, anchor, s, gap_pt=5, **kw):
        kw.setdefault("color", T["ink"])
        kw.setdefault("va", "bottom")
        a = Annotation(s, xy=(1, 0), xycoords=anchor, xytext=(gap_pt, 0), textcoords="offset points",
                       ha="left", zorder=3, **kw)
        self.fig.add_artist(a)
        return a

    def rule(self, x0, x1, y, lw=0.8, color=None):
        self.fig.add_artist(plt.Line2D([self.fx(x0), self.fx(x1)], [self.fy(y), self.fy(y)],
                                       transform=self.fig.transFigure, color=color or T["rule"], linewidth=lw))

    def panel_title(self, x, y, letter, title, unit):
        t = self.text(x, y, f"({letter})", fontsize=10.5, fontweight="bold", va="baseline")
        t2 = self.after(t, title, 4, fontsize=10.5)
        return self.after(t2, unit, 5, fontsize=9, color=T["muted"], style="italic")

    # ---- header -----------------------------------------------------------------------------
    def _header(self, title, subtitle):
        tt = self.text(self.PAD, 34, title.upper(), fontsize=19, fontweight="bold", va="baseline")
        self.after(tt, "research telemetry", 8, fontsize=13, style="italic", color=T["ink2"])
        self.h_sub = self.text(self.PAD, 54, subtitle, fontsize=9, color=T["muted"], va="baseline")
        self.h = {}
        for k, x, label in (("mode", 1020, "flight mode"), ("armed", 1170, "motors"), ("link", 1330, "telemetry")):
            self.h[k + "_dot"] = self.text(x, 34, "", fontsize=10, va="baseline")
            self.h[k] = self.after(self.h[k + "_dot"], "", 3, fontsize=12, fontweight="bold")
            self.text(x, 52, label, fontsize=8.5, color=T["muted"], style="italic", va="baseline")
        self.h["clock"] = self.text(self.W - self.PAD, 34, "", fontsize=15, ha="right", va="baseline")
        self.text(self.W - self.PAD, 52, "elapsed", fontsize=8.5, color=T["muted"], style="italic", ha="right", va="baseline")
        self.rule(self.PAD, self.W - self.PAD, 64, lw=1.4)
        self.rule(self.PAD, self.W - self.PAD, 67, lw=0.5)

    def set_status(self, key, word, status=None, symbol="●"):
        self.h[key].set_text(word)
        self.h[key + "_dot"].set_text(symbol)
        self.h[key + "_dot"].set_color(STATUS[status] if status else T["muted"])

    # ---- 3 x 3 panels -----------------------------------------------------------------------
    def _panels(self):
        top, bottom = 132, self.H - 36
        cols, rows = 3, 3
        gx, gy = 26, 22
        cw = (self.LEFT_X1 - self.PAD - gx * (cols - 1)) / cols
        rh = (bottom - top - gy * (rows - 1)) / rows
        letters = iter("abcdefghi")
        self.panel_x = []
        for r, row in enumerate(PANEL_GRID):
            for c, key in enumerate(row):
                x, y = self.PAD + c * (cw + gx), top + r * (rh + gy)
                ax_x, ax_w = x + 44, cw - 48
                if r == 0:
                    self.panel_x.append((ax_x, ax_w))
                title, unit = PANELS[key][:2]
                self.panel_title(ax_x - 40, y + 14, next(letters), title, unit)
                ax = self.fig.add_axes(self.rect(ax_x, y + 22, ax_w, rh - 22))
                self.ax[key] = ax
                ax.grid(True, axis="both")
                ax.set_axisbelow(True)
                for s_ in ("top", "right"):
                    ax.spines[s_].set_visible(False)
                ax.yaxis.set_major_locator(MaxNLocator(5))
                ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: mmss(v)))
                if r < rows - 1:
                    ax.tick_params(labelbottom=False)
                else:
                    ax.set_xlabel("time since start, m:ss", fontsize=8.5, style="italic", labelpad=2)
                self._series(key, ax)

    def _series(self, key, ax):
        title, unit, series, signed = PANELS[key]
        if signed:
            ax.axhline(0, color=T["spine"], linewidth=0.7, zorder=1)
        handles = []
        for dk, label, slot in series:
            cmd = slot is None
            ln, = ax.plot([], [], color=COMMAND if cmd else SERIES[slot], linewidth=1.0 if cmd else 1.3,
                          linestyle=(0, (4, 2)) if cmd else "-", zorder=2 if cmd else 3)
            self.L[dk] = ln
            handles.append((ln, label))
        if key == "mot":
            ax.axhline(MOTOR_PWM_MAX, color=T["ink"], linewidth=0.8, linestyle=(0, (1, 1.5)), zorder=2)
            ax.text(0.01, MOTOR_PWM_MAX, " PWM max", transform=ax.get_yaxis_transform(), fontsize=7.5,
                    color=T["ink2"], style="italic", va="bottom", ha="left")
        if len(handles) >= 2:
            ax.legend([h for h, _ in handles], [l for _, l in handles], loc="upper right", ncol=len(handles),
                      handlelength=1.6, columnspacing=0.9, handletextpad=0.4, borderpad=0.3, borderaxespad=0.3)
        if key == "yaw":
            self.yaw_note = ax.text(0.02, 0.06, "", transform=ax.transAxes, ha="left", va="bottom",
                                    fontsize=7.5, style="italic", color=T["muted"], zorder=5)

    # ---- mode timeline ------------------------------------------------------------------------
    def _band(self):
        x0 = self.panel_x[0][0]
        x1 = self.panel_x[-1][0] + self.panel_x[-1][1]
        self.text(self.PAD, 98, "modes", fontsize=9, style="italic", color=T["muted"], va="baseline")
        self.band = self.fig.add_axes(self.rect(x0, 82, x1 - x0, 24))
        self._style_band()

    def _style_band(self):
        b = self.band
        b.set_yticks([]); b.set_xticks([])
        for s_ in b.spines.values():
            s_.set_visible(False)
        b.set_ylim(0, 1)

    # ---- right column: track, metrics, events --------------------------------------------------
    def _right(self):
        x0, w = self.RIGHT_X0, self.W - self.PAD - self.RIGHT_X0
        self.panel_title(x0, 92, "j", "Ground track", "m from takeoff")
        ax = self.fig.add_axes(self.rect(x0 + 40, 104, w - 44, 330))
        self.ax["trk"] = ax
        ax.grid(True)
        ax.set_axisbelow(True)
        for s_ in ("top", "right"):
            ax.spines[s_].set_visible(False)
        ax.xaxis.set_major_locator(MaxNLocator(5)); ax.yaxis.set_major_locator(MaxNLocator(5))
        ax.set_xlabel("east, m", fontsize=8.5, style="italic", labelpad=2)
        ax.set_ylabel("north, m", fontsize=8.5, style="italic", labelpad=2)
        self.L["trk"], = ax.plot([], [], color=SERIES[0], linewidth=1.3, zorder=3)
        self.L["home"], = ax.plot([0], [0], "s", markersize=6, color=T["ink"], markeredgecolor=T["page"],
                                  markeredgewidth=1.5, zorder=4)
        self.L["home_lbl"] = ax.annotate("home", xy=(0, 0), xytext=(0, -9), textcoords="offset points",
                                         ha="center", va="top", color=T["ink2"], fontsize=8, style="italic", zorder=4)
        self.L["pos"], = ax.plot([], [], "o", markersize=7, color=T["page"], markeredgecolor=T["ink"],
                                 markeredgewidth=1.8, zorder=6)
        self.L["hdg_arrow"], = ax.plot([], [], color=T["ink"], linewidth=1.3, zorder=5)

        # (k) metrics table
        ty = 500
        self.metrics_title = self.panel_title(x0, ty, "k", "Run metrics", "")
        self.rule(x0, x0 + w, ty + 8, lw=0.9)
        self.m_lbl, self.m_val = [], []
        rh = 21
        for i in range(METRIC_ROWS):
            yy = ty + 26 + i * rh
            self.m_lbl.append(self.text(x0, yy, "", fontsize=9.5, color=T["ink2"], va="baseline"))
            self.m_val.append(self.text(x0 + w, yy, "", fontsize=9.5, ha="right", va="baseline"))
            if i < METRIC_ROWS - 1:
                self.rule(x0, x0 + w, yy + 7, lw=0.5, color=T["grid"])
        self.rule(x0, x0 + w, ty + 26 + (METRIC_ROWS - 1) * rh + 8, lw=0.9)

        # (l) events
        ey = ty + 26 + METRIC_ROWS * rh + 26
        self.log_sub = self.panel_title(x0, ey, "l", "Events", "ArduPilot messages")
        self.log_t, self.log_m = [], []
        for i in range(LOG_LINES):
            yy = ey + 22 + i * 18
            self.log_t.append(self.text(x0 + 52, yy, "", fontsize=8.5, color=T["muted"], ha="right", va="baseline"))
            self.log_m.append(self.text(x0 + 60, yy, "", fontsize=8.5, color=T["ink2"], va="baseline"))
        self.log_wrap = int((w - 64) / 6.2)

    def set_metrics(self, rows, scope):
        self.metrics_title.set_text(scope)
        for i in range(METRIC_ROWS):
            lbl, val = rows[i] if i < len(rows) else ("", "")
            self.m_lbl[i].set_text(lbl); self.m_val[i].set_text(val)

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
        if key in self.L:
            self.L[key].set_data(t, v)

    def set_track(self, xs, ys, heading_deg):
        self.L["trk"].set_data(xs, ys)
        ax = self.ax["trk"]
        pos = ax.get_position()
        r = (pos.height * self.H) / (pos.width * self.W)
        if xs:
            self.L["pos"].set_data([xs[-1]], [ys[-1]])
            x0, x1, y0, y1 = min(xs + [0]), max(xs + [0]), min(ys + [0]), max(ys + [0])
        else:
            x0 = x1 = y0 = y1 = 0.0
        hx = max(25.0, (x1 - x0) / 2 * 1.12, (y1 - y0) / 2 * 1.12 / r)
        hy = hx * r
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ax.set_xlim(cx - hx, cx + hx); ax.set_ylim(cy - hy, cy + hy)
        if xs and isnum(heading_deg):
            L_ = hx * 0.12
            self.L["hdg_arrow"].set_data([xs[-1], xs[-1] + L_ * math.sin(math.radians(heading_deg))],
                                         [ys[-1], ys[-1] + L_ * math.cos(math.radians(heading_deg))])

    def set_mode_markers(self, changes, xmin, label_axes=None):
        self.mode_changes = list(changes)

    def rescale(self, xmin, xmax):
        for k, ax in self.ax.items():
            if k == "trk":
                continue
            ax.set_xlim(xmin, xmax)
            ax.set_xticks(time_ticks(xmin, xmax, target=5))
            ax.relim(); ax.autoscale_view(scalex=False)
            lo, hi = ax.get_ylim()
            if hi - lo < 1e-6:
                ax.set_ylim(lo - 1, hi + 1)
        # mode timeline + faint dividers in every panel
        for a in self._mode_arts:
            a.remove()
        self._mode_arts = []
        b = self.band
        b.set_xlim(xmin, xmax)
        segs = []
        ch = sorted(self.mode_changes)
        for i, (t, mode) in enumerate(ch):
            end = ch[i + 1][0] if i + 1 < len(ch) else xmax
            a, z = max(t, xmin), min(end, xmax)
            if z > a:
                segs.append((a, z, mode))
        px_per_s = b.get_position().width * self.W / max(1e-6, xmax - xmin)
        for i, (a, z, mode) in enumerate(segs):
            self._mode_arts.append(b.axvspan(a, z, color=T["band_a"] if i % 2 == 0 else T["band_b"], linewidth=0))
            if (z - a) * px_per_s >= 6.2 * len(mode) + 8:
                self._mode_arts.append(b.text((a + z) / 2, 0.5, mode, ha="center", va="center", fontsize=8,
                                              color=T["ink"], clip_on=True))
            if a > xmin:
                for k, ax in self.ax.items():
                    if k != "trk":
                        self._mode_arts.append(ax.axvline(a, color="#c9c7bd", linewidth=0.6,
                                                          linestyle=(0, (2, 2)), zorder=1))


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
        for key, (_, _, ser, _) in PANELS.items():
            for dk, *_ in ser:
                if key == "yaw" and not st.have_pid:
                    continue
                db.set_series(dk, *series(st, dk))
        if not st.have_pid:
            db.set_series("yach", *series(st, "yawrate"))
            db.L["ydes"].set_data([], [])
            db.yaw_note.set_text("actual only · run with --pid for the command")
        else:
            db.yaw_note.set_text("")
        xs, ys = ([p[0] for p in st.track], [p[1] for p in st.track]) if st.track else ([], [])
        db.set_track(xs, ys, st.latest["yaw"])
        stale = (time.time() - st.last_rx) > 3 if st.connected else True
        db.set_status("link", "no data" if stale else "live", "critical" if stale else "good", "✕" if stale else "●")
        db.set_status("armed", "armed" if st.armed else "disarmed", "warning" if st.armed else None,
                      "●" if st.armed else "○")
        db.set_status("mode", st.mode, None, "◆")
        db.h["clock"].set_text(mmss(now))
        recent = [r for r in st.rows if now - r["t"] <= window_s]
        m = research_metrics(recent)
        db.set_metrics(metric_rows(m, st.energy_wh, st.armed_time, st.dist_m),
                       f"live, last {window_s:.0f} s")
        db.set_log([(mmss(t), msg) for t, msg in reversed(st.msgs)])
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
                    t = st.now()
                    w.writerow([f"{t:.2f}", st.mode, int(st.armed)] + [st.latest[k] for k in SCALARS])
                    st.rows.append(dict(t=t, mode=st.mode, armed=int(st.armed), **st.latest))
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


def summary_text(s, title, m=None):
    lines = [
        f"Flight test summary: {title}",
        "modes flown (armed)  " + "\n                     ".join(textwrap.wrap(" > ".join(s["modes"]) or "-", 52)),
        f"armed time           {s['armed_s']:.0f} s",
        f"distance             {s['dist']:.0f} m",
        f"altitude             min {fmt(s['alt_min'])}, max {fmt(s['alt_max'])} m above home",
        f"max groundspeed      {fmt(s['gs_max'])} m/s",
        f"max airspeed         {fmt(s['as_max'])} m/s",
        f"mean throttle        {fmt(s['thr_mean'], '{:.0f}')} %",
        f"max |roll| |pitch|   {fmt(s['roll_max'])} / {fmt(s['pitch_max'])} deg",
        f"charge used          {s['mah']:.0f} mAh   (SITL battery model, not calibrated)",
        f"energy used          {s['wh']:.2f} Wh   (SITL battery model, not calibrated)",
    ]
    if m:
        lines += [
            "",
            "Research metrics (armed samples)",
            f"roll error RMS       {fmt(m['roll_rms'], '{:.2f}')} deg   (actual - commanded)",
            f"pitch error RMS      {fmt(m['pitch_rms'], '{:.2f}')} deg",
            f"yaw-rate error RMS   {fmt(m['yaw_rms'], '{:.3f}')} deg/s" + ("" if isnum(m["yaw_rms"]) else "   (run with --pid)"),
            f"lift motors at max   {fmt(m['sat_pct'])} % of VTOL time   (any motor within 20 us of PWM max)",
            f"lift motor spread    {fmt(m['spread'], '{:.0f}')} us mean   (highest minus lowest motor)",
            f"hover motor output   {fmt(m['hover'], '{:.3f}')} of PWM range   (VTOL, |climb| < 0.3 m/s, groundspeed < 3 m/s)",
            f"min voltage          {fmt(m['v_min'], '{:.2f}')} V",
            f"max current          {fmt(m['i_max'])} A",
        ]
    return "\n".join(lines)


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
    m = research_metrics(rows)
    db = Dashboard(title, f"{name} · post-flight report · {len(rows)} samples at 10 Hz")
    t = [r["t"] for r in rows]
    have_pid = any(isnum(r["ydes"]) for r in rows)
    for key, (_, _, ser, _) in PANELS.items():
        for dk, *_ in ser:
            db.set_series(dk, t, [r[dk] for r in rows])
    if not have_pid:
        db.set_series("yach", t, [r["yawrate"] for r in rows])
        db.L["ydes"].set_data([], [])
        db.yaw_note.set_text("actual only · flown without --pid")
    xs, ys = track_from_latlon(rows)
    last_yaw = next((r["yaw"] for r in reversed(rows) if isnum(r["yaw"])), math.nan)
    db.set_track(xs, ys, last_yaw)
    db.set_status("link", "report", None, "■")
    db.set_status("armed", f"{mmss(s['armed_s'])} armed", None, "○")
    db.set_status("mode", s["modes"][-1] if s["modes"] else "-", None, "◆")
    db.h["clock"].set_text(mmss(t[-1] - t[0]) if t else "0:00")
    db.set_metrics(metric_rows(m, s["wh"], s["armed_s"], s["dist"]), "whole flight")
    wrapped = textwrap.wrap(" > ".join(s["modes"]) or "-", db.log_wrap - 2) or ["-"]
    notes = [("modes" if i == 0 else "", ln) for i, ln in enumerate(wrapped)]
    notes += [("altitude", f"max {fmt(s['alt_max'])} m, climb {fmt(s['climb_max'])} m/s, sink {fmt(s['sink_max'])} m/s"),
              ("speed", f"max airspeed {fmt(s['as_max'])} m/s, groundspeed {fmt(s['gs_max'])} m/s")]
    db.log_sub.set_text("flight notes")
    db.set_log(notes)
    changes, prev = [], None
    for r in rows:
        if r["armed"] == 1 and r["mode"] != prev:
            changes.append((r["t"], r["mode"]))
        prev = r["mode"] if r["armed"] == 1 else None
    if t:
        db.set_mode_markers(changes, t[0])
        db.rescale(t[0], t[-1])
    db.fig.savefig(png_path, dpi=110)
    db.fig.savefig(png_path[:-4] + ".pdf")          # vector copy for papers and the thesis
    plt.close(db.fig)
    return s, m


def make_report(csv_path, title="SITL"):
    base = csv_path[:-4] if csv_path.endswith(".csv") else csv_path
    name = os.path.basename(base)
    rows = load_rows(csv_path)
    s, m = report_png(rows, base + ".png", title, name)
    text = summary_text(s, name, m)
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
        print(f"\nSaved: {base}.csv\n       {base}.png\n       {base}.pdf\n       {base}_summary.txt")


if __name__ == "__main__":
    main()
