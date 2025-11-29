#!/usr/bin/env python3
"""
Audio → PWM Servo + Triple Plasma 2040 (white/ red + pattern) + JSON export + OSC

Features
- Plays L/R audio files with per-channel volume, loop, gain & soft limiter
- Scans RMS envelope once, normalises to [0..1] using percentiles
- Maps audio RMS → shared_norm → Servo duty [MinDuty..MaxDuty]
- Maps RMS → LED brightness with:
    * floor & max brightness
    * gamma
    * smoothing
    * optional pixel shuffle (odd/even pattern, all pixels lit)
    * optional red hue when direction reversed
- Optional behaviour:
    * Flip servo direction on "silence" (RMS below threshold for X seconds),
      doing a smooth ramp down → flip → ramp up
- Exports a JSON with RMS analysis and mapping/hardware config
- OSC:
    * /show/start  -> same as clicking "Play & Run"
    * /show/stop   -> same as clicking "Stop"

Notes
- Playback uses a low-passed copy of the audio (~18 kHz) so any ultrasonic
  energy used for servo-driving is reduced at the speakers.
- L/R volumes are auto-calibrated from channel RMS for more balanced loudness.
- Settings autosave to ~/.showplayer_settings.json and reload on startup.
"""

import os, glob, time, json, threading, datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import soundfile as sf
import sounddevice as sd
import serial
import RPi.GPIO as GPIO

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer

SETTINGS_PATH = os.path.expanduser("~/.showplayer_settings.json")

# ------------------------------ Serial helpers ------------------------------

def _find_serial_candidates():
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_id:
        real = [os.path.realpath(p) for p in by_id]
        return real + by_id
    return sorted(list(glob.glob("/dev/ttyACM*")) + list(glob.glob("/dev/ttyUSB*")))

def _drain(ser, wait=0.05):
    time.sleep(wait)
    out = b""
    try:
        while ser.in_waiting:
            out += ser.read(ser.in_waiting)
            time.sleep(0.02)
    except Exception:
        pass
    return out

def _repl_break_and_soft_reset(ser, do_soft=True):
    try:
        ser.write(b'\x03'); time.sleep(0.08)
        ser.write(b'\x03'); time.sleep(0.25)
        if do_soft:
            ser.write(b'\x04'); time.sleep(1.2)
        ser.write(b'\r\n'); time.sleep(0.2)
        _drain(ser)
    except Exception:
        pass

def _send_line(ser, line, think=0.25):
    """Send a line and wait, returning (ok, response_text)."""
    try:
        ser.write(b'\r\n'); time.sleep(0.02)
        ser.write(line.encode('utf-8') + b'\r\n')
        txt = _drain(ser, wait=think).decode('utf-8', 'ignore')
        if "Traceback" in txt or "Error:" in txt:
            return False, txt
        return True, txt
    except Exception as e:
        return False, f"<send_error {e}>"

def _send_line_fast(ser, line):
    """Fire-and-forget sender (not used for LEDs anymore, only if needed)."""
    try:
        ser.write(b'\r\n')
        time.sleep(0.002)
        ser.write(line.encode('utf-8') + b'\r\n')
        return True, ""
    except Exception as e:
        return False, f"<send_fast_error {e}>"

def _paste_block(ser, lines, think=0.4):
    try:
        ser.write(b'\x05'); time.sleep(0.05)  # Ctrl-E
        for ln in lines:
            ser.write(ln.encode('utf-8') + b'\r\n')
        ser.write(b'\x04')  # Ctrl-D
        txt = _drain(ser, wait=think).decode('utf-8', 'ignore')
        if "Traceback" in txt or "Error:" in txt:
            return False, txt
        return True, txt
    except Exception as e:
        return False, f"<paste_error {e}>"

# -------------------------- RMS envelope helper -----------------------------

def compute_rms_envelope(stereo, window=4096, hop=2048):
    n = stereo.shape[0]
    env = []
    for start in range(0, n, hop):
        end = min(start + window, n)
        seg = stereo[start:end, :]
        if seg.size == 0:
            break
        mono = seg.mean(axis=1)
        env.append(float(np.sqrt(np.mean(mono**2) + 1e-12)))
    if not env:
        env = [0.0]
    return np.array(env, dtype=np.float32)

# ------------------------------ Main App ------------------------------------

class AudioPWMDualPlasmaApp:
    def __init__(self, root):
        self.root = root
        root.title("Audio → PWM Servo + Triple Plasma 2040 + JSON export + OSC")
        root.geometry("1020x900")
        root.resizable(False, False)

        # --- Audio file selection ---
        self.left_path = None
        self.right_path = None
        self.left_file_var = tk.StringVar(value="No file selected")
        self.right_file_var = tk.StringVar(value="No file selected")

        # --- Audio device & playback state ---
        self.device_var = tk.StringVar(value="")
        self.loop_enabled = tk.BooleanVar(value=False)
        self.left_volume = tk.DoubleVar(value=1.0)
        self.right_volume = tk.DoubleVar(value=1.0)

        self.audio_data = None      # full-band audio used for analysis
        self.playback_data = None   # low-passed copy used for playback
        self.sr = None
        self.current_frame = 0
        self.is_playing = False

        # envelope + mapping
        self.env = None
        self.env_min = 0.0
        self.env_max = 1.0
        self.env_hop = 2048
        self.scan_done = False
        self.current_env_value = 0.0
        self.led_env_low = None       # percentile low (for norm)
        self.led_env_high = None      # percentile high (for norm)
        self.current_norm = 0.0       # shared 0..1 mapping from RMS

        # channel RMS for calibration
        self.rms_L = None
        self.rms_R = None

        # --- Servo PWM mapping (analog speed) ---
        self.min_duty = tk.DoubleVar(value=5.0)
        self.max_duty = tk.DoubleVar(value=40.0)
        self.stop_decel_time = tk.DoubleVar(value=1.0)
        self.sustain_time = tk.DoubleVar(value=3.0)  # UI only now
        self._sustain_counter = 0

        # GPIO pins
        self.pwm_pin = 18  # BCM18 hardware PWM
        self.dir_pin = 19
        self.en_pin = 20
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.pwm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.en_pin, GPIO.OUT, initial=GPIO.LOW)
        self.pwm_freq_hz = 5000
        self.pwm = GPIO.PWM(self.pwm_pin, self.pwm_freq_hz)
        self.pwm_running = False
        self.servo_enabled_var = tk.BooleanVar(value=False)
        self.servo_dir_cw_var = tk.BooleanVar(value=True)
        self.current_duty = 0.0
        self.servo_peak_duty = 0.0

        # --- LED mapping (global brightness + pattern) ---
        self.led_floor = tk.DoubleVar(value=0.00)    # 0 → full dark at silence
        self.led_max = tk.DoubleVar(value=0.60)      # cap
        self.led_gamma = tk.DoubleVar(value=0.8)     # gamma < 1 => more nuance
        self.led_smooth_time = tk.DoubleVar(value=0.3)  # seconds (LED low-pass)
        self._led_smooth_norm = 0.0                 # internal smoothed feature
        self.current_led_norm = 0.0                 # what we send to brightness mapper

        # --- Behaviour options ---
        self.flip_on_silence_var = tk.BooleanVar(value=False)
        self.red_on_reverse_var = tk.BooleanVar(value=False)
        self.pixel_shuffle_var = tk.BooleanVar(value=False)

        # Silence detection params for direction flip
        self.silence_threshold = 0.02   # shared_norm below this = "silence"
        self.silence_min_time = 0.6     # seconds below threshold before flip

        # Pixel pattern state (0 => even bright, 1 => odd bright)
        self._pattern_mode = 0

        # --- Plasma 2040 state (up to 3) ---
        self.num_leds = tk.IntVar(value=120)
        self.keepalive_enabled = tk.BooleanVar(value=False)  # keepalive OFF by default
        self.devices = []  # list of {"ser": Serial, "port": str}
        self.ser_lock = threading.Lock()

        # --- Threads & flags ---
        self.hardware_running = False
        self.servo_thread = None
        self.led_thread = None

        # --- Status ---
        self.status_var = tk.StringVar(value="Ready")

        # --- OSC control state ---
        self.osc_enabled_var = tk.BooleanVar(value=False)
        self.osc_bind_ip = tk.StringVar(value="0.0.0.0")
        self.osc_port = tk.IntVar(value=9000)
        self.osc_start_path = tk.StringVar(value="/show/start")
        self.osc_stop_path = tk.StringVar(value="/show/stop")
        self.osc_server = None
        self.osc_thread = None

        # --- Numeric label vars for sliders ---
        self.left_vol_label = tk.StringVar(value="1.00")
        self.right_vol_label = tk.StringVar(value="1.00")
        self.led_floor_label = tk.StringVar(value="0.00")
        self.led_max_label = tk.StringVar(value="0.60")
        self.led_gamma_label = tk.StringVar(value="0.80")
        self.led_smooth_label = tk.StringVar(value="0.30 s")
        self.gain_db_label = tk.StringVar(value="0.0 dB")
        self.min_duty_label = tk.StringVar(value="5.0 %")
        self.max_duty_label = tk.StringVar(value="40.0 %")
        self.sustain_time_label = tk.StringVar(value="3.0 s")

        # autosave machinery
        self._save_pending = False

        self._build_ui()
        self._load_audio_devices()
        self._setup_autosave_traces()
        self._load_settings()
        self._ui_tick()

    # ------------------------------ UI layout --------------------------------
    def _build_ui(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        t1 = ttk.Frame(nb, padding=10)
        t2 = ttk.Frame(nb, padding=10)
        t3 = ttk.Frame(nb, padding=10)
        t4 = ttk.Frame(nb, padding=10)
        t5 = ttk.Frame(nb, padding=10)

        nb.add(t1, text="1) Select & Scan")
        nb.add(t2, text="2) Mapping")
        nb.add(t3, text="3) Play & Hardware")
        nb.add(t4, text="Audio Gain")
        nb.add(t5, text="Export JSON")

        # --- Tab 1: Select & Scan ---
        tk.Label(
            t1,
            text="Select Left/Right audio files, then Scan RMS envelope",
            font=("Arial", 13, "bold"),
        ).pack(pady=(0, 8))

        lf = ttk.LabelFrame(t1, text="Left Channel", padding=10)
        lf.pack(fill=tk.X, pady=5)
        ttk.Label(lf, textvariable=self.left_file_var, foreground="blue").pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(lf, text="Browse...", command=self._browse_left).pack(
            side=tk.RIGHT
        )

        rf = ttk.LabelFrame(t1, text="Right Channel", padding=10)
        rf.pack(fill=tk.X, pady=5)
        ttk.Label(rf, textvariable=self.right_file_var, foreground="blue").pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(rf, text="Browse...", command=self._browse_right).pack(
            side=tk.RIGHT
        )

        dev = ttk.LabelFrame(t1, text="Audio Output Device", padding=10)
        dev.pack(fill=tk.X, pady=8)
        self.device_combo = ttk.Combobox(
            dev, textvariable=self.device_var, state="readonly", width=50
        )
        self.device_combo.pack(side=tk.LEFT, padx=5)
        self.device_combo.bind("<<ComboboxSelected>>", lambda e: self._schedule_save_settings())
        ttk.Button(dev, text="Refresh", command=self._load_audio_devices).pack(
            side=tk.RIGHT
        )

        ttk.Button(
            t1,
            text="🔎 Scan RMS Envelope",
            command=self._scan_audio,
            width=24,
        ).pack(pady=8)

        self.scan_info_var = tk.StringVar(value="No scan yet.")
        ttk.Label(t1, textvariable=self.scan_info_var, foreground="gray").pack()

        # --- Tab 2: Mapping ---
        tk.Label(
            t2,
            text="Map RMS envelope → Servo PWM & LED brightness",
            font=("Arial", 13, "bold"),
        ).pack(pady=(0, 8))

        sfm = ttk.LabelFrame(t2, text="Servo PWM (analog speed)", padding=10)
        sfm.pack(fill=tk.X, pady=6)

        ttk.Label(sfm, text="Min duty (%):").grid(row=0, column=0, sticky="w")
        ttk.Scale(
            sfm,
            from_=0.0,
            to=100.0,
            variable=self.min_duty,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(sfm, textvariable=self.min_duty_label).grid(row=0, column=2, sticky="w")

        ttk.Label(sfm, text="Max duty (%):").grid(row=1, column=0, sticky="w", pady=(4,0))
        ttk.Scale(
            sfm,
            from_=0.0,
            to=100.0,
            variable=self.max_duty,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=1, column=1, sticky="w", padx=4, pady=(4,0))
        ttk.Label(sfm, textvariable=self.max_duty_label).grid(row=1, column=2, sticky="w")

        ttk.Label(
            t2,
            text="(0% ≈ 0V, 100% ≈ 3.3V to servo speed input)",
            foreground="gray",
        ).pack(anchor=tk.W, pady=(4, 0))

        sust = ttk.LabelFrame(t2, text="Servo sustain (UI only for now)", padding=10)
        sust.pack(fill=tk.X, pady=6)
        ttk.Label(sust, text="Sustain time (s above high level):").pack(side=tk.LEFT)
        ttk.Scale(
            sust,
            from_=0.0,
            to=5.0,
            variable=self.sustain_time,
            orient=tk.HORIZONTAL,
            length=220,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(sust, textvariable=self.sustain_time_label).pack(side=tk.LEFT, padx=4)

        lf2 = ttk.LabelFrame(
            t2, text="LED brightness (white/red, all pixels or pattern)", padding=10
        )
        lf2.pack(fill=tk.X, pady=6)

        ttk.Label(lf2, text="Brightness floor:").grid(row=0, column=0, sticky="w")
        ttk.Scale(
            lf2,
            from_=0.00,
            to=0.40,
            variable=self.led_floor,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=0, column=1, sticky="w", padx=4)
        ttk.Label(lf2, textvariable=self.led_floor_label).grid(row=0, column=2, sticky="w")

        ttk.Label(lf2, text="Brightness max:").grid(row=1, column=0, sticky="w", pady=(4,0))
        ttk.Scale(
            lf2,
            from_=0.50,
            to=0.90,
            variable=self.led_max,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=1, column=1, sticky="w", padx=4, pady=(4,0))
        ttk.Label(lf2, textvariable=self.led_max_label).grid(row=1, column=2, sticky="w")

        ttk.Label(lf2, text="LED response (gamma):").grid(row=2, column=0, sticky="w", pady=(4,0))
        ttk.Scale(
            lf2,
            from_=0.3,
            to=2.0,
            variable=self.led_gamma,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=2, column=1, sticky="w", padx=4, pady=(4,0))
        ttk.Label(lf2, textvariable=self.led_gamma_label).grid(row=2, column=2, sticky="w")

        ttk.Label(lf2, text="LED smoothing time (s):").grid(row=3, column=0, sticky="w", pady=(4,0))
        ttk.Scale(
            lf2,
            from_=0.0,
            to=2.0,
            variable=self.led_smooth_time,
            orient=tk.HORIZONTAL,
            length=180,
        ).grid(row=3, column=1, sticky="w", padx=4, pady=(4,0))
        ttk.Label(lf2, textvariable=self.led_smooth_label).grid(row=3, column=2, sticky="w")

        ttk.Label(
            lf2,
            text="(LEDs follow RMS: quiet → dark, loud → bright, with floor/max, smoothing & gamma.)",
            foreground="gray",
        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(4,0))

        stopf = ttk.LabelFrame(t2, text="Stop behavior", padding=10)
        stopf.pack(fill=tk.X, pady=6)
        ttk.Label(stopf, text="Stop decel time (s):").pack(side=tk.LEFT)
        ttk.Scale(
            stopf,
            from_=0.20,
            to=3.00,
            variable=self.stop_decel_time,
            orient=tk.HORIZONTAL,
            length=220,
        ).pack(side=tk.LEFT, padx=6)

        # Behaviour options
        beh = ttk.LabelFrame(t2, text="Reactive Behaviour (optional)", padding=10)
        beh.pack(fill=tk.X, pady=6)

        ttk.Checkbutton(
            beh,
            text="Flip servo direction when RMS is silent",
            variable=self.flip_on_silence_var,
        ).grid(row=0, column=0, sticky="w")

        ttk.Checkbutton(
            beh,
            text="Use red hue when direction is reversed",
            variable=self.red_on_reverse_var,
        ).grid(row=1, column=0, sticky="w", pady=(4, 0))

        ttk.Checkbutton(
            beh,
            text="Odd/even pixel shuffle (moving pattern)",
            variable=self.pixel_shuffle_var,
        ).grid(row=2, column=0, sticky="w", pady=(4, 0))

        # --- Tab 3: Play & Hardware ---
        tk.Label(
            t3,
            text="Playback + PWM + Triple Plasma 2040",
            font=("Arial", 13, "bold"),
        ).pack(pady=(0, 8))

        vol = ttk.Frame(t3)
        vol.pack(fill=tk.X, pady=4)
        ttk.Label(vol, text="Left Vol").pack(side=tk.LEFT)
        ttk.Scale(
            vol,
            from_=0,
            to=2.0,
            variable=self.left_volume,
            orient=tk.HORIZONTAL,
            length=200,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(vol, textvariable=self.left_vol_label).pack(side=tk.LEFT)

        ttk.Label(vol, text="Right Vol").pack(side=tk.LEFT, padx=(16,0))
        ttk.Scale(
            vol,
            from_=0,
            to=2.0,
            variable=self.right_volume,
            orient=tk.HORIZONTAL,
            length=200,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Label(vol, textvariable=self.right_vol_label).pack(side=tk.LEFT)

        ttk.Checkbutton(vol, text="🔁 Loop", variable=self.loop_enabled).pack(
            side=tk.LEFT, padx=10
        )

        ledf = ttk.LabelFrame(
            t3, text="Plasma 2040 (up to 3 boards)", padding=10
        )
        ledf.pack(fill=tk.X, pady=6)
        self.plasma_status = ttk.Label(
            ledf, text="Disconnected", foreground="red"
        )
        self.plasma_status.pack(side=tk.LEFT)

        ttk.Button(ledf, text="Connect all", command=self._connect_all_plasma).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Button(ledf, text="Recover", command=self._recover_all).pack(
            side=tk.RIGHT, padx=4
        )
        ttk.Checkbutton(
            ledf, text="LED keepalive", variable=self.keepalive_enabled
        ).pack(side=tk.RIGHT, padx=8)
        ttk.Label(ledf, text="LED count:").pack(side=tk.LEFT, padx=(10, 2))
        ttk.Entry(ledf, textvariable=self.num_leds, width=6).pack(
            side=tk.LEFT, padx=2
        )

        svf = ttk.LabelFrame(t3, text="Servo PWM (analog speed)", padding=10)
        svf.pack(fill=tk.X, pady=6)
        ttk.Checkbutton(
            svf,
            text="Enable servo",
            variable=self.servo_enabled_var,
            command=self._toggle_servo,
        ).pack(side=tk.LEFT, padx=(2, 10))
        ttk.Radiobutton(
            svf,
            text="CW",
            variable=self.servo_dir_cw_var,
            value=True,
            command=self._update_servo_dir,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            svf,
            text="CCW",
            variable=self.servo_dir_cw_var,
            value=False,
            command=self._update_servo_dir,
        ).pack(side=tk.LEFT, padx=(4, 10))
        self.servo_speed_label = ttk.Label(
            svf, text="Duty: 0.0 % (~0.00 V)"
        )
        self.servo_speed_label.pack(side=tk.RIGHT)

        tracker = ttk.Frame(t3)
        tracker.pack(fill=tk.X, pady=(2, 8))
        self.servo_peak_label = ttk.Label(
            tracker, text="Peak duty: 0.0 %"
        )
        self.servo_peak_label.pack(side=tk.LEFT, padx=(2, 10))
        self.servo_util_label = ttk.Label(
            tracker, text="Utilization: 0.0 % of MaxDuty"
        )
        self.servo_util_label.pack(side=tk.LEFT)

        # LED brightness tracker
        self.led_level_label = ttk.Label(
            tracker, text="LED: 0.0 % of max"
        )
        self.led_level_label.pack(side=tk.LEFT, padx=(16, 0))

        ttk.Button(tracker, text="Reset Peak", command=self._reset_servo_peak).pack(
            side=tk.RIGHT
        )

        btns = ttk.Frame(t3)
        btns.pack(pady=8)
        ttk.Button(
            btns,
            text="▶ Play & Run",
            command=self._play_and_run,
            width=16,
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            btns,
            text="⛔ Stop",
            command=self._stop_all,
            width=16,
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            btns,
            text="Clear LEDs",
            command=self._clear_leds,
            width=12,
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            btns,
            text="Test LED ramp",
            command=self._test_led_ramp,
            width=14,
        ).pack(side=tk.LEFT, padx=5)

        # --- OSC Remote Control section in Tab 3 ---
        osc = ttk.LabelFrame(t3, text="OSC Remote Control (optional)", padding=10)
        osc.pack(fill=tk.X, pady=(4, 4))

        ttk.Checkbutton(
            osc,
            text="Enable OSC listener",
            variable=self.osc_enabled_var,
            command=self._toggle_osc,
        ).grid(row=0, column=0, columnspan=4, sticky=tk.W, pady=(0, 6))

        ttk.Label(osc, text="Bind IP:").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(osc, textvariable=self.osc_bind_ip, width=16).grid(
            row=1, column=1, sticky=tk.W, padx=4
        )
        ttk.Label(osc, text="Port:").grid(row=1, column=2, sticky=tk.W)
        ttk.Entry(osc, textvariable=self.osc_port, width=8).grid(
            row=1, column=3, sticky=tk.W, padx=4
        )

        ttk.Label(osc, text="Start path:").grid(row=2, column=0, sticky=tk.W, pady=(4, 0))
        ttk.Entry(osc, textvariable=self.osc_start_path, width=16).grid(
            row=2, column=1, sticky=tk.W, padx=4, pady=(4, 0)
        )
        ttk.Label(osc, text="Stop path:").grid(row=2, column=2, sticky=tk.W, pady=(4, 0))
        ttk.Entry(osc, textvariable=self.osc_stop_path, width=16).grid(
            row=2, column=3, sticky=tk.W, padx=4, pady=(4, 0)
        )

        # --- Tab 4: Audio Gain ---
        tk.Label(
            t4,
            text="Audio Output Gain & Limiter",
            font=("Arial", 13, "bold"),
        ).pack(pady=(0, 8))
        gfrm = ttk.LabelFrame(t4, text="Gain & Limiter", padding=10)
        gfrm.pack(fill=tk.X, pady=6)
        self.output_gain_db = tk.DoubleVar(value=0.0)
        self.soft_limiter = tk.BooleanVar(value=True)
        ttk.Label(gfrm, text="Output Gain (dB):").pack(side=tk.LEFT)
        ttk.Scale(
            gfrm,
            from_=-12,
            to=+12,
            variable=self.output_gain_db,
            orient=tk.HORIZONTAL,
            length=260,
        ).pack(side=tk.LEFT, padx=6)
        ttk.Label(gfrm, textvariable=self.gain_db_label).pack(side=tk.LEFT, padx=4)
        ttk.Checkbutton(
            gfrm, text="Soft limiter (tanh)", variable=self.soft_limiter
        ).pack(side=tk.LEFT, padx=10)

        # --- Tab 5: Export JSON ---
        tk.Label(
            t5,
            text="Export RMS + mapping + hardware config to JSON",
            font=("Arial", 13, "bold"),
        ).pack(pady=(0, 8))
        ttk.Button(
            t5,
            text="💾 Export Show JSON",
            command=self._export_json,
            width=24,
        ).pack(pady=6)
        self.export_info = ttk.Label(
            t5, text="No export yet.", foreground="gray"
        )
        self.export_info.pack()

        # Status bar
        sb = ttk.Frame(self.root)
        sb.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=5)
        ttk.Label(
            sb,
            textvariable=self.status_var,
            relief=tk.SUNKEN,
            anchor=tk.W,
        ).pack(fill=tk.X)

    # ------------------------ Autosave: traces & load/save -------------------

    def _setup_autosave_traces(self):
        vars_to_trace = [
            self.left_volume, self.right_volume,
            self.min_duty, self.max_duty,
            self.sustain_time, self.stop_decel_time,
            self.led_floor, self.led_max, self.led_gamma,
            self.led_smooth_time,
            self.num_leds, self.keepalive_enabled,
            self.osc_enabled_var, self.osc_bind_ip,
            self.osc_port, self.osc_start_path, self.osc_stop_path,
            self.output_gain_db, self.soft_limiter,
            self.loop_enabled, self.device_var,
            self.flip_on_silence_var, self.red_on_reverse_var,
            self.pixel_shuffle_var,
        ]
        for v in vars_to_trace:
            try:
                v.trace_add("write", lambda *args: self._schedule_save_settings())
            except Exception:
                pass

    def _schedule_save_settings(self):
        if self._save_pending:
            return
        self._save_pending = True
        self.root.after(500, self._save_settings)

    def _settings_snapshot(self):
        return {
            "left_path": self.left_path,
            "right_path": self.right_path,
            "device": self.device_var.get(),
            "loop": bool(self.loop_enabled.get()),
            "left_volume": float(self.left_volume.get()),
            "right_volume": float(self.right_volume.get()),
            "min_duty": float(self.min_duty.get()),
            "max_duty": float(self.max_duty.get()),
            "sustain_time": float(self.sustain_time.get()),
            "stop_decel_time": float(self.stop_decel_time.get()),
            "led_floor": float(self.led_floor.get()),
            "led_max": float(self.led_max.get()),
            "led_gamma": float(self.led_gamma.get()),
            "led_smooth_time": float(self.led_smooth_time.get()),
            "num_leds": int(self.num_leds.get()),
            "keepalive": bool(self.keepalive_enabled.get()),
            "osc_enabled": bool(self.osc_enabled_var.get()),
            "osc_bind_ip": self.osc_bind_ip.get(),
            "osc_port": int(self.osc_port.get()),
            "osc_start_path": self.osc_start_path.get(),
            "osc_stop_path": self.osc_stop_path.get(),
            "output_gain_db": float(self.output_gain_db.get()),
            "soft_limiter": bool(self.soft_limiter.get()),
            "flip_on_silence": bool(self.flip_on_silence_var.get()),
            "red_on_reverse": bool(self.red_on_reverse_var.get()),
            "pixel_shuffle": bool(self.pixel_shuffle_var.get()),
        }

    def _save_settings(self):
        self._save_pending = False
        cfg = self._settings_snapshot()
        try:
            with open(SETTINGS_PATH, "w") as f:
                json.dump(cfg, f, indent=2)
        except Exception:
            pass

    def _load_settings(self):
        if not os.path.exists(SETTINGS_PATH):
            return
        try:
            with open(SETTINGS_PATH, "r") as f:
                cfg = json.load(f)
        except Exception:
            return
        try:
            lp = cfg.get("left_path")
            if lp and os.path.exists(lp):
                self.left_path = lp
                self.left_file_var.set(Path(lp).name)
            rp = cfg.get("right_path")
            if rp and os.path.exists(rp):
                self.right_path = rp
                self.right_file_var.set(Path(rp).name)
            dev = cfg.get("device")
            if dev:
                self.device_var.set(dev)
            self.loop_enabled.set(bool(cfg.get("loop", False)))
            self.left_volume.set(float(cfg.get("left_volume", 1.0)))
            self.right_volume.set(float(cfg.get("right_volume", 1.0)))
            self.min_duty.set(float(cfg.get("min_duty", 5.0)))
            self.max_duty.set(float(cfg.get("max_duty", 40.0)))
            self.sustain_time.set(float(cfg.get("sustain_time", 3.0)))
            self.stop_decel_time.set(float(cfg.get("stop_decel_time", 1.0)))
            self.led_floor.set(float(cfg.get("led_floor", 0.00)))
            self.led_max.set(float(cfg.get("led_max", 0.60)))
            self.led_gamma.set(float(cfg.get("led_gamma", 0.8)))
            self.led_smooth_time.set(float(cfg.get("led_smooth_time", 0.3)))
            self.num_leds.set(int(cfg.get("num_leds", 120)))
            self.keepalive_enabled.set(bool(cfg.get("keepalive", False)))
            self.osc_enabled_var.set(bool(cfg.get("osc_enabled", False)))
            self.osc_bind_ip.set(cfg.get("osc_bind_ip", "0.0.0.0"))
            self.osc_port.set(int(cfg.get("osc_port", 9000)))
            self.osc_start_path.set(cfg.get("osc_start_path", "/show/start"))
            self.osc_stop_path.set(cfg.get("osc_stop_path", "/show/stop"))
            self.output_gain_db.set(float(cfg.get("output_gain_db", 0.0)))
            self.soft_limiter.set(bool(cfg.get("soft_limiter", True)))
            self.flip_on_silence_var.set(bool(cfg.get("flip_on_silence", False)))
            self.red_on_reverse_var.set(bool(cfg.get("red_on_reverse", False)))
            self.pixel_shuffle_var.set(bool(cfg.get("pixel_shuffle", False)))
            self.status_var.set("Settings loaded from last session.")
        except Exception as e:
            self.status_var.set(f"Settings load error: {e}")

    # ----------------------------- Audio handling ----------------------------

    def _browse_left(self):
        fn = filedialog.askopenfilename(
            title="Select Left Channel Audio",
            filetypes=[
                ("Audio Files", "*.wav *.mp3 *.flac *.ogg"),
                ("All Files", "*.*"),
            ],
        )
        if fn:
            self.left_path = fn
            self.left_file_var.set(Path(fn).name)
            self.status_var.set(f"Left: {Path(fn).name}")
            self._schedule_save_settings()

    def _browse_right(self):
        fn = filedialog.askopenfilename(
            title="Select Right Channel Audio",
            filetypes=[
                ("Audio Files", "*.wav *.mp3 *.flac *.ogg"),
                ("All Files", "*.*"),
            ],
        )
        if fn:
            self.right_path = fn
            self.right_file_var.set(Path(fn).name)
            self.status_var.set(f"Right: {Path(fn).name}")
            self._schedule_save_settings()

    def _load_audio_devices(self):
        try:
            devs = sd.query_devices()
            outs = [
                f"{i}: {d['name']}"
                for i, d in enumerate(devs)
                if d.get("max_output_channels", 0) > 0
            ]
            self.device_combo["values"] = outs
            if outs and not self.device_var.get():
                default_out = sd.query_devices(kind="output")
                di = default_out["index"]
                for s in outs:
                    if s.startswith(f"{di}:"):
                        self.device_var.set(s)
                        break
        except Exception as e:
            self.status_var.set(f"Audio device query error: {e}")

    # -------------------------- Low-pass helper ------------------------------

    def _lowpass_for_playback(self, data, sr, cutoff=18000.0):
        """Simple first-order low-pass IIR applied once after loading."""
        if data is None or sr <= 0 or cutoff <= 0:
            return data
        rc = 1.0 / (2.0 * np.pi * cutoff)
        dt = 1.0 / float(sr)
        alpha = dt / (rc + dt)
        out = np.empty_like(data)
        for ch in range(data.shape[1]):
            y = 0.0
            xch = data[:, ch]
            och = out[:, ch]
            for i in range(len(xch)):
                x = float(xch[i])
                y = y + alpha * (x - y)
                och[i] = y
        return out

    # --------------------------- Scan RMS envelope ---------------------------

    def _scan_audio(self):
        if not self.left_path or not self.right_path:
            messagebox.showwarning(
                "Missing files",
                "Please select both Left and Right audio files first.",
            )
            return
        try:
            self.status_var.set("Loading audio...")
            L, srL = sf.read(self.left_path, dtype="float32")
            R, srR = sf.read(self.right_path, dtype="float32")
            if srL != srR:
                messagebox.showerror(
                    "Sample rate mismatch",
                    f"Left SR={srL}, Right SR={srR}. They must match.",
                )
                self.status_var.set("Scan aborted (SR mismatch).")
                return
            if L.ndim > 1:
                L = L.mean(axis=1)
            if R.ndim > 1:
                R = R.mean(axis=1)

            rms_L = float(np.sqrt(np.mean(L**2) + 1e-12))
            rms_R = float(np.sqrt(np.mean(R**2) + 1e-12))
            self.rms_L, self.rms_R = rms_L, rms_R

            n = max(len(L), len(R))
            if len(L) < n:
                L = np.pad(L, (0, n - len(L)))
            if len(R) < n:
                R = np.pad(R, (0, n - len(R)))
            self.sr = srL
            self.audio_data = np.column_stack([L, R]).astype(np.float32)
            self.current_frame = 0

            self.playback_data = self._lowpass_for_playback(
                self.audio_data, self.sr, cutoff=18000.0
            )

            self.status_var.set("Computing RMS envelope...")
            self.env = compute_rms_envelope(
                self.audio_data, window=4096, hop=self.env_hop
            )
            self.env_min = float(self.env.min())
            self.env_max = float(self.env.max())
            if abs(self.env_max - self.env_min) < 1e-9:
                self.env_max = self.env_min + 1e-3

            self.led_env_low = float(np.percentile(self.env, 10.0))
            self.led_env_high = float(np.percentile(self.env, 95.0))
            if self.led_env_high <= self.led_env_low:
                self.led_env_high = self.led_env_low + 1e-3

            self.scan_done = True
            dur = n / self.sr
            self.scan_info_var.set(
                f"Scan done: {len(self.env)} frames over {dur:.1f} s,\n"
                f"RMS range = [{self.env_min:.5f}, {self.env_max:.5f}]"
            )

            gL, gR = 1.0, 1.0
            if rms_L > 0 and rms_R > 0:
                ratio = rms_L / rms_R
                if ratio > 1.4:
                    gR = min(2.0, ratio)
                elif (1.0 / ratio) > 1.4:
                    gL = min(2.0, 1.0 / ratio)
            self.left_volume.set(gL)
            self.right_volume.set(gR)

            self.status_var.set(
                f"Scan completed. L/R RMS: {rms_L:.4f}/{rms_R:.4f}, "
                f"gL={gL:.2f}, gR={gR:.2f}"
            )

        except Exception as e:
            self.scan_done = False
            self.env = None
            messagebox.showerror("Scan error", str(e))
            self.status_var.set(f"Scan error: {e}")

    # -------------------------- Servo PWM control ----------------------------

    def _toggle_servo(self):
        enabled = self.servo_enabled_var.get()
        if enabled:
            GPIO.output(self.en_pin, GPIO.HIGH)
            if not self.pwm_running:
                self.pwm.start(0.0)
                self.pwm_running = True
            self.status_var.set("Servo enabled.")
        else:
            GPIO.output(self.en_pin, GPIO.LOW)
            try:
                if self.pwm_running:
                    self.pwm.ChangeDutyCycle(0.0)
                    self.pwm.stop()
                    self.pwm_running = False
            except Exception:
                pass
            GPIO.output(self.pwm_pin, GPIO.LOW)
            self.current_duty = 0.0
            self.status_var.set("Servo disabled.")

    def _update_servo_dir(self):
        cw = self.servo_dir_cw_var.get()
        GPIO.output(self.dir_pin, GPIO.HIGH if cw else GPIO.LOW)

    def _reset_servo_peak(self):
        self.servo_peak_duty = 0.0

    # ---------------------------- Triple Plasma 2040 -------------------------

    def _connect_all_plasma(self):
        def worker():
            try:
                self.plasma_status.config(text="Connecting...", foreground="orange")
                for dev in self.devices:
                    try:
                        dev["ser"].close()
                    except Exception:
                        pass
                self.devices = []

                candidates = _find_serial_candidates() or [
                    "/dev/ttyACM0",
                    "/dev/ttyACM1",
                    "/dev/ttyACM2",
                    "/dev/ttyUSB0",
                ]

                for port in candidates:
                    if len(self.devices) >= 3:
                        break
                    try:
                        ser = serial.Serial(
                            port,
                            115200,
                            timeout=1.5,
                            write_timeout=0.8,
                            inter_byte_timeout=0.2,
                        )
                        time.sleep(1.0)
                        _repl_break_and_soft_reset(ser, do_soft=True)
                        ok, _ = _send_line(ser, "import plasma", think=0.25)
                        if not ok:
                            raise RuntimeError("plasma import failed")

                        _send_line(ser, "from plasma import plasma2040", think=0.25)

                        led_count = int(self.num_leds.get())
                        ok3, _ = _send_line(
                            ser,
                            f"led_strip=plasma.WS2812({led_count},0,0,plasma2040.DAT)",
                            think=0.25,
                        )
                        if not ok3:
                            ok3, _ = _send_line(
                                ser,
                                f"led_strip=plasma.WS2812({led_count},0,0,15)",
                                think=0.25,
                            )
                            if not ok3:
                                raise RuntimeError("WS2812 init failed")

                        ok4, _ = _send_line(ser, "led_strip.start()", think=0.25)
                        if not ok4:
                            raise RuntimeError("led_strip.start failed")

                        helper = [
                            f"NUM={led_count}",
                            "_last=(0,0,0)",
                            "_pmode=0",
                            "_ka_run=False",
                            "def __set_all(r,g,b):",
                            " global _last",
                            " r=int(r); g=int(g); b=int(b)",
                            " if r<0:r=0",
                            " if g<0:g=0",
                            " if b<0:b=0",
                            " _last=(r,g,b)",
                            " for i in range(NUM):",
                            "  led_strip.set_rgb(i,r,g,b)",
                            "def __set_pattern(r1,g1,b1,r2,g2,b2,mode):",
                            " global _last,_pmode",
                            " r1=int(r1); g1=int(g1); b1=int(b1)",
                            " r2=int(r2); g2=int(g2); b2=int(b2)",
                            " if mode<0: mode=0",
                            " mode=mode%2",
                            " _pmode=mode",
                            " for i in range(NUM):",
                            "  if (i%2)==mode:",
                            "   led_strip.set_rgb(i,r1,g1,b1)",
                            "  else:",
                            "   led_strip.set_rgb(i,r2,g2,b2)",
                            " _last=(r1,g1,b1)",
                            "def __ka():",
                            " import time",
                            " global _ka_run,_last",
                            " while _ka_run:",
                            "  for i in range(NUM):",
                            "   led_strip.set_rgb(i,_last[0],_last[1],_last[2])",
                            "  time.sleep(0.3)",
                        ]
                        okh, txt = _paste_block(ser, helper, think=0.4)
                        if not okh:
                            raise RuntimeError(f"helper paste failed: {txt[:120]}")

                        if self.keepalive_enabled.get():
                            _send_line(ser, "_ka_run=True", think=0.2)
                            _send_line(
                                ser,
                                "import _thread; _thread.start_new_thread(__ka,())",
                                think=0.3,
                            )

                        # Start fully OFF (0,0,0)
                        _send_line(ser, "__set_all(0,0,0)", think=0.2)

                        self.devices.append({"ser": ser, "port": port})
                    except Exception as e:
                        print(f"[CONNECT FAIL {port}] {e}")
                        try:
                            ser.close()
                        except Exception:
                            pass
                        continue

                if not self.devices:
                    self.plasma_status.config(text="Failed", foreground="red")
                    self.status_var.set("No Plasma 2040 boards found.")
                else:
                    plist = ", ".join([d["port"] for d in self.devices])
                    self.plasma_status.config(
                        text=f"Connected ({len(self.devices)}): {plist}",
                        foreground="green",
                    )
                    self.status_var.set(
                        f"Connected to {len(self.devices)} Plasma board(s)"
                    )
            except Exception as e:
                self.plasma_status.config(text="Failed", foreground="red")
                self.status_var.set(f"Connect error: {e}")

        threading.Thread(target=worker, daemon=True).start()

    def _recover_all(self):
        for dev in list(self.devices):
            try:
                _repl_break_and_soft_reset(dev["ser"], do_soft=True)
                _send_line(
                    dev["ser"],
                    f"led_strip=plasma.WS2812({int(self.num_leds.get())},0,0,15)",
                    think=0.25,
                )
                _send_line(dev["ser"], "led_strip.start()", think=0.25)
            except Exception:
                pass
        self._connect_all_plasma()

    def _device_set_all_rgb_scaled(self, r, g, b):
        """
        Set RGB on all Plasma boards; uses _send_line with error reporting
        so we see if __set_all is missing or crashing.
        """
        r = int(max(0, min(255, r)))
        g = int(max(0, min(255, g)))
        b = int(max(0, min(255, b)))
        ok_any = False
        with self.ser_lock:
            for dev in list(self.devices):
                try:
                    ok, resp = _send_line(
                        dev["ser"], f"__set_all({r},{g},{b})", think=0.1
                    )
                    if not ok:
                        print(f"[LED ERROR {dev['port']}] {resp}")
                    ok_any = ok_any or ok
                except Exception as e:
                    print(f"[LED EXC {dev.get('port','?')}] {e}")
        return ok_any

    def _device_set_pattern_rgb_scaled(self, r1, g1, b1, r2, g2, b2):
        """
        Use the Micropython __set_pattern helper:
        - mode alternates between 0 and 1 so the "bright" pixels hop
          between even and odd indices.
        """
        r1 = int(max(0, min(255, r1)))
        g1 = int(max(0, min(255, g1)))
        b1 = int(max(0, min(255, b1)))
        r2 = int(max(0, min(255, r2)))
        g2 = int(max(0, min(255, g2)))
        b2 = int(max(0, min(255, b2)))

        ok_any = False
        with self.ser_lock:
            for dev in list(self.devices):
                try:
                    ok, resp = _send_line(
                        dev["ser"],
                        f"__set_pattern({r1},{g1},{b1},{r2},{g2},{b2},{self._pattern_mode})",
                        think=0.1,
                    )
                    if not ok:
                        print(f"[LED PATTERN ERROR {dev['port']}] {resp}")
                    ok_any = ok_any or ok
                except Exception as e:
                    print(f"[LED PATTERN EXC {dev.get('port','?')}] {e}")
        # toggle mode each call so bright pixels hop between even/odd indices
        self._pattern_mode = 1 - self._pattern_mode
        return ok_any

    def _plasma_set_brightness(self, brightness):
        """
        Global brightness + behaviour:
          - 0..1 brightness -> [floor..max] scalar
          - if reversed direction + red_on_reverse: use red instead of white
          - if pixel_shuffle: use odd/even pattern with two brightness levels
            (ensuring at least half of pixels are lit whenever brightness>0)
        """
        floor = float(self.led_floor.get())
        mx = float(self.led_max.get())
        if mx < floor:
            mx, floor = floor, mx

        b = max(0.0, min(1.0, float(brightness)))
        if b <= 1e-4:
            v = 0.0
        else:
            v = floor + b * (mx - floor)

        v = max(0.0, min(mx, v))
        lvl_high = int(255 * v)

        # A dimmer secondary level, but NEVER fully off if lvl_high > 0
        if lvl_high <= 0:
            lvl_low = 0
        else:
            # at least ~12% brightness, or 40% of high, whichever is larger
            lvl_low = max(int(lvl_high * 0.4), int(255 * 0.12))

        reversed_dir = not self.servo_dir_cw_var.get()
        use_red = self.red_on_reverse_var.get() and reversed_dir
        use_pattern = self.pixel_shuffle_var.get()

        if use_red:
            r_hi, g_hi, b_hi = lvl_high, 0, 0
            r_lo, g_lo, b_lo = lvl_low, 0, 0
        else:
            r_hi = g_hi = b_hi = lvl_high
            r_lo = g_lo = b_lo = lvl_low

        if not use_pattern or lvl_high == 0:
            # Simple mode: all pixels same color
            self._device_set_all_rgb_scaled(r_hi, g_hi, b_hi)
        else:
            # Pattern mode: even vs odd indices alternate between hi / low
            self._device_set_pattern_rgb_scaled(
                r_hi, g_hi, b_hi,
                r_lo, g_lo, b_lo
            )

    def _clear_leds(self):
        self._device_set_all_rgb_scaled(0, 0, 0)
        self.status_var.set("LEDs cleared")

    def _test_led_ramp(self):
        """Quick test: sweep LED brightness 0 → 65% → 0 over ~4s, bypassing mapping."""
        if not self.devices:
            messagebox.showwarning("No LEDs", "Connect the Plasma 2040 boards first.")
            return

        def worker():
            self.status_var.set("Running LED test ramp (max 0.65)...")
            max_brightness = 0.65  # hard cap for the test

            # up: 0 → 0.65
            for i in range(50):
                frac = i / 49.0          # 0..1
                b = max_brightness * frac
                lvl = int(255 * b)
                self._device_set_all_rgb_scaled(lvl, lvl, lvl)
                self.current_led_norm = b / max_brightness if max_brightness > 0 else 0.0
                time.sleep(0.04)

            # down: 0.65 → 0
            for i in range(50):
                frac = 1.0 - i / 49.0
                b = max_brightness * frac
                lvl = int(255 * b)
                self._device_set_all_rgb_scaled(lvl, lvl, lvl)
                self.current_led_norm = b / max_brightness if max_brightness > 0 else 0.0
                time.sleep(0.04)

            # back to off at the end
            self._device_set_all_rgb_scaled(0, 0, 0)
            self.current_led_norm = 0.0
            self.status_var.set("LED test ramp finished.")

        threading.Thread(target=worker, daemon=True).start()

    # -------------------------- Playback & loops -----------------------------

    def _apply_gain_and_limiter(self, chunk):
        chunk[:, 0] *= float(self.left_volume.get())
        chunk[:, 1] *= float(self.right_volume.get())
        db = float(self.output_gain_db.get())
        g = 10.0 ** (db / 20.0)
        if abs(db) > 0.05:
            chunk *= g
        if self.soft_limiter.get():
            chunk[:] = np.tanh(chunk)
        np.clip(chunk, -1.0, 1.0, out=chunk)
        return chunk

    def _audio_play_thread(self, device_index):
        data = self.playback_data if self.playback_data is not None else self.audio_data
        sr = self.sr
        blocksize = self.env_hop
        n_frames = data.shape[0]

        def callback(outdata, frames, time_info, status):
            if not self.is_playing:
                outdata[:] = 0
                return
            s = self.current_frame
            e = s + frames
            if self.loop_enabled.get():
                idx = (np.arange(s, e) % n_frames).astype(np.int64)
                chunk = data[idx].copy()
                self.current_frame = e % n_frames
            else:
                e = min(e, n_frames)
                chunk = data[s:e].copy()
                if len(chunk) < frames:
                    pad = np.zeros((frames - len(chunk), 2), dtype=data.dtype)
                    chunk = np.vstack([chunk, pad])
                    self.is_playing = False
                self.current_frame = e

            chunk = self._apply_gain_and_limiter(chunk)
            outdata[:] = chunk

            if self.env is not None and len(self.env) > 0:
                env_idx = min(len(self.env) - 1, s // self.env_hop)
                env_val = float(self.env[env_idx])
                self.current_env_value = env_val

                lo = self.led_env_low if self.led_env_low is not None else self.env_min
                hi = self.led_env_high if self.led_env_high is not None else self.env_max
                if hi <= lo:
                    hi = lo + 1e-3
                raw = (env_val - lo) / (hi - lo)
                raw = max(0.0, min(1.0, raw))
                self.current_norm = raw

        try:
            with sd.OutputStream(
                samplerate=sr,
                channels=2,
                device=device_index,
                callback=callback,
                blocksize=blocksize,
                dtype="float32",
            ):
                while self.is_playing:
                    time.sleep(0.1)
        except Exception as e:
            self.status_var.set(f"Playback error: {e}")
        finally:
            self.is_playing = False
            self.status_var.set("Playback finished.")
            self._stop_all(hardware_only=True)

    def _smooth_direction_flip(self):
        """
        Gently ramp duty to zero, flip direction pin, then ramp back up
        to a duty based on the current RMS norm.
        """
        if not self.servo_enabled_var.get() or not self.pwm_running:
            # Just toggle logical direction + pin
            new_dir = not self.servo_dir_cw_var.get()
            self.servo_dir_cw_var.set(new_dir)
            GPIO.output(self.dir_pin, GPIO.HIGH if new_dir else GPIO.LOW)
            return

        base_dt = 0.03
        steps = 10
        old_duty = float(self.current_duty)

        # Ramp down to 0
        for i in range(steps):
            d = old_duty * (1.0 - (i + 1) / steps)
            try:
                self.pwm.ChangeDutyCycle(max(0.0, d))
                self.current_duty = max(0.0, d)
            except Exception:
                pass
            time.sleep(base_dt)

        # Ensure off
        try:
            self.pwm.ChangeDutyCycle(0.0)
            self.current_duty = 0.0
        except Exception:
            pass

        # Flip direction
        new_dir = not self.servo_dir_cw_var.get()
        self.servo_dir_cw_var.set(new_dir)
        GPIO.output(self.dir_pin, GPIO.HIGH if new_dir else GPIO.LOW)

        # Compute new target duty from current RMS norm
        mn = float(self.min_duty.get())
        mx = float(self.max_duty.get())
        if mx < mn:
            mx, mn = mn, mx
        norm = max(0.0, min(1.0, float(self.current_norm)))
        target_duty = mn + norm * (mx - mn)

        # Ramp up to target
        for i in range(steps):
            d = target_duty * (i + 1) / steps
            try:
                self.pwm.ChangeDutyCycle(max(0.0, d))
                self.current_duty = max(0.0, d)
            except Exception:
                pass
            time.sleep(base_dt)

    def _servo_loop(self):
        base_dt = 0.02
        silence_timer = 0.0
        flip_cooldown = 0.0

        while self.hardware_running:
            # --- Silence-based flip logic ---
            if self.flip_on_silence_var.get() and self.is_playing and self.servo_enabled_var.get():
                norm = max(0.0, min(1.0, float(self.current_norm)))
                if norm < self.silence_threshold:
                    silence_timer += base_dt
                else:
                    silence_timer = 0.0

                if flip_cooldown > 0.0:
                    flip_cooldown -= base_dt

                if (
                    silence_timer >= self.silence_min_time
                    and flip_cooldown <= 0.0
                ):
                    # Smooth direction change
                    self._smooth_direction_flip()
                    silence_timer = 0.0
                    flip_cooldown = 1.0  # avoid multiple flips in one silence window

            # --- Normal duty mapping ---
            if self.servo_enabled_var.get() and self.pwm_running:
                mn = float(self.min_duty.get())
                mx = float(self.max_duty.get())
                if mx < mn:
                    mx, mn = mn, mx
                norm = float(self.current_norm)
                norm = max(0.0, min(1.0, norm))

                duty = mn + norm * (mx - mn)
                duty = max(0.0, min(100.0, duty))
                try:
                    self.pwm.ChangeDutyCycle(duty)
                    self.current_duty = duty
                    if duty > self.servo_peak_duty:
                        self.servo_peak_duty = duty
                except Exception:
                    pass
            else:
                try:
                    if self.pwm_running:
                        self.pwm.ChangeDutyCycle(0.0)
                        self.current_duty = 0.0
                except Exception:
                    pass

            time.sleep(base_dt)

    def _led_loop(self):
        """
        LEDs follow RMS (current_norm):
          - smoothing
          - gamma
          - floor/max mapping
          - optional pattern & red direction
        """
        dt = 0.05
        while self.hardware_running:
            if self.devices and self.env is not None:
                raw = float(self.current_norm)
                raw = max(0.0, min(1.0, raw))

                tau = float(self.led_smooth_time.get())
                if tau <= 0.0:
                    self._led_smooth_norm = raw
                else:
                    alpha = dt / (tau + dt)
                    self._led_smooth_norm += alpha * (raw - self._led_smooth_norm)

                gamma = float(self.led_gamma.get())
                if gamma <= 0:
                    gamma = 1.0
                norm_led = self._led_smooth_norm ** gamma

                self._plasma_set_brightness(norm_led)
                self.current_led_norm = norm_led

            time.sleep(dt)

    def _play_and_run(self):
        if not self.scan_done or self.env is None:
            messagebox.showwarning(
                "No scan", "Please run 'Scan RMS Envelope' first."
            )
            return
        if self.audio_data is None or self.sr is None:
            messagebox.showwarning(
                "No audio", "Scan valid audio files first."
            )
            return
        if not self.devices:
            messagebox.showwarning(
                "Plasma not connected",
                "Please connect the Plasma 2040 boards first.",
            )
            return
        dev_str = self.device_var.get()
        if not dev_str:
            messagebox.showwarning(
                "No audio device", "Please select an audio output device."
            )
            return
        if self.is_playing:
            self.status_var.set("Already playing.")
            return

        if self.servo_enabled_var.get():
            if not self.pwm_running:
                self.pwm.start(0.0)
                self.pwm_running = True
            GPIO.output(self.en_pin, GPIO.HIGH)
        else:
            GPIO.output(self.en_pin, GPIO.LOW)

        self.current_frame = 0
        self.current_env_value = 0.0
        self.current_norm = 0.0
        self.current_duty = 0.0
        self._led_smooth_norm = 0.0
        self.current_led_norm = 0.0

        self.is_playing = True
        self.hardware_running = True
        self.servo_thread = threading.Thread(
            target=self._servo_loop, daemon=True
        )
        self.led_thread = threading.Thread(
            target=self._led_loop, daemon=True
        )
        self.servo_thread.start()
        self.led_thread.start()
        device_index = int(dev_str.split(":")[0])
        threading.Thread(
            target=self._audio_play_thread, args=(device_index,), daemon=True
        ).start()
        self.status_var.set("Playback & hardware started.")

    def _stop_all(self, hardware_only=False):
        if not hardware_only:
            self.is_playing = False
        self.hardware_running = False
        self.current_norm = 0.0
        self.current_env_value = 0.0
        self._led_smooth_norm = 0.0
        self.current_led_norm = 0.0

        self.status_var.set("Stopping hardware...")
        decel = float(self.stop_decel_time.get())
        if decel < 0.05:
            decel = 0.05
        try:
            if self.pwm_running:
                start_duty = self.current_duty
                steps = max(1, int(decel / 0.02))
                for i in range(steps):
                    d = start_duty * (1.0 - i / steps)
                    self.pwm.ChangeDutyCycle(max(0.0, d))
                    time.sleep(0.02)
                self.pwm.ChangeDutyCycle(0.0)
                self.current_duty = 0.0
        except Exception:
            pass

        if not hardware_only:
            if self.servo_enabled_var.get():
                self.servo_enabled_var.set(False)
            self._toggle_servo()
            self.status_var.set("Stopped. Servo fully off.")
        else:
            self.status_var.set("Stopped (audio ended). Servo duty=0%.")

    # -------------------------- OSC server integration -----------------------

    def _toggle_osc(self):
        if self.osc_enabled_var.get():
            self._start_osc()
        else:
            self._stop_osc()
        self._schedule_save_settings()

    def _start_osc(self):
        if self.osc_server is not None:
            return
        bind = self.osc_bind_ip.get().strip() or "0.0.0.0"
        try:
            port = int(self.osc_port.get())
        except Exception:
            port = 9000
            self.osc_port.set(port)
        start_path = self.osc_start_path.get().strip() or "/show/start"
        stop_path  = self.osc_stop_path.get().strip()  or "/show/stop"

        disp = Dispatcher()

        def h_start(addr, *args):
            self.root.after(0, self._osc_start_cb)

        def h_stop(addr, *args):
            self.root.after(0, self._osc_stop_cb)

        disp.map(start_path, h_start)
        disp.map(stop_path, h_stop)

        try:
            server = ThreadingOSCUDPServer((bind, port), disp)
        except Exception as e:
            messagebox.showerror("OSC bind failed", f"{e}")
            self.osc_enabled_var.set(False)
            return

        self.osc_server = server

        def loop():
            try:
                server.serve_forever()
            except Exception:
                pass

        self.osc_thread = threading.Thread(target=loop, daemon=True)
        self.osc_thread.start()
        self.status_var.set(f"OSC listening on {bind}:{port} "
                            f"(start={start_path}, stop={stop_path})")

    def _stop_osc(self):
        if self.osc_server is not None:
            try:
                self.osc_server.shutdown()
            except Exception:
                pass
            try:
                self.osc_server.server_close()
            except Exception:
                pass
        self.osc_server = None
        self.osc_thread = None
        self.status_var.set("OSC listener stopped")

    def _osc_start_cb(self):
        if not self.is_playing:
            self._play_and_run()

    def _osc_stop_cb(self):
        self._stop_all()

    # ------------------------------ JSON Export ------------------------------

    def _export_json(self):
        if not self.scan_done or self.env is None:
            messagebox.showwarning("No scan", "Scan RMS before exporting.")
            return
        now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out = {
            "version": 8,
            "created_at": now,
            "audio": {
                "left_path": os.path.abspath(self.left_path)
                if self.left_path
                else None,
                "right_path": os.path.abspath(self.right_path)
                if self.right_path
                else None,
                "samplerate": self.sr,
                "env": {
                    "window": 4096,
                    "hop": self.env_hop,
                    "min": self.env_min,
                    "max": self.env_max,
                    "rms": [round(float(x), 5) for x in self.env.tolist()],
                    "percentile_low": self.led_env_low,
                    "percentile_high": self.led_env_high,
                },
            },
            "mapping": {
                "servo": {
                    "min_duty": float(self.min_duty.get()),
                    "max_duty": float(self.max_duty.get()),
                    "formula": "shared_norm = clamp((env-lo)/(hi-lo),0,1); "
                               "duty = min_duty + shared_norm*(max_duty-min_duty)",
                },
                "leds": {
                    "mode": "global_from_RMS_with_pattern",
                    "floor": float(self.led_floor.get()),
                    "max": float(self.led_max.get()),
                    "gamma": float(self.led_gamma.get()),
                    "smooth_time": float(self.led_smooth_time.get()),
                    "pixel_shuffle": bool(self.pixel_shuffle_var.get()),
                    "red_on_reverse": bool(self.red_on_reverse_var.get()),
                    "formula": (
                        "raw = clamp((env-lo)/(hi-lo),0,1); "
                        "smoothed = lowpass(raw, tau=smooth_time); "
                        "brightness_norm = smoothed**gamma; "
                        "v = floor + brightness_norm*(max-floor); "
                        "lvl_high = v*255; lvl_low = max(lvl_high*0.4, 0.12*255); "
                        "if shuffle: pattern even/odd hi/low; else all hi."
                    ),
                },
                "behaviour": {
                    "flip_on_silence": bool(self.flip_on_silence_var.get()),
                    "silence_threshold": self.silence_threshold,
                    "silence_min_time": self.silence_min_time,
                },
            },
            "hardware": {
                "led_count": int(self.num_leds.get()),
                "plasma_ports": [d.get("port", "?") for d in self.devices],
                "servo_gpio": {
                    "pwm": self.pwm_pin,
                    "dir": self.dir_pin,
                    "enable": self.en_pin,
                    "pwm_freq_hz": self.pwm_freq_hz,
                },
            },
        }
        out_name = f"show_config_{now}.json"
        try:
            Path(out_name).write_text(json.dumps(out, indent=2))
            self.export_info.config(text=f"Exported → {out_name}")
            self.status_var.set(f"JSON exported: {out_name}")
        except Exception as e:
            self.export_info.config(text=f"Export failed: {e}")
            self.status_var.set(f"Export error: {e}")

    # ----------------------------- UI tick & cleanup -------------------------

    def _ui_tick(self):
        duty = float(self.current_duty)
        volts = 3.3 * duty / 100.0
        self.servo_speed_label.config(
            text=f"Duty: {duty:.1f} %  (~{volts:.2f} V)"
        )
        self.servo_peak_label.config(
            text=f"Peak duty: {self.servo_peak_duty:.1f} %"
        )
        try:
            mx = float(self.max_duty.get())
            util = 0.0 if mx <= 0 else 100.0 * duty / mx
        except Exception:
            util = 0.0
        self.servo_util_label.config(
            text=f"Utilization: {util:.1f} % of MaxDuty"
        )

        led_pct = max(0.0, min(100.0, 100.0 * float(self.current_led_norm)))
        self.led_level_label.config(text=f"LED: {led_pct:.1f} % of max")

        self.left_vol_label.set(f"{self.left_volume.get():.2f}")
        self.right_vol_label.set(f"{self.right_volume.get():.2f}")
        self.led_floor_label.set(f"{self.led_floor.get():.2f}")
        self.led_max_label.set(f"{self.led_max.get():.2f}")
        self.led_gamma_label.set(f"{self.led_gamma.get():.2f}")
        self.led_smooth_label.set(f"{self.led_smooth_time.get():.2f} s")
        self.gain_db_label.set(f"{self.output_gain_db.get():.1f} dB")
        self.min_duty_label.set(f"{self.min_duty.get():.1f} %")
        self.max_duty_label.set(f"{self.max_duty.get():.1f} %")
        self.sustain_time_label.set(f"{self.sustain_time.get():.2f} s")

        if self.root and self.root.winfo_exists():
            self.root.after(100, self._ui_tick)

    def cleanup(self):
        self.is_playing = False
        self.hardware_running = False
        self._stop_osc()
        try:
            if self.pwm_running:
                self.pwm.ChangeDutyCycle(0.0)
                self.pwm.stop()
        except Exception:
            pass
        try:
            GPIO.output(self.en_pin, GPIO.LOW)
            GPIO.output(self.pwm_pin, GPIO.LOW)
            GPIO.cleanup()
        except Exception:
            pass
        try:
            self._device_set_all_rgb_scaled(0, 0, 0)
        except Exception:
            pass
        for dev in self.devices:
            try:
                dev["ser"].close()
            except Exception:
                pass
        self.devices = []

# --------------------------------- Main -------------------------------------

def main():
    root = tk.Tk()
    app = AudioPWMDualPlasmaApp(root)
    def on_close():
        app.cleanup()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
