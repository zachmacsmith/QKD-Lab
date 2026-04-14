"""
BB84 Detector Calibration
==========================
Full calibration sequence for the BB84 QKD setup:

  Step 1 — Dark counts          : block beam, measure background
  Step 2 — V_LOW sweep LCVR 1   : find half-wave voltage, save curve PNG
  Step 3 — V_LOW sweep LCVR 2   : find half-wave voltage, save curve PNG
  Step 4 — State verification   : confirm H, V, D, A with calibrated voltages
  Step 5 — Detector correction  : compute efficiency ratio from verified states

Saves calibration.json and two PNG curve plots to the same directory.
Run this before bb84_qkd.py. Re-run whenever alignment or temperature changes.

Dependencies
------------
    pip install serial pyserial KLCserial matplotlib numpy
    Swabian Instruments Time Tagger software (provides TimeTagger module)

LCVR sweep orientation note
----------------------------
    The V_LOW sweep measures the V-port count rate as a function of voltage.
    For this to give a clear peak:
      - LCVR 2 (slow axis 45 deg): V-port peaks cleanly at V_LOW
      - LCVR 1 (slow axis 22.5 deg): V-port shows a shallower peak because
        at V_LOW the output is D (diagonal), which splits 50/50 at the PBS.
        The code plots both channels so the peak is still identifiable.
    If modulation depth is low for LCVR 1, temporarily mount it at 45 deg
    for the sweep, find V_LOW, then remount at 22.5 deg for the experiment.

Channel mapping (matches kit wiring)
-------------------------------------
    Channel 1 = Detector T  (H-port of PBS)
    Channel 2 = Detector A  (V-port of PBS)
"""

import json
import time
import datetime
import pathlib
import math

# Thorlabs Kinesis via pythonnet — no KLCserial needed
# pip install pythonnet
KINESIS_PATH = r"C:\Program Files\Thorlabs\EDU-QOP1"
KINESIS_AVAILABLE = False
try:
    import clr
    import sys as _sys
    _sys.path.insert(0, KINESIS_PATH)
    clr.AddReference(KINESIS_PATH + r"\Thorlabs.MotionControl.DeviceManagerCLI.dll")
    clr.AddReference(KINESIS_PATH + r"\Thorlabs.MotionControl.KCube.LiquidCrystalCLI.dll")
    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
    from Thorlabs.MotionControl.KCube.LiquidCrystalCLI import KCubeLiquidCrystal
    from System import Decimal as _Decimal
    KINESIS_AVAILABLE = True
except Exception as _e:
    print(f"  Kinesis not available: {_e}")

import sys
# Swabian Instruments installs the TimeTagger module to a non-standard path
# on Windows. Add it explicitly so Python can find it regardless of whether
# the PYTHONPATH environment variable was set correctly by the installer.
_TIMETAGGER_PATH = r"C:\Program Files\Swabian Instruments\Time Tagger\driver\python"
if _TIMETAGGER_PATH not in sys.path:
    sys.path.insert(0, _TIMETAGGER_PATH)

try:
    import TimeTagger
    TAGGER_AVAILABLE = True
except ImportError:
    TAGGER_AVAILABLE = False

try:
    import matplotlib
    matplotlib.use("Agg")   # non-interactive — saves to file without needing display
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


# =============================================================================
# CONFIGURATION — edit these before running
# =============================================================================

# KLC101 identification. Use SN (preferred) or port. None = auto-detect.
SN_LCVR1   = '39443416'          # e.g. '39000001'  — LCVR 1, slow axis 22.5 deg
SN_LCVR2   = '39443413'          # e.g. '39000002'  — LCVR 2, slow axis 45 deg
PORT_LCVR1 = None          # e.g. '/dev/tty.usbserial-XXXX'
PORT_LCVR2 = None          # e.g. '/dev/tty.usbserial-YYYY'

# Initial V_LOW estimates — updated by sweep if a better peak is found
V_LOW_LCVR1  = 3.2         # V — initial estimate, updated by sweep
V_LOW_LCVR2  = 3.2         # V — initial estimate, updated by sweep
V_HIGH       = 20.0        # V — near-zero retardance, identity
CARRIER_FREQ = 2000        # Hz

# V_LOW voltage sweep range — extend if peak is not found within this range
SWEEP_V_START = 1.0        # V
SWEEP_V_END   = 8.0        # V
SWEEP_V_STEP  = 0.1        # V  (finer = more accurate but slower)
SWEEP_SETTLE  = 0.15       # s  wait after each voltage step before measuring

# Time Tagger channel numbers
CHANNEL_H = 1              # Detector T — H-port of PBS
CHANNEL_V = 2              # Detector A — V-port of PBS

# Measurement settings
INTEGRATION_TIME_S = 0.5   # s per sample window
N_SAMPLES          = 5     # windows averaged per step during sweep
N_SAMPLES_VERIFY   = 10    # windows averaged for state verification
SETTLE_TIME_S      = 0.1   # s after state change during verification

# Output paths — saved alongside this script
OUT_DIR          = pathlib.Path(__file__).parent
CALIBRATION_FILE = OUT_DIR / "calibration.json"
CURVE_FILE_LCVR1 = OUT_DIR / "sweep_lcvr1.png"
CURVE_FILE_LCVR2 = OUT_DIR / "sweep_lcvr2.png"

SIMULATE = False            # True = run without hardware for testing


# =============================================================================
# HARDWARE HELPERS
# =============================================================================

class KLC101Device:
    """
    Thin wrapper around the Thorlabs Kinesis KCubeLiquidCrystal .NET class.
    Provides set_voltage() and set_freq() matching the interface the rest
    of this script expects.

    If set_voltage() raises AttributeError, the preset params field name
    may differ in your Kinesis version. Run:
        params = device._device.GetPresetParams(1)
        print([m for m in dir(params) if not m.startswith('_')])
    to find the correct field name and update _VOLTAGE_FIELD below.
    """

    _VOLTAGE_FIELD = "Voltage1"       # field name on preset params object
    _FREQ_FIELD    = "OutputFrequency" # field name on preset params object

    def __init__(self, sn: str):
        self.sn      = sn
        self._device = None

    def connect(self) -> None:
        DeviceManagerCLI.BuildDeviceList()
        time.sleep(0.5)   # allow USB enumeration to complete

        # Verify device is visible before trying to create it
        device_list = DeviceManagerCLI.GetDeviceList()
        available   = list(device_list)
        print(f"  Available Kinesis devices: {available}")

        if self.sn not in available:
            raise RuntimeError(
                f"KLC101 serial {self.sn!r} not found in device list.\n"
                f"Available: {available}\n"
                "  → Check USB connection\n"
                "  → Close Kinesis / EDU-QOP1 software before running this script"
            )

        self._device = KCubeLiquidCrystal.CreateKCubeLiquidCrystal(self.sn)
        self._device.Connect(self.sn)
        self._device.WaitForSettingsInitialized(3000)
        self._device.StartPolling(250)
        time.sleep(0.5)
        self._device.EnableDevice()
        time.sleep(0.25)
        print(f"  KLC101 {self.sn}: connected via Kinesis")

    def set_voltage(self, voltage: float) -> None:
        """Set preset 1 voltage (V). Controls LC retardance."""
        params = self._device.GetPresetParams(1)
        try:
            setattr(params, self._VOLTAGE_FIELD, _Decimal(voltage))
        except AttributeError:
            available = [m for m in dir(params) if not m.startswith('_')]
            raise AttributeError(
                f"Field '{self._VOLTAGE_FIELD}' not found on preset params.\n"
                f"Available fields: {available}\n"
                f"Update KLC101Device._VOLTAGE_FIELD with the correct name."
            )
        self._device.SetPresetParams(1, params)

    def set_freq(self, freq: int) -> None:
        """Set preset 1 carrier frequency (Hz)."""
        params = self._device.GetPresetParams(1)
        try:
            setattr(params, self._FREQ_FIELD, _Decimal(freq))
        except AttributeError:
            available = [m for m in dir(params) if not m.startswith('_')]
            raise AttributeError(
                f"Field '{self._FREQ_FIELD}' not found on preset params.\n"
                f"Available fields: {available}\n"
                f"Update KLC101Device._FREQ_FIELD with the correct name."
            )
        self._device.SetPresetParams(1, params)

    def en_hwchan(self) -> None:
        self._device.EnableDevice()

    def dis_hwchan(self) -> None:
        self._device.DisableDevice()

    def disconnect(self) -> None:
        if self._device is not None:
            try:
                self._device.StopPolling()
                self._device.Disconnect()
            except Exception:
                pass
            self._device = None


def connect_klc(sn: str, _port=None) -> KLC101Device:
    """Connect to a KLC101 by serial number using Kinesis DLLs."""
    device = KLC101Device(sn)
    device.connect()
    return device


def set_voltages(klc1, klc2, v1: float, v2: float) -> None:
    klc1.set_voltage(v1)
    klc2.set_voltage(v2)
    time.sleep(SETTLE_TIME_S)


def measure_rates(countrate, n_samples: int, integration_time_s: float):
    """Return (mean_rate_h, mean_rate_v) in Hz averaged over n_samples."""
    rates_h, rates_v = [], []
    integration_ps = int(integration_time_s * 1e12)
    for _ in range(n_samples):
        countrate.startFor(integration_ps, clear=True)
        countrate.waitUntilFinished()
        data = countrate.getData()
        rates_h.append(float(data[0]))
        rates_v.append(float(data[1]))
    return sum(rates_h) / n_samples, sum(rates_v) / n_samples


# =============================================================================
# SIMULATION HELPERS
# =============================================================================

def _sim_retardance(voltage: float, v_halfwave: float = 3.5) -> float:
    """Approximate LC retardance (radians) as a function of voltage."""
    v_th = 0.8
    if voltage <= v_th:
        return 1.5 * math.pi
    ratio = ((v_halfwave - v_th) / (voltage - v_th)) ** 0.6
    return math.pi * ratio


def _sim_transmission(voltage: float, slow_axis_deg: float,
                      v_halfwave: float = 3.5) -> tuple:
    """Simulate (rate_h, rate_v) for a given voltage and slow axis angle."""
    import random
    retardance = _sim_retardance(voltage, v_halfwave)
    theta = math.radians(slow_axis_deg)
    cos2  = math.cos(2 * theta)
    sin2  = math.sin(2 * theta)
    d     = retardance / 2
    e_h   = math.cos(d) + 1j * math.sin(d) * cos2
    e_v   = 1j * math.sin(d) * sin2
    base      = 220000.0
    imbalance = 2.1
    noise     = lambda: random.gauss(0, 300)
    return (
        max(base * imbalance * abs(e_h)**2 + noise(), 0),
        max(base * abs(e_v)**2 + noise(), 0),
    )


def _sim_dark():
    import random
    return 150.0 + random.gauss(0, 10), 120.0 + random.gauss(0, 10)


# =============================================================================
# VOLTAGE SWEEP
# =============================================================================

def sweep_v_low(
    klc_sweep,
    klc_fixed,
    v_fixed: float,
    slow_axis_deg: float,
    countrate,
    dark_h: float,
    dark_v: float,
    sim_v_halfwave: float = 3.5,
) -> tuple:
    """
    Sweep voltage on klc_sweep while holding klc_fixed at v_fixed.
    Returns (voltages, net_rates_h, net_rates_v, v_low_detected).
    """
    n_steps = int(round((SWEEP_V_END - SWEEP_V_START) / SWEEP_V_STEP)) + 1
    voltages, rh_list, rv_list = [], [], []

    for i in range(n_steps):
        v = round(SWEEP_V_START + i * SWEEP_V_STEP, 4)

        if not SIMULATE:
            klc_sweep.set_voltage(v)
            klc_fixed.set_voltage(v_fixed)
            time.sleep(SWEEP_SETTLE)
            rh, rv = measure_rates(countrate, N_SAMPLES, INTEGRATION_TIME_S)
        else:
            rh, rv = _sim_transmission(v, slow_axis_deg, sim_v_halfwave)

        voltages.append(v)
        rh_list.append(max(rh - dark_h, 0.0))
        rv_list.append(max(rv - dark_v, 0.0))

        bar = int(50 * (i + 1) / n_steps)
        print(f"\r  [{'█'*bar}{'░'*(50-bar)}] {v:.2f}V  "
              f"H:{rh_list[-1]:>8.0f}  V:{rv_list[-1]:>8.0f}",
              end="", flush=True)

    print()

    # Detect V_LOW as peak of V-port rate
    peak_idx = rv_list.index(max(rv_list))
    v_low    = voltages[peak_idx]

    # Refine with parabolic interpolation
    if NUMPY_AVAILABLE and 1 <= peak_idx <= len(voltages) - 2:
        lo     = max(0, peak_idx - 2)
        hi     = min(len(voltages), peak_idx + 3)
        v_arr  = np.array(voltages[lo:hi])
        r_arr  = np.array(rv_list[lo:hi])
        coeffs = np.polyfit(v_arr, r_arr, 2)
        if coeffs[0] < 0:
            v_refined = -coeffs[1] / (2 * coeffs[0])
            if SWEEP_V_START <= v_refined <= SWEEP_V_END:
                v_low = round(float(v_refined), 3)

    modulation = (max(rv_list) - min(rv_list)) / max(max(rv_list), 1)
    print(f"  Peak V-port at: {v_low:.3f} V  |  modulation depth: {100*modulation:.1f}%")

    if modulation < 0.5:
        print("  WARNING: Modulation depth < 50%. Check slow axis angle and "
              "input polarization purity.")
        if abs(slow_axis_deg - 45.0) > 1:
            print("  Tip: For cleaner sweep on LCVR 1, temporarily mount at 45°, "
                  "find V_LOW, then remount at 22.5°.")

    return voltages, rh_list, rv_list, v_low


# =============================================================================
# CURVE PLOTTING
# =============================================================================

def plot_sweep(
    title: str,
    voltages: list,
    rates_h: list,
    rates_v: list,
    v_low: float,
    v_high: float,
    out_path: pathlib.Path,
) -> None:
    """Save a two-panel PNG of the voltage sweep curve."""
    if not MATPLOTLIB_AVAILABLE:
        print("  matplotlib not installed — skipping plot. "
              "Run: pip install matplotlib")
        return

    colors = {
        "H":    "#4fc3f7",
        "V":    "#ef9a9a",
        "mod":  "#ce93d8",
        "vlow": "#a5d6a7",
        "bg":   "#16213e",
        "panel":"#1e2a45",
        "grid": "#2a3a5a",
        "text": "#e0e0e0",
    }

    fig = plt.figure(figsize=(10, 7), facecolor=colors["bg"])
    gs  = gridspec.GridSpec(2, 1, figure=fig, hspace=0.48)

    total = [h + v for h, v in zip(rates_h, rates_v)]
    mod   = [v / max(t, 1) * 100 for v, t in zip(rates_v, total)]

    # ── Top: raw H and V rates ────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0])
    ax1.set_facecolor(colors["panel"])
    ax1.plot(voltages, [r / 1000 for r in rates_h],
             color=colors["H"], lw=1.8, label="H port (ch1 — Detector T)")
    ax1.plot(voltages, [r / 1000 for r in rates_v],
             color=colors["V"], lw=1.8, label="V port (ch2 — Detector A)")
    ax1.axvline(v_low,  color=colors["vlow"], ls="--", lw=1.5,
                label=f"V_LOW = {v_low:.3f} V  ←")
    ax1.axvline(v_high, color="#aaaaaa",      ls=":",  lw=1.0, alpha=0.5,
                label=f"V_HIGH = {v_high:.0f} V")

    # Annotate peak
    peak_v_idx = rates_v.index(max(rates_v))
    ax1.annotate(
        f"  λ/2 at {v_low:.3f} V",
        xy=(v_low, rates_v[peak_v_idx] / 1000),
        xytext=(min(v_low + 0.8, SWEEP_V_END - 0.5),
                rates_v[peak_v_idx] / 1000 * 0.8),
        color=colors["vlow"], fontsize=8.5,
        arrowprops=dict(arrowstyle="->", color=colors["vlow"], lw=1.1),
    )

    ax1.set_ylabel("Count rate (kHz)", color=colors["text"])
    ax1.set_title(title, color=colors["text"], fontsize=12, pad=10)
    _style_ax(ax1, colors)

    # ── Bottom: V fraction (modulation) ──────────────────────────────────────
    ax2 = fig.add_subplot(gs[1])
    ax2.set_facecolor(colors["panel"])
    ax2.plot(voltages, mod, color=colors["mod"], lw=1.8,
             label="V / (H+V) × 100%")
    ax2.axvline(v_low, color=colors["vlow"], ls="--", lw=1.5,
                label=f"V_LOW = {v_low:.3f} V")
    ax2.axhline(50, color="#888888", ls=":", lw=0.8, alpha=0.5,
                label="50% line")
    ax2.set_xlabel("Voltage (V)", color=colors["text"])
    ax2.set_ylabel("V fraction (%)", color=colors["text"])
    ax2.set_title("Polarisation fraction to V port", color=colors["text"],
                  fontsize=11, pad=8)
    _style_ax(ax2, colors)

    plt.savefig(out_path, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"  Curve saved: {out_path.name}")


def _style_ax(ax, colors: dict) -> None:
    ax.tick_params(colors=colors["text"])
    ax.spines[:].set_color(colors["grid"])
    ax.grid(True, color=colors["grid"], lw=0.5, alpha=0.7)
    leg = ax.legend(facecolor=colors["bg"], edgecolor=colors["grid"],
                    labelcolor=colors["text"], fontsize=9)
    for t in ax.get_xticklabels() + ax.get_yticklabels():
        t.set_color(colors["text"])


# =============================================================================
# MAIN CALIBRATION SEQUENCE
# =============================================================================

def run_calibration() -> dict:
    print("\n" + "=" * 62)
    print("  BB84 Full Calibration")
    print("=" * 62)
    print(f"\n  Output dir  : {OUT_DIR}")
    print(f"  Sweep range : {SWEEP_V_START}–{SWEEP_V_END} V  "
          f"step {SWEEP_V_STEP} V")
    print(f"  Integration : {INTEGRATION_TIME_S} s × {N_SAMPLES} samples/step\n")

    # ── Connect ───────────────────────────────────────────────────────────────
    if not SIMULATE:
        if not TAGGER_AVAILABLE:
            raise RuntimeError(
                "TimeTagger not found. Install Swabian Instruments software."
            )
        if not KINESIS_AVAILABLE:
            raise RuntimeError(
                f"Kinesis DLLs not loaded. Check KINESIS_PATH = {KINESIS_PATH!r}"
            )
        print("  Connecting Time Tagger...")
        tagger    = TimeTagger.createTimeTagger()
        countrate = TimeTagger.Countrate(
            tagger=tagger, channels=[CHANNEL_H, CHANNEL_V]
        )
        print("  Connecting KLC101 controllers via Kinesis...")
        klc1 = connect_klc(SN_LCVR1)
        klc2 = connect_klc(SN_LCVR2)
        klc1.set_freq(CARRIER_FREQ)
        klc2.set_freq(CARRIER_FREQ)
    else:
        print("  [SIMULATE MODE — no hardware]\n")
        tagger = countrate = klc1 = klc2 = None

    results = {}

    # ── Step 1: Dark counts ───────────────────────────────────────────────────
    print("\n── Step 1: Dark counts ──────────────────────────────────────────")
    print("  Block the laser beam completely, then press Enter.")
    if not SIMULATE:
        input("  > ")
        set_voltages(klc1, klc2, V_HIGH, V_HIGH)
        dark_h, dark_v = measure_rates(
            countrate, N_SAMPLES_VERIFY, INTEGRATION_TIME_S
        )
    else:
        dark_h, dark_v = _sim_dark()

    print(f"  Dark  H: {dark_h:.1f} Hz    Dark  V: {dark_v:.1f} Hz")
    print("  Unblock the laser, then press Enter.")
    if not SIMULATE:
        input("  > ")

    results["dark_rate_h"] = dark_h
    results["dark_rate_v"] = dark_v

    # ── Step 2: V_LOW sweep — LCVR 1 ─────────────────────────────────────────
    print("\n── Step 2: V_LOW sweep — LCVR 1 (slow axis 22.5°) ──────────────")
    print("  LCVR 2 held at V_HIGH.")

    voltages1, rh1, rv1, v_low1 = sweep_v_low(
        klc_sweep      = klc1,
        klc_fixed      = klc2,
        v_fixed        = V_HIGH,
        slow_axis_deg  = 22.5,
        countrate      = countrate,
        dark_h         = dark_h,
        dark_v         = dark_v,
        sim_v_halfwave = 3.5,
    )

    plot_sweep(
        title    = "LCVR 1  (slow axis 22.5°) — Voltage sweep",
        voltages = voltages1,
        rates_h  = rh1,
        rates_v  = rv1,
        v_low    = v_low1,
        v_high   = V_HIGH,
        out_path = CURVE_FILE_LCVR1,
    )

    print(f"\n  Auto-detected V_LOW LCVR 1: {v_low1:.3f} V")
    print("  Open sweep_lcvr1.png to verify. Press Enter to accept or type a value.")
    if not SIMULATE:
        user = input(f"  V_LOW LCVR 1 [{v_low1:.3f}]: ").strip()
        if user:
            v_low1 = float(user)
    print(f"  ✓ V_LOW LCVR 1 = {v_low1:.3f} V")
    results["v_low_lcvr1"] = v_low1

    # ── Step 3: V_LOW sweep — LCVR 2 ─────────────────────────────────────────
    print("\n── Step 3: V_LOW sweep — LCVR 2 (slow axis 45°) ────────────────")
    print("  LCVR 1 held at V_HIGH.")

    voltages2, rh2, rv2, v_low2 = sweep_v_low(
        klc_sweep      = klc2,
        klc_fixed      = klc1,
        v_fixed        = V_HIGH,
        slow_axis_deg  = 45.0,
        countrate      = countrate,
        dark_h         = dark_h,
        dark_v         = dark_v,
        sim_v_halfwave = 3.8,
    )

    plot_sweep(
        title    = "LCVR 2  (slow axis 45°) — Voltage sweep",
        voltages = voltages2,
        rates_h  = rh2,
        rates_v  = rv2,
        v_low    = v_low2,
        v_high   = V_HIGH,
        out_path = CURVE_FILE_LCVR2,
    )

    print(f"\n  Auto-detected V_LOW LCVR 2: {v_low2:.3f} V")
    print("  Open sweep_lcvr2.png to verify. Press Enter to accept or type a value.")
    if not SIMULATE:
        user = input(f"  V_LOW LCVR 2 [{v_low2:.3f}]: ").strip()
        if user:
            v_low2 = float(user)
    print(f"  ✓ V_LOW LCVR 2 = {v_low2:.3f} V")
    results["v_low_lcvr2"] = v_low2

    # ── Step 4: State verification ────────────────────────────────────────────
    print("\n── Step 4: State verification ───────────────────────────────────")
    print(f"  V_LOW_LCVR1={v_low1:.3f} V   V_LOW_LCVR2={v_low2:.3f} V\n")

    states_def = {
        "H": (V_HIGH, V_HIGH),
        "D": (v_low1, V_HIGH),
        "V": (V_HIGH, v_low2),
        "A": (v_low1, v_low2),
    }
    state_rates = {}

    for lbl, (v1, v2) in states_def.items():
        if not SIMULATE:
            set_voltages(klc1, klc2, v1, v2)
            rh, rv = measure_rates(countrate, N_SAMPLES_VERIFY, INTEGRATION_TIME_S)
        else:
            # Simulate combined two-LCVR effect
            import random
            if lbl == "H":
                rh, rv = _sim_transmission(V_HIGH, 0.0, 3.5)
                rh = 220000.0 * 2.1 + random.gauss(0, 300)
                rv = 220000.0 * 0.007 + random.gauss(0, 50)
            elif lbl == "V":
                rh = 220000.0 * 0.007 + random.gauss(0, 50)
                rv = 220000.0 + random.gauss(0, 300)
            elif lbl == "D":
                rh = 220000.0 * 2.1 / 2 + random.gauss(0, 300)
                rv = 220000.0 / 2 + random.gauss(0, 300)
            else:  # A
                rh = 220000.0 * 2.1 / 2 + random.gauss(0, 300)
                rv = 220000.0 / 2 + random.gauss(0, 300)

        net_h = max(rh - dark_h, 1.0)
        net_v = max(rv - dark_v, 1.0)

        if lbl == "H":
            er, er_lbl = net_h / net_v, "H/V"
            ok = er > 20
        elif lbl == "V":
            er, er_lbl = net_v / net_h, "V/H"
            ok = er > 20
        else:
            er, er_lbl = net_h / net_v, "H/V"
            ok = True

        flag = "" if ok else "  ← LOW — V_LOW may need adjustment"
        print(f"  {lbl}: H={net_h:>9.0f} Hz   V={net_v:>9.0f} Hz   "
              f"{er_lbl}={er:.1f}{flag}")

        state_rates[lbl] = {
            "rate_h": rh, "rate_v": rv,
            "net_h":  net_h, "net_v": net_v,
        }

    results["state_rates"] = state_rates

    # ── Step 5: Detector efficiency correction ────────────────────────────────
    print("\n── Step 5: Detector efficiency correction ───────────────────────")

    ratio_from_h = state_rates["H"]["net_h"] / state_rates["H"]["net_v"]
    ratio_from_v = state_rates["V"]["net_h"] / state_rates["V"]["net_v"]
    efficiency_ratio = (ratio_from_h + ratio_from_v) / 2.0

    extinction_ratios = {
        "H": state_rates["H"]["net_h"] / state_rates["H"]["net_v"],
        "V": state_rates["V"]["net_v"] / state_rates["V"]["net_h"],
        "D": state_rates["D"]["net_h"] / state_rates["D"]["net_v"],
        "A": state_rates["A"]["net_h"] / state_rates["A"]["net_v"],
    }

    print(f"  Efficiency ratio from H: {ratio_from_h:.3f}")
    print(f"  Efficiency ratio from V: {ratio_from_v:.3f}")
    print(f"  Combined               : {efficiency_ratio:.3f}")

    if efficiency_ratio > 10 or efficiency_ratio < 0.1:
        print("  WARNING: Very large ratio — correction will be imprecise.")

    print("\n  Extinction ratios:")
    for s, er in extinction_ratios.items():
        flag = "  ← LOW" if s in ("H", "V") and er < 20 else ""
        print(f"    {s}: {er:.1f}{flag}")

    results["efficiency_ratio"]  = efficiency_ratio
    results["extinction_ratios"] = extinction_ratios

    # ── Save JSON ─────────────────────────────────────────────────────────────
    output = {
        "timestamp":          datetime.datetime.now().isoformat(),
        "simulate":           SIMULATE,
        "channel_h":          CHANNEL_H,
        "channel_v":          CHANNEL_V,
        "v_low_lcvr1":        v_low1,
        "v_low_lcvr2":        v_low2,
        "v_high":             V_HIGH,
        "sweep_v_start":      SWEEP_V_START,
        "sweep_v_end":        SWEEP_V_END,
        "sweep_v_step":       SWEEP_V_STEP,
        "integration_time_s": INTEGRATION_TIME_S,
        "n_samples":          N_SAMPLES,
        **results,
    }

    with open(CALIBRATION_FILE, "w") as f:
        json.dump(output, f, indent=2)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if not SIMULATE:
        set_voltages(klc1, klc2, V_HIGH, V_HIGH)
        klc1.dis_hwchan()
        klc2.dis_hwchan()
        klc1.disconnect()
        klc2.disconnect()
        TimeTagger.freeTimeTagger(tagger)

    print(f"\n  Saved: {CALIBRATION_FILE.name}")
    print(f"  Saved: {CURVE_FILE_LCVR1.name}")
    print(f"  Saved: {CURVE_FILE_LCVR2.name}")
    print("\n" + "=" * 62)
    print("  Calibration complete.")
    print("=" * 62 + "\n")
    return output


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    run_calibration()