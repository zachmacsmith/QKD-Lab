"""
BB84 Detector Calibration
==========================
Runs a structured four-state calibration sequence using Alice's LCVRs and
the Swabian Instruments Time Tagger. Saves results to calibration.json in
the same directory.

Run this before bb84_qkd.py. Re-run whenever:
  - LCVR voltages are changed
  - The optical alignment changes
  - A significant temperature change has occurred (>~5 deg C)

Usage
-----
    python calibrate.py

Dependencies
------------
    pip install serial pyserial KLCserial
    Swabian Instruments Time Tagger software (provides TimeTagger module)

Channel mapping (matches kit wiring)
-------------------------------------
    Channel 1 = Detector T  (trigger / H-port of PBS)
    Channel 2 = Detector A  (V-port of PBS)
"""

import json
import time
import datetime
import pathlib
from typing import Optional

try:
    from KLCserial import KLC
    KLC_AVAILABLE = True
except ImportError:
    KLC_AVAILABLE = False

try:
    import TimeTagger
    TAGGER_AVAILABLE = True
except ImportError:
    TAGGER_AVAILABLE = False

# =============================================================================
# CONFIGURATION — edit these before running
# =============================================================================

# KLC101 identification. Use SN (preferred) or port. None = auto-detect.
SN_LCVR1   = None          # e.g. '39000001'  — LCVR 1, slow axis 22.5 deg
SN_LCVR2   = None          # e.g. '39000002'  — LCVR 2, slow axis 45 deg
PORT_LCVR1 = None          # e.g. '/dev/ttyUSB0'
PORT_LCVR2 = None          # e.g. '/dev/ttyUSB1'

# Voltages — V_LOW must be your optically calibrated half-wave voltage.
# Run the voltage sweep in calibrate_v_low() (see bb84_qkd.py) first.
V_LOW_LCVR1  = 3.2         # V — placeholder, calibrate this
V_LOW_LCVR2  = 3.2         # V — placeholder, calibrate this
V_HIGH       = 20.0        # V — near-zero retardance, identity
CARRIER_FREQ = 2000        # Hz

# Time Tagger channel numbers (match your physical wiring to the kit)
CHANNEL_H = 1              # Detector T — H-port of PBS
CHANNEL_V = 2              # Detector A — V-port of PBS

# Measurement settings
INTEGRATION_TIME_S = 0.5   # seconds per sample window
N_SAMPLES          = 10    # windows averaged per state
SETTLE_TIME_S      = 0.1   # seconds to wait after LCVR voltage change

# Output file — saved alongside this script
CALIBRATION_FILE = pathlib.Path(__file__).parent / "calibration.json"

SIMULATE = False            # True = run without hardware for testing

# =============================================================================
# STATE VOLTAGE MAP
# (must match HardwareLayer._STATE_MAP in bb84_qkd.py)
# =============================================================================

STATES = {
    #  label : (lcvr1_low, lcvr2_low)
    "H": (False, False),
    "D": (True,  False),
    "V": (False, True ),
    "A": (True,  True ),
}


# =============================================================================
# HARDWARE HELPERS
# =============================================================================

def connect_klc(sn, port):
    if sn:
        return KLC(SN=sn)
    if port:
        return KLC(port=port)
    return KLC()


def set_state(klc1, klc2, lcvr1_low: bool, lcvr2_low: bool) -> None:
    v1 = V_LOW_LCVR1 if lcvr1_low else V_HIGH
    v2 = V_LOW_LCVR2 if lcvr2_low else V_HIGH
    klc1.set_voltage(v1)
    klc2.set_voltage(v2)
    time.sleep(SETTLE_TIME_S)


def measure_rates(countrate, n_samples: int, integration_time_s: float):
    """
    Average count rates over n_samples windows.
    Returns (mean_rate_h, mean_rate_v) in Hz.
    """
    rates_h, rates_v = [], []
    integration_ps = int(integration_time_s * 1e12)

    for _ in range(n_samples):
        countrate.startFor(integration_ps, clear=True)
        countrate.waitUntilFinished()
        data = countrate.getData()
        rates_h.append(float(data[0]))
        rates_v.append(float(data[1]))

    return (
        sum(rates_h) / n_samples,
        sum(rates_v) / n_samples,
    )


def simulate_rates(state_label: str, dark: bool = False):
    """Simulated rates for testing without hardware."""
    import random
    if dark:
        return 150.0 + random.gauss(0, 10), 120.0 + random.gauss(0, 10)
    base = 220000.0
    imbalance = 2.1   # simulated detector T being ~2x detector A
    extinction = 150.0
    noise = lambda: random.gauss(0, 500)

    if state_label == "H":
        return base * imbalance + noise(), base / extinction + noise()
    elif state_label == "V":
        return base / extinction + noise(), base + noise()
    elif state_label in ("D", "A"):
        return (base * imbalance / 2) + noise(), (base / 2) + noise()


# =============================================================================
# CALIBRATION SEQUENCE
# =============================================================================

def run_calibration():
    print("\n" + "=" * 60)
    print("  BB84 Detector Calibration")
    print("=" * 60)
    print(f"\n  Output: {CALIBRATION_FILE}")
    print(f"  Integration: {INTEGRATION_TIME_S} s x {N_SAMPLES} samples per state")
    print(f"  Channels: H={CHANNEL_H}, V={CHANNEL_V}\n")

    if not SIMULATE:
        if not TAGGER_AVAILABLE:
            raise RuntimeError(
                "TimeTagger module not found. Install Swabian Instruments software."
            )
        if not KLC_AVAILABLE:
            raise RuntimeError(
                "KLCserial not found. Run: pip install serial pyserial KLCserial"
            )
        print("  Connecting to Time Tagger...")
        tagger   = TimeTagger.createTimeTagger()
        countrate = TimeTagger.Countrate(
            tagger=tagger, channels=[CHANNEL_H, CHANNEL_V]
        )
        print("  Connecting to KLC101 controllers...")
        klc1 = connect_klc(SN_LCVR1, PORT_LCVR1)
        klc2 = connect_klc(SN_LCVR2, PORT_LCVR2)
        klc1.en_hwchan()
        klc2.en_hwchan()
    else:
        print("  [SIMULATE MODE — no hardware]\n")
        tagger = countrate = klc1 = klc2 = None

    results = {}

    # ------------------------------------------------------------------
    # Step 1: Dark count measurement
    # ------------------------------------------------------------------
    print("\n── Step 1: Dark counts ──────────────────────────────────────")
    print("  Block the laser beam completely, then press Enter.")
    if not SIMULATE:
        input("  > ")

    if SIMULATE:
        dark_h, dark_v = simulate_rates("H", dark=True)
    else:
        set_state(klc1, klc2, False, False)   # LCVRs to identity, irrelevant here
        dark_h, dark_v = measure_rates(countrate, N_SAMPLES, INTEGRATION_TIME_S)

    print(f"  Dark rate H: {dark_h:.1f} Hz   Dark rate V: {dark_v:.1f} Hz")
    print("\n  Unblock the laser beam, then press Enter.")
    if not SIMULATE:
        input("  > ")

    results["dark_rate_h"] = dark_h
    results["dark_rate_v"] = dark_v

    # ------------------------------------------------------------------
    # Step 2: Per-state count rates
    # ------------------------------------------------------------------
    print("\n── Step 2: State calibration ────────────────────────────────")
    state_rates = {}

    for label, (lcvr1_low, lcvr2_low) in STATES.items():
        print(f"\n  Preparing state: {label}  "
              f"(LCVR1={'V_LOW' if lcvr1_low else 'V_HIGH'}, "
              f"LCVR2={'V_LOW' if lcvr2_low else 'V_HIGH'})")

        if SIMULATE:
            rate_h, rate_v = simulate_rates(label)
        else:
            set_state(klc1, klc2, lcvr1_low, lcvr2_low)
            rate_h, rate_v = measure_rates(countrate, N_SAMPLES, INTEGRATION_TIME_S)

        net_h = max(rate_h - dark_h, 1.0)   # subtract dark, floor at 1 to avoid /0
        net_v = max(rate_v - dark_v, 1.0)
        extinction = net_h / net_v if label in ("H",) else net_v / net_h

        print(f"  Rate H: {rate_h:>10.1f} Hz   Rate V: {rate_v:>10.1f} Hz")
        print(f"  Net  H: {net_h:>10.1f} Hz   Net  V: {net_v:>10.1f} Hz")
        print(f"  Extinction ratio ({'H/V' if label == 'H' else 'V/H' if label == 'V' else 'dominant/other'}): "
              f"{extinction:.1f}")

        state_rates[label] = {
            "rate_h": rate_h,
            "rate_v": rate_v,
            "net_h":  net_h,
            "net_v":  net_v,
        }

    results["state_rates"] = state_rates

    # ------------------------------------------------------------------
    # Step 3: Compute efficiency ratio and per-state correction factors
    # ------------------------------------------------------------------
    print("\n── Step 3: Computing corrections ────────────────────────────")

    # Global efficiency ratio: average imbalance seen across H and V states.
    # At H: all signal should be on H_PORT, so imbalance = net_h / net_v
    # At V: all signal should be on V_PORT, so imbalance = net_h / net_v still
    #        (we want to know how much H over-counts relative to V globally)
    ratio_from_h = state_rates["H"]["net_h"] / state_rates["H"]["net_v"]
    ratio_from_v = state_rates["V"]["net_h"] / state_rates["V"]["net_v"]
    efficiency_ratio = (ratio_from_h + ratio_from_v) / 2.0

    print(f"  Efficiency ratio from H state: {ratio_from_h:.3f}")
    print(f"  Efficiency ratio from V state: {ratio_from_v:.3f}")
    print(f"  Combined efficiency ratio    : {efficiency_ratio:.3f}")
    print(f"  (V detector counts will be multiplied by this factor)")

    if efficiency_ratio > 10 or efficiency_ratio < 0.1:
        print("\n  WARNING: Efficiency ratio is very large. "
              "Detection correction will be imprecise.")
        print("  Consider improving optical alignment before relying on these results.")

    # Per-state extinction ratios — useful for diagnosing LCVR calibration
    extinction_ratios = {
        "H": state_rates["H"]["net_h"] / state_rates["H"]["net_v"],
        "V": state_rates["V"]["net_v"] / state_rates["V"]["net_h"],
        "D": state_rates["D"]["net_h"] / state_rates["D"]["net_v"],
        "A": state_rates["A"]["net_h"] / state_rates["A"]["net_v"],
    }

    print("\n  Extinction ratios (higher = cleaner state preparation):")
    for state, er in extinction_ratios.items():
        flag = ""
        if state in ("H", "V") and er < 20:
            flag = "  ← LOW — check LCVR V_LOW calibration"
        elif state in ("D", "A") and abs(er - 1.0) > 0.2:
            flag = "  ← unexpected imbalance in diagonal state"
        print(f"    {state}: {er:.2f}{flag}")

    results["efficiency_ratio"]  = efficiency_ratio
    results["extinction_ratios"] = extinction_ratios

    # ------------------------------------------------------------------
    # Step 4: Save to JSON
    # ------------------------------------------------------------------
    output = {
        "timestamp":          datetime.datetime.now().isoformat(),
        "simulate":           SIMULATE,
        "channel_h":          CHANNEL_H,
        "channel_v":          CHANNEL_V,
        "v_low_lcvr1":        V_LOW_LCVR1,
        "v_low_lcvr2":        V_LOW_LCVR2,
        "v_high":             V_HIGH,
        "integration_time_s": INTEGRATION_TIME_S,
        "n_samples":          N_SAMPLES,
        **results,
    }

    with open(CALIBRATION_FILE, "w") as f:
        json.dump(output, f, indent=2)

    print(f"\n  Calibration saved to: {CALIBRATION_FILE}")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    if not SIMULATE:
        set_state(klc1, klc2, False, False)   # return to identity
        klc1.dis_hwchan()
        klc2.dis_hwchan()
        klc1.closeKLC()
        klc2.closeKLC()
        TimeTagger.freeTimeTagger(tagger)

    print("\n" + "=" * 60)
    print("  Calibration complete.")
    print("=" * 60 + "\n")

    return output


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    run_calibration()