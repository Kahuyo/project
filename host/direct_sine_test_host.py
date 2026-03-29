#!/usr/bin/env python3
"""
Host-side script adapted to work with ESP32 middle-layer.
- sim_run: same as original (local sim)
- hw_run: sends START/STOP commands to ESP32 over serial (USB-Serial)
Outputs JSON (metadata) to experiments/ indicating which commands were sent and any serial lines received.
"""

import os
import sys
import time
import math
import json
import argparse
import random
import serial

ROOT = os.path.dirname(os.path.abspath(__file__)) + "/.."
EXPERIMENTS_DIR = os.path.join(ROOT, "experiments")
os.makedirs(EXPERIMENTS_DIR, exist_ok=True)

DEFAULT_KP = 0.005
DEFAULT_KD = 0
DEFAULT_FF = 0.1
DEFAULT_RUN_DURATION_S = 16.0

DEFAULT_MODE = 'hw'
DEFAULT_WAVE = 'sine'

PRESETS = {
    '1hz_5_tri': {'mode': 'hw', 'wave': 'tri', 'freq': 1.0, 'amp': 5.0, 'duration': DEFAULT_RUN_DURATION_S, 'kp': DEFAULT_KP, 'ff': DEFAULT_FF},
    '2hz_8_tri': {'mode': 'hw', 'wave': 'tri', 'freq': 2.0, 'amp': 8.0, 'duration': DEFAULT_RUN_DURATION_S, 'kp': DEFAULT_KP, 'ff': DEFAULT_FF},
    'sim_quick': {'mode': 'sim', 'wave': 'sine', 'freq': 1.0, 'amp': 5.0, 'duration': 2.0, 'kp': DEFAULT_KP, 'ff': DEFAULT_FF},
}
PRESET_NAMES = list(PRESETS.keys())

def sim_run(freq, amp, duration, dt=0.01, kp=0.25, kd=0.005, ff_scale=0.4, wave='sine'):
    # simplified sim as in original repo (very small plant)
    K_track = 60.0
    damping = 6.0
    x = 0.0
    v = 0.0
    hist = []
    steps = max(3, int(duration / dt))
    for i in range(steps):
        t = i * dt
        phase = 2.0 * math.pi * freq * t
        if wave == 'tri':
            wave_val = (2.0 / math.pi) * math.asin(math.sin(phase))
            target = amp * wave_val
            desired_vel = amp * 2.0 * math.pi * freq * math.cos(phase)
        else:
            wave_val = math.sin(phase)
            target = amp * wave_val
            desired_vel = amp * 2.0 * math.pi * freq * math.cos(phase)
        pred_target = target + desired_vel * ff_scale * 0.03
        err = pred_target - x
        corr = kp * err
        max_corr = amp * 0.6
        corr = max(-max_corr, min(max_corr, corr))
        cmd = target + corr
        a = K_track * (cmd - x) - damping * v
        v += a * dt
        x += v * dt
        x += random.gauss(0.0, 0.01)
        hist.append({'t': t, 'target': target, 'cmd': cmd, 'fb': x})
    stamp = time.strftime('%Y%m%d_%H%M%S')
    out = {
        'mode': 'sim',
        'freq': freq,
        'amp': amp,
        'duration': duration,
        'history': hist,
    }
    out_file = os.path.join(EXPERIMENTS_DIR, f'direct_sine_sim_{int(freq)}_{int(amp)}_{stamp}.json')
    with open(out_file, 'w') as f:
        json.dump(out, f, indent=2)
    print('Sim saved:', out_file)
    return out_file

def hw_run(freq, amp, duration, serial_port='/dev/ttyUSB0', baud=115200, wave='sine'):
    # Connect to ESP32 over serial and send START/STOP commands
    ser = serial.Serial(serial_port, baud, timeout=1.0)
    cmd = f"START NEEDLE {freq:.3f} {amp:.3f}\\n"
    print('Sending to ESP32:', cmd.strip())
    ser.write(cmd.encode('utf-8'))
    received = []
    t0 = time.time()
    try:
        # read lines during run (optional)
        while time.time() - t0 < duration:
            time.sleep(0.05)
            # read any available lines
            while ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print('ESP32:', line)
                    received.append({'t': time.time()-t0, 'line': line})
    finally:
        stop_cmd = "STOP NEEDLE\\n"
        ser.write(stop_cmd.encode('utf-8'))
        ser.close()

    stamp = time.strftime('%Y%m%d_%H%M%S')
    out = {
        'mode': 'hw',
        'freq': freq,
        'amp': amp,
        'duration': duration,
        'serial_port': serial_port,
        'received': received,
        'timestamp': time.time(),
    }
    out_file = os.path.join(EXPERIMENTS_DIR, f'direct_sine_hw_{int(freq)}_{int(amp)}_{stamp}.json')
    with open(out_file, 'w') as f:
        json.dump(out, f, indent=2)
    print('HW metadata saved:', out_file)
    return out_file

def main():
    parser = argparse.ArgumentParser(description='Direct sine test (sim or hw via ESP32)')
    parser.add_argument('--mode', choices=['sim','hw'], default=DEFAULT_MODE)
    parser.add_argument('--freq', type=float, default=1.0)
    parser.add_argument('--amp', type=float, default=5.0)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--serial', type=str, default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--preset', choices=PRESET_NAMES, default=None)
    args = parser.parse_args()

    if args.preset:
        p = PRESETS[args.preset]
        mode = p['mode']
        freq = p['freq']
        amp = p['amp']
        duration = p['duration']
    else:
        mode = args.mode
        freq = args.freq
        amp = args.amp
        duration = args.duration

    if mode == 'sim':
        sim_run(freq, amp, duration)
    else:
        hw_run(freq, amp, duration, serial_port=args.serial, baud=args.baud)

if __name__ == '__main__':
    main()
