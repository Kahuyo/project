#!/usr/bin/env python3
# Interactive host tool to command ESP32 over serial (similar to original run_machine.py)
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyUSB0'
BAUD = 115200

def send(ser, s):
    ser.write((s.rstrip() + '\\n').encode('utf-8'))
    time.sleep(0.05)
    # read available
    lines = []
    while ser.in_waiting:
        lines.append(ser.readline().decode('utf-8', errors='ignore').strip())
    return lines

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1.0)
    print(f"Connected to {SERIAL_PORT}@{BAUD}")
    print("Commands:")
    print("  start needle <freq> <amp>")
    print("  stop needle")
    print("  start screw <freq> <amp>")
    print("  stop screw")
    print("  set kp <val>")
    print("  status")
    print("  q")
    try:
        while True:
            cmd = input("cmd> ").strip()
            if cmd in ('q','quit','exit'): break
            if not cmd: continue
            out = send(ser, cmd)
            for l in out: print("ESP32:", l)
    finally:
        ser.close()

if __name__ == '__main__':
    main()
