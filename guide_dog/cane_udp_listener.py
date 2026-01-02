#!/usr/bin/env python3
"""
Simple UDP listener for blind-cane messages.

Run this on the machine that interfaces with the cane (or on the cane if it can run Python).
It listens for ASCII messages in the format produced by `guide_dog_haptic.py`:
  CANE P:<p> L:<l> V:<v> F:<f>
and prints parsed values; replace the print block with code that drives your
actuators (PWM, motor controller, BLE/GATT write, etc.).
"""

import socket
import re

HOST = '0.0.0.0'
PORT = 9000

MSG_RE = re.compile(r"CANE\s+P:(?P<p>[-0-9.]+)\s+L:(?P<l>[-0-9.]+)\s+V:(?P<v>[-0-9.]+)\s+F:(?P<f>[-0-9.]+)")

def map_to_actuator(p, l, v, f):
    # Example mapping: convert normalized p/l [-1..1] into PWM duty 0..255
    pwm_push = int((clamp(p, -1.0, 1.0) * 0.5 + 0.5) * 255)
    pwm_lat = int((clamp(l, -1.0, 1.0) * 0.5 + 0.5) * 255)
    vib_strength = int(clamp(v, 0.0, 1.0) * 255)
    return pwm_push, pwm_lat, vib_strength, f

def clamp(x, a, b):
    return max(a, min(b, x))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    print(f"Cane UDP listener running on {HOST}:{PORT}")
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            s = data.decode('ascii', errors='ignore').strip()
            m = MSG_RE.search(s)
            if not m:
                print("Unknown message:", s)
                continue
            p = float(m.group('p'))
            l = float(m.group('l'))
            v = float(m.group('v'))
            f = float(m.group('f'))

            pwm_push, pwm_lat, vib_strength, freq = map_to_actuator(p, l, v, f)
            # Replace the following print with actual actuator code (PWM, serial, BLE)
            print(f"Recv from {addr}: P={p:.3f} L={l:.3f} V={v:.3f} F={f:.1f} -> PWM_push={pwm_push} PWM_lat={pwm_lat} VIB={vib_strength} Freq={freq:.1f}")

    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
