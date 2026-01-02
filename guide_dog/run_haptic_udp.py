#!/usr/bin/env python3
"""
run_haptic_udp.py

Simple runner that uses `guide_dog_haptic` and sends UDP messages to a cane.
Usage: python3 run_haptic_udp.py <cane_host> <cane_port>
"""

import sys
import time
from guide_dog_haptic import HapticDriver, LingJueTraction, Pose


def main():
    if len(sys.argv) < 3:
        print("Usage: run_haptic_udp.py <cane_host> <cane_port>")
        return
    host = sys.argv[1]
    port = int(sys.argv[2])

    driver = HapticDriver(mode="udp", udp_addr=(host, port))
    hj = LingJueTraction(driver)

    # path and pose demo
    path = [(i*0.2, 0.0) for i in range(1, 16)]
    pose = Pose(0.0, -0.2, 0.0)

    print(f"Sending UDP haptic messages to {host}:{port}")
    for step in range(200):
        if step == 80:
            # obstacle simulated by adding an obstacle to the call; LingJueTraction expects list of tuples
            obstacles = [(1.8, 0.2)]
        else:
            obstacles = []

        hj.tick(pose, path, obstacles)
        pose.x += 0.02
        time.sleep(0.05)


if __name__ == '__main__':
    main()
