#!/usr/bin/env python3
"""
guide_dog_haptic.py

A prototype implementation of the "灵觉牵引" adaptive force-feedback algorithm.

Features:
- Computes a 2D force vector on a handheld leash/rod based on path deviation
  and obstacle proximity.
- Generates small pushes/pulls and vibration patterns to convey guidance.
- Pluggable output: print, UDP packet (for a haptic device/robot wirelessRemote),
  or a user-supplied callback that maps the vector to hardware commands.
- Safety limits and adaptive gains to avoid sudden large forces.

This is a high-level research prototype — tune gains and map outputs to your
hardware interface carefully before testing on a person.
"""

import math
import time
import socket
from dataclasses import dataclass
from typing import Callable, List, Tuple, Optional


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class ForceCmd:
    fx: float
    fy: float
    vib_amp: float = 0.0  # vibration amplitude 0..1
    vib_freq: float = 0.0 # vibration frequency Hz


class HapticDriver:
    """Abstracts sending force commands to hardware.

    Modes:
    - print: print to stdout
    - udp: send JSON-like ASCII via UDP to given (host,port)
    - callback: call user-provided function
    """
    def __init__(self, mode: str = "print", udp_addr: Tuple[str,int]=None, callback: Callable[[ForceCmd],None]=None):
        self.mode = mode
        self.udp_addr = udp_addr
        self.callback = callback
        if mode == "udp":
            assert udp_addr is not None
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, cmd: ForceCmd):
        if self.mode == "print":
            print(f"HAPTIC fx={cmd.fx:.3f} fy={cmd.fy:.3f} vibA={cmd.vib_amp:.2f} vibF={cmd.vib_freq:.1f}")
        elif self.mode == "udp":
            msg = f"FX:{cmd.fx:.4f},FY:{cmd.fy:.4f},VA:{cmd.vib_amp:.3f},VF:{cmd.vib_freq:.2f}"
            self.sock.sendto(msg.encode('ascii'), self.udp_addr)
        elif self.mode == "callback" and self.callback:
            self.callback(cmd)
        else:
            raise RuntimeError("Unknown HapticDriver mode")


def make_blind_cane_callback(udp_addr: Tuple[str,int]=None, serial_send: Callable[[bytes],None]=None) -> Callable[[ForceCmd],None]:
    """Return a callback that maps ForceCmd to blind-cane actuation.

    Mapping strategy (example):
    - fx (forward/back) -> short push/pull pulse on cane handle (intensity)
    - fy (lateral) -> left/right asymmetric vibration or brief lateral pulse
    - vib_amp/freq -> continuous vibration pattern during obstacle urgency

    Output format examples:
    - UDP ASCII: "CANE P:0.12 L:-0.08 V:0.50 F:30.0"
    - Serial binary: (left as implementer-specific)
    """
    sock = None
    if udp_addr:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def clamp(x, a, b):
        return max(a, min(b, x))

    def cb(cmd: ForceCmd):
        # map forces into normalized intensities -1..1
        # choose scaling conservatively for a cane actuator
        scale_force = 0.2  # N -> normalized unit; tune per hardware
        p = clamp(cmd.fx * scale_force, -1.0, 1.0)
        l = clamp(cmd.fy * scale_force, -1.0, 1.0)
        va = clamp(cmd.vib_amp, 0.0, 1.0)
        vf = cmd.vib_freq

        # Compose message for cane: push (P), lateral (L), vibration amp/freq
        msg = f"CANE P:{p:.3f} L:{l:.3f} V:{va:.3f} F:{vf:.1f}"

        if sock:
            sock.sendto(msg.encode('ascii'), udp_addr)
        elif serial_send:
            # serial_send expects bytes
            serial_send(msg.encode('ascii'))
        else:
            # fallback to print for debugging
            print("CANE_OUT:", msg)

    return cb


class LingJueTraction:
    """Adaptive force-feedback controller.

    Inputs each control tick:
    - planned path (list of waypoints in world coords)
    - current robot pose and user relative offset
    - obstacle distances (optional)

    Output:
    - ForceCmd to apply to the leash/rod
    """
    def __init__(self, driver: HapticDriver, cell_size: float=0.1):
        self.driver = driver
        # gains
        self.kp_pos = 5.0      # N/m for position error
        self.kd_drift = 1.2    # N/(m/s) for drift damping
        self.k_obst = 8.0      # N/(normalized proximity) for obstacle urgency
        self.max_force = 15.0  # N, safety cap

        # vibration settings
        self.vib_base_freq = 30.0  # Hz baseline for short pulses
        self.vib_max_amp = 1.0

        # internal state
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = None

        self.cell_size = cell_size

    def compute_force(self, pose: Pose, path: List[Tuple[float,float]], obstacles: Optional[List[Tuple[float,float]]]=None) -> ForceCmd:
        """Compute force command.

        path: list of (x,y) world coordinates. First point is current goal waypoint.
        obstacles: list of (x,y) world coords or distances may be passed; here we
        treat them as points and compute proximity.
        """
        tnow = time.time()
        dt = 0.02
        if self.prev_time is not None:
            dt = max(1e-3, tnow - self.prev_time)
        self.prev_time = tnow

        if not path:
            return ForceCmd(0.0, 0.0, 0.0, 0.0)

        # target is first waypoint
        tx, ty = path[0]

        # error in world frame
        ex = tx - pose.x
        ey = ty - pose.y

        # rotate error into user-aligned frame if needed (we assume handle aligned to user)
        # For simplicity we use world frame.

        # derivative (approx)
        edx = (ex - self.prev_error_x)/dt
        edy = (ey - self.prev_error_y)/dt
        self.prev_error_x = ex; self.prev_error_y = ey

        # position-based pull/push: attract toward path along forward axis,
        # lateral component used for gentle guidance
        fx = self.kp_pos * ex + self.kd_drift * edx
        fy = self.kp_pos * ey + self.kd_drift * edy

        # obstacle-based modifier: if obstacles are near, produce a quick retract/vibration
        vib_amp = 0.0
        vib_freq = 0.0
        if obstacles:
            # compute minimum distance to obstacles
            min_d = min(math.hypot(pose.x-ox, pose.y-oy) for (ox,oy) in obstacles)
            # normalize: 0..1 for [inf..0.2m]
            dnorm = max(0.0, min(1.0, (1.5 - min_d)/1.5))
            # push back proportional to urgency
            push = self.k_obst * dnorm
            # apply push opposite to current heading
            fx -= push * math.cos(pose.yaw)
            fy -= push * math.sin(pose.yaw)

            # vibration scales with proximity
            vib_amp = min(self.vib_max_amp, dnorm)
            vib_freq = self.vib_base_freq * (0.5 + 0.5*dnorm)

        # adaptive modulation: reduce force magnitude as user-facing safety
        mag = math.hypot(fx, fy)
        if mag > self.max_force:
            scale = self.max_force / mag
            fx *= scale; fy *= scale

        # small smoothing / deadzone
        if math.hypot(fx,fy) < 0.05:
            fx = fy = 0.0

        return ForceCmd(fx, fy, vib_amp, vib_freq)

    def tick(self, pose: Pose, path: List[Tuple[float,float]], obstacles: Optional[List[Tuple[float,float]]]=None):
        cmd = self.compute_force(pose, path, obstacles)
        self.driver.send(cmd)


def example_run():
    # simple simulation where the robot deviates slightly and an obstacle appears
    driver = HapticDriver(mode="print")
    hj = LingJueTraction(driver)

    # path: straight line from (0,0) to (3,0)
    path = [(i*0.2, 0.0) for i in range(1,16)]

    pose = Pose(0.0, -0.2, 0.0)  # user offset to the right

    obstacles = []
    for step in range(120):
        if step == 50:
            # obstacle appears at (1.8,0.2)
            obstacles.append((1.8, 0.2))
        hj.tick(pose, path, obstacles)
        # naive pose update to simulate moving forward
        pose.x += 0.02
        time.sleep(0.05)


if __name__ == "__main__":
    example_run()
