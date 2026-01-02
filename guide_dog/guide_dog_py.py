#!/usr/bin/env python3
"""
guide_dog_py.py

A simple Python planner that plans from A to B on a 2D grid and sends
velocity commands via UDP to a local bridge which forwards them to the
Unitree SDK (udp_to_sport).

Usage:
  1) build and run udp_to_sport (C++ bridge):
     mkdir build && cd build
     cmake ..
     make udp_to_sport
     ./udp_to_sport <networkInterface> 14551

  2) run this planner sending UDP to the bridge (host is the machine running udp_to_sport):
     python3 guide_dog_py.py <bridge_host> 14551

This is a minimal demo: it computes a straight A* path on a grid, follows
waypoints with a proportional controller, and does a local replan when
an obstacle appears on the next waypoint.
"""

import math
import socket
import sys
import time

class GridAStar:
    def __init__(self, w, h):
        self.w = w; self.h = h
        self.grid = [0]*(w*h)

    def set_obstacle(self, x, y):
        if 0<=x<self.w and 0<=y<self.h:
            self.grid[y*self.w + x] = 1

    def is_occ(self, x, y):
        if not (0<=x<self.w and 0<=y<self.h): return True
        return self.grid[y*self.w + x] != 0

    def plan(self, sx, sy, gx, gy):
        from heapq import heappush, heappop
        def h(x,y): return math.hypot(x-gx, y-gy)
        openq = []
        gscore = { (sx,sy): 0 }
        heappush(openq, (h(sx,sy), sx, sy, None))
        parents = {}
        seen = set()
        while openq:
            f,x,y,par = heappop(openq)
            if (x,y) in seen: continue
            seen.add((x,y)); parents[(x,y)] = par
            if x==gx and y==gy:
                # reconstruct
                path = []
                cur = (x,y)
                while cur:
                    path.append(cur); cur = parents[cur]
                path.reverse(); return path
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx,ny = x+dx, y+dy
                if self.is_occ(nx,ny): continue
                ng = gscore[(x,y)] + math.hypot(dx,dy)
                if (nx,ny) not in gscore or ng < gscore[(nx,ny)]:
                    gscore[(nx,ny)] = ng
                    heappush(openq, (ng + h(nx,ny), nx, ny, (x,y)))
        return None

def send_vel(sock, addr, vx, wz):
    # message format accepted by udp_to_sport
    msg = f"{vx:.3f},{wz:.3f}"
    sock.sendto(msg.encode('ascii'), addr)

def main():
    if len(sys.argv) < 3:
        print("Usage: guide_dog_py.py <bridge_host> <bridge_port>")
        return
    host = sys.argv[1]; port = int(sys.argv[2])
    addr = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # grid and example
    cell = 0.1
    W,H = 80,60
    planner = GridAStar(W,H)
    # static obstacle line with a gap
    for y in range(10,50):
        planner.set_obstacle(40, y)
    for y in range(24,27):
        planner.grid[y*W + 40] = 0

    # start and goal in meters
    sx,sy = 1.0, 1.0
    gx,gy = 6.0, 3.0
    start = (int(sx/cell), int(sy/cell))
    goal = (int(gx/cell), int(gy/cell))

    path = planner.plan(start[0], start[1], goal[0], goal[1])
    if not path:
        print('Plan failed'); return
    print('Planned len', len(path))

    # control loop
    pose_x = (start[0]+0.5)*cell; pose_y = (start[1]+0.5)*cell; yaw = 0.0
    wp_idx = 1
    dt = 0.05
    max_v = 0.5; Kv=1.0; Kow=1.2

    for it in range(4000):
        # simulate dynamic obstacle at some time
        if it == 400:
            # block next waypoint to force replan
            if wp_idx < len(path):
                bx,by = path[wp_idx]
                planner.set_obstacle(bx, by)

        if wp_idx >= len(path):
            send_vel(sock, addr, 0.0, 0.0)
            print('Reached')
            break

        nx,ny = path[wp_idx]
        if planner.is_occ(nx,ny):
            print('Waypoint blocked, replanning...')
            curcell = (int(pose_x/cell), int(pose_y/cell))
            newp = planner.plan(curcell[0], curcell[1], goal[0], goal[1])
            if not newp:
                print('Replan failed; stopping'); send_vel(sock, addr, 0.0, 0.0); break
            path = newp; wp_idx = 1; continue

        wx = (nx+0.5)*cell; wy = (ny+0.5)*cell
        dx = wx - pose_x; dy = wy - pose_y
        dist = math.hypot(dx,dy)
        target_yaw = math.atan2(dy,dx)
        ang_err = target_yaw - yaw
        while ang_err > math.pi: ang_err -= 2*math.pi
        while ang_err < -math.pi: ang_err += 2*math.pi

        if abs(ang_err) > 0.5:
            v = 0.0
            wz = max(-1.0, min(1.0, Kow*ang_err))
        else:
            v = max(-max_v, min(max_v, Kv*dist))
            wz = max(-1.0, min(1.0, Kow*ang_err))

        send_vel(sock, addr, v, wz)

        # simple kinematic update (will be overwritten by real robot feedback when used live)
        pose_x += v * math.cos(yaw) * dt
        pose_y += v * math.sin(yaw) * dt
        yaw += wz * dt

        if dist < 0.12:
            wp_idx += 1

        time.sleep(dt)

    send_vel(sock, addr, 0.0, 0.0)
    sock.close()

if __name__ == '__main__':
    main()
