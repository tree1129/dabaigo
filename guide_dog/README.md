# Guide Dog Example (A* + Local Avoidance)

This example implements a simple 2D grid A* global planner and a lightweight
local controller that follows waypoints while reacting to dynamic obstacles.

Files:
- `guide_dog.cpp`: the planner + controller example.
- `CMakeLists.txt`: build file for the example.

Quick build:

```bash
cd unitree_sdk2-main/example/guide_dog
mkdir -p build && cd build
cmake ..
make -j
./guide_dog
```

What the example does:
- Creates a 80x60 occupancy grid (0.1 m resolution).
- Plans from a start to goal with A*.
- Runs a simulated control loop (20 Hz) producing (v, omega) commands.
- Simulates a moving obstacle; when a waypoint becomes blocked the planner
  replans online.

Integration with SDK:
- The example currently prints `(v, omega)` to stdout. To actually command
  the robot you must map linear/angular velocity to your robot control interface.

Two common approaches:
1) Use a higher-level gait/locomotion controller in the Unitree guide project
   (for example the `unitree_guide` package) to accept velocity commands.
   You can bridge by publishing `(v,omega)` to a topic the controller listens
   to (via ROS) or by converting the velocity into the robot's `LowCmd_` format
   used in `example/go2`.

2) Direct mapping to `LowCmd_` (advanced): convert desired body velocity into
   joint/motor references and publish `LowCmd_` using `ChannelPublisher<unitree_go::msg::dds_::LowCmd_>`.
   This requires understanding the gait controller and motor mapping; use this
   only if you're familiar with the SDK low-level control.

Where to hook code:
- In `guide_dog.cpp`, replace `SendVelCommand` with code that constructs and
  publishes an SDK message. See `example/go2/go2_low_level.cpp` for how to
  initialize a `ChannelPublisher` and publish messages.

Safety notes:
- Test first in simulation (Gazebo or the unitree simulator) before running on
  hardware. Tuning K_v and K_omega is required for smooth behavior.

If you want, I can:
- add a `#ifdef USE_UNITREE_SDK` section to `guide_dog.cpp` that publishes
  `LowCmd_` when compiled with SDK includes, or
- convert the example to a ROS node publishing `geometry_msgs/Twist`.

Tell me which integration target you prefer and I will update the example
to publish commands directly to the robot.
