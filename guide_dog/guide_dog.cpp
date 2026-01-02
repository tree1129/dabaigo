#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>

#ifdef USE_UNITREE_SDK
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#endif

// Guide-dog path planner + simple local controller example.
// - Global planner: A* on a 2D occupancy grid
// - Local controller: follow waypoint with proportional control (v, omega)
// - Obstacle updates: placeholder function; replace with sensor subscriber
// Integration notes are in README.md. To send commands to the robot, map
// (v,omega) into the SDK's command interface (see comments below).

struct GridPos { int x, y; };

struct Node {
    int x, y;
    double g, f;
    Node* parent;
    Node(int x_, int y_) : x(x_), y(y_), g(INFINITY), f(INFINITY), parent(nullptr) {}
};

struct NodeCmp { bool operator()(const Node* a, const Node* b) const { return a->f > b->f; } };

class AStarPlanner {
public:
    AStarPlanner(int w, int h) : width(w), height(h), grid(w*h, 0) {}

    void setObstacle(int x, int y, bool occ) {
        if(inBounds(x,y)) grid[y*width + x] = occ ? 1 : 0;
    }

    bool isOccupied(int x,int y) const { if(!inBounds(x,y)) return true; return grid[y*width + x] != 0; }

    bool plan(GridPos start, GridPos goal, std::vector<GridPos>& path_out) {
        path_out.clear();
        if(!inBounds(start.x,start.y) || !inBounds(goal.x,goal.y)) return false;
        if(isOccupied(goal.x,goal.y)) return false;

        std::vector<Node> nodes; nodes.reserve(width*height);
        nodes.clear();
        for(int y=0;y<height;++y) for(int x=0;x<width;++x) nodes.emplace_back(x,y);

        auto idx = [&](int x,int y){return y*width + x;};

        Node* s = &nodes[idx(start.x,start.y)];
        Node* g = &nodes[idx(goal.x,goal.y)];

        s->g = 0.0; s->f = heuristic(start, goal); s->parent = nullptr;

        std::priority_queue<Node*, std::vector<Node*>, NodeCmp> open;
        open.push(s);
        std::vector<char> closed(width*height, 0);

        const int dx[8] = {1,-1,0,0,1,1,-1,-1};
        const int dy[8] = {0,0,1,-1,1,-1,1,-1};
        const double dcost[8] = {1,1,1,1,1.41421356,1.41421356,1.41421356,1.41421356};

        while(!open.empty()) {
            Node* cur = open.top(); open.pop();
            if(cur == g) break;
            int cx = cur->x, cy = cur->y;
            if(closed[idx(cx,cy)]) continue;
            closed[idx(cx,cy)] = 1;

            for(int k=0;k<8;++k) {
                int nx = cx + dx[k];
                int ny = cy + dy[k];
                if(!inBounds(nx,ny)) continue;
                if(isOccupied(nx,ny)) continue;
                Node* nb = &nodes[idx(nx,ny)];
                double ng = cur->g + dcost[k];
                if(ng < nb->g) {
                    nb->g = ng;
                    GridPos nbpos{nx,ny};
                    nb->f = ng + heuristic(nbpos, goal);
                    nb->parent = cur;
                    open.push(nb);
                }
            }
        }

        if(g->parent == nullptr) return false; // not found

        // reconstruct path
        Node* it = g;
        while(it) { path_out.push_back(GridPos{it->x,it->y}); it = it->parent; }
        std::reverse(path_out.begin(), path_out.end());
        return true;
    }

private:
    int width, height;
    std::vector<char> grid; // 0 free, 1 occupied

    bool inBounds(int x,int y) const { return x>=0 && x<width && y>=0 && y<height; }
    double heuristic(const GridPos& a, const GridPos& b) const {
        double dx = double(a.x-b.x), dy = double(a.y-b.y);
        return std::hypot(dx,dy);
    }
};

// Simple pose representation
struct Pose { double x, y, yaw; };

// Local controller: follow next waypoint with simple proportional law
void computeCmd(const Pose& pose, const GridPos& wp, double cell_size,
                double& v_out, double& omega_out)
{
    double dx = (wp.x+0.5)*cell_size - pose.x;
    double dy = (wp.y+0.5)*cell_size - pose.y;
    double dist = std::hypot(dx,dy);
    double angle_to_wp = std::atan2(dy,dx);
    double ang_err = angle_to_wp - pose.yaw;
    while(ang_err > M_PI) ang_err -= 2*M_PI;
    while(ang_err < -M_PI) ang_err += 2*M_PI;

    double K_v = 0.8;
    double K_omega = 1.2;
    double max_v = 0.6; // m/s
    double max_omega = 1.2; // rad/s

    // if heading error large, rotate in place
    if (std::fabs(ang_err) > 0.5) {
        v_out = 0.0;
        omega_out = std::copysign(std::min(max_omega, K_omega * std::fabs(ang_err)), ang_err);
    } else {
        v_out = std::min(max_v, K_v * dist);
        omega_out = std::copysign(std::min(max_omega, K_omega * std::fabs(ang_err)), ang_err);
    }
}

// Placeholder: map (v,omega) to a robot command. Right now prints to stdout.
// To integrate with the SDK, replace the body with a publisher using ChannelPublisher
// and publish a suitable command message (e.g., a gait controller input or convert
// into LowCmd_ motor values). An outline is in README.md.
// When built with USE_UNITREE_SDK the program will publish high-level
// velocity commands using the SportClient Move(vx,vy,wz) API and
// subscribe to SportModeState_ for pose feedback.
#ifdef USE_UNITREE_SDK
static std::mutex g_pose_mtx;
static Pose g_robot_pose{0,0,0};
static unitree::robot::go2::SportClient* g_sport_client = nullptr;

void HighStateHandler(const void* message)
{
    auto state = *(unitree_go::msg::dds_::SportModeState_*)message;
    Pose p;
    p.x = state.position()[0];
    p.y = state.position()[1];
    p.yaw = state.imu_state().rpy()[2];
    std::lock_guard<std::mutex> lk(g_pose_mtx);
    g_robot_pose = p;
}

void SendVelCommand(double v, double omega) {
    // send high-level velocity to robot via SportClient
    if(g_sport_client) {
        // Move(vx, vy, wz)
        g_sport_client->Move((float)v, 0.0f, (float)omega);
    }
    std::cout << "SDK CMD: v=" << v << " m/s, omega=" << omega << " rad/s\n";
}
#else
void SendVelCommand(double v, double omega) {
    std::cout << "CMD: v=" << v << " m/s, omega=" << omega << " rad/s\n";
}
#endif

// Placeholder: update occupancy grid from sensors. Replace with actual sensor subscription.
void UpdateOccupancyGridSim(AStarPlanner& planner) {
    // Add a moving obstacle example
    static int t = 0; ++t;
    // clear a column and add an obstacle that moves across
    for(int y=0;y<50;++y) planner.setObstacle(20,y,false);
    int ox = 20 + (t/20)%15;
    for(int yy=22; yy<28; ++yy) planner.setObstacle(ox, yy, true);
}

int main(int argc, char** argv) {
    // grid parameters
    const int W = 80, H = 60;
    const double cell_size = 0.1; // meters

    AStarPlanner planner(W,H);

    // Example: set some static obstacles (a wall with a door)
    for(int y=10; y<50; ++y) planner.setObstacle(40,y,true);
    // a gap
    for(int y=24; y<27; ++y) planner.setObstacle(40,y,false);

    // start and goal in world coordinates (meters)
    double sx = 1.0, sy = 1.0;
    double gx = 6.0, gy = 3.0;

    GridPos start{int(sx / cell_size), int(sy / cell_size)};
    GridPos goal{int(gx / cell_size), int(gy / cell_size)};

    std::vector<GridPos> path;

    if(!planner.plan(start, goal, path)) {
        std::cerr << "Initial planning failed.\n";
        return -1;
    }

    std::cout << "Planned path length: " << path.size() << "\n";

    // initial pose (center of start cell)
    Pose pose{(start.x+0.5)*cell_size, (start.y+0.5)*cell_size, 0.0};

    size_t current_wp_idx = 1; // follow the second point (first is start)

#ifdef USE_UNITREE_SDK
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " networkInterface\n";
        return -1;
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // create sport client and subscriber for high-level state
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> state_sub;
    state_sub.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>("rt/sportmodestate"));
    state_sub->InitChannel(std::bind(&HighStateHandler, std::placeholders::_1), 1);

    // wait briefly for state
    sleep(1);

    g_sport_client = &sport_client;
#endif

    // main loop: each iteration we optionally update obstacles, replan if blocked,
    // compute velocity and send command.
    for(int iter=0; iter<2000; ++iter) {
        // simulate sensor updates affecting occupancy
        UpdateOccupancyGridSim(planner);

        // if next waypoint becomes occupied, replan from current cell
        GridPos curcell{int(pose.x / cell_size), int(pose.y / cell_size)};
        if(current_wp_idx < path.size()) {
            GridPos next = path[current_wp_idx];
            if(planner.isOccupied(next.x, next.y)) {
                std::cout << "Waypoint occupied, replanning...\n";
                GridPos goalcell = goal;
                if(!planner.plan(curcell, goalcell, path)) {
                    std::cout << "Replan failed, stopping.\n";
                    SendVelCommand(0,0);
                    break;
                }
                current_wp_idx = 1;
            }
        }

        if(current_wp_idx >= path.size()) {
            std::cout << "Reached goal.\n";
            SendVelCommand(0,0);
            break;
        }

        GridPos wp = path[current_wp_idx];

        double v, omega;
        computeCmd(pose, wp, cell_size, v, omega);

        // simple pose integration to simulate robot movement when not using SDK
        double dt = 0.05; // 20Hz control loop
    #ifdef USE_UNITREE_SDK
        // update pose from real robot state if available
        {
            std::lock_guard<std::mutex> lk(g_pose_mtx);
            pose = g_robot_pose;
        }
    #else
        pose.x += v * std::cos(pose.yaw) * dt;
        pose.y += v * std::sin(pose.yaw) * dt;
        pose.yaw += omega * dt;
    #endif

        // check if waypoint reached
        double wx = (wp.x+0.5)*cell_size; double wy = (wp.y+0.5)*cell_size;
        if(std::hypot(pose.x-wx, pose.y-wy) < 0.12) {
            current_wp_idx++;
        }

        SendVelCommand(v, omega);

        std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    }

    return 0;
}
