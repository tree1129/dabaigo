/**********************************************************************
 UDP -> SportClient bridge
 Listens for ASCII UDP messages containing velocity commands and forwards
 them to the Unitree SportClient API (Move(vx, vy, wz)).

 Usage: udp_to_sport <networkInterface> <udp_port>
 Example message formats (ASCII):
  VX:0.30,WZ:0.10
  0.30,0.10
----------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace unitree::robot;
using namespace unitree::common;

static std::atomic<bool> running(true);

void run_udp_server(int port, float &out_vx, float &out_wz)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return;
    }

    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if (bind(sockfd, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("bind");
        close(sockfd);
        return;
    }

    char buf[256];
    while (running.load()) {
        sockaddr_in cliaddr{};
        socklen_t len = sizeof(cliaddr);
        ssize_t n = recvfrom(sockfd, buf, sizeof(buf)-1, 0, (sockaddr*)&cliaddr, &len);
        if (n <= 0) continue;
        buf[n] = '\0';
        std::string s(buf);

        // try parsing as 'VX:x,WZ:y' or 'x,y'
        float vx=0.0f, wz=0.0f;
        bool ok=false;
        try {
            size_t p = s.find("VX:");
            if (p != std::string::npos) {
                size_t comma = s.find(',', p);
                std::string sx = s.substr(p+3, comma - (p+3));
                vx = std::stof(sx);
                size_t p2 = s.find("WZ:");
                if (p2 != std::string::npos) {
                    std::string sw = s.substr(p2+3);
                    wz = std::stof(sw);
                }
                ok = true;
            } else {
                // try simple comma-separated
                size_t comma = s.find(',');
                if (comma != std::string::npos) {
                    vx = std::stof(s.substr(0,comma));
                    wz = std::stof(s.substr(comma+1));
                    ok = true;
                }
            }
        } catch(...) { ok = false; }

        if (ok) {
            out_vx = vx;
            out_wz = wz;
            // echo back (optional)
            // sendto(sockfd, buf, n, 0, (sockaddr*)&cliaddr, len);
        }
    }

    close(sockfd);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <networkInterface> <udp_port>\n";
        return -1;
    }

    const char* netif = argv[1];
    int udp_port = atoi(argv[2]);

    ChannelFactory::Instance()->Init(0, netif);

    unitree::robot::go2::SportClient sport_client;

    float vx = 0.0f, wz = 0.0f;

    std::thread udp_thread(run_udp_server, udp_port, std::ref(vx), std::ref(wz));

    std::cout << "udp_to_sport running; listening on port " << udp_port << "\n";

    // control loop: forward latest vx/wz to sport client at ~20Hz
    while (true) {
        sport_client.Move(vx, 0.0f, wz);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    running.store(false);
    udp_thread.join();
    return 0;
}
