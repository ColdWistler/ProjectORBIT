/*
 * JSBSim UDP Bridge
 * 
 * This is a simple C++ program that wraps JSBSim and provides UDP output/input
 * for integration with Godot.
 * 
 * Build: g++ -o jsbsim_bridge jsbsim_bridge.cpp -ljsbsim $(pkg-config --cflags --libs simgear-core) -std=c++17
 * 
 * Alternatively, you can use this as a reference to create a GDExtension.
 */

#include <FGFDMExec.h>
#include <FGInitialCondition.h>
#include <FGPropertyManager.h>
#include <FGMatrix33.h>
#include <FGQuaternion.h>
#include <iostream>
#include <string>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// JSBSim configuration
std::string aircraft_path = "./aircraft";
std::string engine_path = "./engine";
std::string model_name = "c172p";
double sim_dt = 1.0/60.0;

// UDP configuration
int udp_port = 12345;
std::string udp_host = "127.0.0.1";

struct ControlInputs {
    double throttle = 0.0;
    double elevator = 0.0;
    double aileron = 0.0;
    double rudder = 0.0;
    double flaps = 0.0;
    double gear = 1.0;
    double mix = 0.0;
    double aileron_trim = 0.0;
    double elevator_trim = 0.0;
    double rudder_trim = 0.0;
};

class JSBSimBridge {
private:
    JSBSim::FGFDMExec* fdm;
    int sock_fd;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    ControlInputs controls;
    
public:
    JSBSimBridge() : fdm(nullptr), sock_fd(-1) {}
    
    bool init(const std::string& model) {
        fdm = new JSBSim::FGFDMExec();
        
        if (!fdm->LoadModel(aircraft_path, engine_path, model)) {
            std::cerr << "Failed to load model: " << model << std::endl;
            return false;
        }
        
        fdm->GetPropertyManager()->SetDouble("simulation/dt", sim_dt);
        
        fdm->RunIC();
        
        return init_udp();
    }
    
    bool init_udp() {
        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = inet_addr(udp_host.c_str());
        server_addr.sin_port = htons(udp_port);
        
        return true;
    }
    
    void set_control(double throttle, double elevator, double aileron, double rudder,
                     double flaps, double gear, double mix_val,
                     double aileron_trim, double elevator_trim, double rudder_trim) {
        controls.throttle = throttle;
        controls.elevator = elevator;
        controls.aileron = aileron;
        controls.rudder = rudder;
        controls.flaps = flaps;
        controls.gear = gear;
        controls.mix = mix_val;
        controls.aileron_trim = aileron_trim;
        controls.elevator_trim = elevator_trim;
        controls.rudder_trim = rudder_trim;
        
        auto propMgr = fdm->GetPropertyManager();
        propMgr->SetDouble("controls/throttle", controls.throttle);
        propMgr->SetDouble("controls/elevator", controls.elevator);
        propMgr->SetDouble("controls/aileron", controls.aileron);
        propMgr->SetDouble("controls/rudder", controls.rudder);
        propMgr->SetDouble("controls/flaps", controls.flaps);
        propMgr->SetDouble("controls/gear", controls.gear);
        propMgr->SetDouble("controls/mix", controls.mix);
        propMgr->SetDouble("controls/aileron-trim", controls.aileron_trim);
        propMgr->SetDouble("controls/elevator-trim", controls.elevator_trim);
        propMgr->SetDouble("controls/rudder-trim", controls.rudder_trim);
    }
    
    bool receive_controls() {
        char buffer[256];
        socklen_t addr_len = sizeof(client_addr);
        
        int len = recvfrom(sock_fd, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                          (struct sockaddr*)&client_addr, &addr_len);
        
        if (len > 0) {
            buffer[len] = '\0';
            
            double vals[10];
            char* token = strtok(buffer, ",");
            int i = 0;
            while (token != nullptr && i < 10) {
                vals[i++] = atof(token);
                token = strtok(nullptr, ",");
            }
            
            if (i >= 6) {
                set_control(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5],
                          (i > 6 ? vals[6] : 0.0),
                          (i > 7 ? vals[7] : 0.0),
                          (i > 8 ? vals[8] : 0.0),
                          (i > 9 ? vals[9] : 0.0));
            }
            return true;
        }
        return false;
    }
    
    void send_state() {
        auto propMgr = fdm->GetPropertyManager();
        
        double lat = propMgr->GetDouble("position/lat-gc-deg") * M_PI / 180.0;
        double lon = propMgr->GetDouble("position/long-gc-deg") * M_PI / 180.0;
        double alt = propMgr->GetDouble("position/h-sl-ft") * 0.3048;
        
        double q0 = propMgr->GetDouble("orientation/phi-rad");
        double q1 = propMgr->GetDouble("orientation/theta-rad");
        double q2 = propMgr->GetDouble("orientation/psi-rad");
        
        double u = propMgr->GetDouble("velocities/u-fps") * 0.3048;
        double v = propMgr->GetDouble("velocities/v-fps") * 0.3048;
        double w = propMgr->GetDouble("velocities/w-fps") * 0.3048;
        
        double p = propMgr->GetDouble("velocities/p-rad_sec");
        double q = propMgr->GetDouble("velocities/q-rad_sec");
        double r = propMgr->GetDouble("velocities/r-rad_sec");
        
        double v_north = propMgr->GetDouble("velocities/v-north-fps") * 0.3048;
        double v_east = propMgr->GetDouble("velocities/v-east-fps") * 0.3048;
        double v_down = propMgr->GetDouble("velocities/v-down-fps") * 0.3048;
        
        double vc = propMgr->GetDouble("velocities/vc-fps") * 0.3048;
        double mach = propMgr->GetDouble("velocities/mach");
        double alpha = propMgr->GetDouble("aero/alpha-deg") * M_PI / 180.0;
        double beta = propMgr->GetDouble("aero/beta-deg") * M_PI / 180.0;
        
        double sim_time = propMgr->GetDouble("simulation/sim-time-secs");
        
        char buffer[512];
        snprintf(buffer, sizeof(buffer), 
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                lat, lon, alt,
                q0, q1, q2,
                u, v, w,
                p, q, r,
                alt, vc, mach, alpha, beta,
                sim_time);
        
        sendto(sock_fd, buffer, strlen(buffer), 0,
              (struct sockaddr*)&client_addr, sizeof(client_addr));
    }
    
    void run() {
        while (true) {
            receive_controls();
            
            fdm->Run();
            
            send_state();
            
            usleep(static_cast<int>(sim_dt * 1000000));
        }
    }
    
    ~JSBSimBridge() {
        if (fdm) delete fdm;
        if (sock_fd >= 0) close(sock_fd);
    }
};

int main(int argc, char** argv) {
    if (argc > 1) model_name = argv[1];
    if (argc > 2) udp_port = atoi(argv[2]);
    
    JSBSimBridge bridge;
    
    if (!bridge.init(model_name)) {
        return 1;
    }
    
    std::cout << "JSBSim UDP Bridge started on port " << udp_port << std::endl;
    std::cout << "Using aircraft: " << model_name << std::endl;
    
    bridge.run();
    
    return 0;
}
