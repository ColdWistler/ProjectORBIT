#!/usr/bin/env python3
"""
JSBSim UDP Bridge - Python Version

This script wraps JSBSim and provides UDP input/output for Godot integration.
Requires JSBSim Python package: pip install jsbsim

Usage:
    python jsbsim_bridge.py [aircraft_model] [udp_port]
    
Example:
    python jsbsim_bridge.py c172p 12345
"""

import socket
import struct
import sys
import os
import time
from math import pi, degrees, radians

# Suppress JSBSim verbose output before import
JSBSIM_AVAILABLE = False
try:
    # Redirect stderr to suppress JSBSim initialization output
    old_stderr = sys.stderr
    sys.stderr = open(os.devnull, 'w')
    import jsbsim
    from jsbsim import FGFDMExec
    sys.stderr.close()
    sys.stderr = old_stderr
    JSBSIM_AVAILABLE = True
except ImportError:
    try:
        sys.stderr.close()
    except:
        pass
    sys.stderr = old_stderr
    print("Warning: JSBSim Python package not installed", file=sys.stderr)
    print("Install with: pip install jsbsim", file=sys.stderr)
    print("Using mock mode for testing...", file=sys.stderr)

DEFAULT_AIRCRAFT = "c172p"
DEFAULT_PORT = 12345
DEFAULT_HOST = "127.0.0.1"

class JSBSimBridge:
    def __init__(self, aircraft_model=DEFAULT_AIRCRAFT, port=DEFAULT_PORT, host=DEFAULT_HOST):
        self.port = port
        self.host = host
        self.aircraft_model = aircraft_model
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.settimeout(0.01)
        
        self.controls = {
            'throttle': 0.0,
            'elevator': 0.0,
            'aileron': 0.0,
            'rudder': 0.0,
            'flaps': 0.0,
            'gear': 1.0,
            'mix': 0.0,
            'aileron_trim': 0.0,
            'elevator_trim': 0.0,
            'rudder_trim': 0.0
        }
        
        self.client_addr = None
        self.fdm = None
        
        if JSBSIM_AVAILABLE:
            self._init_jsbsim()
        else:
            self._init_mock()
    
    def _init_jsbsim(self):
        old_stderr = sys.stderr
        try:
            sys.stderr = open(os.devnull, 'w')
            self.fdm = FGFDMExec(None)
            self.fdm.load_model(self.aircraft_model)
            self.fdm.set_dt(1.0/60.0)
            self.fdm.run_ic()
            sys.stderr.close()
            sys.stderr = old_stderr
            print(f"JSBSim initialized with aircraft: {self.aircraft_model}")
        except Exception as e:
            try:
                sys.stderr.close()
            except:
                pass
            sys.stderr = old_stderr
            print(f"Failed to initialize JSBSim: {e}")
            self._init_mock()
    
    def _init_mock(self):
        self.fdm = None
        self.mock_state = {
            'lat': 0.0,
            'lon': 0.0,
            'alt': 1000.0,
            'phi': 0.0,
            'theta': 0.0,
            'psi': 0.0,
            'u': 0.0,
            'v': 0.0,
            'w': 0.0,
            'p': 0.0,
            'q': 0.0,
            'r': 0.0,
            'vc': 0.0,
            'mach': 0.0,
            'alpha': 0.0,
            'beta': 0.0,
            'sim_time': 0.0
        }
        print("Running in MOCK mode (no real FDM)")
    
    def receive_controls(self):
        try:
            data, addr = self.sock.recvfrom(4096)
            self.client_addr = addr
            
            values = [float(x) for x in data.decode().split(',')]
            
            if len(values) >= 6:
                self.controls['throttle'] = values[0]
                self.controls['elevator'] = values[1]
                self.controls['aileron'] = values[2]
                self.controls['rudder'] = values[3]
                self.controls['flaps'] = values[4]
                self.controls['gear'] = values[5]
                
                if len(values) > 6:
                    self.controls['mix'] = values[6]
                if len(values) > 7:
                    self.controls['aileron_trim'] = values[7]
                if len(values) > 8:
                    self.controls['elevator_trim'] = values[8]
                if len(values) > 9:
                    self.controls['rudder_trim'] = values[9]
                    
        except socket.timeout:
            pass
        except Exception as e:
            pass
    
    def apply_controls_jsbsim(self):
        if not self.fdm:
            return
        
        self.fdm.set_property_value("controls/throttle-cmd-norm", self.controls['throttle'])
        self.fdm.set_property_value("controls/elevator-cmd-norm", self.controls['elevator'])
        self.fdm.set_property_value("controls/aileron-cmd-norm", self.controls['aileron'])
        self.fdm.set_property_value("controls/rudder-cmd-norm", self.controls['rudder'])
        self.fdm.set_property_value("controls/flap-cmd-norm", self.controls['flaps'])
        self.fdm.set_property_value("controls/gear-cmd-norm", self.controls['gear'])
    
    def apply_controls_mock(self):
        ctrl = self.controls
        
        self.mock_state['theta'] += ctrl['elevator'] * 0.001
        self.mock_state['phi'] += ctrl['aileron'] * 0.001
        self.mock_state['psi'] += ctrl['rudder'] * 0.001
        
        throttle = ctrl['throttle']
        speed = throttle * 150.0
        
        self.mock_state['u'] = speed
        self.mock_state['alt'] += (throttle - 0.3) * 0.5
        self.mock_state['alt'] = max(0, self.mock_state['alt'])
        
        self.mock_state['vc'] = speed
        self.mock_state['mach'] = speed / 343.0
        self.mock_state['alpha'] = ctrl['elevator'] * 0.1
        self.mock_state['beta'] = ctrl['aileron'] * 0.1
    
    def step(self):
        self.receive_controls()
        
        if self.fdm:
            self.apply_controls_jsbsim()
            self.fdm.run()
        else:
            self.apply_controls_mock()
            self.mock_state['sim_time'] += 1.0/60.0
        
        self.send_state()
    
    def get_state_jsbsim(self):
        return {
            'lat': self.fdm.get_property_value("position/lat-gc-deg"),
            'lon': self.fdm.get_property_value("position/long-gc-deg"),
            'alt': self.fdm.get_property_value("position/h-sl-ft") * 0.3048,
            'phi': self.fdm.get_property_value("orientation/phi-rad"),
            'theta': self.fdm.get_property_value("orientation/theta-rad"),
            'psi': self.fdm.get_property_value("orientation/psi-rad"),
            'u': self.fdm.get_property_value("velocities/u-fps") * 0.3048,
            'v': self.fdm.get_property_value("velocities/v-fps") * 0.3048,
            'w': self.fdm.get_property_value("velocities/w-fps") * 0.3048,
            'p': self.fdm.get_property_value("velocities/p-rad_sec"),
            'q': self.fdm.get_property_value("velocities/q-rad_sec"),
            'r': self.fdm.get_property_value("velocities/r-rad_sec"),
            'vc': self.fdm.get_property_value("velocities/vc-fps") * 0.3048,
            'mach': self.fdm.get_property_value("velocities/mach"),
            'alpha': self.fdm.get_property_value("aero/alpha-deg") * pi / 180.0,
            'beta': self.fdm.get_property_value("aero/beta-deg") * pi / 180.0,
            'sim_time': self.fdm.get_property_value("simulation/sim-time-secs")
        }
    
    def get_state_mock(self):
        return self.mock_state.copy()
    
    def send_state(self):
        if not self.client_addr:
            return
            
        if self.fdm:
            state = self.get_state_jsbsim()
        else:
            state = self.get_state_mock()
        
        msg = f"{state['lat']},{state['lon']},{state['alt']}," \
              f"{state['phi']},{state['theta']},{state['psi']}," \
              f"{state['u']},{state['v']},{state['w']}," \
              f"{state['p']},{state['q']},{state['r']}," \
              f"{state['alt']},{state['vc']},{state['mach']}," \
              f"{state['alpha']},{state['beta']},{state['sim_time']}"
        
        self.sock.sendto(msg.encode(), self.client_addr)
    
    def run(self):
        print(f"JSBSim Bridge running on {self.host}:{self.port}")
        print(f"Aircraft: {self.aircraft_model}")
        
        try:
            while True:
                self.step()
                time.sleep(1.0/60.0)
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.sock.close()


if __name__ == "__main__":
    aircraft = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_AIRCRAFT
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    
    bridge = JSBSimBridge(aircraft, port)
    bridge.run()
