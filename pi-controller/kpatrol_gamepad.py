#!/usr/bin/env python3
"""
K-Patrol Bot - Joystick/Gamepad Controller
Điều khiển robot bằng tay cầm game (Xbox, PS4, etc.)

Yêu cầu: pip3 install inputs
Sử dụng: python3 kpatrol_gamepad.py [port]
"""

import serial
import time
import sys
import threading
import math
from typing import Optional, Tuple

try:
    from inputs import get_gamepad, devices
    HAS_INPUTS = True
except ImportError:
    HAS_INPUTS = False
    print("⚠️  Thư viện 'inputs' chưa được cài đặt!")
    print("   Chạy: pip3 install inputs")

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# Joystick thresholds
DEADZONE = 0.15
MAX_AXIS = 32767

# ═══════════════════════════════════════════════════════════════════════════════
# MECANUM MIXER
# ═══════════════════════════════════════════════════════════════════════════════

class MecanumMixer:
    """
    Chuyển đổi input joystick thành tốc độ cho 4 bánh Mecanum
    
    Input:
        - x: Strafe (-1 to 1, trái/phải)
        - y: Forward (-1 to 1, tiến/lùi)
        - rotation: Xoay (-1 to 1, trái/phải)
        
    Output:
        - (FL, FR, BL, BR) speeds (-255 to 255)
    """
    
    @staticmethod
    def mix(x: float, y: float, rotation: float, max_speed: int = 255) -> Tuple[int, int, int, int]:
        """
        Tính toán tốc độ cho 4 bánh Mecanum
        
        Công thức Mecanum:
            FL = y + x + rotation
            FR = y - x - rotation
            BL = y - x + rotation
            BR = y + x - rotation
        """
        # Apply deadzone
        if abs(x) < DEADZONE:
            x = 0
        if abs(y) < DEADZONE:
            y = 0
        if abs(rotation) < DEADZONE:
            rotation = 0
            
        # Calculate wheel speeds
        fl = y + x + rotation
        fr = y - x - rotation
        bl = y - x + rotation
        br = y + x - rotation
        
        # Find max for normalization
        max_val = max(abs(fl), abs(fr), abs(bl), abs(br), 1.0)
        
        # Normalize and scale
        fl = int((fl / max_val) * max_speed)
        fr = int((fr / max_val) * max_speed)
        bl = int((bl / max_val) * max_speed)
        br = int((br / max_val) * max_speed)
        
        return (fl, fr, bl, br)


# ═══════════════════════════════════════════════════════════════════════════════
# GAMEPAD CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class GamepadController:
    """Điều khiển bằng gamepad"""
    
    def __init__(self, port: str):
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.running = True
        
        # Gamepad state
        self.left_x = 0.0    # Strafe
        self.left_y = 0.0    # Forward/Backward
        self.right_x = 0.0   # Rotation
        self.max_speed = 200
        
        # Buttons
        self.emergency_stop = False
        
    def connect_serial(self) -> bool:
        """Kết nối Serial với ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=0.5
            )
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.connected = True
            print(f"✅ Serial connected to {self.port}")
            return True
        except Exception as e:
            print(f"❌ Serial connection error: {e}")
            return False
            
    def send_command(self, cmd: str):
        """Gửi lệnh đến ESP32"""
        if self.serial and self.connected:
            try:
                self.serial.write(f"{cmd}\n".encode())
                self.serial.flush()
            except:
                pass
                
    def process_event(self, event):
        """Xử lý event từ gamepad"""
        # Left stick X (Strafe)
        if event.code == 'ABS_X':
            self.left_x = event.state / MAX_AXIS
            
        # Left stick Y (Forward/Backward)
        elif event.code == 'ABS_Y':
            self.left_y = -event.state / MAX_AXIS  # Invert
            
        # Right stick X (Rotation)
        elif event.code == 'ABS_RX':
            self.right_x = event.state / MAX_AXIS
            
        # Right stick Y
        elif event.code == 'ABS_RY':
            pass  # Not used
            
        # Triggers for speed control
        elif event.code == 'ABS_Z':  # Left trigger
            # Reduce speed
            self.max_speed = max(50, 200 - int(event.state / 255 * 150))
            
        elif event.code == 'ABS_RZ':  # Right trigger
            # Boost speed
            self.max_speed = min(255, 200 + int(event.state / 255 * 55))
            
        # D-Pad
        elif event.code == 'ABS_HAT0Y':
            if event.state == -1:
                self.left_y = 1.0  # D-pad Up
            elif event.state == 1:
                self.left_y = -1.0  # D-pad Down
            else:
                self.left_y = 0
                
        elif event.code == 'ABS_HAT0X':
            if event.state == -1:
                self.left_x = -1.0  # D-pad Left
            elif event.state == 1:
                self.left_x = 1.0  # D-pad Right
            else:
                self.left_x = 0
                
        # Buttons
        elif event.code == 'BTN_SOUTH' and event.state == 1:  # A button
            self.send_command("S")  # Stop
            
        elif event.code == 'BTN_EAST' and event.state == 1:  # B button
            self.send_command("OFF")  # Emergency off
            self.emergency_stop = True
            
        elif event.code == 'BTN_WEST' and event.state == 1:  # X button
            self.emergency_stop = False
            
        elif event.code == 'BTN_START' and event.state == 1:  # Start
            self.running = False
            
    def update_motors(self):
        """Cập nhật tốc độ motor"""
        if self.emergency_stop:
            return
            
        # Calculate wheel speeds
        fl, fr, bl, br = MecanumMixer.mix(
            self.left_x,
            self.left_y,
            self.right_x,
            self.max_speed
        )
        
        # Check if all zero
        if fl == 0 and fr == 0 and bl == 0 and br == 0:
            self.send_command("S")
        else:
            # Send individual wheel commands
            # Format: "M FL FR BL BR"
            self.send_command(f"M {fl} {fr} {bl} {br}")
            
    def gamepad_loop(self):
        """Thread đọc gamepad"""
        while self.running:
            try:
                events = get_gamepad()
                for event in events:
                    if event.ev_type in ['Absolute', 'Key']:
                        self.process_event(event)
            except Exception as e:
                print(f"Gamepad error: {e}")
                time.sleep(1)
                
    def run(self):
        """Chạy controller"""
        # Check gamepad
        if not HAS_INPUTS:
            return
            
        gamepads = devices.gamepads
        if not gamepads:
            print("❌ Không tìm thấy gamepad!")
            print("   Kết nối gamepad và thử lại.")
            return
            
        print(f"🎮 Gamepad detected: {gamepads[0]}")
        
        # Connect serial
        if not self.connect_serial():
            return
            
        # Start gamepad thread
        gamepad_thread = threading.Thread(target=self.gamepad_loop, daemon=True)
        gamepad_thread.start()
        
        print("""
╔══════════════════════════════════════════════════════════════════╗
║           🎮 K-PATROL GAMEPAD CONTROLLER 🎮                      ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║   Left Stick  : Move (Forward/Backward/Strafe)                  ║
║   Right Stick : Rotate                                          ║
║   D-Pad       : Direction control                               ║
║                                                                  ║
║   Left Trigger  : Slow down                                     ║
║   Right Trigger : Speed boost                                   ║
║                                                                  ║
║   A Button : Stop (Brake)                                       ║
║   B Button : Emergency OFF                                      ║
║   X Button : Resume from Emergency                              ║
║   Start    : Exit                                               ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

Press START button or Ctrl+C to exit.
""")
        
        # Main loop
        try:
            last_update = 0
            while self.running:
                current = time.time()
                
                # Update motors at 20Hz
                if current - last_update >= 0.05:
                    self.update_motors()
                    last_update = current
                    
                    # Status display
                    status = "⛔ EMERGENCY" if self.emergency_stop else "🟢 ACTIVE"
                    sys.stdout.write(f"\r   Status: {status} | Speed: {self.max_speed} | "
                                   f"X:{self.left_x:+.2f} Y:{self.left_y:+.2f} R:{self.right_x:+.2f}   ")
                    sys.stdout.flush()
                    
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.serial:
                self.send_command("OFF")
                time.sleep(0.1)
                self.serial.close()
            print("\n\n👋 Goodbye!")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    print("""
╔══════════════════════════════════════════════════════════════════╗
║              K-PATROL GAMEPAD CONTROLLER                         ║
╚══════════════════════════════════════════════════════════════════╝
""")
    
    if not HAS_INPUTS:
        print("❌ Thiếu thư viện 'inputs'!")
        print("   Cài đặt: pip3 install inputs")
        return
        
    controller = GamepadController(port)
    controller.run()


if __name__ == "__main__":
    main()
