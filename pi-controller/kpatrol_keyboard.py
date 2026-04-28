#!/usr/bin/env python3
"""
K-Patrol Bot - Keyboard Control Mode
Điều khiển robot bằng bàn phím theo thời gian thực

Sử dụng: python3 kpatrol_keyboard.py [port]
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
import threading
from typing import Optional

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# ═══════════════════════════════════════════════════════════════════════════════
# TERMINAL UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

class Terminal:
    """Quản lý terminal"""
    
    CLEAR = "\033[2J\033[H"
    BOLD = "\033[1m"
    RESET = "\033[0m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    MAGENTA = "\033[95m"
    
    def __init__(self):
        self.old_settings = None
        
    def setup_raw_mode(self):
        """Chuyển sang raw mode để đọc từng phím"""
        import termios
        import tty
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
    def restore(self):
        """Khôi phục terminal"""
        if self.old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def getch(self) -> str:
        """Đọc một phím"""
        import select
        
        if select.select([sys.stdin], [], [], 0.05)[0]:
            ch = sys.stdin.read(1)
            
            # Handle escape sequences
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'UP'
                        if ch3 == 'B': return 'DOWN'
                        if ch3 == 'C': return 'RIGHT'
                        if ch3 == 'D': return 'LEFT'
                return 'ESC'
            return ch
        return ''
    
    @staticmethod
    def clear():
        """Xóa màn hình"""
        print("\033[2J\033[H", end="", flush=True)


# ═══════════════════════════════════════════════════════════════════════════════
# ROBOT CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class RobotController:
    """Điều khiển robot"""
    
    def __init__(self, port: str):
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.speed = 150
        self.current_action = "STOPPED"
        self.responses = []
        self.running = True
        
        # Key mappings
        self.key_actions = {
            'w': ('F', 'FORWARD ↑'),
            'W': ('F', 'FORWARD ↑'),
            'UP': ('F', 'FORWARD ↑'),
            
            's': ('B', 'BACKWARD ↓'),
            'S': ('B', 'BACKWARD ↓'),
            'DOWN': ('B', 'BACKWARD ↓'),
            
            'a': ('SL', 'STRAFE LEFT ←'),
            'A': ('SL', 'STRAFE LEFT ←'),
            'LEFT': ('SL', 'STRAFE LEFT ←'),
            
            'd': ('SR', 'STRAFE RIGHT →'),
            'D': ('SR', 'STRAFE RIGHT →'),
            'RIGHT': ('SR', 'STRAFE RIGHT →'),
            
            'q': ('L', 'ROTATE LEFT ↺'),
            'Q': ('L', 'ROTATE LEFT ↺'),
            
            'e': ('R', 'ROTATE RIGHT ↻'),
            'E': ('R', 'ROTATE RIGHT ↻'),
            
            '7': ('DFL', 'DIAGONAL ↖'),
            '9': ('DFR', 'DIAGONAL ↗'),
            '1': ('DBL', 'DIAGONAL ↙'),
            '3': ('DBR', 'DIAGONAL ↘'),
            
            ' ': ('S', 'STOPPED ⏹'),
            'x': ('OFF', 'EMERGENCY OFF ⛔'),
            'X': ('OFF', 'EMERGENCY OFF ⛔'),
        }
        
    def connect(self) -> bool:
        """Kết nối với ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=0.5
            )
            time.sleep(2)
            self.serial.reset_input_buffer()
            self.connected = True
            
            # Start reader thread
            threading.Thread(target=self._read_responses, daemon=True).start()
            
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối: {e}")
            return False
            
    def disconnect(self):
        """Ngắt kết nối"""
        self.running = False
        if self.serial:
            self.send("OFF")
            time.sleep(0.1)
            self.serial.close()
        self.connected = False
        
    def send(self, cmd: str):
        """Gửi lệnh"""
        if self.serial and self.connected:
            try:
                self.serial.write(f"{cmd}\n".encode())
                self.serial.flush()
            except:
                pass
                
    def _read_responses(self):
        """Thread đọc response"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.responses.append(line)
                        if len(self.responses) > 5:
                            self.responses.pop(0)
            except:
                pass
            time.sleep(0.01)
            
    def handle_key(self, key: str) -> bool:
        """Xử lý phím bấm"""
        if key == 'ESC':
            return False
            
        # Tốc độ
        if key == '+' or key == '=':
            self.speed = min(255, self.speed + 25)
            return True
        if key == '-' or key == '_':
            self.speed = max(50, self.speed - 25)
            return True
            
        # Di chuyển
        if key in self.key_actions:
            cmd, action = self.key_actions[key]
            self.current_action = action
            
            if cmd in ['S', 'OFF']:
                self.send(cmd)
            else:
                self.send(f"{cmd} {self.speed}")
                
        return True


# ═══════════════════════════════════════════════════════════════════════════════
# DISPLAY
# ═══════════════════════════════════════════════════════════════════════════════

def draw_interface(robot: RobotController, term: Terminal):
    """Vẽ giao diện"""
    status_color = term.GREEN if robot.connected else term.RED
    status_text = "CONNECTED" if robot.connected else "DISCONNECTED"
    
    action_color = term.YELLOW if "STOPPED" not in robot.current_action else term.GREEN
    
    print(f"""{term.CYAN}{term.BOLD}
╔══════════════════════════════════════════════════════════════════╗
║           🤖 K-PATROL - KEYBOARD CONTROL MODE 🤖                 ║
╚══════════════════════════════════════════════════════════════════╝{term.RESET}

{term.BOLD}📡 STATUS:{term.RESET}
   Connection: {status_color}{status_text}{term.RESET} ({robot.port})
   Speed:      {term.BLUE}{robot.speed}/255{term.RESET}
   Action:     {action_color}{robot.current_action}{term.RESET}

{term.BOLD}🎮 CONTROLS:{term.RESET}
   ┌─────────────────────────────────────────────────────────────┐
   │                                                             │
   │     [7]↖   [W/↑]   [9]↗        Movement:                   │
   │            FORWARD              W/↑ = Forward              │
   │                                 S/↓ = Backward             │
   │    [A/←]    [·]    [D/→]        A/← = Strafe Left          │
   │   STRAFE           STRAFE       D/→ = Strafe Right         │
   │    LEFT            RIGHT                                    │
   │                                 Rotation:                   │
   │     [1]↙   [S/↓]   [3]↘        Q = Rotate Left             │
   │           BACKWARD              E = Rotate Right            │
   │                                                             │
   │   [Q] ↺ Rotate Left    [E] ↻ Rotate Right                  │
   │                                                             │
   ├─────────────────────────────────────────────────────────────┤
   │   [SPACE] = Stop (Brake)       [+/-] = Speed Up/Down       │
   │   [X] = Emergency OFF          [ESC] = Exit                 │
   └─────────────────────────────────────────────────────────────┘
""")
    
    # Responses
    if robot.responses:
        print(f"{term.BOLD}📨 ESP32 RESPONSES:{term.RESET}")
        for resp in robot.responses[-3:]:
            print(f"   {term.CYAN}└─ {resp}{term.RESET}")
    
    print(f"\n{term.YELLOW}   Press keys to control robot. ESC to exit.{term.RESET}")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    term = Terminal()
    robot = RobotController(port)
    
    # Banner
    print(f"""
{Terminal.CYAN}{Terminal.BOLD}
╔══════════════════════════════════════════════════════════════════╗
║                 K-PATROL KEYBOARD CONTROLLER                      ║
╚══════════════════════════════════════════════════════════════════╝
{Terminal.RESET}""")
    
    # Kết nối
    print(f"🔌 Connecting to {port}...")
    if not robot.connect():
        print(f"\n{Terminal.RED}Không thể kết nối!{Terminal.RESET}")
        print(f"Thử: python3 {sys.argv[0]} /dev/ttyUSB0")
        
        # Liệt kê cổng
        ports = serial.tools.list_ports.comports()
        if ports:
            print(f"\n{Terminal.BOLD}Cổng có sẵn:{Terminal.RESET}")
            for p in ports:
                print(f"   {p.device}: {p.description}")
        return
        
    print(f"{Terminal.GREEN}✅ Connected!{Terminal.RESET}")
    time.sleep(1)
    
    # Setup terminal
    term.setup_raw_mode()
    
    try:
        while True:
            term.clear()
            draw_interface(robot, term)
            
            key = term.getch()
            if key:
                if not robot.handle_key(key):
                    break
                    
    except KeyboardInterrupt:
        pass
    finally:
        term.restore()
        robot.disconnect()
        print(f"\n{Terminal.GREEN}👋 Goodbye!{Terminal.RESET}\n")


if __name__ == "__main__":
    main()
