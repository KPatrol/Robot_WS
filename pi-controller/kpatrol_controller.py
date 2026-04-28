#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════════════════╗
║                    K-PATROL BOT - RASPBERRY PI CONTROLLER                     ║
║                                                                              ║
║  Chương trình điều khiển robot K-Patrol từ Raspberry Pi                      ║
║  Giao tiếp: USB Serial → ESP32-S3 WROOM-1                                    ║
║                                                                              ║
║  Hardware:                                                                   ║
║    - Raspberry Pi 4 (Controller)                                             ║
║    - ESP32-S3 WROOM-1 (Motor Driver Controller)                              ║
║    - ESP32 Dev Module (Encoder Reader) - Optional                            ║
║    - 4x BTS7960 Motor Drivers                                                ║
║    - 4x JGA25-370 Motors with Encoders                                       ║
║    - 4x Mecanum Wheels                                                       ║
║                                                                              ║
║  Author: K-Patrol Team                                                       ║
║  Date: January 2026                                                          ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
import threading
import signal
from typing import Optional, Callable
from dataclasses import dataclass
from enum import Enum

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Config:
    """Cấu hình hệ thống"""
    # Serial ports
    MOTOR_SERIAL_PORT: str = "/dev/ttyACM0"      # ESP32-S3 Motor Controller
    ENCODER_SERIAL_PORT: str = "/dev/ttyUSB0"    # ESP32 Encoder Reader (optional)
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 1.0
    
    # Speed settings
    DEFAULT_SPEED: int = 150
    MIN_SPEED: int = 50
    MAX_SPEED: int = 255
    SPEED_STEP: int = 25
    
    # Timing
    COMMAND_DELAY: float = 0.05
    HEARTBEAT_INTERVAL: float = 5.0
    
    # Movement duration for timed commands (seconds)
    MOVE_DURATION: float = 0.5


class MovementCommand(Enum):
    """Các lệnh di chuyển Mecanum"""
    FORWARD = "F"
    BACKWARD = "B"
    STRAFE_LEFT = "SL"
    STRAFE_RIGHT = "SR"
    ROTATE_LEFT = "L"
    ROTATE_RIGHT = "R"
    DIAGONAL_FL = "DFL"
    DIAGONAL_FR = "DFR"
    DIAGONAL_BL = "DBL"
    DIAGONAL_BR = "DBR"
    STOP = "S"
    OFF = "OFF"


# ═══════════════════════════════════════════════════════════════════════════════
# SERIAL MANAGER
# ═══════════════════════════════════════════════════════════════════════════════

class SerialManager:
    """Quản lý kết nối Serial với ESP32"""
    
    def __init__(self, port: str, baud_rate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self.is_connected = False
        self.lock = threading.Lock()
        self.response_callback: Optional[Callable[[str], None]] = None
        self.reader_thread: Optional[threading.Thread] = None
        self.running = False
        
    def connect(self) -> bool:
        """Kết nối với ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            time.sleep(2)  # Đợi ESP32 khởi động
            self.is_connected = True
            self.running = True
            
            # Xóa buffer
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()
            
            # Bắt đầu thread đọc response
            self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.reader_thread.start()
            
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối {self.port}: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Ngắt kết nối"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.is_connected = False
        
    def send(self, command: str) -> bool:
        """Gửi lệnh đến ESP32"""
        if not self.is_connected or not self.serial:
            return False
        
        try:
            with self.lock:
                self.serial.write(f"{command}\n".encode('utf-8'))
                self.serial.flush()
            return True
        except Exception as e:
            print(f"❌ Lỗi gửi lệnh: {e}")
            return False
    
    def _read_loop(self):
        """Thread đọc response từ ESP32"""
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line and self.response_callback:
                        self.response_callback(line)
            except Exception:
                pass
            time.sleep(0.01)
    
    def set_response_callback(self, callback: Callable[[str], None]):
        """Đặt callback xử lý response"""
        self.response_callback = callback
    
    @staticmethod
    def list_ports() -> list:
        """Liệt kê các cổng Serial có sẵn"""
        ports = serial.tools.list_ports.comports()
        return [(p.device, p.description) for p in ports]


# ═══════════════════════════════════════════════════════════════════════════════
# MOTOR CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class MotorController:
    """Điều khiển 4 motor Mecanum"""
    
    def __init__(self, serial_manager: SerialManager, config: Config):
        self.serial = serial_manager
        self.config = config
        self.current_speed = config.DEFAULT_SPEED
        self.is_moving = False
        self.last_command = None
        
    def set_speed(self, speed: int):
        """Đặt tốc độ motor"""
        self.current_speed = max(self.config.MIN_SPEED, 
                                  min(self.config.MAX_SPEED, speed))
        
    def increase_speed(self):
        """Tăng tốc độ"""
        self.set_speed(self.current_speed + self.config.SPEED_STEP)
        return self.current_speed
        
    def decrease_speed(self):
        """Giảm tốc độ"""
        self.set_speed(self.current_speed - self.config.SPEED_STEP)
        return self.current_speed
    
    def move(self, direction: MovementCommand) -> bool:
        """Di chuyển robot theo hướng"""
        if direction == MovementCommand.STOP:
            cmd = "S"
        elif direction == MovementCommand.OFF:
            cmd = "OFF"
        else:
            cmd = f"{direction.value} {self.current_speed}"
        
        self.last_command = direction
        self.is_moving = direction not in [MovementCommand.STOP, MovementCommand.OFF]
        return self.serial.send(cmd)
    
    def stop(self) -> bool:
        """Dừng robot (brake mode)"""
        return self.move(MovementCommand.STOP)
    
    def emergency_stop(self) -> bool:
        """Dừng khẩn cấp (disable drivers)"""
        return self.move(MovementCommand.OFF)
    
    def test_motor(self, motor_id: int, speed: int = None) -> bool:
        """Test từng motor độc lập"""
        if motor_id < 1 or motor_id > 4:
            return False
        speed = speed or self.current_speed
        return self.serial.send(f"T{motor_id} {speed}")
    
    def test_all_motors(self, speed: int = None) -> bool:
        """Test tất cả motor"""
        speed = speed or self.current_speed
        return self.serial.send(f"TA {speed}")
    
    # Các phương thức di chuyển cụ thể
    def forward(self) -> bool:
        return self.move(MovementCommand.FORWARD)
    
    def backward(self) -> bool:
        return self.move(MovementCommand.BACKWARD)
    
    def strafe_left(self) -> bool:
        return self.move(MovementCommand.STRAFE_LEFT)
    
    def strafe_right(self) -> bool:
        return self.move(MovementCommand.STRAFE_RIGHT)
    
    def rotate_left(self) -> bool:
        return self.move(MovementCommand.ROTATE_LEFT)
    
    def rotate_right(self) -> bool:
        return self.move(MovementCommand.ROTATE_RIGHT)
    
    def diagonal_forward_left(self) -> bool:
        return self.move(MovementCommand.DIAGONAL_FL)
    
    def diagonal_forward_right(self) -> bool:
        return self.move(MovementCommand.DIAGONAL_FR)
    
    def diagonal_backward_left(self) -> bool:
        return self.move(MovementCommand.DIAGONAL_BL)
    
    def diagonal_backward_right(self) -> bool:
        return self.move(MovementCommand.DIAGONAL_BR)


# ═══════════════════════════════════════════════════════════════════════════════
# ENCODER READER (Optional)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class EncoderData:
    """Dữ liệu encoder của 4 motor"""
    fr_count: int = 0
    fl_count: int = 0
    br_count: int = 0
    bl_count: int = 0
    fr_rpm: float = 0.0
    fl_rpm: float = 0.0
    br_rpm: float = 0.0
    bl_rpm: float = 0.0
    timestamp: float = 0.0


class EncoderReader:
    """Đọc dữ liệu encoder từ ESP32 Dev Module"""
    
    def __init__(self, serial_manager: SerialManager):
        self.serial = serial_manager
        self.data = EncoderData()
        self.data_callback: Optional[Callable[[EncoderData], None]] = None
        
    def parse_response(self, line: str):
        """Parse dữ liệu encoder từ ESP32"""
        try:
            # Format: "ENC:FR=1234,FL=1234,BR=1234,BL=1234"
            if line.startswith("ENC:"):
                parts = line[4:].split(",")
                for part in parts:
                    key, value = part.split("=")
                    if key == "FR":
                        self.data.fr_count = int(value)
                    elif key == "FL":
                        self.data.fl_count = int(value)
                    elif key == "BR":
                        self.data.br_count = int(value)
                    elif key == "BL":
                        self.data.bl_count = int(value)
                        
            # Format: "RPM:FR=12.5,FL=12.5,BR=12.5,BL=12.5"
            elif line.startswith("RPM:"):
                parts = line[4:].split(",")
                for part in parts:
                    key, value = part.split("=")
                    if key == "FR":
                        self.data.fr_rpm = float(value)
                    elif key == "FL":
                        self.data.fl_rpm = float(value)
                    elif key == "BR":
                        self.data.br_rpm = float(value)
                    elif key == "BL":
                        self.data.bl_rpm = float(value)
                        
            self.data.timestamp = time.time()
            
            if self.data_callback:
                self.data_callback(self.data)
                
        except Exception:
            pass
    
    def reset_counts(self):
        """Reset encoder counts"""
        self.serial.send("RESET")


# ═══════════════════════════════════════════════════════════════════════════════
# TERMINAL UI
# ═══════════════════════════════════════════════════════════════════════════════

class TerminalUI:
    """Giao diện điều khiển terminal"""
    
    CLEAR_SCREEN = "\033[2J\033[H"
    BOLD = "\033[1m"
    RESET = "\033[0m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    
    def __init__(self, controller: MotorController, config: Config):
        self.controller = controller
        self.config = config
        self.running = True
        self.last_responses = []
        self.max_responses = 5
        
    def clear(self):
        """Xóa màn hình"""
        print(self.CLEAR_SCREEN, end="")
        
    def print_header(self):
        """In header"""
        print(f"{self.CYAN}{self.BOLD}")
        print("╔══════════════════════════════════════════════════════════════════╗")
        print("║           🤖 K-PATROL BOT - MECANUM CONTROLLER 🤖                ║")
        print("║                    Raspberry Pi Terminal                          ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print(f"{self.RESET}")
        
    def print_status(self):
        """In trạng thái hiện tại"""
        status_color = self.GREEN if self.controller.serial.is_connected else self.RED
        status_text = "CONNECTED" if self.controller.serial.is_connected else "DISCONNECTED"
        
        moving_color = self.YELLOW if self.controller.is_moving else self.GREEN
        moving_text = f"MOVING ({self.controller.last_command.name})" if self.controller.is_moving else "STOPPED"
        
        print(f"\n{self.BOLD}📊 TRẠNG THÁI:{self.RESET}")
        print(f"├─ Serial: {status_color}{status_text}{self.RESET} ({self.config.MOTOR_SERIAL_PORT})")
        print(f"├─ Robot:  {moving_color}{moving_text}{self.RESET}")
        print(f"└─ Speed:  {self.BLUE}{self.controller.current_speed}/255{self.RESET}")
        
    def print_controls(self):
        """In hướng dẫn điều khiển"""
        print(f"\n{self.BOLD}🎮 ĐIỀU KHIỂN:{self.RESET}")
        print("┌────────────────────────────────────────────────────────────────┐")
        print("│  Di chuyển:              │  Xoay:           │  Chéo:          │")
        print("│    [W] Tiến              │  [Q] Xoay trái   │  [7] ↖  [9] ↗   │")
        print("│    [S] Lùi               │  [E] Xoay phải   │  [1] ↙  [3] ↘   │")
        print("│    [A] Sang trái         │                  │                 │")
        print("│    [D] Sang phải         │                  │                 │")
        print("├────────────────────────────────────────────────────────────────┤")
        print("│  Tốc độ:                 │  Test Motor:     │  Hệ thống:      │")
        print("│    [+] Tăng tốc          │  [F1-F4] Motor   │  [SPACE] Dừng   │")
        print("│    [-] Giảm tốc          │  [F5] Test All   │  [X] Tắt driver │")
        print("│    [0-9] Đặt tốc độ      │                  │  [ESC] Thoát    │")
        print("└────────────────────────────────────────────────────────────────┘")
        
    def print_responses(self):
        """In phản hồi từ ESP32"""
        if self.last_responses:
            print(f"\n{self.BOLD}📨 PHẢN HỒI TỪ ESP32:{self.RESET}")
            for resp in self.last_responses[-self.max_responses:]:
                print(f"  └─ {resp}")
                
    def add_response(self, response: str):
        """Thêm response mới"""
        self.last_responses.append(response)
        if len(self.last_responses) > self.max_responses:
            self.last_responses.pop(0)
            
    def print_help(self):
        """In hướng dẫn chi tiết"""
        self.clear()
        print(f"{self.BOLD}{self.CYAN}")
        print("╔══════════════════════════════════════════════════════════════════╗")
        print("║                    📖 HƯỚNG DẪN SỬ DỤNG                          ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print(f"{self.RESET}")
        
        print("""
🚀 ĐIỀU KHIỂN CƠ BẢN:
────────────────────────────────────────────────────────────────────
  W/↑        : Di chuyển tiến (Forward)
  S/↓        : Di chuyển lùi (Backward)  
  A/←        : Di chuyển ngang trái (Strafe Left)
  D/→        : Di chuyển ngang phải (Strafe Right)
  Q          : Xoay tại chỗ sang trái (Rotate Left)
  E          : Xoay tại chỗ sang phải (Rotate Right)

🔀 DI CHUYỂN CHÉO (Diagonal):
────────────────────────────────────────────────────────────────────
  7/Home     : Chéo tiến-trái (Diagonal Forward-Left)
  9/PgUp     : Chéo tiến-phải (Diagonal Forward-Right)
  1/End      : Chéo lùi-trái (Diagonal Backward-Left)
  3/PgDn     : Chéo lùi-phải (Diagonal Backward-Right)

⚡ TỐC ĐỘ:
────────────────────────────────────────────────────────────────────
  +/=        : Tăng tốc độ (+25)
  -/_        : Giảm tốc độ (-25)
  0-9        : Đặt tốc độ nhanh (0=50, 9=255)

🔧 TEST MOTOR:
────────────────────────────────────────────────────────────────────
  F1         : Test Motor 1 (Front-Left)
  F2         : Test Motor 2 (Front-Right)
  F3         : Test Motor 3 (Back-Right)
  F4         : Test Motor 4 (Back-Left)
  F5         : Test tất cả motor cùng lúc

🛑 DỪNG & AN TOÀN:
────────────────────────────────────────────────────────────────────
  SPACE      : Dừng robot (Brake mode - giữ vị trí)
  X          : Tắt hoàn toàn driver (Emergency OFF)
  ESC/Ctrl+C : Thoát chương trình

📡 KẾT NỐI:
────────────────────────────────────────────────────────────────────
  R          : Kết nối lại Serial
  P          : Liệt kê các cổng Serial
  I          : Hiển thị thông tin hệ thống
""")
        input("\n  Nhấn ENTER để quay lại...")
        
    def print_system_info(self):
        """In thông tin hệ thống"""
        self.clear()
        print(f"{self.BOLD}{self.CYAN}")
        print("╔══════════════════════════════════════════════════════════════════╗")
        print("║                    ℹ️  THÔNG TIN HỆ THỐNG                         ║")
        print("╚══════════════════════════════════════════════════════════════════╝")
        print(f"{self.RESET}")
        
        print(f"""
📟 CẤU HÌNH PHẦN CỨNG:
────────────────────────────────────────────────────────────────────
  Controller    : Raspberry Pi 4
  Motor Driver  : ESP32-S3 WROOM-1
  Encoder Reader: ESP32 Dev Module (Optional)
  Motor Drivers : 4x BTS7960 (IBT-2) - 43A
  Motors        : 4x JGA25-370 12V DC + Encoder
  Wheels        : 4x Mecanum (2L + 2R)

📡 KẾT NỐI SERIAL:
────────────────────────────────────────────────────────────────────
  Motor Port    : {self.config.MOTOR_SERIAL_PORT}
  Encoder Port  : {self.config.ENCODER_SERIAL_PORT}
  Baud Rate     : {self.config.BAUD_RATE}
  Status        : {'🟢 Connected' if self.controller.serial.is_connected else '🔴 Disconnected'}

⚙️ CẤU HÌNH MOTOR:
────────────────────────────────────────────────────────────────────
  Current Speed : {self.controller.current_speed}/255
  Min Speed     : {self.config.MIN_SPEED}
  Max Speed     : {self.config.MAX_SPEED}
  Speed Step    : {self.config.SPEED_STEP}

📍 GPIO ALLOCATION (ESP32-S3):
────────────────────────────────────────────────────────────────────
  Motor FR: GPIO 4,5,6,7     (PWM Ch 0,1)
  Motor FL: GPIO 15,16,17,18 (PWM Ch 2,3)
  Motor BR: GPIO 8,9,10,11   (PWM Ch 4,5)
  Motor BL: GPIO 12,13,14,21 (PWM Ch 6,7)
""")
        
        # Liệt kê các cổng Serial có sẵn
        print("🔌 CỔNG SERIAL CÓ SẴN:")
        print("────────────────────────────────────────────────────────────────────")
        ports = SerialManager.list_ports()
        if ports:
            for device, desc in ports:
                status = "✓ (đang dùng)" if device == self.config.MOTOR_SERIAL_PORT else ""
                print(f"  {device}: {desc} {status}")
        else:
            print("  Không tìm thấy cổng Serial nào")
            
        input("\n  Nhấn ENTER để quay lại...")


# ═══════════════════════════════════════════════════════════════════════════════
# KEYBOARD INPUT HANDLER
# ═══════════════════════════════════════════════════════════════════════════════

class KeyboardHandler:
    """Xử lý input từ bàn phím"""
    
    def __init__(self):
        self.old_settings = None
        
    def setup(self):
        """Cấu hình terminal cho raw input"""
        import termios
        import tty
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        
    def cleanup(self):
        """Khôi phục cấu hình terminal"""
        if self.old_settings:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
    def get_key(self) -> str:
        """Đọc một phím từ bàn phím"""
        import select
        
        if select.select([sys.stdin], [], [], 0.1)[0]:
            ch = sys.stdin.read(1)
            
            # Xử lý escape sequences (arrow keys, function keys)
            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'UP'
                        if ch3 == 'B': return 'DOWN'
                        if ch3 == 'C': return 'RIGHT'
                        if ch3 == 'D': return 'LEFT'
                        if ch3 == 'H': return 'HOME'
                        if ch3 == 'F': return 'END'
                        # Function keys
                        if ch3 == '1':
                            ch4 = sys.stdin.read(1)
                            if ch4 == '5': sys.stdin.read(1); return 'F5'
                            if ch4 == '7': sys.stdin.read(1); return 'F6'
                            if ch4 == '~': return 'HOME'
                        if ch3 == '4': sys.stdin.read(1); return 'END'
                        if ch3 == '5': sys.stdin.read(1); return 'PGUP'
                        if ch3 == '6': sys.stdin.read(1); return 'PGDN'
                    elif ch2 == 'O':
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'P': return 'F1'
                        if ch3 == 'Q': return 'F2'
                        if ch3 == 'R': return 'F3'
                        if ch3 == 'S': return 'F4'
                        if ch3 == 'H': return 'HOME'
                        if ch3 == 'F': return 'END'
                return 'ESC'
            
            return ch.upper()
        return ''


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN APPLICATION
# ═══════════════════════════════════════════════════════════════════════════════

class KPatrolApp:
    """Ứng dụng chính điều khiển K-Patrol Bot"""
    
    def __init__(self):
        self.config = Config()
        self.serial_manager = SerialManager(
            self.config.MOTOR_SERIAL_PORT,
            self.config.BAUD_RATE,
            self.config.SERIAL_TIMEOUT
        )
        self.controller = MotorController(self.serial_manager, self.config)
        self.ui = TerminalUI(self.controller, self.config)
        self.keyboard = KeyboardHandler()
        self.running = True
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """Xử lý tín hiệu ngắt"""
        self.stop()
        
    def _on_serial_response(self, response: str):
        """Callback khi nhận response từ ESP32"""
        self.ui.add_response(response)
        
    def connect(self) -> bool:
        """Kết nối với ESP32"""
        print(f"🔌 Đang kết nối với ESP32-S3 ({self.config.MOTOR_SERIAL_PORT})...")
        
        if self.serial_manager.connect():
            self.serial_manager.set_response_callback(self._on_serial_response)
            print(f"✅ Đã kết nối thành công!")
            return True
        else:
            print(f"❌ Không thể kết nối!")
            print("💡 Gợi ý:")
            print(f"   - Kiểm tra ESP32-S3 đã cắm USB chưa")
            print(f"   - Thử: ls /dev/tty* | grep -E '(ACM|USB)'")
            print(f"   - Đổi cổng trong config nếu cần")
            return False
    
    def reconnect(self):
        """Kết nối lại"""
        print("🔄 Đang kết nối lại...")
        self.serial_manager.disconnect()
        time.sleep(1)
        return self.connect()
        
    def handle_key(self, key: str):
        """Xử lý phím bấm"""
        if not key:
            return
            
        # Movement keys
        if key in ['W', 'UP']:
            self.controller.forward()
        elif key in ['S', 'DOWN']:
            self.controller.backward()
        elif key in ['A', 'LEFT']:
            self.controller.strafe_left()
        elif key in ['D', 'RIGHT']:
            self.controller.strafe_right()
        elif key == 'Q':
            self.controller.rotate_left()
        elif key == 'E':
            self.controller.rotate_right()
            
        # Diagonal movement
        elif key in ['7', 'HOME']:
            self.controller.diagonal_forward_left()
        elif key in ['9', 'PGUP']:
            self.controller.diagonal_forward_right()
        elif key in ['1', 'END']:
            self.controller.diagonal_backward_left()
        elif key in ['3', 'PGDN']:
            self.controller.diagonal_backward_right()
            
        # Stop commands
        elif key == ' ':
            self.controller.stop()
        elif key == 'X':
            self.controller.emergency_stop()
            
        # Speed control
        elif key in ['+', '=']:
            speed = self.controller.increase_speed()
            print(f"⚡ Tốc độ: {speed}")
        elif key in ['-', '_']:
            speed = self.controller.decrease_speed()
            print(f"⚡ Tốc độ: {speed}")
        elif key.isdigit():
            # 0-9 = quick speed presets
            speed = 50 + int(key) * 22  # 50-248
            self.controller.set_speed(speed)
            print(f"⚡ Tốc độ: {speed}")
            
        # Test motors
        elif key == 'F1':
            self.controller.test_motor(1)
        elif key == 'F2':
            self.controller.test_motor(2)
        elif key == 'F3':
            self.controller.test_motor(3)
        elif key == 'F4':
            self.controller.test_motor(4)
        elif key == 'F5':
            self.controller.test_all_motors()
            
        # System commands
        elif key == 'R':
            self.reconnect()
        elif key == 'P':
            ports = SerialManager.list_ports()
            print("\n🔌 Cổng Serial có sẵn:")
            for device, desc in ports:
                print(f"   {device}: {desc}")
        elif key == 'I':
            self.ui.print_system_info()
        elif key == 'H':
            self.ui.print_help()
        elif key == 'ESC':
            self.running = False
            
    def run(self):
        """Chạy ứng dụng chính"""
        self.ui.clear()
        self.ui.print_header()
        
        if not self.connect():
            print("\n⚠️  Tiếp tục mà không có kết nối? (y/n): ", end="")
            if input().lower() != 'y':
                return
        
        self.keyboard.setup()
        
        try:
            while self.running:
                self.ui.clear()
                self.ui.print_header()
                self.ui.print_status()
                self.ui.print_controls()
                self.ui.print_responses()
                
                print(f"\n{self.ui.YELLOW}[H] Trợ giúp | [I] Thông tin | [ESC] Thoát{self.ui.RESET}")
                
                key = self.keyboard.get_key()
                self.handle_key(key)
                
        except Exception as e:
            print(f"\n❌ Lỗi: {e}")
        finally:
            self.stop()
            
    def stop(self):
        """Dừng ứng dụng"""
        self.running = False
        self.keyboard.cleanup()
        
        # Dừng robot an toàn
        if self.serial_manager.is_connected:
            self.controller.emergency_stop()
            time.sleep(0.1)
            self.serial_manager.disconnect()
            
        print("\n\n👋 Đã thoát K-Patrol Controller. Tạm biệt!")


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    """Entry point"""
    print("""
╔══════════════════════════════════════════════════════════════════════════════╗
║                                                                              ║
║    ██╗  ██╗      ██████╗  █████╗ ████████╗██████╗  ██████╗ ██╗              ║
║    ██║ ██╔╝      ██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔═══██╗██║              ║
║    █████╔╝ █████╗██████╔╝███████║   ██║   ██████╔╝██║   ██║██║              ║
║    ██╔═██╗ ╚════╝██╔═══╝ ██╔══██║   ██║   ██╔══██╗██║   ██║██║              ║
║    ██║  ██╗      ██║     ██║  ██║   ██║   ██║  ██║╚██████╔╝███████╗         ║
║    ╚═╝  ╚═╝      ╚═╝     ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝         ║
║                                                                              ║
║                    🤖 MECANUM ROBOT CONTROLLER 🤖                            ║
║                                                                              ║
╚══════════════════════════════════════════════════════════════════════════════╝
    """)
    
    app = KPatrolApp()
    app.run()


if __name__ == "__main__":
    main()
