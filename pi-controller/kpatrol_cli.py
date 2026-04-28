#!/usr/bin/env python3
"""
K-Patrol Bot - Interactive Command Line Interface
Giao diện dòng lệnh tương tác để điều khiển robot

Sử dụng: python3 kpatrol_cli.py
"""

import serial
import serial.tools.list_ports
import time
import sys
import readline
import os
from typing import Optional

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 1.0

# ═══════════════════════════════════════════════════════════════════════════════
# COLORS
# ═══════════════════════════════════════════════════════════════════════════════

class Colors:
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"


# ═══════════════════════════════════════════════════════════════════════════════
# SERIAL CONNECTION
# ═══════════════════════════════════════════════════════════════════════════════

class SerialConnection:
    """Kết nối Serial với ESP32-S3"""
    
    def __init__(self, port: str = DEFAULT_PORT):
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        
    def connect(self) -> bool:
        """Kết nối với ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=TIMEOUT
            )
            time.sleep(2)  # Đợi ESP32 khởi động
            self.connected = True
            
            # Xóa buffer và đọc thông tin khởi động
            self.serial.reset_input_buffer()
            
            print(f"{Colors.GREEN}✅ Đã kết nối với {self.port}{Colors.RESET}")
            self._read_startup()
            return True
            
        except Exception as e:
            print(f"{Colors.RED}❌ Lỗi kết nối: {e}{Colors.RESET}")
            self.connected = False
            return False
            
    def disconnect(self):
        """Ngắt kết nối"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
        print(f"{Colors.YELLOW}🔌 Đã ngắt kết nối{Colors.RESET}")
        
    def send(self, command: str, show_response: bool = True) -> str:
        """Gửi lệnh và nhận phản hồi"""
        if not self.connected or not self.serial:
            print(f"{Colors.RED}❌ Chưa kết nối!{Colors.RESET}")
            return ""
            
        try:
            # Gửi lệnh
            self.serial.write(f"{command}\n".encode('utf-8'))
            self.serial.flush()
            time.sleep(0.1)
            
            # Đọc phản hồi
            response = ""
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response += line + "\n"
                    if show_response:
                        print(f"{Colors.CYAN}← {line}{Colors.RESET}")
                        
            return response.strip()
            
        except Exception as e:
            print(f"{Colors.RED}❌ Lỗi: {e}{Colors.RESET}")
            return ""
            
    def _read_startup(self):
        """Đọc thông tin khởi động từ ESP32"""
        time.sleep(0.5)
        while self.serial.in_waiting:
            line = self.serial.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"{Colors.BLUE}  {line}{Colors.RESET}")
                
    @staticmethod
    def list_ports():
        """Liệt kê các cổng Serial"""
        ports = serial.tools.list_ports.comports()
        print(f"\n{Colors.BOLD}🔌 Cổng Serial có sẵn:{Colors.RESET}")
        if ports:
            for p in ports:
                print(f"   {p.device}: {p.description}")
        else:
            print(f"   {Colors.RED}Không tìm thấy cổng nào{Colors.RESET}")
        print()


# ═══════════════════════════════════════════════════════════════════════════════
# COMMAND PARSER
# ═══════════════════════════════════════════════════════════════════════════════

class CommandParser:
    """Xử lý và thực thi các lệnh"""
    
    def __init__(self, connection: SerialConnection):
        self.conn = connection
        self.speed = 150
        self.commands = {
            # Di chuyển cơ bản
            'f': ('forward', 'Tiến'),
            'b': ('backward', 'Lùi'),
            'l': ('left', 'Sang trái'),
            'r': ('right', 'Sang phải'),
            'rl': ('rotate_left', 'Xoay trái'),
            'rr': ('rotate_right', 'Xoay phải'),
            's': ('stop', 'Dừng'),
            'off': ('off', 'Tắt driver'),
            
            # Di chuyển chéo
            'dfl': ('diagonal_fl', 'Chéo tiến-trái'),
            'dfr': ('diagonal_fr', 'Chéo tiến-phải'),
            'dbl': ('diagonal_bl', 'Chéo lùi-trái'),
            'dbr': ('diagonal_br', 'Chéo lùi-phải'),
            
            # Test motor
            't1': ('test_m1', 'Test Motor 1 (FL)'),
            't2': ('test_m2', 'Test Motor 2 (FR)'),
            't3': ('test_m3', 'Test Motor 3 (BR)'),
            't4': ('test_m4', 'Test Motor 4 (BL)'),
            'ta': ('test_all', 'Test tất cả motor'),
        }
        
    def parse(self, input_str: str) -> bool:
        """Parse và thực thi lệnh"""
        parts = input_str.strip().lower().split()
        if not parts:
            return True
            
        cmd = parts[0]
        args = parts[1:] if len(parts) > 1 else []
        
        # Lệnh hệ thống
        if cmd in ['quit', 'exit', 'q']:
            return False
        elif cmd == 'help' or cmd == 'h' or cmd == '?':
            self.show_help()
        elif cmd == 'ports':
            SerialConnection.list_ports()
        elif cmd == 'connect':
            port = args[0] if args else DEFAULT_PORT
            self.conn.disconnect()
            self.conn.port = port
            self.conn.connect()
        elif cmd == 'disconnect':
            self.conn.disconnect()
        elif cmd == 'status':
            self.show_status()
        elif cmd == 'speed':
            if args:
                self.speed = max(50, min(255, int(args[0])))
                print(f"⚡ Tốc độ đặt: {self.speed}")
            else:
                print(f"⚡ Tốc độ hiện tại: {self.speed}")
        elif cmd == 'raw':
            # Gửi lệnh raw đến ESP32
            raw_cmd = ' '.join(args)
            self.conn.send(raw_cmd)
            
        # Lệnh di chuyển
        elif cmd in self.commands:
            self._execute_movement(cmd, args)
            
        # Gửi trực tiếp đến ESP32
        else:
            self.conn.send(input_str)
            
        return True
        
    def _execute_movement(self, cmd: str, args: list):
        """Thực thi lệnh di chuyển"""
        speed = int(args[0]) if args else self.speed
        
        cmd_map = {
            'f': f'F {speed}',
            'b': f'B {speed}',
            'l': f'SL {speed}',
            'r': f'SR {speed}',
            'rl': f'L {speed}',
            'rr': f'R {speed}',
            's': 'S',
            'off': 'OFF',
            'dfl': f'DFL {speed}',
            'dfr': f'DFR {speed}',
            'dbl': f'DBL {speed}',
            'dbr': f'DBR {speed}',
            't1': f'T1 {speed}',
            't2': f'T2 {speed}',
            't3': f'T3 {speed}',
            't4': f'T4 {speed}',
            'ta': f'TA {speed}',
        }
        
        if cmd in cmd_map:
            action_name = self.commands[cmd][1]
            print(f"{Colors.YELLOW}→ {action_name} (speed={speed}){Colors.RESET}")
            self.conn.send(cmd_map[cmd])
            
    def show_help(self):
        """Hiển thị trợ giúp"""
        print(f"""
{Colors.BOLD}{Colors.CYAN}╔══════════════════════════════════════════════════════════════════╗
║               📖 K-PATROL CLI - DANH SÁCH LỆNH                   ║
╚══════════════════════════════════════════════════════════════════╝{Colors.RESET}

{Colors.BOLD}🚀 DI CHUYỂN CƠ BẢN:{Colors.RESET}
   f [speed]      Tiến (Forward)
   b [speed]      Lùi (Backward)
   l [speed]      Sang trái (Strafe Left)
   r [speed]      Sang phải (Strafe Right)
   rl [speed]     Xoay trái (Rotate Left)
   rr [speed]     Xoay phải (Rotate Right)

{Colors.BOLD}🔀 DI CHUYỂN CHÉO:{Colors.RESET}
   dfl [speed]    Chéo tiến-trái
   dfr [speed]    Chéo tiến-phải
   dbl [speed]    Chéo lùi-trái
   dbr [speed]    Chéo lùi-phải

{Colors.BOLD}🛑 DỪNG:{Colors.RESET}
   s              Dừng (Brake mode)
   off            Tắt hoàn toàn driver (Emergency OFF)

{Colors.BOLD}🔧 TEST MOTOR:{Colors.RESET}
   t1 [speed]     Test Motor 1 (Front-Left)
   t2 [speed]     Test Motor 2 (Front-Right)
   t3 [speed]     Test Motor 3 (Back-Right)
   t4 [speed]     Test Motor 4 (Back-Left)
   ta [speed]     Test tất cả motor

{Colors.BOLD}⚙️ CẤU HÌNH:{Colors.RESET}
   speed [value]  Đặt/xem tốc độ mặc định (50-255)
   status         Hiển thị trạng thái
   ports          Liệt kê cổng Serial

{Colors.BOLD}🔌 KẾT NỐI:{Colors.RESET}
   connect [port] Kết nối với ESP32
   disconnect     Ngắt kết nối

{Colors.BOLD}📤 LỆNH RAW:{Colors.RESET}
   raw <command>  Gửi lệnh trực tiếp đến ESP32

{Colors.BOLD}🚪 THOÁT:{Colors.RESET}
   quit / q       Thoát chương trình

{Colors.YELLOW}💡 Gợi ý: Có thể gõ trực tiếp lệnh ESP32 (F 100, T1 150, ...){Colors.RESET}
""")
        
    def show_status(self):
        """Hiển thị trạng thái"""
        status = "🟢 Connected" if self.conn.connected else "🔴 Disconnected"
        print(f"""
{Colors.BOLD}📊 TRẠNG THÁI:{Colors.RESET}
   Serial: {status} ({self.conn.port})
   Speed:  {self.speed}/255
""")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def print_banner():
    """In banner"""
    print(f"""
{Colors.CYAN}{Colors.BOLD}
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║   ██╗  ██╗      ██████╗  █████╗ ████████╗██████╗  ██████╗ ██╗   ║
║   ██║ ██╔╝      ██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔═══██╗██║   ║
║   █████╔╝ █████╗██████╔╝███████║   ██║   ██████╔╝██║   ██║██║   ║
║   ██╔═██╗ ╚════╝██╔═══╝ ██╔══██║   ██║   ██╔══██╗██║   ██║██║   ║
║   ██║  ██╗      ██║     ██║  ██║   ██║   ██║  ██║╚██████╔╝██║   ║
║   ╚═╝  ╚═╝      ╚═╝     ╚═╝  ╚═╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═╝   ║
║                                                                  ║
║          🤖 COMMAND LINE INTERFACE - MECANUM ROBOT 🤖            ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
{Colors.RESET}
   Gõ {Colors.YELLOW}'help'{Colors.RESET} để xem danh sách lệnh
   Gõ {Colors.YELLOW}'quit'{Colors.RESET} để thoát
""")


def main():
    """Entry point"""
    print_banner()
    
    # Tạo kết nối
    conn = SerialConnection()
    parser = CommandParser(conn)
    
    # Thử kết nối tự động
    print(f"{Colors.YELLOW}🔌 Đang kết nối với {DEFAULT_PORT}...{Colors.RESET}")
    if not conn.connect():
        print(f"\n{Colors.YELLOW}💡 Gõ 'ports' để xem danh sách cổng{Colors.RESET}")
        print(f"{Colors.YELLOW}   Gõ 'connect /dev/ttyXXX' để kết nối{Colors.RESET}\n")
    
    # Main loop
    try:
        while True:
            try:
                # Prompt
                status_icon = "🟢" if conn.connected else "🔴"
                prompt = f"{status_icon} {Colors.BOLD}kpatrol>{Colors.RESET} "
                
                # Đọc lệnh
                cmd = input(prompt)
                
                # Parse và thực thi
                if not parser.parse(cmd):
                    break
                    
            except KeyboardInterrupt:
                print("\n")
                continue
                
    except EOFError:
        pass
    finally:
        # Dọn dẹp
        if conn.connected:
            conn.send("OFF", show_response=False)
            conn.disconnect()
            
        print(f"\n{Colors.GREEN}👋 Tạm biệt!{Colors.RESET}\n")


if __name__ == "__main__":
    main()
