#!/usr/bin/env python3
"""
K-Patrol Bot - System Diagnostics & Testing
Công cụ chẩn đoán và test hệ thống robot

Sử dụng: python3 kpatrol_diagnostics.py [port]
"""

import serial
import serial.tools.list_ports
import time
import sys
from typing import Optional, List, Tuple
from dataclasses import dataclass
from enum import Enum

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# ═══════════════════════════════════════════════════════════════════════════════
# COLORS
# ═══════════════════════════════════════════════════════════════════════════════

class C:
    """Terminal colors"""
    RESET = "\033[0m"
    BOLD = "\033[1m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    
    @staticmethod
    def ok(text: str) -> str:
        return f"{C.GREEN}✅ {text}{C.RESET}"
    
    @staticmethod
    def fail(text: str) -> str:
        return f"{C.RED}❌ {text}{C.RESET}"
    
    @staticmethod
    def warn(text: str) -> str:
        return f"{C.YELLOW}⚠️  {text}{C.RESET}"
    
    @staticmethod
    def info(text: str) -> str:
        return f"{C.CYAN}ℹ️  {text}{C.RESET}"


# ═══════════════════════════════════════════════════════════════════════════════
# TEST RESULT
# ═══════════════════════════════════════════════════════════════════════════════

class TestStatus(Enum):
    PASSED = "PASSED"
    FAILED = "FAILED"
    WARNING = "WARNING"
    SKIPPED = "SKIPPED"


@dataclass
class TestResult:
    name: str
    status: TestStatus
    message: str
    details: Optional[str] = None


# ═══════════════════════════════════════════════════════════════════════════════
# DIAGNOSTICS
# ═══════════════════════════════════════════════════════════════════════════════

class RobotDiagnostics:
    """Chạy các bài test chẩn đoán robot"""
    
    def __init__(self, port: str):
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.results: List[TestResult] = []
        
    def connect(self) -> bool:
        """Kết nối với ESP32"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=BAUD_RATE,
                timeout=2.0
            )
            time.sleep(2)
            self.serial.reset_input_buffer()
            return True
        except Exception as e:
            print(f"{C.fail(f'Không thể kết nối: {e}')}")
            return False
            
    def disconnect(self):
        """Ngắt kết nối"""
        if self.serial:
            self.send_command("OFF")
            time.sleep(0.1)
            self.serial.close()
            
    def send_command(self, cmd: str, wait: float = 0.2) -> str:
        """Gửi lệnh và đọc response"""
        if not self.serial:
            return ""
            
        try:
            self.serial.write(f"{cmd}\n".encode())
            self.serial.flush()
            time.sleep(wait)
            
            response = ""
            while self.serial.in_waiting:
                response += self.serial.readline().decode('utf-8', errors='ignore')
                
            return response.strip()
        except (serial.SerialException, OSError, UnicodeDecodeError):
            return ""
            
    def add_result(self, name: str, status: TestStatus, message: str, details: str = None):
        """Thêm kết quả test"""
        self.results.append(TestResult(name, status, message, details))
        
        # Print result
        icon = {
            TestStatus.PASSED: C.GREEN + "✅",
            TestStatus.FAILED: C.RED + "❌",
            TestStatus.WARNING: C.YELLOW + "⚠️",
            TestStatus.SKIPPED: C.BLUE + "⏭️"
        }[status]
        
        print(f"   {icon} {name}: {message}{C.RESET}")
        if details:
            for line in details.split('\n'):
                print(f"      {C.CYAN}└─ {line}{C.RESET}")
                
    # ═══════════════════════════════════════════════════════════════════════════
    # TEST CASES
    # ═══════════════════════════════════════════════════════════════════════════
    
    def test_serial_ports(self) -> TestResult:
        """Test 1: Kiểm tra cổng Serial"""
        print(f"\n{C.BOLD}[TEST 1] Kiểm tra cổng Serial...{C.RESET}")
        
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            self.add_result(
                "Serial Ports",
                TestStatus.FAILED,
                "Không tìm thấy cổng Serial nào"
            )
            return
            
        found_esp32 = False
        port_info = []
        
        for p in ports:
            info = f"{p.device}: {p.description}"
            port_info.append(info)
            
            # Check if ESP32
            if any(x in p.description.lower() for x in ['cp210', 'ch340', 'ftdi', 'usb', 'serial']):
                if p.device == self.port:
                    found_esp32 = True
                    
        if found_esp32:
            self.add_result(
                "Serial Ports",
                TestStatus.PASSED,
                f"Tìm thấy {len(ports)} cổng, ESP32 tại {self.port}",
                "\n".join(port_info)
            )
        else:
            self.add_result(
                "Serial Ports",
                TestStatus.WARNING,
                f"Tìm thấy {len(ports)} cổng, nhưng không chắc chắn ESP32",
                "\n".join(port_info)
            )
            
    def test_connection(self) -> bool:
        """Test 2: Kiểm tra kết nối"""
        print(f"\n{C.BOLD}[TEST 2] Kiểm tra kết nối ESP32...{C.RESET}")
        
        if self.connect():
            self.add_result(
                "Connection",
                TestStatus.PASSED,
                f"Đã kết nối với {self.port} @ {BAUD_RATE} baud"
            )
            return True
        else:
            self.add_result(
                "Connection",
                TestStatus.FAILED,
                f"Không thể kết nối với {self.port}"
            )
            return False
            
    def test_communication(self):
        """Test 3: Kiểm tra giao tiếp"""
        print(f"\n{C.BOLD}[TEST 3] Kiểm tra giao tiếp Serial...{C.RESET}")
        
        # Send test command
        response = self.send_command("S", 0.5)
        
        if response:
            self.add_result(
                "Communication",
                TestStatus.PASSED,
                "ESP32 phản hồi lệnh",
                response[:200] if len(response) > 200 else response
            )
        else:
            self.add_result(
                "Communication",
                TestStatus.WARNING,
                "Không nhận được phản hồi (có thể firmware không echo)"
            )
            
    def test_motor_individual(self, motor_id: int, motor_name: str):
        """Test từng motor"""
        print(f"\n{C.BOLD}[TEST] Motor {motor_id} ({motor_name})...{C.RESET}")
        
        # Gửi lệnh test
        response = self.send_command(f"T{motor_id} 100", 0.3)
        
        print(f"   {C.YELLOW}→ Đang chạy Motor {motor_id} với tốc độ 100...{C.RESET}")
        time.sleep(2)
        
        # Dừng motor
        self.send_command("OFF", 0.1)
        
        # Hỏi user
        while True:
            result = input(f"   {C.CYAN}Motor {motor_id} hoạt động? (y=Đúng/n=Không quay/r=Quay ngược/s=Bỏ qua): {C.RESET}").lower()
            
            if result == 'y':
                self.add_result(
                    f"Motor {motor_id} ({motor_name})",
                    TestStatus.PASSED,
                    "Hoạt động bình thường"
                )
                return True
            elif result == 'n':
                self.add_result(
                    f"Motor {motor_id} ({motor_name})",
                    TestStatus.FAILED,
                    "Motor không quay",
                    "Kiểm tra:\n- Dây nguồn 12V\n- Kết nối driver BTS7960\n- Dây tín hiệu từ ESP32"
                )
                return False
            elif result == 'r':
                self.add_result(
                    f"Motor {motor_id} ({motor_name})",
                    TestStatus.WARNING,
                    "Motor quay ngược chiều",
                    "Cần đảo dây M+/M- tại driver BTS7960"
                )
                return False
            elif result == 's':
                self.add_result(
                    f"Motor {motor_id} ({motor_name})",
                    TestStatus.SKIPPED,
                    "Bỏ qua"
                )
                return True
            else:
                print("   Vui lòng chọn y, n, r hoặc s")
                
    def test_all_motors(self):
        """Test tất cả motor"""
        motors = [
            (1, "Front-Left"),
            (2, "Front-Right"),
            (3, "Back-Right"),
            (4, "Back-Left")
        ]
        
        print(f"\n{C.BOLD}{'='*60}{C.RESET}")
        print(f"{C.BOLD}[TEST] KIỂM TRA TỪNG MOTOR{C.RESET}")
        print(f"{C.BOLD}{'='*60}{C.RESET}")
        print(f"\n{C.YELLOW}⚠️  CẢNH BÁO: Nhấc robot lên khỏi mặt đất để an toàn!{C.RESET}")
        input(f"\n   Nhấn ENTER để bắt đầu test motor...")
        
        for motor_id, motor_name in motors:
            self.test_motor_individual(motor_id, motor_name)
            time.sleep(0.5)
            
    def test_mecanum_movement(self):
        """Test các hướng di chuyển Mecanum"""
        print(f"\n{C.BOLD}{'='*60}{C.RESET}")
        print(f"{C.BOLD}[TEST] KIỂM TRA DI CHUYỂN MECANUM{C.RESET}")
        print(f"{C.BOLD}{'='*60}{C.RESET}")
        
        movements = [
            ("F", "Forward (Tiến)"),
            ("B", "Backward (Lùi)"),
            ("SL", "Strafe Left (Sang trái)"),
            ("SR", "Strafe Right (Sang phải)"),
            ("L", "Rotate Left (Xoay trái)"),
            ("R", "Rotate Right (Xoay phải)"),
        ]
        
        print(f"\n{C.YELLOW}⚠️  ĐẶT ROBOT TRÊN MẶT PHẲNG RỘNG!{C.RESET}")
        input(f"\n   Nhấn ENTER để bắt đầu test di chuyển...")
        
        for cmd, name in movements:
            print(f"\n{C.BOLD}[TEST] {name}...{C.RESET}")
            input(f"   Nhấn ENTER để thực hiện {name}...")
            
            # Di chuyển
            self.send_command(f"{cmd} 120", 0.1)
            print(f"   {C.YELLOW}→ Robot đang di chuyển...{C.RESET}")
            time.sleep(2)
            self.send_command("S", 0.1)
            
            # Kiểm tra
            while True:
                result = input(f"   {C.CYAN}Di chuyển đúng hướng? (y/n/s=bỏ qua): {C.RESET}").lower()
                
                if result == 'y':
                    self.add_result(name, TestStatus.PASSED, "Di chuyển đúng")
                    break
                elif result == 'n':
                    self.add_result(
                        name,
                        TestStatus.FAILED,
                        "Di chuyển sai hướng",
                        "Kiểm tra:\n- Hướng lắp bánh Mecanum\n- Thứ tự đấu nối motor\n- Cờ INVERTED trong firmware"
                    )
                    break
                elif result == 's':
                    self.add_result(name, TestStatus.SKIPPED, "Bỏ qua")
                    break
                    
        # Dừng
        self.send_command("OFF", 0.1)
        
    def test_emergency_stop(self):
        """Test chức năng dừng khẩn cấp"""
        print(f"\n{C.BOLD}[TEST] Emergency Stop...{C.RESET}")
        
        # Start motors
        self.send_command("TA 100", 0.1)
        print(f"   {C.YELLOW}→ Đang chạy tất cả motor...{C.RESET}")
        time.sleep(1)
        
        # Emergency stop
        print(f"   {C.RED}→ Gửi lệnh OFF...{C.RESET}")
        response = self.send_command("OFF", 0.3)
        
        result = input(f"   {C.CYAN}Tất cả motor đã dừng? (y/n): {C.RESET}").lower()
        
        if result == 'y':
            self.add_result(
                "Emergency Stop",
                TestStatus.PASSED,
                "Dừng khẩn cấp hoạt động"
            )
        else:
            self.add_result(
                "Emergency Stop",
                TestStatus.FAILED,
                "Motor không dừng ngay lập tức"
            )
            
    def print_summary(self):
        """In tổng kết"""
        print(f"\n{C.BOLD}{'='*60}{C.RESET}")
        print(f"{C.BOLD}📊 TỔNG KẾT KẾT QUẢ TEST{C.RESET}")
        print(f"{C.BOLD}{'='*60}{C.RESET}\n")
        
        passed = sum(1 for r in self.results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in self.results if r.status == TestStatus.FAILED)
        warnings = sum(1 for r in self.results if r.status == TestStatus.WARNING)
        skipped = sum(1 for r in self.results if r.status == TestStatus.SKIPPED)
        
        print(f"   {C.GREEN}✅ Passed:  {passed}{C.RESET}")
        print(f"   {C.RED}❌ Failed:  {failed}{C.RESET}")
        print(f"   {C.YELLOW}⚠️  Warning: {warnings}{C.RESET}")
        print(f"   {C.BLUE}⏭️  Skipped: {skipped}{C.RESET}")
        
        print(f"\n{C.BOLD}Chi tiết:{C.RESET}")
        for r in self.results:
            icon = {
                TestStatus.PASSED: "✅",
                TestStatus.FAILED: "❌",
                TestStatus.WARNING: "⚠️",
                TestStatus.SKIPPED: "⏭️"
            }[r.status]
            print(f"   {icon} {r.name}: {r.message}")
            
        if failed > 0:
            print(f"\n{C.RED}{C.BOLD}⚠️  Có {failed} test thất bại! Cần kiểm tra lại phần cứng.{C.RESET}")
        elif warnings > 0:
            print(f"\n{C.YELLOW}{C.BOLD}⚠️  Có {warnings} cảnh báo. Xem xét sửa chữa.{C.RESET}")
        else:
            print(f"\n{C.GREEN}{C.BOLD}🎉 Tất cả test đều PASSED! Robot hoạt động tốt.{C.RESET}")
            
    def run_full_diagnostics(self):
        """Chạy toàn bộ chẩn đoán"""
        print(f"""
{C.CYAN}{C.BOLD}
╔══════════════════════════════════════════════════════════════════╗
║            🔧 K-PATROL ROBOT DIAGNOSTICS 🔧                      ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║   Công cụ chẩn đoán sẽ kiểm tra:                                ║
║   1. Cổng Serial                                                 ║
║   2. Kết nối ESP32                                               ║
║   3. Giao tiếp Serial                                            ║
║   4. Từng motor (FL, FR, BR, BL)                                ║
║   5. Các hướng di chuyển Mecanum                                ║
║   6. Chức năng dừng khẩn cấp                                    ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
{C.RESET}""")
        
        input(f"{C.YELLOW}Nhấn ENTER để bắt đầu chẩn đoán...{C.RESET}")
        
        # Test 1: Serial ports
        self.test_serial_ports()
        
        # Test 2: Connection
        if not self.test_connection():
            print(f"\n{C.RED}Không thể tiếp tục do lỗi kết nối.{C.RESET}")
            self.print_summary()
            return
            
        # Test 3: Communication
        self.test_communication()
        
        # Test 4: Motors
        self.test_all_motors()
        
        # Test 5: Mecanum movement
        do_movement = input(f"\n{C.CYAN}Bạn có muốn test di chuyển Mecanum? (y/n): {C.RESET}").lower()
        if do_movement == 'y':
            self.test_mecanum_movement()
            
        # Test 6: Emergency stop
        self.test_emergency_stop()
        
        # Cleanup
        self.disconnect()
        
        # Summary
        self.print_summary()


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    
    diag = RobotDiagnostics(port)
    
    try:
        diag.run_full_diagnostics()
    except KeyboardInterrupt:
        print(f"\n\n{C.YELLOW}Đã hủy bởi người dùng.{C.RESET}")
        diag.disconnect()
    except Exception as e:
        print(f"\n{C.RED}Lỗi: {e}{C.RESET}")
        diag.disconnect()


if __name__ == "__main__":
    main()
