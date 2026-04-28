#!/usr/bin/env python3
"""
K-Patrol Bot - Mecanum Movement Test
Script để test các chuyển động Mecanum

Chạy: python3 test_mecanum_movement.py
"""

import serial
import time
import sys

# Cấu hình Serial
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def connect_serial():
    """Kết nối Serial với ESP32-S3"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to {SERIAL_PORT}")
        
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        return ser
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

def send_command(ser, cmd):
    """Gửi lệnh"""
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    while ser.in_waiting:
        print(f"← {ser.readline().decode('utf-8', errors='ignore').strip()}")

def run_movement_test(ser, name, command, speed=150, duration=2):
    """Chạy một test di chuyển"""
    print(f"\n▶️  {name}...")
    send_command(ser, f"{command} {speed}")
    time.sleep(duration)
    send_command(ser, "S")
    time.sleep(0.5)

def main():
    print("="*60)
    print("🤖 K-PATROL BOT - MECANUM MOVEMENT TEST")
    print("="*60)
    
    ser = connect_serial()
    
    print("\n⚠️  ĐẶT ROBOT TRÊN MẶT PHẲNG RỘNG!")
    print("Robot sẽ di chuyển theo các hướng khác nhau.\n")
    
    input("Press ENTER to start movement test...")
    
    speed = 150  # Tốc độ vừa phải
    duration = 2  # Thời gian mỗi test (giây)
    
    movements = [
        ("1. TIẾN (Forward)", "F"),
        ("2. LÙI (Backward)", "B"),
        ("3. SANG TRÁI (Strafe Left)", "L"),
        ("4. SANG PHẢI (Strafe Right)", "R"),
        ("5. XOAY TRÁI (Rotate Left)", "RL"),
        ("6. XOAY PHẢI (Rotate Right)", "RR"),
        ("7. CHÉO TRƯỚC-TRÁI (Diagonal FL)", "DFL"),
        ("8. CHÉO TRƯỚC-PHẢI (Diagonal FR)", "DFR"),
        ("9. CHÉO SAU-TRÁI (Diagonal BL)", "DBL"),
        ("10. CHÉO SAU-PHẢI (Diagonal BR)", "DBR"),
    ]
    
    print("\n" + "-"*60)
    print("BẮT ĐẦU TEST CÁC CHUYỂN ĐỘNG")
    print("-"*60)
    
    for name, cmd in movements:
        input(f"\n{name}\nPress ENTER to execute...")
        run_movement_test(ser, name, cmd, speed, duration)
        
        result = input("Chuyển động đúng? (y/n): ").lower()
        if result != 'y':
            print(f"⚠️  {name} - CẦN KIỂM TRA LẠI!")
    
    print("\n" + "="*60)
    print("✅ MOVEMENT TEST COMPLETE!")
    print("="*60)
    
    # Kiểm tra kết quả
    print("\n📋 CHECKLIST:")
    print("□ Robot tiến/lùi đúng hướng?")
    print("□ Robot đi ngang (strafe) đúng?")
    print("□ Robot xoay tại chỗ đúng?")
    print("□ Robot đi chéo đúng hướng?")
    print("\nNếu có vấn đề, kiểm tra:")
    print("- Hướng lắp bánh Mecanum (roller tạo hình X)")
    print("- Đấu nối dây motor (đảo M+/M- nếu quay ngược)")
    print("- Vị trí motor (FL, FR, RL, RR)")
    
    ser.close()

if __name__ == "__main__":
    main()
