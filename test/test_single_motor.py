#!/usr/bin/env python3
"""
K-Patrol Bot - Test Single Motor
Script để test từng motor một cách độc lập

Chạy: python3 test_single_motor.py
"""

import serial
import time
import sys

# Cấu hình Serial
SERIAL_PORT = '/dev/ttyACM0'  # Thay đổi nếu cần (có thể là /dev/ttyUSB0)
BAUD_RATE = 115200

def connect_serial():
    """Kết nối Serial với ESP32-S3"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Đợi ESP32 khởi động
        print(f"✅ Connected to {SERIAL_PORT}")
        
        # Đọc thông tin khởi động
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        return ser
    except Exception as e:
        print(f"❌ Error connecting: {e}")
        print("\n💡 Tips:")
        print("  - Kiểm tra ESP32-S3 đã kết nối USB chưa")
        print("  - Thử: ls /dev/tty* | grep -E '(ACM|USB)'")
        print("  - Thay đổi SERIAL_PORT trong script")
        sys.exit(1)

def send_command(ser, cmd):
    """Gửi lệnh và đọc phản hồi"""
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    
    response = ""
    while ser.in_waiting:
        response += ser.readline().decode('utf-8', errors='ignore')
    
    if response:
        print(f"← {response.strip()}")
    
    return response

def test_single_motor(ser, motor_num, speed=100, duration=2):
    """Test một motor cụ thể"""
    print(f"\n🔧 Testing Motor {motor_num} at speed {speed}...")
    
    # Chạy motor
    send_command(ser, f"T{motor_num} {speed}")
    time.sleep(duration)
    
    # Dừng motor
    send_command(ser, "S")
    print(f"✅ Motor {motor_num} test complete")

def test_all_motors_sequence(ser, speed=100, duration=2):
    """Test tất cả motor theo thứ tự"""
    print("\n" + "="*50)
    print("🔄 TESTING ALL MOTORS SEQUENTIALLY")
    print("="*50)
    
    motor_names = {
        1: "Front Left (Trước Trái)",
        2: "Front Right (Trước Phải)",
        3: "Rear Left (Sau Trái)",
        4: "Rear Right (Sau Phải)"
    }
    
    for motor_num in range(1, 5):
        print(f"\n📍 Motor {motor_num}: {motor_names[motor_num]}")
        input(f"   Press ENTER to test Motor {motor_num}...")
        
        test_single_motor(ser, motor_num, speed, duration)
        
        # Hỏi kết quả
        result = input(f"   Motor {motor_num} quay đúng chiều? (y/n/r=reverse): ").lower()
        if result == 'n':
            print(f"   ⚠️  Motor {motor_num} KHÔNG quay - kiểm tra kết nối!")
        elif result == 'r':
            print(f"   ⚠️  Motor {motor_num} quay NGƯỢC - cần đảo dây M+/M-")
    
    print("\n" + "="*50)
    print("✅ Sequential test complete!")
    print("="*50)

def test_all_motors_together(ser, speed=100, duration=3):
    """Test tất cả motor cùng lúc"""
    print("\n" + "="*50)
    print("🔄 TESTING ALL MOTORS TOGETHER")
    print("="*50)
    
    input("Press ENTER to run all 4 motors...")
    
    send_command(ser, f"TA {speed}")
    print(f"⏱️  Running for {duration} seconds...")
    time.sleep(duration)
    
    send_command(ser, "S")
    print("✅ All motors test complete!")

def interactive_mode(ser):
    """Chế độ tương tác - gửi lệnh trực tiếp"""
    print("\n" + "="*50)
    print("🎮 INTERACTIVE MODE")
    print("="*50)
    print("Nhập lệnh trực tiếp (gõ 'H' để xem help, 'Q' để thoát)")
    print("")
    
    while True:
        try:
            cmd = input("→ ").strip()
            
            if cmd.upper() == 'Q' or cmd.upper() == 'QUIT':
                send_command(ser, "S")  # Dừng motor trước khi thoát
                print("👋 Goodbye!")
                break
            
            if cmd:
                send_command(ser, cmd)
                
        except KeyboardInterrupt:
            send_command(ser, "S")
            print("\n👋 Stopped!")
            break

def main():
    print("="*50)
    print("🤖 K-PATROL BOT - MOTOR TEST UTILITY")
    print("="*50)
    
    # Kết nối
    ser = connect_serial()
    
    while True:
        print("\n📋 MENU:")
        print("  1. Test từng motor (tuần tự)")
        print("  2. Test tất cả motor cùng lúc")
        print("  3. Test motor cụ thể")
        print("  4. Interactive mode (gửi lệnh tự do)")
        print("  5. Exit")
        print("")
        
        choice = input("Chọn (1-5): ").strip()
        
        if choice == '1':
            test_all_motors_sequence(ser)
        elif choice == '2':
            test_all_motors_together(ser)
        elif choice == '3':
            motor = input("Số motor (1-4): ").strip()
            speed = input("Tốc độ (1-255): ").strip()
            if motor.isdigit() and speed.isdigit():
                test_single_motor(ser, int(motor), int(speed))
        elif choice == '4':
            interactive_mode(ser)
        elif choice == '5':
            send_command(ser, "S")
            print("👋 Goodbye!")
            break
        else:
            print("❌ Invalid choice")
    
    ser.close()

if __name__ == "__main__":
    main()
