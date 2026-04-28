#!/usr/bin/env python3
"""
K-Patrol Bot - Setup Wizard
Trình hướng dẫn cài đặt và kiểm tra từng motor
"""

import serial
import time
import sys
import os

# Cấu hình Serial
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def connect_serial():
    print("🔌 Đang kết nối với ESP32-S3...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print(f"✅ Đã kết nối: {SERIAL_PORT}")
        # Xóa buffer
        while ser.in_waiting:
            ser.readline()
        return ser
    except Exception as e:
        print(f"❌ Lỗi kết nối: {e}")
        print("💡 Gợi ý: Kiểm tra cáp USB hoặc cổng Serial (/dev/ttyACM0)")
        sys.exit(1)

def send_command(ser, cmd):
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        # print(f"   [ESP32]: {line}")

def wizard_step_motor(ser, motor_id, motor_name):
    clear_screen()
    print("="*60)
    print(f"⚙️  CẤU HÌNH MOTOR {motor_id}: {motor_name}")
    print("="*60)
    print("\n⚠️  CHÚ Ý: Hãy nhấc robot lên khỏi mặt đất để an toàn!")
    print(f"   Chúng ta sẽ kiểm tra Motor {motor_id} độc lập.")
    print("   Các motor khác sẽ bị ngắt điện hoàn toàn.")
    print("")
    
    input("👉 Nhấn ENTER để bắt đầu chạy thử...")
    
    print(f"\n🔄 Đang chạy Motor {motor_id} chiều TIẾN (Forward)...")
    send_command(ser, f"T{motor_id} 100")
    
    print("   Motor đang quay...")
    time.sleep(3)
    
    send_command(ser, "OFF")
    print("🛑 Đã dừng motor.")
    
    print("\n❓ CÂU HỎI KIỂM TRA:")
    print(f"1. Motor {motor_id} có quay không?")
    print("2. Nếu quay, nó có quay theo chiều TIẾN không?")
    print("   (Bánh xe phải quay về phía trước của robot)")
    print("")
    
    while True:
        ans = input("👉 Kết quả (y=Đúng/n=Không quay/r=Quay ngược): ").lower().strip()
        
        if ans == 'y':
            print(f"✅ Motor {motor_id} đã cấu hình ĐÚNG!")
            time.sleep(1)
            return True
        elif ans == 'n':
            print(f"\n❌ LỖI: Motor {motor_id} không quay!")
            print("👉 Kiểm tra lại:")
            print("   - Dây nguồn 12V vào driver BTS7960")
            print("   - Dây nối từ driver vào motor")
            print("   - Dây tín hiệu từ ESP32 xuống driver")
            input("\nNhấn ENTER để thử lại...")
            return wizard_step_motor(ser, motor_id, motor_name)
        elif ans == 'r':
            print(f"\n⚠️  LỖI: Motor {motor_id} quay NGƯỢC chiều!")
            print("👉 HÀNH ĐỘNG CẦN LÀM:")
            print("   1. Tắt nguồn robot")
            print("   2. Đảo vị trí 2 dây động cơ (Đỏ <-> Trắng) tại cầu đấu driver")
            print("   3. Bật nguồn lại")
            input("\nNhấn ENTER sau khi đã sửa xong để thử lại...")
            return wizard_step_motor(ser, motor_id, motor_name)
        else:
            print("Vui lòng chọn y, n hoặc r")

def main():
    clear_screen()
    print("="*60)
    print("🤖 K-PATROL BOT - SETUP WIZARD")
    print("="*60)
    print("Công cụ hỗ trợ cấu hình từng motor độc lập.")
    print("Đảm bảo an toàn và chính xác trước khi chạy toàn hệ thống.")
    print("")
    
    ser = connect_serial()
    
    motors = [
        (1, "FRONT LEFT (Trước Trái)"),
        (2, "FRONT RIGHT (Trước Phải)"),
        (3, "REAR LEFT (Sau Trái)"),
        (4, "REAR RIGHT (Sau Phải)")
    ]
    
    # Reset trạng thái
    send_command(ser, "OFF")
    
    for mid, mname in motors:
        if not wizard_step_motor(ser, mid, mname):
            print("\n❌ Cấu hình thất bại. Đang thoát...")
            send_command(ser, "OFF")
            return

    clear_screen()
    print("="*60)
    print("🎉 CHÚC MỪNG! CẤU HÌNH HOÀN TẤT")
    print("="*60)
    print("Tất cả 4 motor đã được kiểm tra:")
    print("✅ Đấu nối đúng")
    print("✅ Chiều quay đúng")
    print("✅ Hoạt động độc lập tốt")
    print("")
    print("Bây giờ bạn có thể chạy script 'test_mecanum_movement.py'")
    print("để kiểm tra phối hợp chuyển động.")
    print("")
    
    send_command(ser, "OFF")
    ser.close()

if __name__ == "__main__":
    main()
