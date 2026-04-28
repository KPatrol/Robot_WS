#!/bin/bash
# K-Patrol Bot - Quick Setup Script for Raspberry Pi
# Chạy: chmod +x setup.sh && ./setup.sh

echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║           K-PATROL BOT - RASPBERRY PI SETUP                      ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# Check if running on Raspberry Pi
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model)
    echo "📟 Detected: $MODEL"
else
    echo "⚠️  Warning: Không phát hiện Raspberry Pi"
fi
echo ""

# Update system
echo "📦 Đang cập nhật hệ thống..."
sudo apt-get update -y

# Install Python dependencies
echo ""
echo "🐍 Đang cài đặt Python dependencies..."
sudo apt-get install -y python3-pip python3-serial

# Install pyserial
echo ""
echo "📡 Đang cài đặt pyserial..."
pip3 install pyserial --break-system-packages 2>/dev/null || pip3 install pyserial

# Install pynmea2 for NEO-6M GPS NMEA parsing (V5.2 outdoor mode)
echo ""
echo "🛰️  Đang cài đặt pynmea2 (GPS NEO-6M support)..."
pip3 install pynmea2 --break-system-packages 2>/dev/null || pip3 install pynmea2

# Install paho-mqtt for MQTT bridge
echo ""
echo "📨 Đang cài đặt paho-mqtt..."
pip3 install paho-mqtt --break-system-packages 2>/dev/null || pip3 install paho-mqtt

# Install inputs (optional, for gamepad)
echo ""
echo "🎮 Đang cài đặt inputs (gamepad support)..."
pip3 install inputs --break-system-packages 2>/dev/null || pip3 install inputs

# Install udev rules for stable serial port symlinks
echo ""
echo "🔗 Đang cài đặt udev rules cho serial symlinks..."
if [ -f "99-kpatrol-serial.rules" ]; then
    sudo cp 99-kpatrol-serial.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "   ✓ Đã cài rules → /dev/ttyKPATROL_{MOTOR,ENCODER,GPS}"
else
    echo "   ⚠️  Bỏ qua (không tìm thấy 99-kpatrol-serial.rules)"
fi

# Add user to dialout group for serial access
echo ""
echo "🔑 Đang cấu hình quyền Serial..."
sudo usermod -a -G dialout $USER

# Make scripts executable
echo ""
echo "⚙️  Đang cấu hình scripts..."
chmod +x *.py

# Check for ESP32
echo ""
echo "🔌 Đang tìm kiếm ESP32..."
SERIAL_PORTS=$(ls /dev/tty* 2>/dev/null | grep -E '(ACM|USB)')

if [ -n "$SERIAL_PORTS" ]; then
    echo "   Tìm thấy các cổng Serial:"
    echo "$SERIAL_PORTS" | while read port; do
        echo "   ✓ $port"
    done
else
    echo "   ⚠️  Không tìm thấy cổng Serial"
    echo "   Hãy kết nối ESP32-S3 qua USB"
fi

echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                    ✅ SETUP HOÀN TẤT!                            ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║                                                                  ║"
echo "║  Các công cụ có sẵn:                                            ║"
echo "║                                                                  ║"
echo "║  1. python3 kpatrol_diagnostics.py  - Chẩn đoán hệ thống       ║"
echo "║  2. python3 kpatrol_cli.py          - Command line interface   ║"
echo "║  3. python3 kpatrol_keyboard.py     - Điều khiển bàn phím      ║"
echo "║  4. python3 kpatrol_gamepad.py      - Điều khiển gamepad       ║"
echo "║  5. python3 kpatrol_controller.py   - Full controller          ║"
echo "║                                                                  ║"
echo "║  ⚠️  Hãy LOGOUT và LOGIN lại để áp dụng quyền Serial!          ║"
echo "║                                                                  ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""
