#!/usr/bin/env python3
"""
K-Patrol Sensor Reader - Read Encoder + ToF data from ESP32 Dev Module

This script reads sensor data from ESP32 Dev Module via USB Serial.
The ESP32 is running encoder_reader_4motors.ino with ToF sensors support.

Data Formats from ESP32:
  - ENC:fr_cnt,fl_cnt,br_cnt,bl_cnt,fr_rpm,fl_rpm,br_rpm,bl_rpm
  - TOF:front,front_left,front_right,left,right,back
  - SAFETY:zone,min_distance (zone: 0=SAFE, 1=SLOW, 2=WARNING, 3=DANGER)

Usage:
  python3 sensor_reader.py                    # Default /dev/ttyUSB0
  python3 sensor_reader.py --port /dev/ttyUSB1
  python3 sensor_reader.py --json             # Output as JSON

Author: K-Patrol Team
Date: February 25, 2026
"""

import serial
import time
import argparse
import json
import sys
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any
from enum import IntEnum

# ============================================================================
# DATA CLASSES
# ============================================================================

class SafetyZone(IntEnum):
    SAFE = 0
    SLOW = 1
    WARNING = 2
    DANGER = 3

@dataclass
class EncoderData:
    """Encoder readings for all 4 motors"""
    fr_count: int = 0
    fl_count: int = 0
    br_count: int = 0
    bl_count: int = 0
    fr_rpm: float = 0.0
    fl_rpm: float = 0.0
    br_rpm: float = 0.0
    bl_rpm: float = 0.0
    timestamp: float = 0.0

@dataclass
class ToFData:
    """ToF sensor readings (distance in mm)"""
    front: int = 9999
    front_left: int = 9999
    front_right: int = 9999
    left: int = 9999
    right: int = 9999
    back: int = 9999
    timestamp: float = 0.0

@dataclass
class SafetyData:
    """Safety zone status"""
    zone: SafetyZone = SafetyZone.SAFE
    min_distance: int = 9999
    timestamp: float = 0.0
    
    @property
    def zone_name(self) -> str:
        return self.zone.name
    
    @property
    def is_danger(self) -> bool:
        return self.zone == SafetyZone.DANGER
    
    @property
    def speed_factor(self) -> float:
        """Speed multiplier based on safety zone"""
        factors = {
            SafetyZone.SAFE: 1.0,
            SafetyZone.SLOW: 0.8,
            SafetyZone.WARNING: 0.5,
            SafetyZone.DANGER: 0.0
        }
        return factors.get(self.zone, 1.0)

@dataclass
class SensorData:
    """Combined sensor data"""
    encoder: EncoderData
    tof: ToFData
    safety: SafetyData

# ============================================================================
# SENSOR READER CLASS
# ============================================================================

class SensorReader:
    """
    Reads sensor data from ESP32 Dev Module via Serial.
    
    ESP32 Dev is connected to Pi via USB (/dev/ttyUSB0)
    Baud rate: 115200
    """
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        
        # Current sensor data
        self.encoder = EncoderData()
        self.tof = ToFData()
        self.safety = SafetyData()
        
        # Callbacks
        self.on_encoder_update = None
        self.on_tof_update = None
        self.on_safety_update = None
        self.on_danger_detected = None
    
    def connect(self) -> bool:
        """Connect to ESP32 via serial"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(0.5)  # Wait for connection
            print(f"✓ Connected to {self.port} @ {self.baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected from serial")
    
    def send_command(self, cmd: str):
        """Send a command to ESP32"""
        if self.serial and self.serial.is_open:
            self.serial.write(f"{cmd}\n".encode())
    
    def reset_encoders(self):
        """Reset encoder counts to 0"""
        self.send_command('R')
    
    def request_tof(self):
        """Request ToF data"""
        self.send_command('T')
    
    def request_encoder(self):
        """Request encoder data"""
        self.send_command('E')
    
    def request_safety(self):
        """Request safety status"""
        self.send_command('S')
    
    def set_json_mode(self, enabled: bool = True):
        """Set JSON output mode on ESP32"""
        self.send_command('J' if enabled else 'M')
    
    def scan_i2c(self):
        """Scan I2C bus"""
        self.send_command('I')
    
    def parse_line(self, line: str) -> bool:
        """
        Parse a line of data from ESP32.
        Returns True if data was parsed successfully.
        """
        line = line.strip()
        if not line:
            return False
        
        timestamp = time.time()
        
        try:
            # Encoder data: ENC:fr,fl,br,bl,rpm_fr,rpm_fl,rpm_br,rpm_bl
            if line.startswith('ENC:'):
                values = line[4:].split(',')
                if len(values) >= 8:
                    self.encoder = EncoderData(
                        fr_count=int(values[0]),
                        fl_count=int(values[1]),
                        br_count=int(values[2]),
                        bl_count=int(values[3]),
                        fr_rpm=float(values[4]),
                        fl_rpm=float(values[5]),
                        br_rpm=float(values[6]),
                        bl_rpm=float(values[7]),
                        timestamp=timestamp
                    )
                    if self.on_encoder_update:
                        self.on_encoder_update(self.encoder)
                    return True
            
            # ToF data: TOF:front,front_left,front_right,left,right,back
            elif line.startswith('TOF:'):
                values = line[4:].split(',')
                if len(values) >= 6:
                    self.tof = ToFData(
                        front=int(values[0]),
                        front_left=int(values[1]),
                        front_right=int(values[2]),
                        left=int(values[3]),
                        right=int(values[4]),
                        back=int(values[5]),
                        timestamp=timestamp
                    )
                    if self.on_tof_update:
                        self.on_tof_update(self.tof)
                    return True
            
            # Safety data: SAFETY:zone,min_distance
            elif line.startswith('SAFETY:'):
                values = line[7:].split(',')
                if len(values) >= 2:
                    zone = SafetyZone(int(values[0]))
                    was_danger = self.safety.is_danger
                    
                    self.safety = SafetyData(
                        zone=zone,
                        min_distance=int(values[1]),
                        timestamp=timestamp
                    )
                    
                    if self.on_safety_update:
                        self.on_safety_update(self.safety)
                    
                    # Trigger danger callback
                    if self.safety.is_danger and not was_danger:
                        if self.on_danger_detected:
                            self.on_danger_detected(self.safety)
                    
                    return True
            
            # JSON data
            elif line.startswith('{'):
                data = json.loads(line)
                # Parse JSON format from ESP32
                if 'enc' in data:
                    enc = data['enc']
                    self.encoder = EncoderData(
                        fr_count=enc.get('fr', {}).get('cnt', 0),
                        fl_count=enc.get('fl', {}).get('cnt', 0),
                        br_count=enc.get('br', {}).get('cnt', 0),
                        bl_count=enc.get('bl', {}).get('cnt', 0),
                        fr_rpm=enc.get('fr', {}).get('rpm', 0),
                        fl_rpm=enc.get('fl', {}).get('rpm', 0),
                        br_rpm=enc.get('br', {}).get('rpm', 0),
                        bl_rpm=enc.get('bl', {}).get('rpm', 0),
                        timestamp=timestamp
                    )
                if 'tof' in data:
                    tof = data['tof']
                    self.tof = ToFData(
                        front=tof.get('front', 9999),
                        front_left=tof.get('front_left', 9999),
                        front_right=tof.get('front_right', 9999),
                        left=tof.get('left', 9999),
                        right=tof.get('right', 9999),
                        back=tof.get('back', 9999),
                        timestamp=timestamp
                    )
                if 'safety' in data:
                    safety = data['safety']
                    zone_str = safety.get('zone', 'SAFE')
                    zone = SafetyZone[zone_str] if zone_str in SafetyZone.__members__ else SafetyZone.SAFE
                    self.safety = SafetyData(
                        zone=zone,
                        min_distance=safety.get('min', 9999),
                        timestamp=timestamp
                    )
                return True
                
        except (ValueError, IndexError, KeyError, json.JSONDecodeError) as e:
            # Ignore parse errors for non-data lines
            pass
        
        return False
    
    def read_line(self) -> Optional[str]:
        """Read a line from serial"""
        if self.serial and self.serial.is_open and self.serial.in_waiting:
            try:
                line = self.serial.readline().decode('utf-8', errors='ignore')
                return line.strip()
            except serial.SerialException:
                return None
        return None
    
    def read_and_parse(self) -> bool:
        """Read a line and parse it. Returns True if data was parsed."""
        line = self.read_line()
        if line:
            return self.parse_line(line)
        return False
    
    def get_all_data(self) -> SensorData:
        """Get all current sensor data"""
        return SensorData(
            encoder=self.encoder,
            tof=self.tof,
            safety=self.safety
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert all sensor data to dictionary"""
        return {
            'encoder': asdict(self.encoder),
            'tof': asdict(self.tof),
            'safety': {
                'zone': self.safety.zone.name,
                'zone_value': self.safety.zone.value,
                'min_distance': self.safety.min_distance,
                'speed_factor': self.safety.speed_factor,
                'timestamp': self.safety.timestamp
            }
        }
    
    def to_json(self) -> str:
        """Convert all sensor data to JSON string"""
        return json.dumps(self.to_dict(), indent=2)


# ============================================================================
# MAIN - EXAMPLE USAGE
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description='K-Patrol Sensor Reader')
    parser.add_argument('--port', '-p', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--json', '-j', action='store_true',
                        help='Output as JSON')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Quiet mode (only print data)')
    args = parser.parse_args()
    
    # Create reader
    reader = SensorReader(port=args.port, baudrate=args.baud)
    
    # Set callbacks
    def on_danger(safety: SafetyData):
        print(f"\n⚠️  DANGER! Obstacle at {safety.min_distance}mm!")
        # Here you could send emergency stop command to motor controller
    
    reader.on_danger_detected = on_danger
    
    # Connect
    if not reader.connect():
        sys.exit(1)
    
    if not args.quiet:
        print("\nReading sensor data... (Ctrl+C to exit)\n")
    
    try:
        last_print = 0
        while True:
            # Read and parse data
            if reader.read_and_parse():
                if args.json:
                    # Print JSON every 500ms
                    now = time.time()
                    if now - last_print >= 0.5:
                        print(reader.to_json())
                        last_print = now
                elif not args.quiet:
                    # Print human-readable data
                    now = time.time()
                    if now - last_print >= 0.5:
                        print(f"\r[ENC] FR:{reader.encoder.fr_count:6d} FL:{reader.encoder.fl_count:6d} "
                              f"BR:{reader.encoder.br_count:6d} BL:{reader.encoder.bl_count:6d} | "
                              f"[ToF] F:{reader.tof.front:4d} FL:{reader.tof.front_left:4d} "
                              f"FR:{reader.tof.front_right:4d} L:{reader.tof.left:4d} R:{reader.tof.right:4d} | "
                              f"[{reader.safety.zone_name:7s}]", end='')
                        last_print = now
            
            time.sleep(0.01)  # Small delay to prevent CPU hogging
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        reader.disconnect()


if __name__ == '__main__':
    main()
