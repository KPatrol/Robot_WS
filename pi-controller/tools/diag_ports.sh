#!/bin/bash
# K-Patrol port diagnostic — run on the Pi to figure out why motor / encoder
# boards aren't responding to commands while the D1 R32 peripheral hub is fine.
#
# Usage:  bash diag_ports.sh
#
# Symptom: only the D1 R32 (relay / horn / OLED) reacts to web commands.
# The two ESP32 boards have power but neither motor commands nor encoder /
# ToF telemetry reach the Pi.

set -u

bold() { printf "\033[1m%s\033[0m\n" "$*"; }
ok()   { printf "  \033[32m✓\033[0m %s\n" "$*"; }
bad()  { printf "  \033[31m✗\033[0m %s\n" "$*"; }
info() { printf "  • %s\n" "$*"; }

bold "──── 1. udev symlinks present? ────────────────────────────────────────"
for link in /dev/ttyKPATROL_MOTOR /dev/ttyKPATROL_ENCODER /dev/kpatrol-periph; do
  if [ -L "$link" ]; then
    target=$(readlink -f "$link")
    ok "$link -> $target"
  else
    bad "$link MISSING"
  fi
done

bold "──── 2. Raw /dev/tty* nodes ────────────────────────────────────────────"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | sed 's/^/  /'

bold "──── 3. USB enumeration (lsusb) ────────────────────────────────────────"
lsusb | sed 's/^/  /'

bold "──── 4. Per-device VID:PID + USB port (KERNELS path) ─────────────────"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  echo "  $dev"
  udevadm info -a -n "$dev" 2>/dev/null \
    | grep -E 'ATTRS\{idVendor\}|ATTRS\{idProduct\}|KERNELS==.*1-1' \
    | head -6 | sed 's/^/      /'
done

bold "──── 5. udev rules installed? ──────────────────────────────────────────"
if [ -f /etc/udev/rules.d/99-kpatrol-serial.rules ]; then
  ok "/etc/udev/rules.d/99-kpatrol-serial.rules present"
  info "Modified: $(stat -c '%y' /etc/udev/rules.d/99-kpatrol-serial.rules)"
else
  bad "/etc/udev/rules.d/99-kpatrol-serial.rules NOT installed!"
  info "Fix:  sudo cp ~/kpatrol/pi-controller/99-kpatrol-serial.rules /etc/udev/rules.d/"
  info "      sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

bold "──── 6. Last 30 kpatrol service log lines ──────────────────────────────"
journalctl -u kpatrol --no-pager -n 30 2>/dev/null \
  | grep -E '\[Motor\]|\[Encoder\]|\[Periph\]|Connection|FAIL|OK' \
  | sed 's/^/  /' || info "(no journalctl entries — service running?)"

bold "──── 7. Quick read test on each port (3 s capture) ─────────────────────"
for link in /dev/ttyKPATROL_MOTOR /dev/ttyKPATROL_ENCODER /dev/kpatrol-periph; do
  if [ -L "$link" ]; then
    echo "  --- $link (3 s sniff) ---"
    timeout 3 stdbuf -oL cat "$link" 2>/dev/null | head -8 | sed 's/^/      /' \
      || info "(no data — port closed or board silent)"
  fi
done

bold "──── 8. Cable / power hint check ───────────────────────────────────────"
# ESP32-S3 uses native USB; if the cable is power-only there will be no
# 303a:1001 enumeration even though the LED stays lit on the board.
if ! lsusb | grep -qi "303a:1001"; then
  bad "ESP32-S3 (303a:1001) NOT enumerated — likely power-only USB cable"
  info "Swap the S3 USB cable for a known-good data cable and re-check."
else
  ok "ESP32-S3 (303a:1001) enumerated"
fi
ch340_count=$(lsusb | grep -c "1a86:7523")
if [ "$ch340_count" -lt 2 ]; then
  bad "Only $ch340_count CH340 device(s) enumerated — expected 2 (encoder + periph)"
  info "Check the second CH340 cable / hub port."
else
  ok "$ch340_count CH340 devices enumerated"
fi

echo
bold "Done. Most common causes:"
echo "  1. ESP32-S3 USB cable is power-only (no data lines) → swap cable."
echo "  2. ESP32 Dev encoder plugged into wrong USB hub slot — udev expects"
echo "     KERNELS==\"1-1.3\". If it's at 1-1.2 / 1-1.4 the symlink stays empty."
echo "  3. udev rules not installed or not reloaded after editing."
echo "  4. kpatrol.service started BEFORE udev finished — try:"
echo "       sudo systemctl restart kpatrol"
