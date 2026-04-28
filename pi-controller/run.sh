#!/bin/bash
# K-Patrol Bot - Quick Launcher Script
# Chạy: ./run.sh [mode]
#
# Modes:
#   cli       - Command line interface (default)
#   keyboard  - Keyboard control mode
#   gamepad   - Gamepad/joystick controller
#   diag      - Diagnostics & testing
#   full      - Full controller with UI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Banner
echo -e "${CYAN}"
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                    🤖 K-PATROL BOT LAUNCHER 🤖                   ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Check for ESP32 connection
ESP32_PORT=""
if [ -e /dev/ttyACM0 ]; then
    ESP32_PORT="/dev/ttyACM0"
elif [ -e /dev/ttyUSB0 ]; then
    ESP32_PORT="/dev/ttyUSB0"
fi

if [ -n "$ESP32_PORT" ]; then
    echo -e "${GREEN}✅ ESP32 detected: $ESP32_PORT${NC}"
else
    echo -e "${YELLOW}⚠️  ESP32 not detected. Please connect via USB.${NC}"
    echo -e "${YELLOW}   Available ports:${NC}"
    ls /dev/tty* 2>/dev/null | grep -E '(ACM|USB|AMA)' || echo "   (none found)"
    echo ""
fi

# Parse mode
MODE="${1:-cli}"

case "$MODE" in
    cli)
        echo -e "${CYAN}Starting CLI mode...${NC}"
        python3 kpatrol_cli.py $ESP32_PORT
        ;;
    keyboard|kb)
        echo -e "${CYAN}Starting Keyboard control mode...${NC}"
        python3 kpatrol_keyboard.py $ESP32_PORT
        ;;
    gamepad|gp)
        echo -e "${CYAN}Starting Gamepad control mode...${NC}"
        python3 kpatrol_gamepad.py $ESP32_PORT
        ;;
    diag|test)
        echo -e "${CYAN}Starting Diagnostics...${NC}"
        python3 kpatrol_diagnostics.py $ESP32_PORT
        ;;
    full|controller)
        echo -e "${CYAN}Starting Full Controller...${NC}"
        python3 kpatrol_controller.py
        ;;
    help|--help|-h)
        echo "Usage: ./run.sh [mode]"
        echo ""
        echo "Available modes:"
        echo "  cli       - Command line interface (default)"
        echo "  keyboard  - Keyboard control mode"
        echo "  gamepad   - Gamepad/joystick controller"
        echo "  diag      - Diagnostics & testing"
        echo "  full      - Full controller with UI"
        echo ""
        echo "Examples:"
        echo "  ./run.sh             # Run CLI mode"
        echo "  ./run.sh keyboard    # Run keyboard control"
        echo "  ./run.sh diag        # Run diagnostics"
        ;;
    *)
        echo -e "${RED}Unknown mode: $MODE${NC}"
        echo "Use './run.sh help' for available modes"
        exit 1
        ;;
esac
