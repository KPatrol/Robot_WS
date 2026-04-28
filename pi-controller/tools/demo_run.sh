#!/usr/bin/env bash
#
# demo_run.sh — One-command K-Patrol demo launcher
# =================================================
# Automates: reset state → set mode → show live status
# Run this ON THE PI after services are already up.
#
# Usage:
#   ./demo_run.sh                 # start SMART_COVERAGE + live monitor
#   ./demo_run.sh monitor         # monitor only (no state reset)
#   ./demo_run.sh stop            # stop robot, back to MANUAL
#   ./demo_run.sh home            # send robot to marker HOME
#   ./demo_run.sh reset           # reset odom + grid + markers only
#
# Requirements on Pi:
#   - mosquitto-clients (mosquitto_pub, mosquitto_sub)
#   - jq (optional, for pretty status display)
#

set -e

BROKER="103.81.84.43"
USER="alphaasimov2024"
PASS="gvB3DtGfus6U"
TOPIC_NAV="kpatrol/nav_command"
TOPIC_STATUS="kpatrol/nav_status"
TOPIC_MARKERS="kpatrol/markers"

pub() {
    mosquitto_pub -h "$BROKER" -u "$USER" -P "$PASS" -t "$TOPIC_NAV" -m "$1"
}

reset_state() {
    echo "→ Reset odometry and grid..."
    pub '{"action": "reset_odom"}'
    sleep 0.3
    pub '{"action": "reset_grid"}'
    sleep 0.3
    echo "  done."
}

start_smart_coverage() {
    echo "→ Starting SMART_COVERAGE @ speed 60..."
    pub '{"mode": "SMART_COVERAGE", "speed": 60}'
    echo "  done. Robot will begin patrolling inside marker boundary."
}

stop_robot() {
    echo "→ Stopping robot (MANUAL mode)..."
    pub '{"mode": "MANUAL"}'
    echo "  stopped."
}

return_home() {
    echo "→ Sending robot home (marker ID 0)..."
    pub '{"mode": "RETURN_HOME"}'
}

monitor() {
    if command -v jq >/dev/null 2>&1; then
        FILTER='{
            mode: .mode,
            vfh: .vfh_action,
            pose: .pos,
            yaw: .yaw,
            boundary: .boundary.mode,
            home: .boundary.has_home,
            corners: (.boundary.corners | length),
            anchors: (.boundary.anchors | map({id, samples, confident}))
        }'
        echo "→ Live status (Ctrl+C to stop)..."
        mosquitto_sub -h "$BROKER" -u "$USER" -P "$PASS" -t "$TOPIC_STATUS" 2>/dev/null \
            | while IFS= read -r line; do
                echo "$line" | jq -c "$FILTER" 2>/dev/null || echo "$line"
            done
    else
        echo "→ Live status (raw — install jq for pretty print)..."
        mosquitto_sub -h "$BROKER" -u "$USER" -P "$PASS" -t "$TOPIC_STATUS"
    fi
}

watch_markers() {
    if command -v jq >/dev/null 2>&1; then
        echo "→ Live ArUco detections (Ctrl+C to stop)..."
        mosquitto_sub -h "$BROKER" -u "$USER" -P "$PASS" -t "$TOPIC_MARKERS" 2>/dev/null \
            | jq -c '{n: .count, markers: [.markers[] | {id, dist: .distance_m, fwd: .forward_m, left: .left_m, deg: .bearing_deg}]}'
    else
        mosquitto_sub -h "$BROKER" -u "$USER" -P "$PASS" -t "$TOPIC_MARKERS"
    fi
}

show_setup_help() {
    cat <<'EOF'
┌─────────────────────────────────────────────────────────────┐
│                K-PATROL MARKER BOUNDARY SETUP               │
└─────────────────────────────────────────────────────────────┘

POSITION LAYOUT (viewed from above):

        CORNER 1 ●─ ─ ─ ─ ─ ─ ─ ─ ─● CORNER 2
                 │                 │
                 │       HOME ●    │
                 │                 │
                 │       🤖↑       │   (robot start pose,
                 │      (start)    │    FACING corner 1-2 edge)
                 │                 │
        CORNER 4 ●─ ─ ─ ─ ─ ─ ─ ─ ─● CORNER 3

  ID 0 = HOME        → pillar placed near robot start (0.3-0.6 m in front)
  ID 1 = CORNER 1    → top-left  (forward-left of robot, ~1.2-1.5 m)
  ID 2 = CORNER 2    → top-right (forward-right)
  ID 3 = CORNER 3    → bottom-right (behind robot, right)
  ID 4 = CORNER 4    → bottom-left  (behind robot, left)

  Order of corners matters: 1 → 2 → 3 → 4 is CLOCKWISE from the
  robot's perspective looking forward.

ROBOT PLACEMENT:
  - Robot facing toward the CENTER of the polygon (toward HOME)
  - Within ~30-50 cm of HOME marker so first detection is easy
  - On level ground, clear of obstacles within 25 cm

OBSTACLE DESIGN (optional, for showcase):
  - Small box 15-20 cm tall (ToF sensor height range)
  - Place ~0.8 m in front of robot, slightly offset
  - Must be INSIDE the polygon, not blocking markers

LIGHTING:
  - Even indoor lighting, no strong backlight on markers
  - Avoid direct sunlight on markers (washes out contrast)

VERIFY BEFORE DEMO:
  1.  ./demo_run.sh watch-markers   # confirm ArUco detection works
  2.  ./demo_run.sh monitor         # confirm boundary mode progresses
  3.  drive robot manually until all 5 markers seen
      (boundary.mode should be "polygon", corners: 4, home: true)
  4.  ./demo_run.sh                 # start SMART_COVERAGE
EOF
}

case "${1:-start}" in
    start)
        reset_state
        sleep 0.5
        start_smart_coverage
        echo
        monitor
        ;;
    monitor)
        monitor
        ;;
    markers|watch-markers)
        watch_markers
        ;;
    stop)
        stop_robot
        ;;
    home)
        return_home
        ;;
    reset)
        reset_state
        ;;
    help|setup)
        show_setup_help
        ;;
    *)
        cat <<USAGE
demo_run.sh — K-Patrol demo helper

Commands:
  $(basename "$0")           reset state + start SMART_COVERAGE + monitor
  $(basename "$0") monitor   subscribe nav_status (pretty)
  $(basename "$0") markers   subscribe kpatrol/markers (live detections)
  $(basename "$0") stop      stop robot (MANUAL mode)
  $(basename "$0") home      send robot back to marker 0 (HOME)
  $(basename "$0") reset     reset odometry + grid
  $(basename "$0") help      show setup layout diagram
USAGE
        exit 1
        ;;
esac
