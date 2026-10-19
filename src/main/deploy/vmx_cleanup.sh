#!/bin/bash
# VMX storage / vision check and cleanup. Run over ssh on the robot:
#
#   bash /home/lvuser/deploy/vmx_cleanup.sh            # only show the state
#   bash /home/lvuser/deploy/vmx_cleanup.sh --clean    # + clear our logs/temp
#   bash /home/lvuser/deploy/vmx_cleanup.sh --clean --system
#                                  # + system journal and package cache (sudo)
#   bash /home/lvuser/deploy/vmx_cleanup.sh --restart-vision
#                                  # kill the Python vision; the robot code
#                                  # starts it again within a few seconds
#
# The robot program also trims these logs by itself every 30 s; this script
# is for when the storage is already full and the program cannot start.

HOME_DIR=/home/lvuser
CLEAN=0
SYSTEM=0
RESTART=0
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --system) SYSTEM=1 ;;
    --restart-vision) RESTART=1 ;;
    *) echo "unknown option: $arg"; exit 1 ;;
  esac
done

echo "== Storage =="
df -h / "$HOME_DIR" 2>/dev/null | awk '!seen[$0]++'
echo
echo "== Biggest items in $HOME_DIR =="
du -sh "$HOME_DIR"/* "$HOME_DIR"/.[!.]* 2>/dev/null | sort -h | tail -10
echo
echo "== Biggest system logs =="
du -sh /var/log/* 2>/dev/null | sort -h | tail -5
echo
echo "== Memory (RAM) =="
free -h 2>/dev/null
echo
echo "== Vision process =="
if pgrep -af vision_aruco.py >/dev/null; then
  pgrep -af vision_aruco.py
else
  echo "NOT running"
fi
if [ -f "$HOME_DIR/vision_data.json" ]; then
  age=$(( $(date +%s) - $(stat -c %Y "$HOME_DIR/vision_data.json") ))
  echo "vision_data.json updated ${age}s ago (should be 0-1 s)"
fi
echo "Last lines of vision_aruco.log:"
tail -5 "$HOME_DIR/vision_aruco.log" 2>/dev/null
echo

if [ "$CLEAN" = 1 ]; then
  echo "== Cleaning our files =="
  : > "$HOME_DIR/vision_aruco.log" 2>/dev/null && echo "vision_aruco.log cleared"
  rm -f "$HOME_DIR/pickup_log.old.csv"
  if [ -f "$HOME_DIR/pickup_log.csv" ]; then
    tail -c 200000 "$HOME_DIR/pickup_log.csv" > "$HOME_DIR/pickup_log.old.csv"
    rm -f "$HOME_DIR/pickup_log.csv"
    echo "pickup_log.csv -> pickup_log.old.csv (last 200 KB kept)"
  fi
  find "$HOME_DIR" -maxdepth 1 -name '.vision_data.json.*.tmp' -delete
  find "$HOME_DIR/deploy" -maxdepth 1 -name '.color_ranges.*.tmp' -delete 2>/dev/null
  find "$HOME_DIR" -maxdepth 2 -name 'core' -type f -delete 2>/dev/null
  echo "temp files removed"
fi

if [ "$SYSTEM" = 1 ]; then
  echo "== Cleaning system logs and package cache (sudo) =="
  sudo journalctl --vacuum-size=50M 2>/dev/null
  sudo find /var/log -name '*.gz' -delete 2>/dev/null
  sudo find /var/log -name '*.[0-9]' -delete 2>/dev/null
  sudo apt-get clean 2>/dev/null
fi

if [ "$RESTART" = 1 ]; then
  echo "== Restarting vision =="
  pkill -9 -f vision_aruco.py && echo "vision killed; the robot code restarts it"
fi

if [ "$CLEAN" = 1 ] || [ "$SYSTEM" = 1 ]; then
  echo
  echo "== Storage after cleanup =="
  df -h / 2>/dev/null
fi
