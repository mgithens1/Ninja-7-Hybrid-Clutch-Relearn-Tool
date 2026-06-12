#!/bin/bash
# Kawasaki Ninja 7 Hybrid Clutch Relearn Tool
# Runs the UDS diagnostic session for clutch calibration

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$(dirname "$SCRIPT_DIR")/Downloads/venv"

# Try common locations for the venv
for dir in "$HOME/Downloads" "$SCRIPT_DIR"; do
    if [ -d "$dir/venv/bin" ]; then
        VENV_DIR="$dir/venv"
        break
    fi
done

echo "=== Kawasaki Ninja 7 Hybrid Clutch Relearn ==="
echo ""

# Activate venv
if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "[ERROR] Python venv not found. Create it first:"
    echo "  cd ~/Downloads"
    echo "  python3 -m venv venv"
    echo "  source venv/bin/activate"
    echo "  pip install python-can"
    exit 1
fi

source "$VENV_DIR/bin/activate"

# Check if CAN interface exists
if ! ip link show can0 &>/dev/null; then
    echo "[INFO] can0 not found — attempting to bring it up..."
    echo "[INFO] Make sure your UCAN adapter is plugged in."
    sudo ip link set can0 up type can bitrate 500000
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to bring up can0. Check that the UCAN adapter is connected."
        exit 1
    fi
    echo "[OK] can0 is up at 500kbps"
else
    # Check if it's already UP
    STATE=$(ip -br link show can0 2>/dev/null | awk '{print $2}')
    if [ "$STATE" != "UP" ]; then
        echo "[INFO] can0 exists but is down — bringing it up..."
        sudo ip link set can0 up type can bitrate 500000
        echo "[OK] can0 is up at 500kbps"
    else
        echo "[OK] can0 is already up at 500kbps"
    fi
fi

echo ""
echo "=== Checklist ==="
echo "  1. Engine is OFF"
echo "  2. Key switch is ON"
echo "  3. Kill switch is set to RUN"
echo "  4. Instrument cluster is fully active"
echo ""
echo "Starting diagnostic session..."
echo ""

# Run the script
python3 ~/Downloads/kawasaki_clutch_relearn.py

# Clean shutdown
echo ""
echo "=== Remember ==="
echo "Turn the key switch to OFF to write calibration values to ECU memory."
echo ""