#!/bin/bash
# Wrapper: rebind cdc_acm driver before running rnodeconf.
#
# The Linux cdc_acm driver has a bug where read_urbs_free bitmap is not
# properly reset between port close/reopen, causing FIONREAD to return 0
# even though data arrives at the USB level. Rebinding the driver forces
# acm_probe to reinitialize the bitmap.
#
# Usage: ./rnodeconf-rebind.sh [rnodeconf args...]
#   e.g. ./rnodeconf-rebind.sh /dev/ttyACM2 --info

set -euo pipefail

# Auto-detect the STM32 CDC ACM interface (VID 0483, PID 5740)
find_intf() {
    for p in /sys/bus/usb/drivers/cdc_acm/*:1.0; do
        [ -e "$p" ] || continue
        realp=$(readlink -f "$p")
        vid=$(cat "$realp/../idVendor" 2>/dev/null)
        pid=$(cat "$realp/../idProduct" 2>/dev/null)
        if [ "$vid" = "0483" ] && [ "$pid" = "5740" ]; then
            basename "$p"
            return 0
        fi
    done
    return 1
}

INTF=$(find_intf) || { echo "Error: STM32 CDC device not found"; exit 1; }

# Unbind
sudo -A sh -c "echo '$INTF' > /sys/bus/usb/drivers/cdc_acm/unbind" 2>/dev/null || true
sleep 0.3

# Rebind
sudo -A sh -c "echo '$INTF' > /sys/bus/usb/drivers/cdc_acm/bind"
sleep 0.3

# Wait for tty device to appear (up to 3s)
PORT="${1:-/dev/ttyACM2}"
for _ in $(seq 1 30); do
    [ -e "$PORT" ] && break
    sleep 0.1
done

if [ ! -e "$PORT" ]; then
    echo "Error: $PORT did not appear after rebind"
    exit 1
fi

exec rnodeconf "$@"
