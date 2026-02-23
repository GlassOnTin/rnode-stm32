#!/bin/bash
# Upload firmware via USB HID bootloader, then rebind cdc_acm so the
# serial port works immediately (the CDC→HID→CDC re-enumeration leaves
# the kernel tty in a stale state on Linux).

set -euo pipefail

pio run -t upload

# Find the cdc_acm interface path for VID 0483:PID 5740
# Wait up to 10s for the device to re-enumerate after upload
INTF=""
for i in $(seq 1 20); do
    for p in /sys/bus/usb/drivers/cdc_acm/*:1.0; do
        [ -e "$p" ] || continue
        realp=$(readlink -f "$p")
        vid=$(cat "$realp/../idVendor" 2>/dev/null)
        pid=$(cat "$realp/../idProduct" 2>/dev/null)
        if [ "$vid" = "0483" ] && [ "$pid" = "5740" ]; then
            INTF=$(basename "$p")
            break 2
        fi
    done
    sleep 0.5
done

if [ -z "$INTF" ]; then
    echo "Warning: could not find STM32 CDC device to rebind"
    exit 0
fi

echo "Rebinding cdc_acm ($INTF)..."
sudo -A sh -c "echo '$INTF' > /sys/bus/usb/drivers/cdc_acm/unbind"
sleep 0.5
sudo -A sh -c "echo '$INTF' > /sys/bus/usb/drivers/cdc_acm/bind"
sleep 2
echo "Done — serial port ready"
