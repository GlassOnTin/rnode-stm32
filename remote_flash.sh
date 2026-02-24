#!/bin/bash
# Flash firmware to E22P on remote OpenWrt router over SSH.
#
# The router's CDC ACM driver doesn't support DTR ioctl via hid-flash,
# so we use a Python helper to trigger the HID bootloader, then flash.
#
# Usage:  ./remote_flash.sh [host] [env]
#         ./remote_flash.sh root@192.168.0.2 diag_e22p

set -euo pipefail

HOST="${1:-root@192.168.0.2}"
ENV="${2:-rnode_e22p}"
FW=".pio/build/$ENV/firmware.bin"

if [ ! -f "$FW" ]; then
    echo "Firmware not found. Building..."
    pio run -e "$ENV"
fi

echo "Stopping rnsd..."
ssh "$HOST" '/etc/init.d/rnsd stop' || true

echo "Copying firmware..."
scp -O "$FW" "$HOST":/tmp/firmware.bin

echo "Entering HID bootloader..."
ssh "$HOST" 'python3 -c "
import os, fcntl, struct, time, termios, glob

fd = os.open(\"/dev/ttyACM0\", os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[4] = termios.B115200
attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)

TIOCMBIS = 0x5416
TIOCMBIC = 0x5417
TIOCM_DTR = 0x002

# Toggle DTR 4 times (STM32duino counts toggles, triggers on >3)
for i in range(4):
    fcntl.ioctl(fd, TIOCMBIS, struct.pack(\"I\", TIOCM_DTR))
    time.sleep(0.05)
    fcntl.ioctl(fd, TIOCMBIC, struct.pack(\"I\", TIOCM_DTR))
    time.sleep(0.05)

# Send 1EAF magic to activate BL_HID reset (writes backup register + NVIC_SystemReset)
os.write(fd, b\"1EAF\")
os.close(fd)

# Wait for HID bootloader (1209:BEBA)
for i in range(50):
    time.sleep(0.1)
    for d in glob.glob(\"/sys/bus/usb/devices/*/idVendor\"):
        base = os.path.dirname(d)
        try:
            vid = open(d).read().strip()
            pid = open(os.path.join(base, \"idProduct\")).read().strip()
            if vid == \"1209\" and pid == \"beba\":
                print(f\"HID bootloader ready ({(i+1)*0.1:.1f}s)\")
                raise SystemExit(0)
        except SystemExit:
            raise
        except:
            pass
print(\"ERROR: HID bootloader not found after 5s\")
raise SystemExit(1)
"'

echo "Flashing..."
ssh "$HOST" '/tmp/hid-flash /tmp/firmware.bin ttyACM0'

if [ "$ENV" = "rnode_e22p" ]; then
    echo "Starting rnsd..."
    sleep 3
    ssh "$HOST" '/etc/init.d/rnsd start'
fi

echo "Done. Flashed $ENV."
