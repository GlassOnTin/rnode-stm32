# Flashing

Firmware uploads go over the existing USB-C cable using a 2KB [HID bootloader](https://github.com/Serasidis/STM32_HID_Bootloader) (driverless on all platforms). SWD is only needed once to flash the bootloader itself.

The `hid-flash` CLI tool is required. PlatformIO does not bundle it — build from [source](https://github.com/Serasidis/STM32_HID_Bootloader/tree/master/cli) (`make` in the `cli/` directory) and install to PATH.

## Local flashing

```bash
./upload.sh          # build, flash, rebind serial port (Linux)
pio run -t upload    # build and flash only (macOS/Windows)
```

On Linux, use `upload.sh` — it runs the upload then rebinds the `cdc_acm` driver so the serial port works immediately. The CDC→HID→CDC USB re-enumeration during upload leaves the kernel tty in a stale state otherwise.

## Remote flashing

When the E22P is connected to a remote Linux host (e.g. an OpenWrt router), use `remote_flash.sh`:

```bash
./remote_flash.sh root@192.168.0.2
```

Or manually:

```bash
pio run -e rnode_e22p
scp -O .pio/build/rnode_e22p/firmware.bin root@host:/tmp/firmware.bin
```

On OpenWrt, `hid-flash` cannot toggle DTR to enter the bootloader (the CDC ACM driver doesn't support DTR ioctl). Use Python to trigger the STM32duino `BL_HID` entry sequence instead:

```bash
ssh root@host 'python3 -c "
import os, fcntl, struct, time, termios, glob
fd = os.open(\"/dev/ttyACM0\", os.O_RDWR | os.O_NOCTTY)
attrs = termios.tcgetattr(fd)
attrs[4] = attrs[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, attrs)
TIOCMBIS, TIOCMBIC, TIOCM_DTR = 0x5416, 0x5417, 0x002
for i in range(4):
    fcntl.ioctl(fd, TIOCMBIS, struct.pack(\"I\", TIOCM_DTR))
    time.sleep(0.05)
    fcntl.ioctl(fd, TIOCMBIC, struct.pack(\"I\", TIOCM_DTR))
    time.sleep(0.05)
os.write(fd, b\"1EAF\")
os.close(fd)
for i in range(50):
    time.sleep(0.1)
    for d in glob.glob(\"/sys/bus/usb/devices/*/idVendor\"):
        base = os.path.dirname(d)
        try:
            vid = open(d).read().strip()
            pid = open(os.path.join(base, \"idProduct\")).read().strip()
            if vid == \"1209\" and pid == \"beba\":
                raise SystemExit(0)
        except SystemExit: raise
        except: pass
raise SystemExit(1)
"'
ssh root@host '/tmp/hid-flash /tmp/firmware.bin ttyACM0'
```

The bootloader entry works by toggling DTR 4 times (STM32duino's `DTR_TOGGLING_SEQ` counter) then sending the `1EAF` magic string, which triggers `BL_HID` to write a magic value to the backup register and reset into HID bootloader mode.

The remote host needs `hid-flash` (build from source for its architecture) and USB HID kernel support. On OpenWrt this means installing `kmod-usb-hid` and `kmod-hid-generic` — without them the bootloader device (1209:BEBA) is invisible and `hid-flash` fails silently.

## One-time bootloader setup (SWD)

The E22P board has SWD pads (SWDIO, SWCLK) accessible via CON2. Flash the HID bootloader once:

```bash
# Download and extract bootloader (STM32F103 low/medium density, LED on PC13)
wget -O /tmp/stm32_binaries.zip \
  https://github.com/Serasidis/STM32_HID_Bootloader/releases/download/2.2.2/stm32_binaries.zip
unzip -j /tmp/stm32_binaries.zip \
  "stm32_binaries/F103/low_and_medium_density/hid_generic_pc13.bin" -d /tmp

# Flash bootloader at 0x08000000
openocd -f interface/jlink.cfg -c "transport select swd" \
  -f target/stm32f1x.cfg -c "reset_config none separate" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase /tmp/hid_generic_pc13.bin 0x08000000" \
  -c "reset run" -c "shutdown"

# Flash firmware at 0x08000800 (first time only, subsequent uploads use USB)
openocd -f interface/jlink.cfg -c "transport select swd" \
  -f target/stm32f1x.cfg -c "reset_config none separate" \
  -c "init" -c "reset halt" \
  -c "flash write_image erase .pio/build/rnode_e22p/firmware.bin 0x08000800" \
  -c "reset run" -c "shutdown"
```

## Flash layout

```
0x08000000  HID Bootloader (2 KB)
0x08000800  RNode Firmware  (62 KB available)
0x08010000  End of flash
```

## Notes

- **JLink via PlatformIO often fails** on this board because the nRF5340-DK's onboard J-Link cannot assert the STM32's NRST pin over the SWD wires alone. Use the **openocd commands above** with `reset_config none separate` which avoids the reset pin dependency.
- The factory firmware has **read protection (RDP)** enabled. You must unlock it first, which triggers a mass erase:
  ```bash
  openocd -f interface/jlink.cfg -c "transport select swd" \
    -f target/stm32f1x.cfg -c "reset_config none separate" \
    -c "init; reset halt; flash protect 0 0 last off; reset halt; flash write_image erase firmware.bin 0x08000000; reset run; shutdown"
  ```
- An **nRF5340-DK** (or any board with an onboard J-Link) can be used as an SWD programmer by routing its debug-out header to the E22P.
- SWD can still flash firmware directly to `0x08000800` as a fallback if USB upload is unavailable.
- **OpenWrt/embedded Linux hosts** need `kmod-usb-hid` and `kmod-hid-generic` installed. The HID bootloader enumerates as a USB HID device (1209:BEBA) — without these modules the kernel can't bind it and `hid-flash` silently fails.

## Linux udev rules

The STM32's USB CDC implementation requires `hupcl` to be disabled on the tty, otherwise the kernel drops DTR on port close and the tty becomes unresponsive until rebound. Install these rules to fix this automatically:

```bash
# HID bootloader permissions (for upload without sudo)
sudo tee /etc/udev/rules.d/99-stm32-hid-bootloader.rules << 'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="beba", MODE="0666"
EOF

# CDC serial port fix (disable hupcl to prevent stale tty)
sudo tee /etc/udev/rules.d/99-stm32-cdc.rules << 'EOF'
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", RUN+="/bin/stty -F /dev/%k -hupcl"
EOF

sudo udevadm control --reload-rules
```
