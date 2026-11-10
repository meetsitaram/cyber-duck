#!/bin/bash
# On a fresh Raspberry Pi OS Lite, 64-bit, in the folder holding this file. Not yet tried on a real Pi.
set -e
sudo apt-get update
sudo apt-get install -y python3-venv i2c-tools
sudo raspi-config nonint do_i2c 0
sudo usermod -aG dialout,i2c "$USER"
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt adafruit-blinka adafruit-circuitpython-bno08x
# The bus board is on the Pi's UART (GPIO 14 and 15, pins 8 and 10), not USB: the full UART (PL011) on those pins, and no login console on it.
# Bluetooth stays on, for a gamepad, moved to the mini UART (miniuart-bt): a gamepad sends little, and the mini UART can't hold the servos' 1 Mbaud.
# miniuart-bt needs a fixed core clock for the mini UART's baud: core_freq=250 (a little slower GPU and SDRAM, nothing the controller uses)
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
sudo sed -i '/^dtoverlay=disable-bt/d' /boot/firmware/config.txt
for line in 'enable_uart=1' 'dtoverlay=miniuart-bt' 'core_freq=250'; do
  grep -q "^$line" /boot/firmware/config.txt || echo "$line" | sudo tee -a /boot/firmware/config.txt
done
sudo systemctl enable hciuart 2>/dev/null || true
echo 'Done. Reboot, then: ls -l /dev/serial0 (it should point at ttyAMA0) and i2cdetect -y 1 (the BNO085 is at 0x4a or 0x4b). For a gamepad: bluetoothctl, then scan on, pair, trust and connect.'
