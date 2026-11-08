# ROKI

ESP-NOW LR telemetry link for two Seeed Studio XIAO ESP32-C3 boards.

## Structure

- `transmitter/` — autonomous mobile board firmware. It detects supported IMU, barometer and temperature/humidity sensors through I2C and sends telemetry over ESP-NOW LR.
- `receiver/` — USB-connected ground board firmware with OLED status display.
- `monitor/` — Windows ground-station application, telemetry logging, playback, calibration UI and 3D orientation/trajectory view.
- `common/` — radio protocol shared by both ESP32 firmwares.

Build the transmitter or receiver with PlatformIO from its respective directory. Run `monitor/launch_roki_monitor.vbs` to start the Windows monitor.

Field logs and per-PC monitor settings are deliberately not committed.
