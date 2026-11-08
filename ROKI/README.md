# ROKI

ESP-NOW LR telemetry link for two Seeed Studio XIAO ESP32-C3 boards.

## Structure

- `transmitter/` — autonomous mobile board firmware. It detects supported IMU, barometer and temperature/humidity sensors through I2C and sends telemetry over ESP-NOW LR.
- `receiver/` — USB-connected ground board firmware with OLED status display.
- `monitor/` — Windows ground-station application, telemetry logging, playback, calibration UI and 3D orientation/trajectory view.
- `common/` — radio protocol shared by both ESP32 firmwares.

Build the transmitter or receiver with PlatformIO from its respective directory. Install the monitor dependency once with `py -m pip install -r monitor/requirements.txt`, then run `monitor/launch_roki_monitor.vbs`. The launcher resolves its own directory and has no machine-specific paths.

Per-PC monitor settings and routine field logs are not committed. One selected
reference CSV is tracked to reproduce the recorded barometric noise and radio
loss regression tests.

## Experimental estimation and operator display

- **Vertical velocity** is a fused IMU/barometer **estimate**, not an independent
  calibrated speed sensor. The OLED receiver shows its sign, quality
  (`VALID`, `DEGR`, `INVALID`), barometer readiness and ZUPT state; its second
  page (alternating every five seconds) shows radio/USB diagnostics.
- **ZUPT** requires a quiet gyro/accelerometer, short-term barometric stability,
  and a roughly four-second pressure-height trend below 0.035 m/s. This reduces
  false zero-velocity updates during slow constant climbs/descents. It cannot
  distinguish true rest from *arbitrarily slow* constant motion; wind or a
  pressure transient may temporarily prevent ZUPT at rest.
- **Horizontal velocity and position cannot be independently determined** from
  IMU+barometer alone over an extended interval: accelerometer bias and
  attitude error accumulate into drift. The monitor therefore does not draw a
  horizontal dead-reckoned track by default; it is available only via the
  clearly marked experimental checkbox. The confirmed 3D **orientation**
  display continues to operate without that checkbox. No reliable 3D speed is
  exported in the radio protocol.
- **Playback timing** uses the board's monotonic `tx_ms` when available,
  preserving sample cadence even if USB batches packets. Host timestamps are
  used at transmitter restarts or when TX timing is unavailable.
- **Diagnostics:** the monitor shows both raw and calibrated acceleration norms
  in g on the calibration tab. At rest the calibrated vector should be near
  1 g. The USB serial parser counts and exposes malformed input lines without
  terminating the receive loop.

## Acceptance tests

From the repository root, without connecting hardware:

```sh
python -m unittest discover -s ROKI/tests -p 'test_*.py' -v
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I ROKI/common ROKI/tests/test_baro_stillness.cpp -o roki_baro_test
./roki_baro_test
```

Build each firmware from its own PlatformIO directory (`pio run`). Flashing
and on-device measurements are **separate acceptance steps**; successful
mathematical simulations and desktop builds do not establish flight accuracy.
After flashing both boards, verify startup, the OLED's two views, a 10-second
stationary segment, a deliberately slow ascent/descent, yaw direction, restart
recovery and packet-loss counters. Preserve the CSV and firmware commit SHA
for every trial. Avoid treating IMU-estimated velocity as a safety-critical
navigation measurement without independent validation.
