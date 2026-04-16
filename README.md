# SensESP Engine Monitor (ESPHome -> Signal K)

This repository contains a starter SensESP firmware that publishes common engine
signals to Signal K:

- `propulsion.main.temperature` (Kelvin)
- `propulsion.main.oilPressure` (Pa)
- `propulsion.main.revolutions` (Hz)

## What this implements

To transition from ESPHome to Signal K with SensESP, this repository now
includes:

1. A PlatformIO project configured for ESP32 + SensESP.
2. A firmware scaffold (`/src/main.cpp`) with analog and RPM sensor pipelines.
3. Build/run instructions for producing firmware.

## Information needed to complete your exact migration

To fully replicate your existing ESPHome setup, provide:

- The blog post describing the ESPHome solution.
- The current ESPHome YAML/config files.

Those details are needed to map calibration, pin assignments, update rates, and
exact Signal K paths.

## Build firmware

Prerequisites:

- Python 3
- PlatformIO CLI (`pip install platformio`)

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Serial monitor:

```bash
pio device monitor
```
