# SensESP Engine Monitor (ESPHome -> Signal K)

This repository contains a starter SensESP firmware that publishes common engine
signals to Signal K:

- `propulsion.main.coolantTemperature` (Kelvin)
- `propulsion.main.oilPressure` (Pa)
- `propulsion.main.revolutions` (Hz)
- `propulsion.main.auxiliaryTemperature.rawWaterElbow` (Kelvin, DS18B20)
- `electrical.alternators.main.temperature` (Kelvin, DS18B20)
- `propulsion.main.auxiliaryTemperature.coolantArea` (Kelvin, DS18B20)
- `propulsion.main.auxiliaryTemperature.thermostat` (Kelvin, DS18B20)

Current defaults are intentionally generic and should be calibrated for your
hardware (for example: coolant volts->°C slope, oil pressure volts->Pa slope,
and tachometer pulses-per-revolution).

## What this implements

To transition from ESPHome to Signal K with SensESP, this repository now
includes:

1. A PlatformIO project configured for ESP32 + SensESP.
2. A firmware scaffold (`/src/main.cpp`) with analog and RPM sensor pipelines.
3. Build/run instructions for producing firmware.

## Pinout diagram

```text
ESP32 pinout used by this project

GPIO34  <- Analog coolant temperature input
GPIO35  <- Analog oil pressure input
GPIO4   <- RPM pulse input
GPIO25  <-> DS18B20 1-Wire data bus
3V3     -> DS18B20 VCC (all probes)
GND     -> DS18B20 GND (all probes)

4.7k pull-up resistor between GPIO25 (data) and 3V3
```

## DS18B20 setup (from the ESPHome example)

- 1-Wire data pin: `GPIO25`
- Update interval: `10s`
- Preconfigured addresses in `src/main.cpp`:
  - `0x2c3c18e381b14128` (Temperature 1 / Raw-water elbow)
  - `0xf43c9ae38186c228` (Temperature 2 / Alternator)
  - `0x5c3c09e3810b9328` (Temperature 3 / Coolant area)
  - `0x443c7de381c5a828` (Temperature 4 / Thermostat)

If your probes have different addresses, update the
`kDallasSensorAddresses[]` array in `src/main.cpp`.

## Information needed to complete your exact migration

To fully replicate your existing ESPHome setup, provide:

- Documentation describing your ESPHome solution (blog post, notes, or design).
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
