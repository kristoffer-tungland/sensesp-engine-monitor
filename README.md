# SensESP Engine Monitoring System

ESP32 + SensESP firmware for publishing engine telemetry to Signal K:

- 4x DS18B20 temperatures
- Oil pressure (INA219, bar)
- Fuel level (INA219, % as 0.0-1.0)
- Engine RPM (tach pulse input)
- Engine running state
- Engine hours

## Pinout

| Sensor | Type | GPIO Pin | Connection Details |
|---|---|---:|---|
| DS18B20 (x4) | 1-Wire | 25 | Data line with 4.7k pull-up resistor |
| Oil INA219 | I2C | 21 (SDA), 22 (SCL) | INA219 at `0x40` |
| Fuel INA219 | I2C | 21 (SDA), 22 (SCL) | INA219 at `0x41` |
| Tachometer input | Pulse | 26 | Via optocoupler, rising edge count |

## Build and flash

```bash
python3 -m platformio run
python3 -m platformio run --target upload
python3 -m platformio device monitor
```

## Signal K paths

- `propulsion.engine.temperature.rawWater` (K)
- `propulsion.engine.temperature.alternator` (K)
- `propulsion.engine.temperature.coolant` (K)
- `propulsion.engine.temperature.thermostat` (K)
- `propulsion.engine.oilPressure` (bar)
- `tanks.fuel.level` (ratio 0.0-1.0)
- `propulsion.engine.revolutions` (rpm)
- `propulsion.engine.isRunning` (bool)
- `propulsion.engine.hours` (hours)

## Notes

- Oil/fuel conversion constants are in `src/main.cpp` and should be calibrated to your senders.
- SensESP networking is configured through the standard SensESP provisioning workflow.

