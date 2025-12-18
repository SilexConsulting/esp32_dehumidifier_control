# Dehumidifier Control

An ESP32-based controller for dehumidifiers, featuring a robust Finite State Machine (FSM) to handle compressor protection, defrost cycles, and web-based monitoring/configuration.

## Overview

This project replaces the control board of a standard dehumidifier. It uses an ESP32 Relay DevKit to manage the compressor and fan based on temperature and demand sensors that are part of the dehumidifier.

### Key Features
- **Compressor Protection**: Anti-short-cycle timer to prevent frequent restarts.
- **Fan Management**: Configurable fan lead (starts after compressor) and lag (continues after compressor stops) times.
- **Defrost Logic**: Automatic defrost mode based on evaporator temperature.
- **Timed Fallback**: If the temperature sensor fails, the system falls back to a timed run/rest cycle.
- **Web Interface**: Real-time monitoring of temperature, states, and runtime statistics.
- **OTA Updates**: Update firmware over-the-air via the web interface.
- **WiFi Manager**: Easy WiFi setup via an Access Point portal.
- **Statistics**: All-time and session-based tracking of compressor and fan runtimes.

## Project Structure

```text
.
├── include/            # Project headers (standard PIO)
├── lib/                # Local libraries
├── src/
│   ├── main.cpp        # Entry point, Web Server, and App logic
│   ├── fsm.cpp/h       # Dehumidifier Finite State Machine
│   ├── lfs.h           # LittleFS helper for persistent storage
│   └── serial_setup.h  # Serial communication initialization
├── test/               # Unit tests (standard PIO)
├── platformio.ini      # PlatformIO configuration
└── LICENSE             # Project license
```

## Requirements

### Hardware
- **Microcontroller**: ESP32 (Tested on [ESP32 2 Channel Relay Development Board](https://www.amazon.co.uk/dp/B0DS6SB1XB))
- **Sensors**:
    - NTC Thermistor for evaporator temperature.
    - Dry contact for Humidity Demand (e.g., humidistat).
    - Dry contact for Bucket/Tank Full sensor.
- **Actuators**:
    - Relays for Compressor and Fan control.
    - Status LEDs (Running, Defrost, Bucket Full).

### Software
- [CLion IDE](https://www.jetbrains.com/clion/)
- [Platform IO](https://platformio.org/)

## Hardware Mapping (Default GPIOs)

| Function | GPIO | Notes                                   |
| :--- | :--- |:----------------------------------------|
| **Compressor Relay** | 16 | Output                                  |
| **Fan Relay** | 17 | Output                                  |
| **Humidity Demand** | 25 | Input (Pull-up, Closed to GND = Demand) |
| **Bucket Sensor** | 26 | Input (Pull-up, Closed to GND = OK)     |
| **Temp Sensor (NTC)**| 35 | ADC Input (Biased with 12k resistor)  |
| **Running LED** | 15 | Output                                  |
| **Defrost LED** | 4  | Output                                  |
| **Bucket LED** | 19 | Output                                  |
| **Boot Button** | 0  | Standard ESP32 boot pin                 |

## Setup & Run

### 1. Build and Upload
Connect your ESP32-EVB and use the Pio Pluign to build and upload the firmware. You need to hold the Boot button and reset the board to enter bootloader mode to upload the firmware.

### 2. WiFi Configuration
On the first boot, or if the saved WiFi is unavailable, the ESP32 will start an Access Point named `ESP32-Dehumidifier` (or similar). Connect to it and follow the portal to configure your WiFi credentials.

### 3. Web Interface
Once connected to your network, find the ESP32's IP address (visible in the Serial monitor at 115200 baud).
- **Dashboard**: `http://<device-ip>/`
- **OTA Update**: `http://<device-ip>/update`
- **Status API**: `http://<device-ip>/api/status`

## Configuration

Most parameters can be adjusted via the Web UI:
- **Defrost Stop Temp**: Temperature at which defrost ends.
- **Defrost Restart Temp**: Temperature at which normal operation resumes.
- **Fallback Timing**: Run/Rest durations if the sensor fails.

## Finite State Machine (FSM) States

- `ST_IDLE`: No demand or bucket is full.
- `ST_ANTI_SHORT`: Waiting for the minimum compressor-off time to expire.
- `ST_STARTING_COMP`: Compressor is on, waiting for fan lead time.
- `ST_RUNNING`: Both compressor and fan are running.
- `ST_DEFROST`: Compressor off, fan running to clear ice.
- `ST_FAN_LAG`: Compressor off, fan continues for the lag duration.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
**TODO**:
- [ ] Document specific NTC Steinhart-Hart coefficients used.
- [ ] Add circuit diagram/schematic link.
- [ ] Add unit tests for FSM transitions.
