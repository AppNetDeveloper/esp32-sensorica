# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
Multi-Sensor IoT Universal - Professional ESP32 firmware supporting 4 sensor types with web configuration and OTA updates.

## Build and Development Commands

### Core PlatformIO Commands
```bash
# Build firmware
pio run

# Upload firmware to ESP32
pio run --target upload --upload-port /dev/ttyUSB0

# Upload filesystem (web panel HTML)
pio run --target uploadfs

# Clean build
pio run --target clean

# Serial monitor
pio device monitor
```

### OTA Deployment
```bash
# Deploy new version to OTA server
./deploy_script.sh 1.1.0

# This script:
# 1. Copies .pio/build/esp32dev/firmware.bin to versioned file
# 2. Uploads to ota.boisolo.com/multi-sensor-iot/
# 3. Updates version.json with checksum
```

### Firmware Version Management
- Update `FW_VERSION` in `platformio.ini` for each release
- Firmware naming convention: `multi-sensor-iot-{version}.bin`
- OTA server path: `http://ota.boisolo.com/multi-sensor-iot/`

## Architecture Overview

### Hardware Platform
- **WT32-ETH01**: ESP32 with Ethernet + WiFi hybrid capability
- **HC-SR04**: Ultrasonic distance sensor (1-400cm range)
- **Triple LED System**: Status (green), Error (red), Configuration (blue)
- **Configuration Button**: GPIO 12 with multi-mode operation

### Operating Modes
1. **Normal Mode**: Continuous sensor operation with MQTT publishing
2. **Bridge Mode** (2s button press): Ethernet maintained + WiFi AP for config
3. **Hotspot Mode** (5s button press): WiFi-only configuration mode

### Key Architectural Components

#### Configuration System
- **Persistent Storage**: ESP32 Preferences API for non-volatile config
- **Web Panel**: HTML file served from LittleFS (`/data/config.html`)
- **Multi-tab Interface**: Network, MQTT, Device, System configuration
- **Real-time Validation**: Client-side and server-side input validation

#### OTA Update System
- **Automatic Checking**: Every 5 minutes via HTTP to version.json
- **Version Comparison**: Semantic versioning with rollback protection
- **Safety Mechanisms**: Boot counting, checksum verification, automatic rollback
- **Fail-safe**: Only applies updates after successful download and verification

#### Multi-tasking Architecture
```cpp
// FreeRTOS tasks for concurrent operation
- sensorTask(): Ultrasonic sensor readings (50ms intervals)
- mqttTask(): MQTT connection management and publishing
- otaTask(): Periodic update checking
- Web Server: Configuration interface (when in bridge/hotspot mode)
```

#### LED Status Indication
- **Green (GPIO 4)**: System OK - solid when Ethernet + MQTT connected
- **Red (GPIO 5)**: Error indication - blinking when connection issues
- **Blue (GPIO 2)**: Configuration mode - solid in bridge, dual-blink in hotspot

### Data Structures

#### Configuration Structures
```cpp
struct NetworkConfig {
  bool dhcpEnabled;
  String staticIP, gateway, subnet, dns1, dns2;
};

struct MQTTConfig {
  String server, username, password, topic, clientId;
  int port, keepAlive;
};

struct DeviceConfig {
  String deviceName, location;
  int sensorInterval, readingsCount;
  bool debugMode;
};
```

#### System Status Tracking
```cpp
struct SystemStatus {
  unsigned long uptime, lastMQTTConnection, lastSensorReading;
  unsigned int mqttConnectionAttempts, otaUpdatesCount, systemRestarts;
  float currentDistance;
  bool ethConnected, mqttConnected;
  int freeHeap;
};
```

### Web Panel Integration
- **Static Content**: Served from LittleFS (`/config.html`)
- **REST API**: `/api/status` endpoint for real-time system monitoring
- **Form Handling**: POST to `/save` updates persistent configuration
- **Security**: Access only via physical button press (bridge/hotspot modes)

### MQTT Integration
- **Topic Structure**: Configurable, default `medidor/altura`
- **Payload Format**: JSON with distance, device info, and timestamp
- **Connection Management**: Automatic reconnection with exponential backoff
- **QoS Level**: Configurable, default QoS 1

## File Structure Context

### Critical Files
- `src/medidor-altura-ultrasonido.ino`: Main firmware (Multi-Sensor IoT)
- `data/config.html`: Web configuration panel
- `version.json`: OTA update information
- `platformio.ini`: Build configuration with WT32-ETH01 pin mappings

### Build Configuration
- **Filesystem**: LittleFS for web content storage
- **Libraries**: PubSubClient (MQTT), ArduinoJson (JSON parsing), ESP32WebServer
- **Board Flags**: WT32-ETH01 specific Ethernet pin configuration

## Development Patterns

### Configuration Persistence
- All settings stored in ESP32 Preferences namespace "sensor-config"
- Changes survive OTA updates and device restarts
- Factory reset available via web panel

### Error Handling
- Comprehensive logging with structured event format
- Graceful degradation when network/MQTT unavailable
- LED-based error indication for field diagnostics

### Safety Mechanisms
- OTA updates require successful download before installation
- Boot count protection prevents boot loops
- Configuration validation prevents invalid network/MQTT settings

## GPIO Pin Assignments (WT32-ETH01)
- GPIO 25: Ultrasonic Trigger
- GPIO 26: Ultrasonic Echo
- GPIO 12: Configuration Button (pull-up)
- GPIO 2: Configuration LED (blue)
- GPIO 4: Status LED (green)
- GPIO 5: Error LED (red)

## OTA Server Configuration
- **Server**: ota.boisolo.com/multi-sensor-iot/
- **Version File**: `version.json` with firmware metadata
- **Firmware Path**: `/multi-sensor-iot/multi-sensor-iot-{version}.bin`
- **Update Frequency**: Every 5 minutes (300,000ms)