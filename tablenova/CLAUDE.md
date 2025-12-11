# CLAUDE.md - Project Development Guide

This file provides essential guidance for Claude Code when working with the Multi-Sensor IoT Universal project.

## Project Overview
Multi-Sensor IoT Universal - Professional ESP32 firmware supporting 4 sensor types with dual Ethernet/WiFi connectivity, web configuration, and OTA updates.

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
./deploy_script_ftp.sh 1.1.0

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
2. **Bridge Mode** (3s button press): Ethernet maintained + WiFi AP for config
3. **Hotspot Mode** (10s button press): WiFi-only configuration mode
4. **Auto-Hotspot**: Automatic hotspot if no previous configuration exists

### Connection Modes
- **MODE_ETHERNET**: Ethernet only
- **MODE_WIFI**: WiFi only
- **MODE_DUAL_ETH_WIFI**: Ethernet primary + WiFi backup

### Key Architectural Components

#### Configuration System
- **Persistent Storage**: ESP32 Preferences API (survives OTA updates)
- **Web Panel**: HTML file served from LittleFS (`/data/config.html`)
- **Multi-tab Interface**: Network, WiFi, Connection, MQTT, Device, Sensor, System
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

struct WiFiConfig {
  String ssid, password;
  bool enabled;
  int channel;
  bool hidden;
};

struct MQTTConfig {
  String server, username, password, topic, clientId;
  int port, keepAlive;
};

struct DeviceConfig {
  String deviceName, location;
  int sensorInterval, readingsCount;
  bool debugMode;
  int sensorType; // 0=ultrasonido, 1=1 botón, 2=2 botones, 3=vibración
  int connectionMode; // 0=Ethernet, 1=WiFi, 2=Dual
  // ... additional sensor-specific configs
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
- **Topic Structure**: Configurable, default `multi-sensor/iot/`
- **Payload Format**: JSON with device info, sensor type, timestamp
- **Connection Management**: Automatic reconnection with exponential backoff
- **QoS Level**: Configurable, default QoS 1

## File Structure Context

### Critical Files
- `src/multi-sensor-iot.ino`: Main firmware (renamed from medidor-altura-ultrasonido.ino)
- `data/config.html`: Web configuration panel with 7 tabs
- `version.json`: OTA update information
- `platformio.ini`: Build configuration with WT32-ETH01 pin mappings
- `README.md`: Comprehensive documentation (consolidated from all .md files)

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
```
GPIO 25: Ultrasonic Trigger / Pulsador 1
GPIO 26: Ultrasonic Echo / Pulsador 2
GPIO 12: Configuration Button (pull-up)
GPIO 2: Configuration LED (blue)
GPIO 4: Status LED (green)
GPIO 5: Error LED (red)
GPIO 13: Pulsador 1 (alternativo)
GPIO 14: Pulsador 2 (alternativo)
GPIO 32: Sensor de vibraciones SW-420
```

## OTA Server Configuration
- **Server**: ota.boisolo.com/multi-sensor-iot/
- **Version File**: `version.json` with firmware metadata
- **Firmware Path**: `/multi-sensor-iot/multi-sensor-iot-{version}.bin`
- **Update Frequency**: Every 5 minutes (300,000ms)

## Key Features Implementation

### Auto-Hotspot Mode
- Detects when no WiFi or MQTT configuration exists
- Automatically enters hotspot mode for first-time setup
- Creates "ESP32-Hotspot" AP with captive portal

### Connection Mode Management
- Real-time connection monitoring every 10 seconds
- Automatic failover in dual mode
- Web interface allows mode switching without reboot

### Persistent Configuration
- Uses ESP32 Preferences API with namespace "sensor-config"
- All settings survive OTA updates
- Auto-backup on configuration changes

### Multi-Sensor Support
- 4 sensor types supported via enum
- Dynamic pin configuration based on sensor type
- Sensor-specific MQTT topics and payload formats

## Testing and Deployment

### Local Testing
```bash
# Build and test locally
pio run
pio device monitor

# Test web interface
# Enter bridge mode (3s button press)
# Connect to ESP32-Bridge WiFi
# Access http://192.168.4.1
```

### OTA Deployment Testing
```bash
# Deploy to test server
./deploy_script_ftp.sh test-version

# Verify HTTP access
curl -I http://ota.boisolo.com/multi-sensor-iot/version.json
curl -I http://ota.boisolo.com/multi-sensor-iot/multi-sensor-iot-test-version.bin
```

### Production Deployment
```bash
# Deploy production version
./deploy_script_ftp.sh 1.1.0

# Monitor OTA logs on devices
# Check for automatic updates within 5 minutes
```

## Important Notes

- **Bluetooth Removed**: Eliminated to optimize flash space (firmware now 79.9% of flash)
- **Auto-Hotspot**: New feature for zero-configuration setup
- **Connection Modes**: Three modes (Ethernet, WiFi, Dual) with web control
- **Documentation Consolidated**: All .md files merged into comprehensive README.md
- **Firmware Renamed**: Changed from medidor-altura-ultrasonido.ino to multi-sensor-iot.ino

## Current Status
- ✅ Compilation successful (1.04MB of 1.31MB flash)
- ✅ WiFi + Ethernet control implemented
- ✅ Web interface updated with connection modes
- ✅ Auto-hotspot mode implemented
- ✅ Persistent configuration across OTA
- ✅ File renamed and documentation consolidated