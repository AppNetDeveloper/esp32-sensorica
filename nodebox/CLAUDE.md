# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
Multi-Sensor IoT Universal - Professional ESP32 firmware for WT32-ETH01 with 4 sensor types support, dual Ethernet/WiFi connectivity, web configuration, and automatic OTA updates.

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
# Deploy new version (auto-increments patch version)
./deploy_script_ftp.sh

# Or using Python script (includes filesystem support)
python3 deploy_ota.py
```

The deploy script:
1. Reads current version from `platformio.ini` (FW_VERSION)
2. Auto-increments patch version (X.Y.Z -> X.Y.Z+1)
3. Updates version in `platformio.ini` and `multi-sensor-iot.ino`
4. Runs clean build
5. Uploads firmware to `ota.boisolo.com/multi-sensor-iot/`
6. Creates and uploads `version.json` with SHA256 checksum

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

#### FreeRTOS Task Architecture
```cpp
// Core 0 - Sensor Task (10KB stack, priority 1)
xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);

// Core 1 - Network & OTA Tasks (10KB stack each)
xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 10000, NULL, 1, NULL, 1);
xTaskCreatePinnedToCore(otaTask, "OTA Task", 10000, NULL, 2, NULL, 1);
```

- **sensorTask()**: Created only for SENSOR_ULTRASONIC type. Runs at 50ms intervals with median filtering (10 readings).
- **mqttTask()**: Handles MQTT connection with exponential backoff reconnection (base 1s, max 10 attempts).
- **otaTask()**: Checks for updates every 30 seconds (not 5 minutes as documentation elsewhere states).
- **WebServer**: Dynamically created only in bridge/hotspot modes.

#### Configuration System
- **Persistent Storage**: ESP32 Preferences API with namespace "sensor-config" - survives OTA updates
- **Web Panel**: HTML file served from LittleFS (`/data/config.html`)
- **Multi-tab Interface**: Network, WiFi, Connection, MQTT, Device, Sensor, System
- **Real-time Validation**: Client-side and server-side input validation

#### OTA Update System
- **Automatic Checking**: Every 30 seconds via HTTP to `version.json`
- **Version Comparison**: Semantic versioning with rollback protection
- **Safety Mechanisms**: Boot counting, checksum verification, automatic rollback
- **Fail-safe**: Only applies updates after successful download and verification

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
- **Libraries**: PubSubClient (MQTT), ArduinoJson (JSON parsing), ESP32WebServer (fork)
- **Board Flags**: WT32-ETH01 specific Ethernet pin configuration (LAN8720 PHY)

### Build Environments
- **esp32dev** (default): Production build with `-Os` optimization, debug level 3
- **esp32dev_debug**: Debug build with `-O0` (no optimization), debug level 4, additional WiFi/HTTP debug flags

```bash
# Build with default environment
pio run

# Build with debug environment
pio run -e esp32dev_debug
```

## Development Patterns

### Adding New Sensor Types

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
- **Server**: `ota.boisolo.com/multi-sensor-iot/`
- **Version File**: `version.json` with firmware metadata (includes version, url, checksum, mandatory flag)
- **Firmware Path**: `/multi-sensor-iot/multi-sensor-iot-{version}.bin`
- **Update Frequency**: Every 30 seconds (30,000ms)
- **Checksum**: SHA256 format in `version.json`

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
# Deploy to OTA server (auto-increments version)
./deploy_script_ftp.sh

# Or using Python script (includes filesystem support)
python3 deploy_ota.py

# Verify HTTP access
curl -I http://ota.boisolo.com/multi-sensor-iot/version.json
```

Note: Devices will check for updates within 30 seconds of the new version.json being uploaded.

## Development Patterns

### Adding New Sensor Types
1. Add new enum value to `SensorType` in multi-sensor-iot.ino
2. Add GPIO pin defines at top of file
3. Extend `DeviceConfig` struct with sensor-specific settings
4. Create dedicated sensor task function
5. Add case in task creation switch statement
6. Update web config.html with new sensor options
7. Handle MQTT topic configuration for new sensor

### Modifying Web Interface
- Edit `data/config.html` for UI changes
- After modifications, run `pio run --target uploadfs` to update filesystem
- Web server only runs in bridge/hotspot modes (security feature)

### Thread Safety
- **sensorMutex**: SemaphoreHandle_t protecting shared sensor data
- Used for synchronizing access to `readings[]`, `filteredValue`, and `newDataAvailable`
- Always acquire mutex before reading/modifying shared sensor data in tasks