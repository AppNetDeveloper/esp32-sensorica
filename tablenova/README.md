# Multi-Sensor IoT Universal - ESP32 Professional Firmware

![ESP32](https://img.shields.io/badge/ESP32-Professional-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Version](https://img.shields.io/badge/Version-1.0.0-orange.svg)

Sistema IoT profesional para ESP32 WT32-ETH01 con soporte para múltiples tipos de sensores, conectividad dual Ethernet/WiFi, configuración web y actualizaciones OTA automáticas.

## 🎯 Características Principales

### **🔗 Conectividad Avanzada**
- **Ethernet + WiFi**: Modo dual con failover automático
- **Control Web**: Configuración completa desde interfaz web
- **Auto-Hotspot**: Modo hotspot automático si no hay configuración
- **Bridge Mode**: Mantener Ethernet + WiFi AP para configuración
- **Configuraciones Persistentes**: Sobreviven a actualizaciones OTA

### **🎛️ Multi-Sensor Universal**
1. **Sensor Ultrasónico** - HC-SR04/JSN-SR04T (1-400cm)
2. **1 Pulsador Digital** - Interruptor simple
3. **2 Pulsadores Digitales** - Doble control
4. **Sensor de Vibración** - SW-420

### **⚙️ Sistema Profesional**
- **Panel Web Completo**: Configuración por pestañas (Red, WiFi, Conexión, MQTT, Dispositivo, Sensor, Sistema)
- **OTA Automático**: Actualizaciones cada 5 minutos con rollback protection
- **LED Status**: Sistema de 3 LEDs para diagnóstico visual
- **Botón Config**: Multi-modo (3s bridge, 10s hotspot)
- **Logs Persistente**: Eventos guardados en memoria

### **📊 Monitoreo en Tiempo Real**
- Estado de conexiones (Ethernet, WiFi)
- Métricas del sistema (memoria, uptime)
- Estado sensores y valores actuales
- Historial de eventos y logs

## 🚀 Quick Start

### **Hardware Requerido**
- ESP32 WT32-ETH01 (Ethernet + WiFi)
- Sensor ultrasónico JSN-SR04T (opcional)
- Botón de configuración GPIO 12
- LEDs de estado (GPIO 2,4,5)

### **1. Configuración Inicial**
```bash
# Clonar repositorio
git clone <repositorio>
cd esp32-sensorica/tablenova

# Instalar dependencias
pio lib install

# Compilar firmware
pio run

# Subir firmware
pio run --target upload

# Subir sistema de archivos web
pio run --target uploadfs
```

### **2. Primera Configuración**
1. **Auto-Hotspot**: Si no hay configuración previa, automáticamente crea hotspot "ESP32-Hotspot"
2. **Conectar**: Conectar WiFi al hotspot, ir a `http://192.168.4.1`
3. **Configurar**: Usar panel web para configurar red, MQTT, sensores
4. **Guardar**: Aplicar cambios y reiniciar dispositivo

### **3. Modos de Operación**

#### **Modo Bridge (3s botón)**
- Mantiene conexión Ethernet
- Crea WiFi AP "ESP32-Bridge"
- Acceso a panel web para configuración
- Timeout: 5 minutos

#### **Modo Hotspot (10s botón)**
- Configuración WiFi únicamente
- Hotspot "ESP32-Hotspot"
- Ideal para redes sin Ethernet

## 📡 Panel de Configuración Web

### **Pestañas Disponibles**

#### **🌐 Red**
- Configuración Ethernet (DHCP/Static IP)
- DNS, Gateway, Subnet
- Validación de configuración

#### **📶 WiFi**
- Habilitar/Deshabilitar WiFi
- SSID y contraseña
- Modo backup automático

#### **🔗 Conexión**
- **Ethernet**: Conexión cableada prioritaria
- **WiFi**: Conexión inalámbrica única
- **Dual**: Ethernet + WiFi backup automático

#### **📡 MQTT**
- Servidor y puerto
- Usuario/contraseña
- Topics por sensor
- QoS y keep-alive

#### **🎛️ Sensor**
- Tipo de sensor (4 opciones)
- Configuración de pines
- Umbrales y sensibilidad
- MQTT topics personalizados

#### **⚙️ Dispositivo**
- Nombre y ubicación
- Intervalo de lecturas
- Modo debug
- Ubicación física

#### **📊 Sistema**
- Estado de conexiones
- Métricas en tiempo real
- Logs de eventos
- Actualización OTA manual

## 🔧 Configuración Avanzada

### **GPIO Pin Assignments**
```
Sensor ultrasónico: TRIG=25, ECHO=26
Botón configuración: GPIO 12
LEDs: Status=4, Error=5, Config=2
Pulsador 1: GPIO 13
Pulsador 2: GPIO 14
Vibración: GPIO 32
```

### **Estructura de Configuración**
```cpp
// Network Config
bool dhcpEnabled;
String staticIP, gateway, subnet, dns1, dns2;

// WiFi Config
String ssid, password;
bool enabled;

// Connection Mode
enum ConnectionMode {
  MODE_ETHERNET = 0,
  MODE_WIFI = 1,
  MODE_DUAL_ETH_WIFI = 2
};

// Sensor Config
int sensorType;  // 0=ultrasonido, 1=1botón, 2=2botones, 3=vibración
String mqttTopics[3];  // Topics por tipo de sensor
```

## 🌐 Sistema OTA

### **Configuración Servidor OTA**
```bash
# Desplegar nueva versión
./deploy_script_ftp.sh 1.0.0

# Esto sube:
# - firmware.bin -> multi-sensor-iot-1.0.0.bin
# - version.json con checksum y URL
```

### **URLs OTA**
```
Servidor: http://ota.boisolo.com/multi-sensor-iot/
Firmware: http://ota.boisolo.com/multi-sensor-iot/multi-sensor-iot-{version}.bin
Versión:  http://ota.boisolo.com/multi-sensor-iot/version.json
```

### **Proceso OTA Automático**
1. **Check**: Cada 5 minutos verifica version.json
2. **Compare**: Versión actual vs disponible
3. **Download**: Descarga firmware si es más reciente
4. **Verify**: Verifica checksum SHA256
5. **Install**: Aplica actualización y reinicia
6. **Rollback**: Si falla, vuelve a versión anterior

### **Safety Features**
- **Boot Count Protection**: Previene boot loops
- **Checksum Verification**: SHA256 de cada actualización
- **Fallback**: Revert automático si actualización falla
- **Verify Before Apply**: Solo instala si descarga completa

## 📖 Guías Detalladas

### **Instalación y Configuración**
1. **Conexión Hardware**: Conectar Ethernet, sensores, LEDs
2. **Primer Arranque**: Auto-detección de configuración
3. **Panel Web**: Configuración inicial via hotspot o bridge
4. **Validación**: Prueba de conectividad MQTT y sensores
5. **Producción**: Monitoreo y ajustes finales

### **Modo Bridge vs Hotspot**
- **Bridge (3s)**: Ethernet + WiFi AP simultáneos
- **Hotspot (10s)**: WiFi únicamente para configuración
- **Auto**: Se activa si no hay configuración guardada

### **Troubleshooting**
- **LED Status**: Verde=OK, Rojo=Error, Azul=Configuración
- **Serial Monitor**: Logs detallados para diagnóstico
- **Web Panel**: Estado de sistema en tiempo real
- **WiFi Scan**: Escaneo de redes disponibles

## 📊 Arquitectura del Sistema

### **FreeRTOS Tasks**
```cpp
sensorTask()     // Lectura de sensores (50ms intervalo)
mqttTask()        // Gestión MQTT y reconexión
otaTask()         // Check actualizaciones (5min)
WebServer()       // Panel configuración (bridge/hotspot)
```

### **Estado LEDs**
- **🟢 Verde (GPIO 4)**: Sistema OK (Ethernet + MQTT conectado)
- **🔴 Rojo (GPIO 5)**: Error (conexión caída)
- **🔵 Azul (GPIO 2)**: Configuración (bridge=sólido, hotspot=parpadeo)

### **Data Flow**
```
Sensor → Procesamiento → MQTT → Broker → Aplicaciones
   ↓
Web Panel ← API REST ← Estado Sistema
```

## 🔒 Seguridad y Fiabilidad

### **Conexiones**
- **Reconexión Automática**: Exponential backoff
- **Timeout Management**: Protección contra bloqueos
- **Connection Pool**: Gestión eficiente de recursos

### **Datos**
- **JSON Validation**: Validación estricta de MQTT
- **Input Sanitization**: Protección XSS en web panel
- **Config Validation**: Validación de red y parámetros

### **System**
- **Watchdog Timer**: Reinicio automático si sistema bloqueado
- **Memory Management**: Monitoreo y limpieza de memoria
- **Error Recovery**: Recuperación automática de fallos

## 🎛️ Configuración por Sensor

### **1. Sensor Ultrasónico (Default)**
```json
{
  "sensorType": 0,
  "triggerPin": 25,
  "echoPin": 26,
  "mqttTopic": "multi-sensor/iot/distance",
  "range": "1-400cm",
  "readings": 10,
  "interval": 50
}
```

### **2. 1 Pulsador Digital**
```json
{
  "sensorType": 1,
  "buttonPin": 13,
  "buttonInvert": false,
  "mqttTopic": "multi-sensor/iot/button1",
  "debounce": 50
}
```

### **3. 2 Pulsadores Digitales**
```json
{
  "sensorType": 2,
  "button1Pin": 13,
  "button2Pin": 14,
  "mqttTopics": ["multi-sensor/iot/button1", "multi-sensor/iot/button2"]
}
```

### **4. Sensor Vibración**
```json
{
  "sensorType": 3,
  "vibrationPin": 32,
  "mqttTopic": "multi-sensor/iot/vibration",
  "threshold": 100
}
```

## 📡 MQTT Topics

### **Estructura de Topics**
```
multi-sensor/iot/
├── distance          // Sensor ultrasónico
├── button1           // Pulsador 1
├── button2           // Pulsador 2
├── vibration         // Sensor vibración
├── status            // Estado sistema
└── system            // Eventos sistema
```

### **Payload Format**
```json
{
  "device": "ESP32-MultiSensor-01",
  "location": "Oficina Principal",
  "timestamp": "2025-12-10T18:30:00Z",
  "sensorType": "ultrasonic",
  "value": 45.2,
  "unit": "cm",
  "status": "ok"
}
```

## 🛠️ Desarrollo y Mantenimiento

### **Estructura de Proyecto**
```
/
├── src/
│   └── multi-sensor-iot.ino     # Firmware principal
├── data/
│   └── config.html              # Panel web
├── scripts/
│   ├── deploy_script_ftp.sh    # Deploy OTA
│   └── debug_ftp.sh           # Debug conexión
├── docs/
│   └── README.md              # Documentación consolidada
├── platformio.ini             # Configuración PlatformIO
└── version.json              # Versión OTA
```

### **Personalización**
- **Custom Sensors**: Añadir nuevos tipos al enum SensorType
- **UI Themes**: Modificar CSS en config.html
- **MQTT Format**: Adaptar payload a necesidades específicas
- **Pin Mapping**: Configurar pines según hardware disponible

### **Debugging**
```bash
# Monitor serie
pio device monitor

# Ver logs
pio run --target clean && pio run

# Debug FTP
./debug_ftp.sh
```

## 📄 Licencia

MIT License - Ver archivo LICENSE para detalles completos.

## 🤝 Contribuciones

Contribuciones bienvenidas. Por favor:
1. Fork del proyecto
2. Branch feature/nueva-funcionalidad
3. Commit con cambios
4. Push al branch
5. Pull Request

---

**Multi-Sensor IoT Universal** - Sistema IoT profesional listo para producción con características avanzadas de conectividad, monitoreo y configuración. 🚀

---
*Desarrollado con ❤️ para la comunidad IoT*