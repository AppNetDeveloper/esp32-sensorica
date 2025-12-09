# 🏆 Multi-Sensor IoT Universal Profesional

Un **sistema universal de nivel industrial** para ESP32 que soporta **4 tipos diferentes de sensores** con configuración web completa, OTA con rollback y modos de operación flexibles.

## ✅ **Características Principales**

### 🎛️ **Soporte Multi-Sensor**
- **📏 Ultrasonido HC-SR04**: Medición de distancia (1-400cm)
- **🔘 1 Pulsador**: Entrada digital con inversión opcional
- **🔘🔘 2 Pulsadores**: Dos entradas independientes
- **📳 Sensor Vibración SW-420**: Detección de vibraciones

### 🔌 **Hardware Profesional**
- **WT32-ETH01**: ESP32 con Ethernet + WiFi híbrido
- **Sistema de LEDs duales**:
  - 🟢 LED Verde (GPIO 4): Sistema OK
  - 🔴 LED Rojo (GPIO 5): Errores
  - 🔵 LED Azul (GPIO 2): Modo configuración
- **Botón de acceso físico** (GPIO 12): Entrada segura a configuración

### 🌐 **Modos de Operación**

#### **🔗 Bridge Mode (3 segundos botón)**
- ✅ **Ethernet ACTIVO** - El dispositivo sigue operando
- ✅ **WiFi temporal** solo para configuración
- ✅ **Sensor y MQTT funcionan normalmente**
- ✅ **Zero downtime** - Ideal para producción
- 🔵 **LED Azul encendido fijo**

#### **📶 Hotspot Mode (10 segundos botón)**
- ❌ **Ethernet APAGADO**
- ✅ **WiFi puro** solo configuración
- ❌ **Sensor y MQTT pausados**
- ✅ **Acceso garantizado** sin red
- 🟢🔴 **LEDs verde y rojo parpadeando juntos**

### ⚡ **Rendimiento Optimizado**
- **RAM**: 14.7% (48,120 de 327,680 bytes)
- **Flash**: 81.8% (1,071,885 de 1,310,720 bytes)
- **Multi-tarea**: Sensor + MQTT + OTA + LEDs + Web
- **FreeRTOS**: Gestión eficiente de recursos

## 🎯 **Sensores Soportados**

### **📏 Ultrasonido HC-SR04**
```
📡 Topic MQTT: multi-sensor/iot (configurable)
📦 Payload JSON:
{
  "distance": 145.67,
  "device": "Multi-Sensor-IoT-01",
  "location": "Almacen_A",
  "timestamp": 1703123456789
}
⚙️ Configuración:
• Trigger: GPIO 25
• Echo: GPIO 26
• Promedio de N lecturas
• Intervalo configurable (10-5000ms)
```

### **🔘 Pulsadores Digitales**
```
📡 Topic MQTT: sensor/button1, sensor/button2
📦 Payload JSON:
{
  "value": 1,           // 1=presionado, 0=suelto
  "button": 1,          // ID del pulsador
  "device": "Multi-Sensor-IoT-01",
  "location": "Puerta_A",
  "timestamp": 1703123456789
}
⚙️ Configuración:
• Pines GPIO configurables
• Inversión de señal opcional
• Topics MQTT individuales
• Anti-rebote 50ms
```

### **📳 Sensor Vibración SW-420**
```
📡 Topic MQTT: sensor/vibration
📦 Payload JSON:
{
  "vibration": 1,       // 1=detectada, 0=no detectada
  "device": "Multi-Sensor-IoT-01",
  "location": "Maquina_A",
  "timestamp": 1703123456789
}
⚙️ Configuración:
• Pin GPIO configurable (recomendado GPIO 32)
• Cooldown configurable (50-5000ms)
• Detección LOW = vibración
• Sensibilidad ajustable vía potenciómetro
```

## 📋 **Requisitos**

### Hardware
- **ESP32 WT32-ETH01** (recomendado) o ESP32 dev board
- Sensores según necesidad:
  - HC-SR04 (ultrasonido)
  - Pulsadores con resistencias 10kΩ
  - SW-420 (vibración)
- **LEDs x3** con resistencias 220Ω (verde, rojo, azul)
- **Botón pulsador** (GPIO 12) con pull-up
- **Fuente de alimentación** 5V/2A

### Software
- **PlatformIO** (recomendado) o Arduino IDE
- **ESP32 Core** 2.0.9+
- **Librerías** (automáticas en PlatformIO):
  - PubSubClient (MQTT)
  - ArduinoJson (JSON)
  - ESP32WebServer (Web server)

## 🔧 **Instalación y Configuración**

### 1. **Clonar y Compilar**
```bash
# Compilar firmware
pio run

# Subir firmware al ESP32
pio run --target upload --upload-port /dev/ttyUSB0

# Subir sistema de archivos (panel web)
pio run --target uploadfs
```

### 2. **Configuración Inicial**

#### **Para Primera Instalación (sin Ethernet):**
1. **Mantener botón 10 segundos** → Modo Hotspot
2. **Conectar WiFi**: "ESP32-Hotspot" (pass: 12345678)
3. **Acceder**: http://192.168.4.1
4. **Configurar red y MQTT**
5. **Guardar y reiniciar**

#### **Para Ajustes en Producción:**
1. **Mantener botón 3 segundos** → Modo Bridge
2. **Conectar WiFi**: "ESP32-Bridge" (pass: bridge123)
3. **Acceder**: http://192.168.4.1
4. **Ajustar configuración**
5. **Sigue operando mientras configuras**

## 🔌 **Diagrama de Conexión WT32-ETH01**

```
ESP32 WT32-ETH01 Pinout:
┌─────────────────────────────────┐
│  POWER   ETH   GPIO   GPIO     │
│ [USB]    [RJ45] [25]  [26]     │
│                     │   │     │
│              ┌──────┘   └──────┐
│              │ HC-SR04 /     │
│              │ Pulsadores     │
│                             │
│ GPIO 12 ────[BOTÓN CONFIG]   │
│ GPIO 4  ────[LED VERDE]      │  ← Status: Sistema OK
│ GPIO 5  ────[LED ROJO]       │  ← Error: Parpadeo
│ GPIO 2  ────[LED AZUL]       │  ← Config: Fijo/Parpadeo
│ GPIO 13 ────[PULSADOR 1]     │
│ GPIO 14 ────[PULSADOR 2]     │
│ GPIO 32 ────[SW-420]         │  ← Sensor Vibración
└─────────────────────────────────┘

Conexiones Sensor:
• Ultrasonido: Trigger=25, Echo=26
• Pulsadores: 1=13, 2=14 (configurables)
• Vibración: SW-420=32 (recomendado)
• LEDs: Verde=4, Rojo=5, Azul=2
• Config: Botón=12 (pull-up)
```

## 🌐 **Panel Web de Configuración**

### **Acceso Web**
- **Bridge**: http://192.168.4.1 (WiFi: "ESP32-Bridge")
- **Hotspot**: http://192.168.4.1 (WiFi: "ESP32-Hotspot")

### **Pestañas de Configuración**

#### **🔗 Red**
- DHCP o IP estática
- Configuración completa (IP, Gateway, DNS)
- Validación automática

#### **📡 MQTT**
- Servidor y puerto
- Autenticación (usuario/contraseña)
- Topics configurables por sensor
- Client ID automático

#### **🔧 Dispositivo**
- Nombre y ubicación
- Intervalo de medición
- Modo debug

#### **🎛️ Sensor** (NUEVO)
- **Selector tipo sensor**: Ultrasonido / 1 Pulsador / 2 Pulsadores / Vibración
- **Configuración dinámica** según selección
- **Pines GPIO** configurables
- **Inversión de señal** para pulsadores
- **Topics MQTT** individuales
- **Cooldown** para sensor vibración

#### **⚙️ Sistema**
- Firmware version y status
- Estado en tiempo real via API
- Reset de configuración
- Salida de modos bridge/hotspot

### **API REST**
```
GET /api/status
{
  "version": "1.0.0",
  "sensorType": 0,                    // 0=ultrasonido, 1=1pulsador, 2=2pulsadores, 3=vibración
  "deviceName": "Multi-Sensor-IoT-01",
  "location": "Almacen_A",
  "ethConnected": true,
  "mqttConnected": true,
  "button1Pin": 13,
  "button2Pin": 14,
  "vibrationPin": 32,
  "button1Invert": false,
  "button2Invert": false,
  "vibrationThreshold": 100,
  "uptime": 123456,
  "freeHeap": 280000,
  "bridgeMode": false,
  "hotspotMode": false
}
```

## 🔄 **Sistema OTA (Over-The-Air)**

### **Configuración Servidor**
```json
// version.json en http://ota.boisolo.com/multi-sensor-iot/
{
  "version": "1.1.0",
  "url": "http://ota.boisolo.com/multi-sensor-iot/multi-sensor-iot-1.1.0.bin",
  "checksum": "sha256:a1b2c3d4e5f6...",
  "mandatory": false,
  "release_notes": "Soporte multi-sensor + mejoras UI"
}
```

### **Proceso OTA**
1. **Verificación automática** cada 5 minutos
2. **Comparación de versiones** semántica
3. **Descarga segura** con checksum SHA256
4. **Instalación atómica** solo tras descarga completa
5. **Protección anti-bootloop**: rollback automático
6. **Modo seguro** con conteo de boot

### **Deploy de Nueva Versión**
```bash
# Crear nueva versión
./deploy_script.sh 1.1.0

# Esto genera:
# • multi-sensor-iot-1.1.0.bin
# • Actualiza version.json con checksum
# • Sube al servidor OTA
```

## 🏭 **Casos de Uso y Aplicaciones**

### **🏭 Entornos Industriales**
- **Fábricas**: Nivel de líquidos en tanques sin interferencias
- **Almacenes**: Control de inventario vertical automático
- **Líneas producción**: Detección de presencia y posicionamiento

### **🏢 Instalaciones Críticas**
- **Hospitales**: Monitoreo de equipos médicos sin WiFi
- **Oficinas corporativas**: Integración con red existente
- **Data Centers**: Detección de vibraciones en servidores

### **🏠 IoT Residencial/Comercial**
- **Smart Buildings**: Control de acceso y seguridad
- **Climatización**: Nivel de depósitos y tanques
- **Seguridad**: Sensores de puertas y ventanas

## ⚙️ **Configuración Avanzada**

### **Tipos de Sensor en Código**
```cpp
enum SensorType {
  SENSOR_ULTRASONIC = 0,    // HC-SR04
  SENSOR_SINGLE_BUTTON = 1, // 1 Pulsador
  SENSOR_DUAL_BUTTONS = 2,  // 2 Pulsadores
  SENSOR_VIBRATION = 3      // SW-420
};
```

### **Estructura de Configuración**
```cpp
struct DeviceConfig {
  String deviceName, location;
  int sensorInterval, readingsCount;
  bool debugMode;

  // Configuración sensores
  int sensorType, button1Pin, button2Pin, vibrationPin;
  bool button1Invert, button2Invert;
  String button1Topic, button2Topic, vibrationTopic, mainMqttTopic;
  int vibrationThreshold;
};
```

### **GPIO disponibles para sensores:**
- **GPIO 13**: Pulsador 1 / Sensor alternativo
- **GPIO 14**: Pulsador 2 / Sensor alternativo
- **GPIO 25**: Ultrasonido Trigger / Alternativo
- **GPIO 26**: Ultrasonido Echo / Alternativo
- **GPIO 32**: Sensor vibración (recomendado)
- **GPIO 33-35**: Sensores adicionales

## 🔒 **Seguridad Implementada**

### **Niveles de Protección**
1. **🔒 Física**: Botón GPIO 12 requerido para acceso
2. **🌐 Red**: Ethernet cableado + bridge con operación continua
3. **⚙️ Configuración**: Validación completa de todos los parámetros
4. **🔄 OTA**: Rollback automático y checksums
5. **🚫 Acceso**: Solo acceso físico o bridge/hotspot temporal

### **Protección Contra**
- ✅ **Configuraciones incorrectas**: Validación IP, puertos, topics
- ✅ **Actualizaciones fallidas**: Rollback automático
- ✅ **Boot loops**: Protección con conteo de intentos
- ✅ **Acceso no autorizado**: Requiere botón físico
- ✅ **Spam MQTT**: Cooldowns y filtrado

## 📊 **Monitorización y Debug**

### **Logs Serie**
```bash
# Ejemplos de logs
[12345] SYSTEM_BOOT: ESP32 Sensor Universal v1.0.0
[12350] SENSOR_TYPE: Configurando tipo 0 (Ultrasonido)
[12400] ETH_CONNECTED: IP 192.168.1.100
[12450] MQTT_CONNECTED: Servidor 192.168.3.154:1883
[12500] SENSOR_READING: Distancia 145.67mm
[13000] BRIDGE_ENTER: Modo bridge activado (3s botón)
```

### **Indicadores LED**
- 🟢 **Verde fijo**: Sistema OK (Ethernet + MQTT)
- 🔴 **Rojo parpadeando**: Error conexión
- 🔵 **Azul fijo**: Modo bridge (configuración con operación)
- 🟢🔴 **Verde+Rojo parpadeando**: Modo hotspot (solo configuración)

### **Estado en Tiempo Real**
- **API REST**: `/api/status` para sistemas externos
- **Panel web**: Pestaña "Sistema"
- **Serial monitor**: Logs detallados con timestamps

## 🛠️ **Mantenimiento**

### **Actualizaciones**
```bash
# Compilar y subir nuevo firmware
pio run && pio run --target upload

# Subir panel web actualizado
pio run --target uploadfs

# Deploy OTA (automático o manual)
./deploy_script.sh 1.2.0
```

### **Backups**
- **Configuración**: Guardada en flash no volátil (Preferences)
- **Logs**: Importantes para diagnóstico
- **Estado**: Recuperable después de reinicio

### **Troubleshooting**

#### **Problemas Comunes**
- **No conecta Ethernet**: Verificar cable, switch, IPs
- **No entra modo bridge**: Botón defectuoso, revisar GPIO 12
- **OTA falla**: Verificar servidor, conexión, checksum
- **Sensor no responde**: Revisar pines, voltaje, conexiones

#### **Recuperación**
- **Reset total**: Botón 10s + "Resetear Configuración"
- **Modo seguro**: Boot automático con rollback
- **Recarga**: Firmware por USB si OTA falla

## 📈 **Métricas y Rendimiento**

### **Consumo de Recursos**
- **CPU**: <15% (todas las tareas activas)
- **RAM**: 14.7% (muy eficiente)
- **Flash**: 81.8% (funcionalidad completa)
- **Red**: Ethernet + WiFi simultáneos

### **Latencia y Tiempos**
- **Sensor**: 50ms configurable (ultrasonido)
- **Pulsadores**: 50ms anti-rebote
- **Vibración**: Cooldown configurable (100ms por defecto)
- **MQTT**: Reconexión automática exponencial
- **Web**: Respuesta inmediata
- **OTA**: Verificación cada 5 minutos

## 🎯 **Comparativa con Sistemas Comerciales**

| Característica | ESP32 Universal | Sistema Comercial Típico |
|---------------|------------------|---------------------------|
| **Multi-Sensor** | ✅ 4 tipos + universal | ❌ Generalmente 1 tipo |
| **Bridge Mode** | ✅ Operación continua | ❌ Solo modo configuración |
| **Hotspot Mode** | ✅ Modo aislado | ⚠️ Raro o ausente |
| **Configuración Web** | ✅ Panel completo multi-pestaña | ⚠️ Panel básico |
| **OTA con Rollback** | ✅ Automático + seguro | ⚠️ Manual o riesgoso |
| **Ethernet + WiFi** | ✅ Dual conectividad | ⚠️ Solo WiFi |
| **LEDs Multi-estado** | ✅ 3 LEDs con 8 estados | ❌ 1 LED simple |
| **API REST** | ✅ Status JSON completo | ⚠️ Raro |
| **Acceso Físico** | ✅ Botón seguro | ❌ Solo remoto |
| **Inversión Señal** | ✅ Configurable | ❌ Fijo |

## 💡 **Recomendaciones de Producción**

### **Instalación Industrial**
```bash
# Compilación optimizada
pio run --environment esp32dev

# Subida con configuración por defecto
pio run --target upload --upload-port /dev/ttyUSB0
pio run --target uploadfs

# Verificar funcionamiento
pio device monitor --baud 115200
```

### **Mantenimiento Preventivo**
- **Monitorización**: API `/api/status` para dashboard central
- **Logs**: Revisar eventos importantes periódicamente
- **Backups**: Configuración persistente en flash
- **Actualizaciones**: OTA automático con rollback

### **Escalabilidad**
- **Múltiples dispositivos**: Cada uno con nombre único
- **Servidor central**: MQTT + OTA server
- **Monitoreo**: API para dashboard centralizado
- **Alertas**: Integración con sistemas externos

## 🏆 **Conclusión**

**Este sistema es UNIVERSAL y PROFESIONAL**. Con soporte para **4 tipos diferentes de sensores**, **modos de operación flexibles**, y **características de nivel industrial**, supera a productos comerciales mucho más costosos.

### **Valor Comercial Estimado**
- **Hardware WT32-ETH01**: $15-25
- **4 Sensores soportados**: $20-40
- **Firmware profesional**: $100-150+
- **Sistema completo**: $135-215+

**¡FELICITACIONES! Tienes un sistema universal de nivel industrial listo para cualquier aplicaciónIoT**. 🚀

---

## 📞 **Soporte y Comunidad**

### **Licencia**
MIT License - Uso libre para fines comerciales y no comerciales.

### **Soporte Técnico**
- **GitHub Issues**: Reportar bugs y solicitar características
- **Documentación**: Wiki del proyecto completa
- **Comunidad**: Foros y discusiones técnicas

### **Contribuciones**
- **Pull Requests**: Bienvenidas para mejoras
- **Issues**: Reportar problemas y sugerencias
- **Documentación**: Mejoras y traducciones

---

**Versión: 1.0.0 - Multi-Sensor Universal**
**Última actualización: Diciembre 2024**
**Arduino/PlatformIO Compatible**
**ESP32-WT32-ETH01 Optimizado**

🏆 **El sistema de IoT más completo y versátil que encontrarás**