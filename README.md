# esp32-sensorica

<div align="center">

![ESP32](https://img.shields.io/badge/ESP32-WT32--ETH01-blue.svg)
![License](https://img.shields.io/badge/License-Apache%202.0-green.svg)
![Version](https://img.shields.io/badge/Firmware-1.6.39-orange.svg)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Ready-red.svg)

**Firmware IoT profesional para ESP32 WT32-ETH01**

Soporte multi-sensor, conectividad dual Ethernet/WiFi, panel de configuración web y actualizaciones OTA automáticas.

</div>

---

## Sobre este repositorio

Este repositorio contiene el proyecto **nodebox**: el firmware *Multi-Sensor IoT Universal*
para placas ESP32 **WT32-ETH01**. Es una solución lista para producción que permite
desplegar nodos de sensores industriales con conectividad cableada e inalámbrica y
gestión remota del ciclo de vida del firmware.

Todo el código, los scripts de despliegue y la documentación detallada viven dentro
de la carpeta [`nodebox/`](nodebox/).

## Características principales

- **Conectividad dual** — Ethernet (LAN8720) como conexión primaria con failover
  automático a WiFi, o WiFi únicamente.
- **Multi-sensor universal** — 5 tipos de sensor seleccionables por configuración:
  ultrasónico (HC-SR04 / JSN-SR04T), 1 pulsador, 2 pulsadores, vibración (SW-420) y
  vibración + pulsador combinados.
- **Panel de configuración web** — Interfaz de 7 pestañas (Red, WiFi, Conexión, MQTT,
  Dispositivo, Sensor, Sistema) servida desde el propio dispositivo.
- **Actualizaciones OTA** — Verificación automática cada 30 s con verificación SHA256,
  protección contra boot loops y rollback automático.
- **Publicación MQTT** — Payloads JSON con reconexión automática y backoff exponencial.
- **Configuración persistente** — Los ajustes se guardan en la NVS del ESP32 y
  sobreviven a las actualizaciones OTA.
- **Diagnóstico visual** — 3 LEDs de estado y botón de configuración multi-modo
  (3 s = modo bridge, 10 s = modo hotspot).

## Estructura del repositorio

```
esp32-sensorica/
├── nodebox/                  Proyecto principal (firmware Multi-Sensor IoT)
│   ├── src/                  Firmware ESP32 (multi-sensor-iot.ino)
│   ├── data/                 Panel web (config.html)
│   ├── platformio.ini        Configuración de compilación PlatformIO
│   ├── deploy_all.sh         Despliegue local por USB
│   ├── deploy_ota.py         Despliegue OTA con auto-incremento de versión
│   ├── README.md             Documentación completa del firmware
│   └── PINOUT.md             Mapa de pines y conexiones de sensores
├── libraries/                Librerías Arduino utilizadas
└── LICENSE.md                Licencia Apache 2.0
```

## Hardware

| Componente | Especificación |
|------------|----------------|
| Microcontrolador | ESP32 WT32-ETH01 (Ethernet + WiFi) |
| Sensores soportados | HC-SR04 / JSN-SR04T, SW-420, pulsadores |
| Alimentación | 5V DC |

El mapa completo de pines GPIO y los diagramas de conexión de cada sensor están en
[`nodebox/PINOUT.md`](nodebox/PINOUT.md).

## Quick Start

Requiere [PlatformIO](https://platformio.org/). El ESP32 debe estar conectado por USB.

```bash
# Clonar el repositorio
git clone https://github.com/appnetdeveloper/esp32-sensorica.git
cd esp32-sensorica/nodebox

# Compilar el firmware
pio run

# Subir firmware al ESP32
pio run --target upload

# Subir el panel web (filesystem LittleFS)
pio run --target uploadfs

# Monitor serie (opcional)
pio device monitor
```

O usar el script de despliegue completo, que limpia, compila y sube firmware + filesystem:

```bash
./deploy_all.sh
```

Para publicar una nueva versión en el servidor OTA (incrementa la versión
automáticamente):

```bash
python3 deploy_ota.py
```

## Documentación

La guía completa del firmware —modos de operación, configuración de cada sensor,
panel web, sistema OTA, formato de mensajes MQTT, arquitectura FreeRTOS y
troubleshooting— está en:

- [`nodebox/README.md`](nodebox/README.md) — Documentación detallada del firmware.
- [`nodebox/PINOUT.md`](nodebox/PINOUT.md) — Pinout y diagramas de conexión.

## Licencia

Distribuido bajo la licencia Apache 2.0. Consulta el archivo [LICENSE.md](LICENSE.md)
para los términos completos.
