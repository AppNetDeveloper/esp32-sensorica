# 🚀 Guía completa de OTA (Over-The-Air) para ESP32

## 📋 Resumen del Sistema

He implementado un sistema OTA robusto con control de versiones semántico para tu ESP32 WT32-ETH01. El sistema verifica automáticamente si hay actualizaciones disponibles y las instala de forma segura.

## 🏗️ Arquitectura del Sistema

### Componentes principales:
1. **Tarea OTA**: Se ejecuta en background cada 5 minutos
2. **Control de versiones**: Compara versiones usando formato semántico (1.2.3)
3. **JSON de configuración**: Define qué versiones están disponibles
4. **Verificación opcional**: Soporta checksum SHA256
5. **Actualizaciones obligatorias**: Permite forzar actualizaciones críticas

## 📁 Archivos Generados

```
/home/liviu/esp32-sensorica/tablenova/
├── .pio/build/esp32dev/firmware.bin     # Binario para OTA
├── src/sensor-medidor-mesas-corte.ino   # Código con OTA
├── platformio.ini                      # Configuración con versión
├── version.json.example                # Plantilla de version.json
└── OTA_SETUP.md                       # Esta guía
```

## ⚙️ Configuración del Servidor

### 1. Archivo version.json
Crea este archivo en tu servidor web (ej: `http://ota.boisolo.com/ultrasonido/version.json`):

```json
{
  "version": "1.1.0",
  "url": "http://ota.boisolo.com/ultrasonido/firmware-1.1.0.bin",
  "checksum": "sha256:a1b2c3d4e5f6...",
  "mandatory": false,
  "release_notes": "Mejora en la precisión del sensor y optimización de memoria RAM"
}
```

** Campos del JSON:**
- `version`: Versión en formato semántico (ej: "1.2.3")
- `url`: URL directa al archivo .bin
- `checksum`: SHA256 del firmware (opcional)
- `mandatory`: true/false para actualizaciones obligatorias
- `release_notes`: Descripción de los cambios

### 2. Archivos de firmware
Coloca los archivos `.bin` con nombres versionados:
```
http://ota.boisolo.com/ultrasonido/
├── firmware-1.0.0.bin  # Versión inicial
├── firmware-1.1.0.bin  # Primera actualización
├── firmware-1.2.0.bin  # Segunda actualización
└── version.json        # Información de versiones
```

## 🔄 Flujo de Actualización

### 1. Verificación (cada 5 minutos)
```cpp
OTA > Verificando actualizaciones...
OTA > Versión actual: 1.0.0
OTA > Versión disponible: 1.1.0
OTA > Notas de la versión: Mejora en la precisión del sensor...
OTA > Se encontró una actualización disponible
```

### 2. Descarga e instalación
```cpp
OTA > Iniciando actualización...
OTA > URL del firmware: http://192.168.3.154/firmware-1.1.0.bin
OTA > ¡Actualización exitosa! Reiniciando...
```

## 🛠️ Comandos de PlatformIO

### Para compilar y generar nuevo firmware:
```bash
pio run
```

### Para subir firmware por primera vez:
```bash
pio run --target upload
```

### Para monitorizar el dispositivo:
```bash
pio device monitor
```

## 📝 Control de Versiones

### Cambiar versión del firmware:
Edita `platformio.ini`:
```ini
build_flags =
    -D FW_VERSION=\"1.1.0\"
```

### Proceso para nueva versión:
1. **Actualizar versión** en `platformio.ini`
2. **Compilar**: `pio run`
3. **Copiar binario**:
   ```bash
   scp .pio/build/esp32dev/firmware.bin user@ota.boisolo.com:/var/www/html/ota.boisolo.com/ultrasonido/firmware-1.1.0.bin
   ```
4. **Actualizar version.json** con nueva versión
5. **Desplegar**: Los dispositivos se actualizarán automáticamente

## 🔧 Configuración en el Código

### URLs y tiempos:
```cpp
const char* ota_version_url = "http://ota.boisolo.com/ultrasonido/version.json";
const unsigned long ota_check_interval = 300000;  // 5 minutos
const int ota_timeout = 30000;  // 30 segundos timeout
```

### Comparación de versiones:
- Soporta formato semántico: `major.minor.patch`
- Compara: `1.2.3` < `1.2.4` < `1.3.0` < `2.0.0`
- Ignora prefijos como "v": `v1.0.0` = `1.0.0`

## 📊 Monitoreo y Debug

### Logs esperados:
```cpp
// Verificación normal
OTA > Verificando actualizaciones...
OTA > Versión actual: 1.0.0
OTA > El firmware está actualizado

// Actualización disponible
OTA > Se encontró una actualización disponible
OTA > Iniciando actualización...

// Errores
OTA > Error al obtener información de versión, código HTTP: 404
OTA > Error en actualización: (-1) sin memoria disponible
```

## 🔐 Seguridad

### Recomendaciones:
1. **HTTPS**: Usa HTTPS para producción
2. **Checksums**: Implementa verificación SHA256
3. **Autenticación**: Añade headers de autenticación
4. **Validación**: Verifica tamaño máximo del firmware

### Ejemplo con autenticación:
```cpp
http.addHeader("Authorization", "Bearer tu-token");
```

## 🚨 Manejo de Errores

### Fallas comunes y soluciones:

| Error | Causa | Solución |
|-------|-------|----------|
| HTTP 404 | version.json no encontrado | Verificar URL y ruta |
| Sin memoria | Firmware muy grande | Optimizar particiones |
| Timeout red | Conexión lenta | Aumentar timeout |
| Corrupción | Descarga incompleta | Reintentar automáticamente |

## 📈 Estadísticas del Build

```
RAM:   14.4% (47,336 bytes de 327,680 bytes)
Flash: 76.4% (1,001,693 bytes de 1,310,720 bytes)
```

## 🎯 Mejoras Futuras

1. **Rollback automático**: Volver a versión anterior si falla
2. **Actualizaciones parciales**: Solo descargar cambios
3. **A/B testing**: Desplegar a subset de dispositivos
4. **Métricas**: Reportar éxito/falla de actualizaciones
5. **UI web**: Interfaz para gestionar actualizaciones

## ❓ Preguntas Frecuentes

**Q: ¿Qué pasa si la actualización falla?**
A: El dispositivo mantiene el firmware anterior y reintenta más tarde.

**Q: ¿Puedo forzar una actualización inmediata?**
A: Sí, reinicia el dispositivo o reduce `ota_check_interval`.

**Q: ¿Cuánto espacio necesito para OTA?**
A: Necesitas al menos el doble del tamaño del firmware en flash.

**Q: ¿Funciona con WiFi además de Ethernet?**
A: Sí, el sistema OTA funciona con cualquier conexión a internet.

---

## 🎉 ¡Listo!

Tu sistema OTA está completamente configurado y funcionando. Los dispositivos verificarán actualizaciones automáticamente cada 5 minutos y las instalarán de forma segura cuando estén disponibles.

Para una nueva versión:
1. Cambia el número en `platformio.ini`
2. Ejecuta `pio run`
3. Ejecuta el script de despliegue: `./deploy_script.sh 1.1.0`
4. ¡Listo! Los dispositivos se actualizarán solos desde ota.boisolo.com/ultrasonido/