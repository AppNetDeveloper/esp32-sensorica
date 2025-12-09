# 🌐 Sistema de Configuración Web para Multi-Sensor IoT Universal

## 📋 Resumen del Sistema

He implementado un sistema completo de configuración web con acceso físico, modos bridge/hotspot y soporte para 4 tipos de sensores. El sistema permite configurar todas las variables del dispositivo sin necesidad de reprogramar.

## 🎯 Características Principales

### ✅ **Modo Bridge (3 segundos botón)**
- **Botón GPIO 12**: Mantener presionado por 3 segundos para entrar en modo bridge
- **Ethernet ACTIVO**: El dispositivo sigue operando normalmente
- **WiFi temporal**: Solo para configuración, sin interrumpir operación
- **LED Azul**: Encendido fijo en modo bridge
- **WiFi AP**: Crea red `ESP32-Bridge` con contraseña `bridge123`

### 🔥 **Modo Hotspot (10 segundos botón)**
- **Botón GPIO 12**: Mantener presionado por 10 segundos para modo hotspot
- **Ethernet APAGADO**: Máximo ahorro de energía
- **WiFi puro**: Solo configuración, sensor y MQTT pausados
- **LEDs Verde+Rojo**: Parpadeando juntos
- **WiFi AP**: Crea red `ESP32-Hotspot` con contraseña `12345678`

### ⚙️ **Panel de Configuración Web Multi-pestaña**
- **5 Pestañas**: Red, MQTT, Dispositivo, Sensor, Sistema
- **Configuración dinámica**: Formas se adaptan según tipo de sensor
- **Validación en tiempo real**: Campos obligatorios y formatos válidos
- **Responsive**: Funciona en móviles y tablets
- **Guardado automático**: Configuración persistente en flash

### 🎛️ **Soporte Multi-Sensor**
- **Selector de tipo sensor**: Ultrasonido / 1 Pulsador / 2 Pulsadores / Vibración
- **Configuración dinámica**: Pines, topics, e inversores según sensor
- **MQTT individual**: Topics separados para cada sensor
- **Validación específica**: Configuración adecuada para cada tipo

### 🔐 **Protección contra Fallos OTA**
- **Contador de boots**: Detecta reinicios fallidos
- **Rollback automático**: Vuelve al firmware anterior si falla
- **Modo seguro**: Entrada automática en modo bridge si hay problemas

## 🚀 **Cómo Usar el Sistema**

### **1. Entrar en Modo Bridge (Operación Continua)**
```
1. Mantener presionado el botón (GPIO 12) por 3 segundos
2. LED Azul se enciende fijo, el dispositivo sigue operando
3. Conectar el móvil al WiFi "ESP32-Bridge" (contraseña: bridge123)
4. Abrir el navegador y visitar: http://192.168.4.1
```

### **2. Entrar en Modo Hotspot (Configuración Pura)**
```
1. Mantener presionado el botón (GPIO 12) por 10 segundos
2. LEDs Verde+Rojo parpadean juntos, Ethernet se apaga
3. Conectar el móvil al WiFi "ESP32-Hotspot" (contraseña: 12345678)
4. Abrir el navegador y visitar: http://192.168.4.1
```

### **3. Configurar Parámetros**

#### **🌐 Red**
- **DHCP**: Activar/desactivar DHCP
- **IP Estática**: Configurar IP manual si DHCP está desactivado
- **Gateway**: Puerta de enlace
- **Máscara**: Subred (ej: 255.255.255.0)
- **DNS**: Servidores DNS primario y secundario

#### **📡 MQTT**
- **Servidor**: IP del broker MQTT
- **Puerto**: Puerto MQTT (ej: 1883)
- **Autenticación**: Usuario y contraseña (opcionales)
- **Topics**: Configurables según tipo de sensor
- **Client ID**: Identificador único para MQTT
- **Keep Alive**: Tiempo de conexión keepalive

#### **🔧 Dispositivo**
- **Nombre**: Nombre descriptivo del dispositivo
- **Ubicación**: Lugar donde está instalado
- **Intervalo Sensor**: Tiempo entre lecturas (ms)
- **Lecturas Promedio**: Número de lecturas para promedio
- **Modo Debug**: Activar logs detallados

#### **🎛️ Sensor (NUEVO)**
- **Tipo Sensor**: Selector ultrasonido/1 pulsador/2 pulsadores/vibración
- **Pines GPIO**: Configurables según tipo
- **Inversión**: Para pulsadores (active low/high)
- **Topics MQTT**: Individuales por sensor
- **Cooldown**: Para sensor vibración

#### **💾 Sistema**
- **Versión Firmware**: Muestra versión actual
- **MAC Address**: Dirección física del dispositivo
- **Estado del Sistema**: Información completa del dispositivo
- **Resetear Configuración**: Vuelve a valores por defecto
- **Salir Modo Config**: Reinicia en modo normal

### **4. Guardar y Salir**
```
1. Configurar todos los parámetros necesarios
2. Seleccionar tipo de sensor y configurar sus parámetros específicos
3. Hacer clic en "💾 Guardar Configuración"
4. Esperar confirmación
5. El dispositivo se reiniciará automáticamente en modo normal
```

## 📊 **Valores por Defecto**

### **Red**
- DHCP: Activado
- IP Estática: 192.168.1.100
- Gateway: 192.168.1.1
- Subred: 255.255.255.0
- DNS1: 8.8.8.8
- DNS2: 8.8.4.4

### **MQTT**
- Servidor: 192.168.3.154
- Puerto: 1883
- Topic: multi-sensor/iot
- Client ID: Auto-generado
- Keep Alive: 60 segundos

### **Dispositivo**
- Nombre: Multi-Sensor-IoT-01
- Ubicación: Almacen_A
- Intervalo Sensor: 50ms
- Lecturas Promedio: 10
- Modo Debug: Desactivado

### **Sensor**
- Tipo: Ultrasonido HC-SR04
- Trigger: GPIO 25
- Echo: GPIO 26
- Pulsador 1: GPIO 13
- Pulsador 2: GPIO 14
- Vibración: GPIO 32
- Inversión: No
- Cooldown Vibración: 100ms

## 🔧 **Almacenamiento Persistente**

### **Donde se guardan los datos**
- **Memoria flash**: Usando Preferences API
- **Partición**: "multi-sensor-config"
- **No se borra**: Los datos sobreviven a actualizaciones OTA

### **Variables guardadas**
- Toda la configuración de red (DHCP, IPs, DNS)
- Configuración completa de MQTT
- Parámetros del dispositivo
- Configuración de sensores (tipo, pines, inversores, topics)
- Contadores de sistema para protección OTA

## 🛡️ **Protección contra Fallos**

### **Sistema de Rollback**
1. **Contador de boots**: Registra cada reinicio
2. **Tiempo de estabilidad**: Verifica que el sistema esté estable 2 minutos
3. **Máximo 3 boots fallidos**: Después de 3 fallos, entra en modo seguro
4. **Rollback automático**: Vuelve al firmware anterior si hay problemas

### **Recuperación**
```
Si el dispositivo no arranca correctamente:
1. Mantener presionado el botón GPIO 12 por 3 segundos
2. Entrará en modo bridge automáticamente
3. Configurar parámetros o resetear a valores por defecto
4. El sistema se marcará como "recuperado" y funcionará normalmente
```

## 📝 **Logs Serie (Debug)**

### **Modo Bridge**
```
=== MODO BRIDGE ACTIVADO ===
Ethernet activo, operación normal
LED Azul encendido fijo
AP IP address: 192.168.4.1
Servidor web iniciado
Conéctate a: ESP32-Bridge (pass: bridge123)
Luego visita: http://192.168.4.1
```

### **Modo Hotspot**
```
=== MODO HOTSPOT ACTIVADO ===
Ethernet apagado, máxima eficiencia
LEDs Verde+Rojo parpadeando juntos
AP IP address: 192.168.4.1
Servidor web iniciado
Conéctate a: ESP32-Hotspot (pass: 12345678)
Luego visita: http://192.168.4.1
```

### **Protección OTA**
```
OTA > ¡Demasiados intentos de boot fallidos! Entrando en modo seguro
OTA > Iniciando rollback al firmware anterior...
OTA > Rollback completado. Reiniciando...
```

### **Configuración**
```
Sistema de preferencias inicializado
Configuración cargada exitosamente
MQTT > Servidor: 192.168.3.154:1883
```

## 🔄 **Flujo de Operación Normal**

### **Arranque**
1. **Verificar botón**: Si está presionado → modo bridge
2. **Cargar configuración**: Lee valores de flash
3. **Iniciar Ethernet**: Conexión a red
4. **Conectar MQTT**: Usa configuración guardada
5. **Iniciar tareas**: Sensor, MQTT, OTA

### **Modo Bridge**
1. **Detener tareas normales**: Sensor y MQTT se pausan
2. **Iniciar WiFi AP**: Crea red local
3. **Iniciar servidor web**: Sirve páginas de configuración
4. **Esperar configuración**: Usuario configura parámetros
5. **Guardar y reiniciar**: Aplica cambios y vuelve a modo normal

### **Protección OTA**
1. **Verificación de boots**: Contador y tiempo
2. **Detección de fallos**: Boots fallidos consecutivos
3. **Modo seguro**: Entrada automática en modo bridge
4. **Recuperación**: Opción de rollback o reconfiguración

## 🎯 **Casos de Uso**

### **1. Primera Configuración**
```
1. Conectar alimentación
2. Mantener botón presionado 10 segundos (modo hotspot)
3. Conectar móvil al WiFi "ESP32-Hotspot"
4. Configurar MQTT, red y tipo de sensor
5. Guardar y reiniciar
```

### **2. Cambiar Servidor MQTT**
```
1. Mantener botón presionado 3 segundos (modo bridge)
2. Conectar móvil al WiFi "ESP32-Bridge"
3. Ir a pestaña MQTT
4. Cambiar servidor y puerto
5. Guardar configuración
6. Dispositivo se reconectará automáticamente
```

### **3. Cambiar a IP Estática**
```
1. Mantener botón presionado 3 segundos (modo bridge)
2. Conectar móvil al WiFi "ESP32-Bridge"
3. Ir a pestaña Red
4. Desactivar DHCP
5. Configurar IP estática, gateway y DNS
6. Guardar y reiniciar
```

### **4. Cambiar Tipo de Sensor**
```
1. Mantener botón presionado 3 segundos (modo bridge)
2. Ir a pestaña Sensor
3. Seleccionar nuevo tipo de sensor
4. Configurar pines GPIO y parámetros específicos
5. Guardar y reiniciar
```

### **5. Recuperación tras Fallo OTA**
```
1. Si el dispositivo reinicia continuamente, entrar en modo bridge
2. El sistema detectará boots fallidos
3. Configurar parámetros correctos
4. O resetear a valores por defecto
5. El sistema se marcará como recuperado
```

## ✨ **Ventajas del Sistema**

1. **Universal**: Soporta 4 tipos diferentes de sensores
2. **Acceso físico**: Solo se puede configurar localmente (seguro)
3. **Operación continua**: Modo bridge no interrumpe funcionamiento
4. **Sin reprogramación**: No necesita IDE o cables USB
5. **Configuración persistente**: Sobrevive a actualizaciones
6. **Protección automática**: Recuperación ante fallos con rollback
7. **Intuitivo**: Interfaz web moderna y responsiva
8. **Validación**: Previene configuraciones incorrectas
9. **Multi-modo**: Bridge y hotspot según necesidad
10. **Debug completo**: Logs detallados para diagnóstico

## 🚨 **Consideraciones de Seguridad**

- **Acceso físico**: El botón físico previene acceso remoto no autorizado
- **WiFi temporal**: El modo bridge solo funciona con acceso físico
- **Validación**: Los datos se validan antes de guardar
- **Rollback automático**: Protege contra actualizaciones fallidas
- **Defaults seguros**: Valores por defecto seguros y funcionales