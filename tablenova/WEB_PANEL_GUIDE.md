# 🌐 Sistema de Configuración Web para ESP32 Sensor

## 📋 Resumen del Sistema

He implementado un sistema completo de configuración web con acceso físico, modo bridge y protección contra fallos OTA. El sistema permite configurar todas las variables del dispositivo sin necesidad de reprogramar.

## 🎯 Características Principales

### ✅ **Modo Bridge con Botón Físico**
- **Botón GPIO 12**: Mantener presionado por 3 segundos para entrar en modo bridge
- **LED GPIO 2**: Indicador visual del modo bridge (encendido cuando está activo)
- **WiFi AP**: Crea red `Sensor-Config` con contraseña `sensor2024`
- **IP del servidor**: `192.168.4.1` (acceso automático desde dispositivos conectados)

### ⚙️ **Panel de Configuración Web**
- **4 Pestañas**: Red, MQTT, Dispositivo, Sistema
- **Validación en tiempo real**: Campos obligatorios y formatos válidos
- **Responsive**: Funciona en móviles y tablets
- **Guardado automático**: Configuración persistente en flash

### 🔐 **Protección contra Fallos OTA**
- **Contador de boots**: Detecta reinicios fallidos
- **Rollback automático**: Vuelve al firmware anterior si falla
- **Modo seguro**: Entrada automática en modo bridge si hay problemas

## 🚀 **Cómo Usar el Sistema**

### **1. Entrar en Modo Bridge**
```
1. Mantener presionado el botón (GPIO 12) por 3 segundos
2. El LED (GPIO 2) se encenderá indicando modo bridge
3. Conectar el móvil al WiFi "Sensor-Config" (contraseña: sensor2024)
4. Abrir el navegador y visitar: http://192.168.4.1
```

### **2. Configurar Parámetros**

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
- **Topic**: Topic donde publicar datos del sensor
- **Client ID**: Identificador único para MQTT
- **Keep Alive**: Tiempo de conexión keepalive

#### **🔧 Dispositivo**
- **Nombre**: Nombre descriptivo del dispositivo
- **Ubicación**: Lugar donde está instalado
- **Intervalo Sensor**: Tiempo entre lecturas (ms)
- **Cantidad Lecturas**: Número de lecturas para mediana
- **Modo Debug**: Activar logs detallados

#### **💾 Sistema**
- **Versión Firmware**: Muestra versión actual
- **MAC Address**: Dirección física del dispositivo
- **Estado del Sistema**: Información completa del dispositivo
- **Resetear Configuración**: Vuelve a valores por defecto
- **Salir Modo Bridge**: Reinicia en modo normal

### **3. Guardar y Salir**
```
1. Configurar todos los parámetros necesarios
2. Hacer clic en "💾 Guardar Configuración"
3. Esperar confirmación
4. El dispositivo se reiniciará automáticamente en modo normal
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
- Topic: sensor/distance
- Client ID: Auto-generado
- Keep Alive: 60 segundos

### **Dispositivo**
- Nombre: ESP32-Sensor
- Ubicación: Desconocida
- Intervalo Sensor: 50ms
- Cantidad Lecturas: 10
- Modo Debug: Desactivado

## 🔧 **Almacenamiento Persistente**

### **Donde se guardan los datos**
- **Memoria flash**: Usando Preferences API
- **Partición**: "sensor-config"
- **No se borra**: Los datos sobreviven a actualizaciones OTA

### **Variables guardadas**
- Toda la configuración de red (DHCP, IPs, DNS)
- Configuración completa de MQTT
- Parámetros del dispositivo
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
LED indicador encendido
AP IP address: 192.168.4.1
Servidor web iniciado
Conéctate a: Sensor-Config
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
2. Mantener botón presionado 3 segundos
3. Conectar móvil al WiFi del sensor
4. Configurar MQTT y red
5. Guardar y reiniciar
```

### **2. Cambiar Servidor MQTT**
```
1. Entrar en modo bridge
2. Ir a pestaña MQTT
3. Cambiar servidor y puerto
4. Guardar configuración
5. Dispositivo se reconectará automáticamente
```

### **3. Cambiar a IP Estática**
```
1. Entrar en modo bridge
2. Ir a pestaña Red
3. Desactivar DHCP
4. Configurar IP estática, gateway y DNS
5. Guardar y reiniciar
```

### **4. Recuperación tras Fallo OTA**
```
1. Si el dispositivo reinicia continuamente, entrar en modo bridge
2. El sistema detectará boots fallidos
3. Configurar parámetros correctos
4. O resetear a valores por defecto
5. El sistema se marcará como recuperado
```

## ✨ **Ventajas del Sistema**

1. **Acceso físico**: Solo se puede configurar localmente (seguro)
2. **Sin reprogramación**: No necesita IDE o cables USB
3. **Configuración persistente**: Sobrevive a actualizaciones
4. **Protección automática**: Recuperación ante fallos
5. **Intuitivo**: Interfaz web moderna y responsiva
6. **Validación**: Previene configuraciones incorrectas
7. **Debug completo**: Logs detallados para diagnóstico

## 🚨 **Consideraciones de Seguridad**

- **Acceso físico**: El botón físico previene acceso remoto no autorizado
- **WiFi temporal**: El modo bridge solo funciona con acceso físico
- **Validación**: Los datos se validan antes de guardar
- **Rollback automático**: Protege contra actualizaciones fallidas
- **Defaults seguros**: Valores por defecto seguros y funcionales