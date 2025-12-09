#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiAP.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <Preferences.h>

// -- INICIO: SECCIÓN DE CONFIGURACIÓN MULTI-SENSOR --

// TIPOS DE SENSORES SOPORTADOS
enum SensorType {
  SENSOR_ULTRASONIC = 0,    // HC-SR04 - Distancia
  SENSOR_SINGLE_BUTTON = 1, // 1 Pulsador digital
  SENSOR_DUAL_BUTTONS = 2,  // 2 Pulsadores digitales
  SENSOR_VIBRATION = 3      // Sensor de vibraciones SW-420
};

// -- PINES PARA SENSORES (CONFIGURABLES) --
#define TRIG_PIN 25      // Ultrasonido Trigger / Pulsador 1
#define ECHO_PIN 26      // Ultrasonido Echo / Pulsador 2
#define BUTTON1_PIN 13   // Pulsador 1 (alternativo)
#define BUTTON2_PIN 14   // Pulsador 2 (alternativo)
#define VIBRATION_PIN 32 // Sensor de vibraciones SW-420

// -- PINES PARA MODO CONFIGURACIÓN --
#define CONFIG_BUTTON_PIN 12  // Botón para entrar en modo bridge/hotspot
#define CONFIG_LED_PIN 2     // LED indicador de modo bridge/hotspot
#define STATUS_LED_PIN 4     // LED verde para estado online
#define ERROR_LED_PIN 5      // LED rojo para estado de error

// -- CONFIGURACIÓN WIFI AP MODO BRIDGE/HOTSPOT --
const char* hotspot_ssid = "ESP32-Hotspot";
const char* hotspot_password = "12345678";

// Configuración para modo bridge
const char* ap_ssid = "ESP32-Bridge";
const char* ap_password = "bridge123";

// -- TIMERS PARA BOTÓN --
const unsigned long bridgeButtonHoldTime = 3000;    // 3 segundos para modo bridge
const unsigned long hotspotButtonHoldTime = 10000;  // 10 segundos para modo hotspot

// -- FIN: NUEVA SECCIÓN DE CONFIGURACIÓN DEL SENSOR --


// -- INICIO: SECCIÓN DE CONFIGURACIÓN ETHERNET PARA WT32-ETH01 --

// Configuración específica y verificada para la placa WT32-ETH01
#define ETH_PHY_TYPE   ETH_PHY_LAN8720
#define ETH_PHY_ADDR   1
#define ETH_PHY_POWER  16  // El pin de alimentación del PHY en la WT32-ETH01 es el GPIO 16
#define ETH_PHY_MDC    23
#define ETH_PHY_MDIO   18
#define ETH_CLK_MODE   ETH_CLOCK_GPIO0_IN // El modo de reloj para la WT32-ETH01 es entrada en GPIO 0

// Variable para saber si estamos conectados a la red
static bool eth_connected = false;

// -- FIN: SECCIÓN DE CONFIGURACIÓN ETHERNET --


// Configuración de MQTT - Ahora usa configuración dinámica

// La librería PubSubClient necesita una instancia de WiFiClient (que es compatible con ETH).
WiFiClient espClient;
PubSubClient client(espClient);

// El resto de tus variables globales permanece igual
const int NUM_READINGS = 10;
volatile int readings[NUM_READINGS];
volatile int readingIndex = 0;
volatile int filteredValue = 0;
volatile bool newDataAvailable = false;
volatile int lastPublishedValue = -1;
SemaphoreHandle_t sensorMutex;
unsigned long lastReconnectAttempt = 0;
unsigned long baseReconnectInterval = 1000;
int reconnectAttempts = 0;
const int maxReconnectAttempts = 10;
unsigned long lastSensorRestart = 0;
const unsigned long sensorRestartInterval = 1800000;

// -- Configuración OTA --
const char* ota_version_url = "http://ota.boisolo.com/ultrasonido/version.json";  // URL del JSON de versiones
const char* firmware_base_url = "http://ota.boisolo.com/ultrasonido/";  // URL base para los archivos de firmware
const unsigned long ota_check_interval = 300000;  // Revisar actualizaciones cada 5 minutos
unsigned long lastOtaCheck = 0;
const int ota_timeout = 30000;  // Timeout para actualización OTA (30 segundos)

// -- SISTEMA DE CONFIGURACIÓN PERSISTENTE --

// Estructura para configuración de red
struct NetworkConfig {
  bool dhcpEnabled;
  String staticIP;
  String gateway;
  String subnet;
  String dns1;
  String dns2;
};

// Estructura para configuración MQTT
struct MQTTConfig {
  String server;
  int port;
  String username;
  String password;
  String topic;
  String clientId;
  int keepAlive;
};

// Estructura para configuración del dispositivo
struct DeviceConfig {
  String deviceName;
  String location;
  int sensorInterval;
  int readingsCount;
  bool debugMode;

  // NUEVO: Configuración de sensores
  int sensorType; // 0=ultrasonido, 1=1 pulsador, 2=2 pulsadores, 3=vibración
  int button1Pin, button2Pin, vibrationPin;
  bool button1Invert, button2Invert;
  String button1Topic, button2Topic, vibrationTopic;
  String mainMqttTopic; // Topic principal para ultrasonido
  int vibrationThreshold; // Umbral de sensibilidad para vibraciones
};

// Variables globales de configuración
NetworkConfig networkConfig;
MQTTConfig mqttConfig;
DeviceConfig deviceConfig;

// Variables para estado de pulsadores
bool button1State = false;
bool button2State = false;
bool lastButton1State = false;
bool lastButton2State = false;
unsigned long lastButton1Change = 0;
unsigned long lastButton2Change = 0;

// Variables para sensor de vibraciones
bool vibrationState = false;
bool lastVibrationState = false;
unsigned long lastVibrationChange = 0;
unsigned long vibrationCooldown = 100; // 100ms cooldown entre detecciones

// Variables para modo bridge/hotspot
bool bridgeMode = false;
bool hotspotMode = false;
WebServer* configServer = NULL;
Preferences preferences;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
const unsigned long bridgeModeTimeout = 300000;   // 5 minutos timeout en modo bridge
unsigned long bridgeModeEnterTime = 0;
unsigned long hotspotModeEnterTime = 0;

// Variables de estado del sistema
struct SystemStatus {
  unsigned long uptime;
  unsigned long lastMQTTConnection;
  unsigned long lastSensorReading;
  unsigned int mqttConnectionAttempts;
  unsigned int otaUpdatesCount;
  unsigned int systemRestarts;
  float currentDistance;
  int wifiSignalStrength;
  int freeHeap;
  float cpuUsage;
};

SystemStatus systemStatus;

// -- Estructura para información de versión --
struct FirmwareInfo {
  String version;
  String url;
  String checksum;
  bool mandatory;
  String release_notes;
};

// -- Prototipos de funciones OTA --
bool checkForUpdates();
bool performOTAUpdate(const FirmwareInfo& firmwareInfo);
String getCurrentFirmwareVersion();
bool compareVersions(const String& current, const String& available);
String calculateSHA256(const String& filePath);

// Prototipos de tareas y funciones
void WiFiEvent(WiFiEvent_t event);
bool reconectarMQTT();
void sensorTask(void *pvParameters);

// Funciones de lectura de sensores
void readButtons();
void readVibrationSensor();
void publishButtonState(int buttonId, bool state, String topic);
void publishVibrationState(bool state);
void mqttTask(void *pvParameters);
void otaTask(void *pvParameters);

// Prototipos de funciones de configuración
void initializePreferences();
void loadConfiguration();
void saveConfiguration();
void resetToDefaults();
void setupSensorPins();
void checkConfigButton();
void enterBridgeMode();
void exitBridgeMode();
void enterHotspotMode();
void exitHotspotMode();
void handleConfigWebRequests();
String getConfigFormHTML();
void handleRoot();
void handleSaveConfig();
void handleReset();
void handleStatus();
bool validateNetworkConfig(String ip, String gateway, String subnet);
bool validateMQTTConfig(String server, int port);

// Nuevas funciones para LEDs y modo hotspot
void initializeLEDs();
void updateStatusLEDs();
void setOnlineStatus();
void setErrorStatus();
void clearErrorStatus();
void blinkLEDs(int times, int onTime, int offTime);

// Funciones de protección OTA
bool safeOTACheck();
void rollbackToFirmware();
void markBootAttempt();

// Nuevas funciones de estado y mejoras
void updateSystemStatus();
void initializeSystemStatus();
void checkBridgeTimeout();
String generateSystemStatusJSON();
void logSystemEvent(String event, String details = "");


// El resto de tus funciones auxiliares (sortArray, calcularMediana, etc.) no necesitan cambios.
void sortArray(int arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        int temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}
int calcularMediana(int arr[], int n) {
  int temp[n];
  for (int i = 0; i < n; i++) {
    temp[i] = arr[i];
  }
  sortArray(temp, n);
  return (temp[n/2 - 1] + temp[n/2]) / 2;
}
String generarClientId() {
  uint8_t mac[6];
  ETH.macAddress(mac); // Usamos la MAC de Ethernet
  char id[25];
  sprintf(id, "ESP32Client-%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(id);
}


void setup() {
  Serial.begin(115200);
  delay(500); // Pequeña pausa para que el monitor serie se estabilice
  Serial.println("\n\n--- INICIANDO MULTI-SENSOR IOT UNIVERSAL (WT32-ETH01) ---");

  sensorMutex = xSemaphoreCreateMutex();

  // Configuración de pines para LEDs y botón de configuración
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CONFIG_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  // La configuración de pines de sensor se hará después de cargar configuración

  // Inicializar LEDs
  initializeLEDs();

  // Inicializar sistema de configuración persistente
  Serial.println("Inicializando sistema de configuración...");
  initializePreferences();
  loadConfiguration();

  // Configurar pines de sensor según tipo
  setupSensorPins();

  // Inicializar sistema de estado y monitoreo
  Serial.println("Inicializando sistema de estado...");
  initializeSystemStatus();

  // Verificar si debemos entrar en modo bridge o hotspot (botón presionado al inicio)
  checkConfigButton();

  if (bridgeMode) {
    enterBridgeMode();
    return; // Salir de setup si estamos en modo bridge
  }

  if (hotspotMode) {
    enterHotspotMode();
    return; // Salir de setup si estamos en modo hotspot
  }

  // Marcar intento de boot (para protección OTA)
  markBootAttempt();

  // -- INICIO: INICIALIZACIÓN DE RED --

  Serial.println("Configurando conexión Ethernet...");
  WiFi.onEvent(WiFiEvent);

  Serial.println("Intentando conectar a la red por RJ45...");
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);
  
  // -- FIN: INICIALIZACIÓN DE RED --

  // Configurar MQTT con valores guardados
  client.setServer(mqttConfig.server.c_str(), mqttConfig.port);

  Serial.println("Sensor de ultrasonido JSN-SR04T configurado.");
  Serial.print("MQTT Server: ");
  Serial.println(mqttConfig.server);
  Serial.print("Topic: ");
  Serial.println(mqttConfig.topic);
  Serial.println("------------------------------------");

  lastSensorRestart = millis();

  // Creación de tareas
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(otaTask, "OTA Task", 10000, NULL, 2, NULL, 1);  // Prioridad 2 para OTA
}

void loop() {
  // Si estamos en modo bridge, manejar solicitudes web y verificar botón
  if (bridgeMode) {
    checkConfigButton(); // Verificar si se presiona el botón para salir del modo bridge
    checkBridgeTimeout(); // Verificar timeout del modo bridge
    if (configServer) {
      configServer->handleClient();
    }
    delay(10);
    return;
  }

  // Si estamos en modo hotspot, manejar solicitud web y verificar botón
  if (hotspotMode) {
    checkConfigButton(); // Verificar si se presiona el botón para salir del modo hotspot

    // Parpadear ambos LEDs en modo hotspot
    static unsigned long lastHotspotBlink = 0;
    static bool hotspotBlinkState = false;

    if (millis() - lastHotspotBlink > 500) { // Parpadeo cada 500ms
      hotspotBlinkState = !hotspotBlinkState;
      digitalWrite(STATUS_LED_PIN, hotspotBlinkState ? HIGH : LOW);
      digitalWrite(ERROR_LED_PIN, hotspotBlinkState ? HIGH : LOW);
      lastHotspotBlink = millis();
    }

    if (configServer) {
      configServer->handleClient();
    }
    delay(10);
    return;
  }

  // Operación normal - leer sensores no-ultrasonidos
  readButtons();
  readVibrationSensor();

  // Operación normal - actualizar estado del sistema y LEDs
  updateSystemStatus();
  updateStatusLEDs();

  // Operación normal - reinicio programado
  if (millis() - lastSensorRestart >= sensorRestartInterval) {
    logSystemEvent("SCHEDULED_RESTART", "Reinicio programado del sistema");
    Serial.println("Reinicio programado del ESP32...");
    ESP.restart();
  }
  delay(1000);
}

// Tarea para leer el sensor de ultrasonido (compatible con JSN-SR04T en modo por defecto)
void sensorTask(void *pvParameters) {
  long duration;
  int distance;

  for (;;) {
    // Generar el pulso de disparo (trigger)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Medir la duración del pulso de eco (echo)
    duration = pulseIn(ECHO_PIN, HIGH);

    // NUEVO: Log de depuración para ver la lectura cruda del sensor
    Serial.print("Sensor RAW > Duracion del pulso (us): ");
    Serial.print(duration);

    // Calcular la distancia en milímetros
    distance = duration * 0.343 / 2;

    // NUEVO: Log de depuración para ver la distancia calculada
    Serial.print(" | Distancia calculada (mm): ");
    Serial.println(distance);
    
    // Proteger acceso al arreglo y al índice
    if (xSemaphoreTake(sensorMutex, (TickType_t) 10) == pdTRUE) {
      readings[readingIndex] = distance;
      readingIndex++;
      xSemaphoreGive(sensorMutex);
      
      // Cuando se acumulen las lecturas, calcular la mediana
      if (readingIndex >= NUM_READINGS) {
        if (xSemaphoreTake(sensorMutex, (TickType_t) 10) == pdTRUE) {
          int med = calcularMediana((int*)readings, NUM_READINGS);
          filteredValue = med;
          newDataAvailable = true;
          // Serial.print("Nueva mediana calculada (mm): "); // Descomentar para depuración intensiva
          // Serial.println(filteredValue);
          readingIndex = 0;
          xSemaphoreGive(sensorMutex);
        }
      }
    }
    
    // Espera 50 ms para la siguiente lectura para evitar interferencias
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Funciones de lectura de sensores
void readButtons() {
  if (deviceConfig.sensorType == SENSOR_SINGLE_BUTTON || deviceConfig.sensorType == SENSOR_DUAL_BUTTONS) {
    // Leer Pulsador 1
    bool currentButton1State = digitalRead(deviceConfig.button1Pin);

    // Aplicar inversión si está configurada
    if (deviceConfig.button1Invert) {
      currentButton1State = !currentButton1State;
    }

    // Detectar cambios
    if (currentButton1State != lastButton1State && millis() - lastButton1Change > 50) {
      lastButton1State = currentButton1State;
      lastButton1Change = millis();
      button1State = currentButton1State;

      // Publicar estado
      publishButtonState(1, button1State, deviceConfig.button1Topic);

      if (deviceConfig.debugMode) {
        Serial.print("DEBUG > Pulsador 1: ");
        Serial.println(button1State ? "PRESIONADO" : "SUELTO");
      }
    }

    // Leer Pulsador 2 (solo para modo dual)
    if (deviceConfig.sensorType == SENSOR_DUAL_BUTTONS) {
      bool currentButton2State = digitalRead(deviceConfig.button2Pin);

      // Aplicar inversión si está configurada
      if (deviceConfig.button2Invert) {
        currentButton2State = !currentButton2State;
      }

      // Detectar cambios
      if (currentButton2State != lastButton2State && millis() - lastButton2Change > 50) {
        lastButton2State = currentButton2State;
        lastButton2Change = millis();
        button2State = currentButton2State;

        // Publicar estado
        publishButtonState(2, button2State, deviceConfig.button2Topic);

        if (deviceConfig.debugMode) {
          Serial.print("DEBUG > Pulsador 2: ");
          Serial.println(button2State ? "PRESIONADO" : "SUELTO");
        }
      }
    }
  }
}

void readVibrationSensor() {
  if (deviceConfig.sensorType == SENSOR_VIBRATION) {
    bool currentVibrationState = digitalRead(deviceConfig.vibrationPin);

    // Detectar vibración (LOW = vibración detectada en SW-420)
    bool vibrationDetected = (currentVibrationState == LOW);

    // Aplicar cooldown para evitar múltiples detecciones rápidas
    if (vibrationDetected && millis() - lastVibrationChange > deviceConfig.vibrationThreshold) {
      lastVibrationChange = millis();
      vibrationState = true;

      // Publicar detección de vibración
      publishVibrationState(true);

      if (deviceConfig.debugMode) {
        Serial.println("DEBUG > ¡VIBRACIÓN DETECTADA!");
      }
    } else if (!vibrationDetected && vibrationState) {
      // Resetear estado cuando no hay vibración
      vibrationState = false;
    }
  }
}

void publishButtonState(int buttonId, bool state, String topic) {
  if (client.connected()) {
    StaticJsonDocument<200> doc;
    char jsonBuffer[200];

    doc["value"] = state ? 1 : 0;
    doc["button"] = buttonId;
    doc["device"] = deviceConfig.deviceName;
    doc["location"] = deviceConfig.location;
    doc["timestamp"] = millis();

    serializeJson(doc, jsonBuffer);

    bool result = client.publish(topic.c_str(), jsonBuffer);
    if (deviceConfig.debugMode) {
      Serial.print("MQTT > Publicando botón ");
      Serial.print(buttonId);
      Serial.print(" a ");
      Serial.print(topic);
      Serial.print(": ");
      Serial.print(result ? "OK" : "FALLÓ");
      Serial.print(" -> ");
      Serial.println(jsonBuffer);
    }
  }
}

void publishVibrationState(bool state) {
  if (client.connected()) {
    StaticJsonDocument<200> doc;
    char jsonBuffer[200];

    doc["vibration"] = state ? 1 : 0;
    doc["device"] = deviceConfig.deviceName;
    doc["location"] = deviceConfig.location;
    doc["timestamp"] = millis();

    serializeJson(doc, jsonBuffer);

    bool result = client.publish(deviceConfig.vibrationTopic.c_str(), jsonBuffer);
    if (deviceConfig.debugMode) {
      Serial.print("MQTT > Publicando vibración a ");
      Serial.print(deviceConfig.vibrationTopic);
      Serial.print(": ");
      Serial.print(result ? "OK" : "FALLÓ");
      Serial.print(" -> ");
      Serial.println(jsonBuffer);
    }
  }
}

// Tarea de MQTT (sin cambios)
void mqttTask(void *pvParameters) {
  unsigned long lastPublishTime = 0;
  for (;;) {
    if (!eth_connected) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    if (!client.connected()) {
      reconectarMQTT();
    }
    
    client.loop();
    
    if (millis() - lastPublishTime >= 500) {
      lastPublishTime = millis();
      
      if (newDataAvailable) {
        StaticJsonDocument<200> doc;
        char jsonBuffer[512];
        doc["value"] = filteredValue;
        serializeJson(doc, jsonBuffer);
        
        Serial.print("MQTT > Publicando valor: ");
        Serial.println(jsonBuffer);
        
        if (client.publish(mqttConfig.topic.c_str(), jsonBuffer)) {
          // Serial.println("MQTT > Datos enviados correctamente."); // Descomentar si se necesita confirmación
          lastPublishedValue = filteredValue;
          newDataAvailable = false;
        } else {
          Serial.println("MQTT > Error al enviar datos, MQTT desconectado. Se reintentará...");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Manejador de eventos de red (sin cambios)
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("LOG RED > ETH Iniciado");
      ETH.setHostname("wt32-eth01-sensor"); 
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("LOG RED > Cable de red enchufado.");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("LOG RED > ¡CONEXIÓN ESTABLECIDA!");
      Serial.print("LOG RED >   MAC: ");
      Serial.println(ETH.macAddress());
      Serial.print("LOG RED >   IPv4: ");
      Serial.println(ETH.localIP());
      Serial.print("LOG RED >   Velocidad: ");
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      if (ETH.fullDuplex()) {
        Serial.println("LOG RED >   Modo: Full Duplex");
      }
      Serial.println("------------------------------------");
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("LOG RED > Cable de red desenchufado.");
      eth_connected = false;
      break;
    default:
      break;
  }
}


// Función para reconectar a MQTT (sin cambios)
bool reconectarMQTT() {
  String clientId = mqttConfig.clientId.length() > 0 ? mqttConfig.clientId : generarClientId();
  Serial.print("MQTT > Intentando reconectar con ClientID: ");
  Serial.println(clientId);
  Serial.print("MQTT > Servidor: ");
  Serial.print(mqttConfig.server);
  Serial.print(":");
  Serial.println(mqttConfig.port);

  // Intentar conectar con autenticación si está configurada
  bool connected = false;
  if (mqttConfig.username.length() > 0) {
    connected = client.connect(clientId.c_str(), mqttConfig.username.c_str(), mqttConfig.password.c_str());
  } else {
    connected = client.connect(clientId.c_str());
  }

  if (connected) {
    Serial.println("MQTT > Conectado al broker.");
    return true;
  } else {
    Serial.print("MQTT > Fallo la conexión, rc=");
    Serial.println(client.state());
    return false;
  }
}


void reiniciarESP32() {
  Serial.println("Reiniciando ESP32...");
  delay(1000);
  ESP.restart();
}

// -- FUNCIONES OTA --

// Tarea para verificar y realizar actualizaciones OTA
void otaTask(void *pvParameters) {
  for (;;) {
    // Esperar a que haya conexión Ethernet antes de verificar actualizaciones
    if (eth_connected) {
      if (checkForUpdates()) {
        Serial.println("OTA > Se encontró una actualización, procediendo con la instalación...");
        // checkForUpdates() ya maneja la actualización si es necesaria
      }
    }

    // Esperar el intervalo de verificación OTA
    vTaskDelay(ota_check_interval / portTICK_PERIOD_MS);
  }
}

// Verificar si hay actualizaciones disponibles
bool checkForUpdates() {
  Serial.println("OTA > Verificando actualizaciones...");

  HTTPClient http;
  http.setTimeout(ota_timeout);

  // Obtener información de versión del servidor
  if (!http.begin(ota_version_url)) {
    Serial.println("OTA > No se pudo conectar al servidor de versiones");
    return false;
  }

  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK) {
    Serial.print("OTA > Error al obtener información de versión, código HTTP: ");
    Serial.println(httpCode);
    http.end();
    return false;
  }

  // Parsear JSON de versión
  String payload = http.getString();
  http.end();

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("OTA > Error al parsear JSON de versión: ");
    Serial.println(error.c_str());
    return false;
  }

  // Extraer información del firmware
  FirmwareInfo firmwareInfo;
  firmwareInfo.version = doc["version"].as<String>();
  firmwareInfo.url = doc["url"].as<String>();
  firmwareInfo.checksum = doc["checksum"].as<String>();
  firmwareInfo.mandatory = doc["mandatory"] | false;
  firmwareInfo.release_notes = doc["release_notes"].as<String>();

  // Obtener versión actual
  String currentVersion = getCurrentFirmwareVersion();
  Serial.print("OTA > Versión actual: ");
  Serial.println(currentVersion);
  Serial.print("OTA > Versión disponible: ");
  Serial.println(firmwareInfo.version);

  // Si hay notas de lanzamiento, mostrarlas
  if (firmwareInfo.release_notes.length() > 0) {
    Serial.print("OTA > Notas de la versión: ");
    Serial.println(firmwareInfo.release_notes);
  }

  // Comparar versiones
  if (compareVersions(currentVersion, firmwareInfo.version)) {
    Serial.println("OTA > Se encontró una actualización disponible");

    if (firmwareInfo.mandatory) {
      Serial.println("OTA > Actualización obligatoria");
    }

    // Realizar la actualización
    return performOTAUpdate(firmwareInfo);
  } else {
    Serial.println("OTA > El firmware está actualizado");
    return false;
  }
}

// Realizar la actualización OTA
bool performOTAUpdate(const FirmwareInfo& firmwareInfo) {
  Serial.println("OTA > Iniciando actualización...");
  Serial.print("OTA > URL del firmware: ");
  Serial.println(firmwareInfo.url);

  // Detener tareas críticas pero mantener MQTT activo para reportar estado
  vTaskSuspendAll();

  HTTPClient http;
  http.setTimeout(ota_timeout);

  if (!http.begin(firmwareInfo.url)) {
    Serial.println("OTA > No se pudo conectar al servidor de firmware");
    xTaskResumeAll();
    return false;
  }

  // Iniciar la actualización OTA
  httpUpdate.rebootOnUpdate(true);

  // Configurar headers para la actualización
  http.addHeader("User-Agent", "ESP32-OTA-Client");

  t_httpUpdate_return ret = httpUpdate.update(http);

  bool success = false;

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("OTA > Error en actualización: (%d) %s\n",
                   httpUpdate.getLastError(),
                   httpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("OTA > No hay actualizaciones disponibles");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("OTA > ¡Actualización exitosa! Reiniciando...");
      success = true;
      break;
  }

  http.end();

  // Si la actualización falló, reanudar tareas
  if (!success) {
    xTaskResumeAll();
    Serial.println("OTA > Actualización fallida, reanudando operaciones normales");
  }

  return success;
}

// Obtener versión actual del firmware
String getCurrentFirmwareVersion() {
  const esp_partition_t *running_partition = esp_ota_get_running_partition();
  if (running_partition != NULL) {
    esp_app_desc_t app_desc;
    if (esp_ota_get_partition_description(running_partition, &app_desc) == ESP_OK) {
      Serial.printf("OTA > Versión del app_desc: %s\n", app_desc.version);
      return String(app_desc.version);
    }
  }

  // Si no podemos obtener la versión de app_desc, usamos la versión de compilación
  Serial.println("OTA > Usando versión de compilación");
  return String(FW_VERSION);
}

// Comparar versiones usando formato semántico (ej: 1.2.3 > 1.2.2)
bool compareVersions(const String& current, const String& available) {
  Serial.println("OTA > Comparando versiones...");

  // Eliminar prefijos como 'v' si existen
  String currentClean = current;
  String availableClean = available;

  if (currentClean.startsWith("v")) currentClean = currentClean.substring(1);
  if (availableClean.startsWith("v")) availableClean = availableClean.substring(1);

  // Extraer números de versión
  int current_major = 0, current_minor = 0, current_patch = 0;
  int available_major = 0, available_minor = 0, available_patch = 0;

  // Parsear versión actual
  sscanf(currentClean.c_str(), "%d.%d.%d", &current_major, &current_minor, &current_patch);

  // Parsear versión disponible
  sscanf(availableClean.c_str(), "%d.%d.%d", &available_major, &available_minor, &available_patch);

  Serial.printf("OTA > Versión actual: %d.%d.%d\n", current_major, current_minor, current_patch);
  Serial.printf("OTA > Versión disponible: %d.%d.%d\n", available_major, available_minor, available_patch);

  // Comparar versiones
  if (available_major > current_major) return true;
  if (available_major == current_major && available_minor > current_minor) return true;
  if (available_major == current_major && available_minor == current_minor && available_patch > current_patch) return true;

  return false;
}

// Calcular checksum SHA256 de un archivo (opcional, para verificación de integridad)
String calculateSHA256(const String& filePath) {
  // Esta función es opcional para verificación básica
  // Para implementación completa, necesitarías descargar el archivo y calcular SHA256
  // Por ahora, retornamos un string vacío
  Serial.println("OTA > Verificación de checksum no implementada (opcional)");
  return "";
}

// =====================================================================
// FUNCIÓNES DE CONFIGURACIÓN PERSISTENTE
// =====================================================================

void initializePreferences() {
  preferences.begin("sensor-config", false);
  Serial.println("Sistema de preferencias inicializado");
}

void loadConfiguration() {
  // Cargar configuración de red
  networkConfig.dhcpEnabled = preferences.getBool("dhcp", true);
  networkConfig.staticIP = preferences.getString("staticIP", "192.168.1.100");
  networkConfig.gateway = preferences.getString("gateway", "192.168.1.1");
  networkConfig.subnet = preferences.getString("subnet", "255.255.255.0");
  networkConfig.dns1 = preferences.getString("dns1", "8.8.8.8");
  networkConfig.dns2 = preferences.getString("dns2", "8.8.4.4");

  // Cargar configuración MQTT
  mqttConfig.server = preferences.getString("mqttServer", "192.168.3.154");
  mqttConfig.port = preferences.getInt("mqttPort", 1883);
  mqttConfig.username = preferences.getString("mqttUser", "");
  mqttConfig.password = preferences.getString("mqttPass", "");
  mqttConfig.topic = preferences.getString("mqttTopic", "sensor/distance");
  mqttConfig.clientId = preferences.getString("mqttClientId", generarClientId());
  mqttConfig.keepAlive = preferences.getInt("mqttKeepAlive", 60);

  // Cargar configuración del dispositivo
  deviceConfig.deviceName = preferences.getString("deviceName", "Multi-Sensor-IoT-01");
  deviceConfig.location = preferences.getString("location", "Desconocida");
  deviceConfig.sensorInterval = preferences.getInt("sensorInterval", 50);
  deviceConfig.readingsCount = preferences.getInt("readingsCount", 10);
  deviceConfig.debugMode = preferences.getBool("debugMode", false);

  // Cargar configuración de sensores
  deviceConfig.sensorType = preferences.getInt("sensorType", SENSOR_ULTRASONIC);
  deviceConfig.button1Pin = preferences.getInt("button1Pin", BUTTON1_PIN);
  deviceConfig.button2Pin = preferences.getInt("button2Pin", BUTTON2_PIN);
  deviceConfig.vibrationPin = preferences.getInt("vibrationPin", VIBRATION_PIN);
  deviceConfig.button1Invert = preferences.getBool("button1Invert", false);
  deviceConfig.button2Invert = preferences.getBool("button2Invert", false);
  deviceConfig.button1Topic = preferences.getString("button1Topic", "sensor/button1");
  deviceConfig.button2Topic = preferences.getString("button2Topic", "sensor/button2");
  deviceConfig.vibrationTopic = preferences.getString("vibrationTopic", "sensor/vibration");
  deviceConfig.mainMqttTopic = preferences.getString("mainMqttTopic", "multi-sensor/iot");
  deviceConfig.vibrationThreshold = preferences.getInt("vibrationThreshold", 100); // 100ms por defecto

  Serial.println("Configuración cargada exitosamente");
}

void saveConfiguration() {
  // Guardar configuración de red
  preferences.putBool("dhcp", networkConfig.dhcpEnabled);
  preferences.putString("staticIP", networkConfig.staticIP);
  preferences.putString("gateway", networkConfig.gateway);
  preferences.putString("subnet", networkConfig.subnet);
  preferences.putString("dns1", networkConfig.dns1);
  preferences.putString("dns2", networkConfig.dns2);

  // Guardar configuración MQTT
  preferences.putString("mqttServer", mqttConfig.server);
  preferences.putInt("mqttPort", mqttConfig.port);
  preferences.putString("mqttUser", mqttConfig.username);
  preferences.putString("mqttPass", mqttConfig.password);
  preferences.putString("mqttTopic", mqttConfig.topic);
  preferences.putString("mqttClientId", mqttConfig.clientId);
  preferences.putInt("mqttKeepAlive", mqttConfig.keepAlive);

  // Guardar configuración del dispositivo
  preferences.putString("deviceName", deviceConfig.deviceName);
  preferences.putString("location", deviceConfig.location);
  preferences.putInt("sensorInterval", deviceConfig.sensorInterval);
  preferences.putInt("readingsCount", deviceConfig.readingsCount);
  preferences.putBool("debugMode", deviceConfig.debugMode);

  // Guardar configuración de sensores
  preferences.putInt("sensorType", deviceConfig.sensorType);
  preferences.putInt("button1Pin", deviceConfig.button1Pin);
  preferences.putInt("button2Pin", deviceConfig.button2Pin);
  preferences.putInt("vibrationPin", deviceConfig.vibrationPin);
  preferences.putBool("button1Invert", deviceConfig.button1Invert);
  preferences.putBool("button2Invert", deviceConfig.button2Invert);
  preferences.putString("button1Topic", deviceConfig.button1Topic);
  preferences.putString("button2Topic", deviceConfig.button2Topic);
  preferences.putString("vibrationTopic", deviceConfig.vibrationTopic);
  preferences.putString("mainMqttTopic", deviceConfig.mainMqttTopic);
  preferences.putInt("vibrationThreshold", deviceConfig.vibrationThreshold);

  Serial.println("Configuración guardada exitosamente");
}

void setupSensorPins() {
  Serial.print("Configurando pines para tipo de sensor: ");
  Serial.println(deviceConfig.sensorType);

  // Primero, resetear todos los pines de sensor a estado seguro
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON1_PIN, INPUT);
  pinMode(BUTTON2_PIN, INPUT);
  pinMode(VIBRATION_PIN, INPUT);

  switch (deviceConfig.sensorType) {
    case SENSOR_ULTRASONIC:
      Serial.println("Configurando sensor ultrasónico HC-SR04");
      pinMode(TRIG_PIN, OUTPUT);
      pinMode(ECHO_PIN, INPUT);
      digitalWrite(TRIG_PIN, LOW);
      break;

    case SENSOR_SINGLE_BUTTON:
      Serial.println("Configurando 1 pulsador");
      pinMode(deviceConfig.button1Pin, INPUT_PULLUP);
      break;

    case SENSOR_DUAL_BUTTONS:
      Serial.println("Configurando 2 pulsadores");
      pinMode(deviceConfig.button1Pin, INPUT_PULLUP);
      pinMode(deviceConfig.button2Pin, INPUT_PULLUP);
      break;

    case SENSOR_VIBRATION:
      Serial.println("Configurando sensor de vibraciones SW-420");
      pinMode(deviceConfig.vibrationPin, INPUT_PULLUP);
      break;

    default:
      Serial.println("Tipo de sensor no reconocido, usando ultrasonido por defecto");
      deviceConfig.sensorType = SENSOR_ULTRASONIC;
      pinMode(TRIG_PIN, OUTPUT);
      pinMode(ECHO_PIN, INPUT);
      digitalWrite(TRIG_PIN, LOW);
      break;
  }
}

void resetToDefaults() {
  preferences.clear();

  // Valores por defecto
  networkConfig.dhcpEnabled = true;
  networkConfig.staticIP = "192.168.1.100";
  networkConfig.gateway = "192.168.1.1";
  networkConfig.subnet = "255.255.255.0";
  networkConfig.dns1 = "8.8.8.8";
  networkConfig.dns2 = "8.8.4.4";

  mqttConfig.server = "192.168.3.154";
  mqttConfig.port = 1883;
  mqttConfig.username = "";
  mqttConfig.password = "";
  mqttConfig.topic = "sensor/distance";
  mqttConfig.clientId = generarClientId();
  mqttConfig.keepAlive = 60;

  deviceConfig.deviceName = "Multi-Sensor-IoT-01";
  deviceConfig.location = "Desconocida";
  deviceConfig.sensorInterval = 50;
  deviceConfig.readingsCount = 10;
  deviceConfig.debugMode = false;

  // Valores por defecto para sensores
  deviceConfig.sensorType = SENSOR_ULTRASONIC;
  deviceConfig.button1Pin = BUTTON1_PIN;
  deviceConfig.button2Pin = BUTTON2_PIN;
  deviceConfig.vibrationPin = VIBRATION_PIN;
  deviceConfig.button1Invert = false;
  deviceConfig.button2Invert = false;
  deviceConfig.button1Topic = "sensor/button1";
  deviceConfig.button2Topic = "sensor/button2";
  deviceConfig.vibrationTopic = "sensor/vibration";
  deviceConfig.mainMqttTopic = "multi-sensor/iot";
  deviceConfig.vibrationThreshold = 100; // 100ms cooldown por defecto

  saveConfiguration();
  Serial.println("Configuración restablecida a valores por defecto");
}

void checkConfigButton() {
  static unsigned long buttonPressStart = 0;
  static bool currentlyPressed = false;

  bool buttonState = digitalRead(CONFIG_BUTTON_PIN);

  if (buttonState == LOW && !currentlyPressed) {
    // Botón presionado
    buttonPressStart = millis();
    currentlyPressed = true;
    Serial.println("Botón de configuración presionado...");
  } else if (buttonState == HIGH && currentlyPressed) {
    // Botón liberado
    currentlyPressed = false;
    unsigned long pressDuration = millis() - buttonPressStart;

    if (pressDuration >= 10000) { // 10 segundos para modo hotspot
      if (!bridgeMode && !hotspotMode) {
        Serial.println("Entrando en modo hotspot (10 segundos)");
        enterHotspotMode();
      }
    } else if (pressDuration >= bridgeButtonHoldTime) { // 3 segundos para modo bridge
      if (!bridgeMode && !hotspotMode) {
        Serial.println("Entrando en modo bridge");
        enterBridgeMode();
      }
    }
  }
}

void enterBridgeMode() {
  Serial.println("=== MODO BRIDGE ACTIVADO ===");
  Serial.println("LED indicador encendido");

  digitalWrite(CONFIG_LED_PIN, HIGH);
  bridgeModeEnterTime = millis();

  // Detener tareas normales si están corriendo
  // Las tareas FreeRTOS se detendrán solas al entrar en modo bridge

  // Iniciar modo AP (manteniendo Ethernet conectado)
  WiFi.mode(WIFI_AP_STA);  // STA mantiene Ethernet, AP para configuración
  WiFi.softAP(ap_ssid, ap_password);

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(apIP);
  Serial.print("Ethernet status: ");
  Serial.println(eth_connected ? "Connected" : "Disconnected");

  // Iniciar servidor web
  if (!configServer) {
    configServer = new WebServer(80);
  }

  setupWebServer();
  configServer->begin();

  Serial.println("Servidor web iniciado");
  Serial.print("Conéctate a: ");
  Serial.println(ap_ssid);
  Serial.print("Luego visita: http://");
  Serial.println(apIP);

  logSystemEvent("BRIDGE_SETUP", "AP iniciado, servidor web listo");
}

void exitBridgeMode() {
  Serial.println("Saliendo del modo bridge...");
  bridgeMode = false;
  digitalWrite(CONFIG_LED_PIN, LOW);

  if (configServer) {
    configServer->stop();
    delete configServer;
    configServer = NULL;
  }

  WiFi.mode(WIFI_OFF);

  delay(1000);
  ESP.restart();
}

void enterHotspotMode() {
  Serial.println("=== MODO HOTSPOT ACTIVADO ===");
  Serial.println("Iniciando hotspot de configuración...");

  hotspotMode = true;
  hotspotModeEnterTime = millis();

  // Apagar ambos LEDs inicialmente
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);

  // Configurar WiFi como hotspot puro (sin bridge)
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Hotspot", "12345678"); // Contraseña fija para hotspot

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("Hotspot IP address: ");
  Serial.println(apIP);

  // Iniciar servidor web
  if (!configServer) {
    configServer = new WebServer(80);
  }

  setupWebServer();
  configServer->begin();

  Serial.println("Servidor web iniciado en modo hotspot");
  Serial.print("Conéctate a: ESP32-Hotspot");
  Serial.print("Luego visita: http://");
  Serial.println(apIP);

  logSystemEvent("HOTSPOT_ENTER", "Modo hotspot activado por botón físico (5 segundos)");
}

void exitHotspotMode() {
  Serial.println("Saliendo del modo hotspot...");
  hotspotMode = false;

  // Apagar ambos LEDs
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);

  if (configServer) {
    configServer->stop();
    delete configServer;
    configServer = NULL;
  }

  WiFi.mode(WIFI_OFF);

  delay(1000);
  ESP.restart();
}

void initializeLEDs() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  pinMode(CONFIG_LED_PIN, OUTPUT);

  // Inicializar LEDs apagados
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  digitalWrite(CONFIG_LED_PIN, LOW);

  Serial.println("LEDs inicializados");
}

void updateStatusLEDs() {
  // Solo actualizar LEDs si no estamos en modo bridge o hotspot
  if (bridgeMode || hotspotMode) return;

  static unsigned long lastLEDUpdate = 0;
  static bool errorBlinkState = false;

  // Actualizar cada 500ms
  if (millis() - lastLEDUpdate < 500) return;
  lastLEDUpdate = millis();

  // Determinar estados
  bool systemOK = eth_connected && client.connected();
  bool hasError = !eth_connected || !client.connected();

  // LED verde (STATUS) - encendido si todo está OK
  if (systemOK) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }

  // LED rojo (ERROR) - parpadea si hay error, apagado si todo está OK
  if (hasError) {
    errorBlinkState = !errorBlinkState;
    digitalWrite(ERROR_LED_PIN, errorBlinkState ? HIGH : LOW);
  } else {
    digitalWrite(ERROR_LED_PIN, LOW);
  }
}

void blinkLEDs(int times, int onDuration, int offDuration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(onDuration);
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(offDuration);
  }
}

void setupWebServer() {
  configServer->on("/", HTTP_GET, handleRoot);
  configServer->on("/save", HTTP_POST, handleSaveConfig);
  configServer->on("/reset", HTTP_POST, handleReset);
  configServer->on("/status", HTTP_GET, handleStatus);
  configServer->on("/api/status", HTTP_GET, [](){
    // API endpoint para status en tiempo real
    configServer->sendHeader("Access-Control-Allow-Origin", "*");
    configServer->send(200, "application/json", generateSystemStatusJSON());
  });
  configServer->on("/exit", HTTP_POST, [](){
    if (bridgeMode) {
      logSystemEvent("BRIDGE_EXIT", "Usuario solicitó salir del modo bridge");
      configServer->send(200, "text/html",
        "<html><body><h2>Saliendo del modo bridge...</h2><p>El dispositivo se reiniciará en modo normal.</p></body></html>");
      delay(1000);
      exitBridgeMode();
    } else if (hotspotMode) {
      logSystemEvent("HOTSPOT_EXIT", "Usuario solicitó salir del modo hotspot");
      configServer->send(200, "text/html",
        "<html><body><h2>Saliendo del modo hotspot...</h2><p>El dispositivo se reiniciará en modo normal.</p></body></html>");
      delay(1000);
      exitHotspotMode();
    }
  });

  configServer->onNotFound([](){
    configServer->send(404, "text/plain", "Not found");
  });
}

// =====================================================================
// FUNCIONES DE PROTECCIÓN OTA
// =====================================================================

void markBootAttempt() {
  preferences.putUInt("bootCount", preferences.getUInt("bootCount", 0) + 1);
  preferences.putULong64("lastBootTime", millis());
}

bool safeOTACheck() {
  // Verificar si el dispositivo ha arrancado correctamente
  unsigned long lastBoot = preferences.getULong64("lastBootTime", 0);
  unsigned long currentTime = millis();

  // Si el último boot fue hace menos de 2 minutos, está bien
  if (lastBoot > 0 && (currentTime - lastBoot) < 120000) {
    return true;
  }

  // Verificar contador de boots fallidos
  unsigned int bootCount = preferences.getUInt("bootCount", 0);
  if (bootCount > 3) {
    Serial.println("¡Demasiados intentos de boot fallidos! Entrando en modo seguro");
    return false;
  }

  return true;
}

void rollbackToFirmware() {
  Serial.println("Iniciando rollback al firmware anterior...");

  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                                            ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);

  if (factory != NULL) {
    esp_ota_set_boot_partition(factory);
    Serial.println("Rollback completado. Reiniciando...");
    ESP.restart();
  } else {
    Serial.println("No se encontró partición de fábrica. Reiniciando...");
    ESP.restart();
  }
}

// =====================================================================
// FUNCIONES DEL WEB PANEL
// =====================================================================

String getConfigFormHTML() {
  String html = "";

  // Leer el archivo HTML base
  File file = LittleFS.open("/config.html", "r");
  if (!file) {
    // Si no se puede leer el archivo, retornar un error simple
    return "<html><body><h1>Error: No se puede cargar la pagina de configuracion</h1></body></html>";
  }

  // Leer el contenido y hacer reemplazos dinamicos
  while (file.available()) {
    String line = file.readStringUntil('\n');

    // Reemplazar valores dinamicos
    line.replace("value=\"1883\"", "value=\"" + String(mqttConfig.port) + "\"");
    line.replace("value=\"60\"", "value=\"" + String(mqttConfig.keepAlive) + "\"");
    line.replace("value=\"50\"", "value=\"" + String(deviceConfig.sensorInterval) + "\"");
    line.replace("value=\"10\"", "value=\"" + String(deviceConfig.readingsCount) + "\"");
    line.replace("placeholder=\"192.168.1.100\"", "value=\"" + mqttConfig.server + "\"");
    line.replace("placeholder=\"sensor/distance\"", "value=\"" + mqttConfig.topic + "\"");
    line.replace("placeholder=\"Multi-Sensor-IoT-01\"", "value=\"" + deviceConfig.deviceName + "\"");
    line.replace("placeholder=\"Oficina Principal\"", "value=\"" + deviceConfig.location + "\"");

    // Checkbox states
    if (!networkConfig.dhcpEnabled) {
      line.replace("checked onchange=\"toggleStaticIP()\"", "onchange=\"toggleStaticIP()\"");
      line.replace("checked", "");
    }

    if (deviceConfig.debugMode) {
      line.replace("name=\"debugMode\"", "name=\"debugMode\" checked");
    }

    html += line + "\n";
  }

  file.close();
  return html;
}

void handleRoot() {
  configServer->send(200, "text/html", getConfigFormHTML());
}

void handleSaveConfig() {
  String message = "<html><body><div class='container'><h1>Configuración Guardada</h1>";

  // Guardar configuración de red
  networkConfig.dhcpEnabled = configServer->hasArg("dhcpEnabled");
  networkConfig.staticIP = configServer->arg("staticIP");
  networkConfig.gateway = configServer->arg("gateway");
  networkConfig.subnet = configServer->arg("subnet");
  networkConfig.dns1 = configServer->arg("dns1");
  networkConfig.dns2 = configServer->arg("dns2");

  // Guardar configuración MQTT
  mqttConfig.server = configServer->arg("mqttServer");
  mqttConfig.port = configServer->arg("mqttPort").toInt();
  mqttConfig.username = configServer->arg("mqttUsername");
  mqttConfig.password = configServer->arg("mqttPassword");
  mqttConfig.topic = configServer->arg("mqttTopic");
  mqttConfig.clientId = configServer->arg("mqttClientId");
  mqttConfig.keepAlive = configServer->arg("mqttKeepAlive").toInt();

  // Guardar configuración del dispositivo
  deviceConfig.deviceName = configServer->arg("deviceName");
  deviceConfig.location = configServer->arg("location");
  deviceConfig.sensorInterval = configServer->arg("sensorInterval").toInt();
  deviceConfig.readingsCount = configServer->arg("readingsCount").toInt();
  deviceConfig.debugMode = configServer->hasArg("debugMode");

  // Cargar configuración de sensores desde el formulario
  deviceConfig.sensorType = configServer->arg("sensorType").toInt();

  // Configuración para todos los tipos de sensores
  if (configServer->hasArg("button1Pin")) {
    deviceConfig.button1Pin = configServer->arg("button1Pin").toInt();
  }
  if (configServer->hasArg("dualButton1Pin")) {
    deviceConfig.button1Pin = configServer->arg("dualButton1Pin").toInt();
  }

  if (configServer->hasArg("button2Pin")) {
    deviceConfig.button2Pin = configServer->arg("button2Pin").toInt();
  }

  if (configServer->hasArg("vibrationPin")) {
    deviceConfig.vibrationPin = configServer->arg("vibrationPin").toInt();
  }

  // Inversiones de señal
  deviceConfig.button1Invert = configServer->hasArg("button1Invert") || configServer->hasArg("dualButton1Invert");
  deviceConfig.button2Invert = configServer->hasArg("button2Invert");

  // Topics MQTT
  if (configServer->hasArg("button1Topic")) {
    deviceConfig.button1Topic = configServer->arg("button1Topic");
  }
  if (configServer->hasArg("dualButton1Topic")) {
    deviceConfig.button1Topic = configServer->arg("dualButton1Topic");
  }

  if (configServer->hasArg("button2Topic")) {
    deviceConfig.button2Topic = configServer->arg("button2Topic");
  }

  if (configServer->hasArg("vibrationTopic")) {
    deviceConfig.vibrationTopic = configServer->arg("vibrationTopic");
  }

  if (configServer->hasArg("mainMqttTopic")) {
    deviceConfig.mainMqttTopic = configServer->arg("mainMqttTopic");
  }

  // Umbral de vibración
  if (configServer->hasArg("vibrationThreshold")) {
    deviceConfig.vibrationThreshold = configServer->arg("vibrationThreshold").toInt();
  }

  // Validar configuración
  bool configValid = true;
  String errorMessage = "";

  if (mqttConfig.server.length() == 0) {
    configValid = false;
    errorMessage += "El servidor MQTT es requerido<br>";
  }

  if (!networkConfig.dhcpEnabled) {
    if (!validateNetworkConfig(networkConfig.staticIP, networkConfig.gateway, networkConfig.subnet)) {
      configValid = false;
      errorMessage += "La configuración de IP estática no es válida<br>";
    }
  }

  if (!validateMQTTConfig(mqttConfig.server, mqttConfig.port)) {
    configValid = false;
    errorMessage += "La configuración MQTT no es válida<br>";
  }

  if (configValid) {
    saveConfiguration();
    message += "<div class='status success'>✅ Configuración guardada exitosamente!</div>";
    message += "<p>El dispositivo se reiniciará en modo normal.</p>";
    message += "<button onclick='setTimeout(function(){ window.location.href=\"/exit\"; }, 2000);'>Continuar</button>";
  } else {
    message += "<div class='status error'>❌ Error en la configuración:</div>";
    message += "<p>" + errorMessage + "</p>";
    message += "<button onclick='history.back()'>Volver</button>";
  }

  message += "</div></body></html>";
  configServer->send(200, "text/html", message);

  if (configValid) {
    delay(2000);
    exitBridgeMode();
  }
}

void handleReset() {
  resetToDefaults();
  String message = R"(
    <html><body>
    <div class='container'>
        <h1>Configuración Reset</h1>
        <div class='status success'>✅ Configuración restablecida a valores por defecto</div>
        <p>El dispositivo se reiniciará...</p>
        <script>setTimeout(function(){ window.location.href='/exit'; }, 3000);</script>
    </div>
    </body></html>
  )";
  configServer->send(200, "text/html", message);
}

void handleStatus() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Estado del Sistema</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        h1 { color: #333; text-align: center; }
        h3 { color: #555; border-bottom: 2px solid #4CAF50; padding-bottom: 5px; }
        .status { padding: 10px; margin: 10px 0; border-radius: 4px; background-color: #dff0d8; }
        button { background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; }
    </style>
</head>
<body>
    <div class="container">
        <h1>📊 Estado del Sistema</h1>
        <div class="status">
            <h3>Información del Dispositivo</h3>
            <p><strong>Nombre:</strong> )" + deviceConfig.deviceName + R"(</p>
            <p><strong>Ubicación:</strong> )" + deviceConfig.location + R"(</p>
            <p><strong>Versión Firmware:</strong> )" + String(FW_VERSION) + R"(</p>
            <p><strong>MAC Address:</strong> )" + ETH.macAddress() + R"(</p>
            <p><strong>Modo Bridge:</strong> )" + String(bridgeMode ? "Activado" : "Desactivado") + R"(</p>
        </div>

        <div class="status">
            <h3>Configuración Red</h3>
            <p><strong>DHCP:</strong> )" + String(networkConfig.dhcpEnabled ? "Activado" : "Desactivado") + R"(</p>
            <p><strong>IP Estática:</strong> )" + networkConfig.staticIP + R"(</p>
            <p><strong>Gateway:</strong> )" + networkConfig.gateway + R"(</p>
        </div>

        <div class="status">
            <h3>Configuración MQTT</h3>
            <p><strong>Servidor:</strong> )" + mqttConfig.server + R"(:)" + String(mqttConfig.port) + R"(</p>
            <p><strong>Topic:</strong> )" + mqttConfig.topic + R"(</p>
            <p><strong>Client ID:</strong> )" + mqttConfig.clientId + R"(</p>
        </div>

        <button onclick='window.location.href=\"/\"'>Volver</button>
    </div>
</body>
</html>
)";
  configServer->send(200, "text/html", html);
}

bool validateIP(String ip) {
  int parts[4];
  int part = 0;
  String currentPart = "";

  for (int i = 0; i <= ip.length(); i++) {
    char c = (i < ip.length()) ? ip.charAt(i) : '.';

    if (c == '.') {
      if (currentPart.length() == 0) return false;
      parts[part++] = currentPart.toInt();
      currentPart = "";

      if (part > 3) return false;
      if (parts[part-1] < 0 || parts[part-1] > 255) return false;
    } else if (c >= '0' && c <= '9') {
      currentPart += c;
      if (currentPart.length() > 3) return false;
    } else {
      return false;
    }
  }

  return (part == 3);
}

bool validateNetworkConfig(String ip, String gateway, String subnet) {
  if (ip.length() == 0 || gateway.length() == 0 || subnet.length() == 0) {
    return false;
  }

  // Validar formatos de IP
  if (!validateIP(ip) || !validateIP(gateway) || !validateIP(subnet)) {
    return false;
  }

  // Validaciones básicas de red
  if (subnet.equals("255.255.255.255") || subnet.equals("0.0.0.0")) {
    return false;
  }

  // Validar que IP y gateway estén en la misma subred (validación básica)
  if (!ip.substring(0, ip.lastIndexOf('.')).equals(gateway.substring(0, gateway.lastIndexOf('.')))) {
    // Permitir si es una configuración especial, pero registrar advertencia
    if (deviceConfig.debugMode) {
      Serial.println("ADVERTENCIA: IP y gateway en diferentes subredes");
    }
  }

  return true;
}

bool validateMQTTConfig(String server, int port) {
  if (server.length() == 0) {
    return false;
  }

  if (port < 1 || port > 65535) {
    return false;
  }

  // Validar que el servidor no sea localhost o 127.0.0.1 (para ESP32 no tiene sentido)
  if (server.equals("127.0.0.1") || server.equals("localhost")) {
    return false;
  }

  // Validar formato básico de hostname/IP
  if (server.charAt(0) == '.' || server.charAt(server.length()-1) == '.') {
    return false;
  }

  return true;
}

// =====================================================================
// FUNCIONES DE ESTADO Y MEJORAS DEL SISTEMA
// =====================================================================

void initializeSystemStatus() {
  memset(&systemStatus, 0, sizeof(SystemStatus));

  // Cargar contadores de estadísticas
  systemStatus.otaUpdatesCount = preferences.getUInt("otaUpdatesCount", 0);
  systemStatus.systemRestarts = preferences.getUInt("systemRestarts", 0) + 1;

  // Guardar el nuevo reinicio
  preferences.putUInt("systemRestarts", systemStatus.systemRestarts);

  logSystemEvent("SYSTEM_BOOT", "Version: " + String(FW_VERSION) + ", Restarts: " + String(systemStatus.systemRestarts));
}

void updateSystemStatus() {
  systemStatus.uptime = millis();
  systemStatus.freeHeap = ESP.getFreeHeap();
  systemStatus.wifiSignalStrength = WiFi.RSSI();

  // Calcular uso aproximado de CPU (basado en tiempo de ejecución)
  static unsigned long lastTaskTime = 0;
  unsigned long currentTime = millis();
  if (lastTaskTime > 0) {
    systemStatus.cpuUsage = min(100.0f, (100.0f * (currentTime - lastTaskTime)) / 1000.0f);
  }
  lastTaskTime = currentTime;
}

void checkBridgeTimeout() {
  if (bridgeMode && (millis() - bridgeModeEnterTime > bridgeModeTimeout)) {
    logSystemEvent("BRIDGE_TIMEOUT", "Saliendo del modo bridge por timeout");

    // Enviar mensaje al cliente si está conectado
    String message = R"(
    <html>
    <head><title>Timeout del Modo Bridge</title></head>
    <body>
      <h1>⏰ Tiempo límite alcanzado</h1>
      <p>El modo bridge ha finalizado automáticamente después de 5 minutos.</p>
      <p>El dispositivo se está reiniciando en modo normal...</p>
      <script>setTimeout(function(){ window.close(); }, 3000);</script>
    </body>
    </html>
    )";

    configServer->send(200, "text/html", message);
    delay(500);

    exitBridgeMode();
  }
}

String generateSystemStatusJSON() {
  updateSystemStatus();

  String json = "{";
  json += "\"version\":\"" + String(FW_VERSION) + "\",";
  json += "\"uptime\":" + String(systemStatus.uptime) + ",";
  json += "\"deviceName\":\"" + deviceConfig.deviceName + "\",";
  json += "\"location\":\"" + deviceConfig.location + "\",";
  json += "\"currentDistance\":" + String(systemStatus.currentDistance, 2) + ",";
  json += "\"wifiSignal\":" + String(systemStatus.wifiSignalStrength) + ",";
  json += "\"freeHeap\":" + String(systemStatus.freeHeap) + ",";
  json += "\"cpuUsage\":" + String(systemStatus.cpuUsage, 2) + ",";
  json += "\"mqttConnected\":" + String(client.connected() ? "true" : "false") + ",";
  json += "\"lastMQTTConnection\":" + String(systemStatus.lastMQTTConnection) + ",";
  json += "\"mqttAttempts\":" + String(systemStatus.mqttConnectionAttempts) + ",";
  json += "\"otaUpdates\":" + String(systemStatus.otaUpdatesCount) + ",";
  json += "\"systemRestarts\":" + String(systemStatus.systemRestarts) + ",";
  json += "\"bridgeMode\":" + String(bridgeMode ? "true" : "false") + ",";
  json += "\"hotspotMode\":" + String(hotspotMode ? "true" : "false") + ",";
  json += "\"ethConnected\":" + String(eth_connected ? "true" : "false") + ",";

  // Configuración de sensores
  json += "\"sensorType\":" + String(deviceConfig.sensorType) + ",";
  json += "\"button1Pin\":" + String(deviceConfig.button1Pin) + ",";
  json += "\"button2Pin\":" + String(deviceConfig.button2Pin) + ",";
  json += "\"vibrationPin\":" + String(deviceConfig.vibrationPin) + ",";
  json += "\"button1Invert\":" + String(deviceConfig.button1Invert ? "true" : "false") + ",";
  json += "\"button2Invert\":" + String(deviceConfig.button2Invert ? "true" : "false") + ",";
  json += "\"vibrationThreshold\":" + String(deviceConfig.vibrationThreshold) + ",";
  json += "\"button1Topic\":\"" + deviceConfig.button1Topic + "\",";
  json += "\"button2Topic\":\"" + deviceConfig.button2Topic + "\",";
  json += "\"vibrationTopic\":\"" + deviceConfig.vibrationTopic + "\",";
  json += "\"mainMqttTopic\":\"" + deviceConfig.mainMqttTopic + "\",";

  // Configuración básica del dispositivo
  json += "\"deviceName\":\"" + deviceConfig.deviceName + "\",";
  json += "\"location\":\"" + deviceConfig.location + "\",";
  json += "\"sensorInterval\":" + String(deviceConfig.sensorInterval) + ",";
  json += "\"readingsCount\":" + String(deviceConfig.readingsCount) + ",";
  json += "\"debugMode\":" + String(deviceConfig.debugMode ? "true" : "false") + "";

  json += "}";

  return json;
}

void logSystemEvent(String event, String details) {
  String timestamp = String(millis() / 1000);
  String logEntry = "[" + timestamp + "] " + event;
  if (details.length() > 0) {
    logEntry += ": " + details;
  }

  // Solo mostrar logs si está en modo debug o es un evento importante
  if (deviceConfig.debugMode || event.equals("SYSTEM_BOOT") || event.equals("BRIDGE_ENTER") || event.equals("OTA_UPDATE")) {
    Serial.println(logEntry);
  }

  // En modo bridge, también guardar logs importantes en memoria para diagnóstico
  if (bridgeMode && (event.equals("SYSTEM_BOOT") || event.equals("OTA_UPDATE") || event.equals("BRIDGE_ENTER"))) {
    String logKey = "log_" + String(millis());
    preferences.putString(logKey.c_str(), logEntry);

    // Mantener solo los últimos 10 logs
    static String logKeys[10];
    static int logIndex = 0;
    if (logIndex < 10) {
      logKeys[logIndex++] = logKey;
    }

    // Borrar logs antiguos si hay más de 10
    if (logIndex >= 10) {
      String oldLog = "log_" + String(millis() - 600000); // 10 minutos atrás
      preferences.remove(oldLog.c_str());
    }
  }
}

