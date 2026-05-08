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

enum SensorType {
  SENSOR_ULTRASONIC = 0,        // HC-SR04 - Distancia
  SENSOR_SINGLE_BUTTON = 1,     // 1 Pulsador digital
  SENSOR_DUAL_BUTTONS = 2,      // 2 Pulsadores digitales
  SENSOR_VIBRATION = 3,         // Sensor de vibraciones SW-420
  SENSOR_VIBRATION_BUTTON = 4   // Vibración + 1 Pulsador
};

enum ConnectionMode {
  MODE_ETHERNET = 0,   // Ethernet
  MODE_WIFI = 1,       // WiFi
  MODE_DUAL_ETH_WIFI = 2  // Ethernet primario + WiFi backup
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

static bool eth_connected = false;

WiFiClient espClient;
PubSubClient client(espClient);

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
const char* ota_version_url = "http://ota.boisolo.com/multi-sensor-iot/version.json";  // URL del JSON de versiones
const char* firmware_base_url = "http://ota.boisolo.com/multi-sensor-iot/";  // URL base para los archivos de firmware
const unsigned long ota_check_interval = 30000;   // Revisar actualizaciones cada 30 segundos
unsigned long lastOtaCheck = 0;
const int ota_timeout = 30000;  // Timeout para actualización OTA (30 segundos)

struct NetworkConfig {
  bool dhcpEnabled;
  String staticIP;
  String gateway;
  String subnet;
  String dns1;
  String dns2;
};

struct MQTTConfig {
  String server;
  int port;
  String username;
  String password;
  String topic;
  String clientId;
  int keepAlive;
};


struct DeviceConfig {
  String deviceName;
  String location;
  int sensorInterval;
  int readingsCount;
  bool debugMode;

  int sensorType; // 0=ultrasonido, 1=1 pulsador, 2=2 pulsadores, 3=vibración
  int button1Pin, button2Pin, vibrationPin;
  bool button1Invert, button2Invert;
  String button1Topic, button2Topic, vibrationTopic;
  String mainMqttTopic; // Topic principal para ultrasonido
  int vibrationThreshold; // Umbral de sensibilidad para vibraciones
  int vibrationMode; // 0=golpe (solo publica 1), 1=vibracion (publica 1 y 0)

  int connectionMode; // 0=Ethernet, 1=WiFi, 2=Bluetooth, 3=Dual ETH+WiFi, 4=Dual WiFi+BT
};


struct WiFiConfig {
  String ssid;
  String password;
  bool enabled;
  int channel;
  bool hidden;
};

NetworkConfig networkConfig;
MQTTConfig mqttConfig;
DeviceConfig deviceConfig;
WiFiConfig wifiConfig;

bool button1State = false;
bool button2State = false;
bool lastButton1State = false;
bool lastButton2State = false;
unsigned long lastButton1Change = 0;
unsigned long lastButton2Change = 0;

bool vibrationState = false;
bool lastVibrationState = false;
unsigned long lastVibrationChange = 0;
unsigned long vibrationCooldown = 100; // 100ms cooldown entre detecciones

bool bridgeMode = false;
bool hotspotMode = false;
WebServer* configServer = NULL;
Preferences preferences;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
const unsigned long bridgeModeTimeout = 300000;   // 5 minutos timeout en modo bridge
unsigned long bridgeModeEnterTime = 0;
unsigned long hotspotModeEnterTime = 0;


bool wifiConnected = false;
bool ethernetConnected = false;
int currentConnectionMode = MODE_ETHERNET;
unsigned long lastConnectionCheck = 0;
const unsigned long connectionCheckInterval = 10000; // Revisar conexión cada 10 segundos

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

struct FirmwareInfo {
  String version;
  String url;
  String filesystemUrl;  // Nueva: URL del filesystem
  String checksum;
  bool mandatory;
  String release_notes;
};

bool checkForUpdates();
bool checkForUpdatesSafe();
bool performOTAUpdate(const FirmwareInfo& firmwareInfo);
bool performSafeOTAUpdate(const FirmwareInfo& firmwareInfo);
bool performFilesystemOTAUpdate(const String& filesystemUrl);
String getCurrentFirmwareVersion();
bool compareVersions(const String& current, const String& available);
String calculateSHA256(const String& filePath);

void WiFiEvent(WiFiEvent_t event);
bool reconectarMQTT();
void sensorTask(void *pvParameters);

void readButtons();
void readVibrationSensor();
void publishButtonState(int buttonId, bool state, String topic);
void publishVibrationState(bool state);
void publishConnectionStatus();
void mqttTask(void *pvParameters);
void otaTask(void *pvParameters);

void initializePreferences();
void loadConfiguration();
void saveConfiguration();
void resetToDefaults();
void setupSensorPins();
void checkConfigButton();
void initializeConnectionModules();
void setupWiFi();
void checkConnectionMode();
void switchConnectionMode(int newMode);
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

void initializeLEDs();
void updateStatusLEDs();
void setOnlineStatus();
void setErrorStatus();
void clearErrorStatus();
void blinkLEDs(int times, int onTime, int offTime);

bool safeOTACheck();
void rollbackToFirmware();
void markBootAttempt();

void updateSystemStatus();
void initializeSystemStatus();
void checkBridgeTimeout();
String generateSystemStatusJSON();
void logSystemEvent(String event, String details = "");

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

  // Generar ClientID único con timestamp para evitar conflictos al reconectar
  unsigned long timestamp = millis();
  uint16_t randomNum = random(0xFFFF);

  char id[40];
  sprintf(id, "ESP32Client-%02X%02X%02X%02X%02X%02X-%04X-%08lX",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], randomNum, timestamp);
  return String(id);
}


void setup() {
  Serial.begin(115200);
  delay(500); // Pequeña pausa para que el monitor serie se estabilice
  Serial.println("\n\n--- INICIANDO MULTI-SENSOR IOT UNIVERSAL (WT32-ETH01) ---");

  // RESET COMPLETO DE VARIABLES OTA (v1.6.) - Forzar actualización sin backoff
  // IMPORTANTE: Las variables se resetean en otaTask() para evitar problemas de declaración
  Serial.println("🔄 [RESET OTA] Las variables OTA se resetearán en otaTask() - v1.6.");
  Serial.println("🔄 [RESET OTA] Backoff será eliminado - verificará inmediatamente");

  sensorMutex = xSemaphoreCreateMutex();

  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CONFIG_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  initializeLEDs();

  Serial.println("Inicializando sistema de configuración...");
  initializePreferences();
  loadConfiguration();
  setupSensorPins();
  Serial.println("Inicializando sistema de estado...");
  initializeSystemStatus();
  Serial.println("Inicializando módulos de conexión...");
  initializeConnectionModules();

  bool hasConfig = false;
  if (wifiConfig.enabled && wifiConfig.ssid.length() > 0) {
    hasConfig = true;
    Serial.println("Configuración WiFi encontrada");
  }
  if (mqttConfig.server.length() > 0) {
    hasConfig = true;
    Serial.println("Configuración MQTT encontrada");
  }

  if (!hasConfig) {
    Serial.println("📡 No hay configuración previa - Entrando en modo hotspot automático");
    hotspotMode = true;
    enterHotspotMode();
    return;
  }

  
  
checkConfigButton();

  if (bridgeMode) {
    enterBridgeMode();
    return;
  }

  if (hotspotMode) {
    enterHotspotMode();
    return;
  }

  markBootAttempt();


  Serial.println("Configurando conexión Ethernet...");
  WiFi.onEvent(WiFiEvent);

  Serial.println("Intentando conectar a la red por RJ45...");
  ETH.begin(ETH_PHY_ADDR, ETH_PHY_POWER, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_TYPE, ETH_CLK_MODE);

  // Configurar MQTT con valores guardados
  client.setServer(mqttConfig.server.c_str(), mqttConfig.port);

  Serial.println("Sensor de ultrasonido JSN-SR04T configurado.");
  Serial.print("MQTT Server: ");
  Serial.println(mqttConfig.server);
  Serial.print("Topic: ");
  Serial.println(mqttConfig.topic);
  Serial.println("------------------------------------");

  lastSensorRestart = millis();

  // Crear tareas según el tipo de sensor
  switch(deviceConfig.sensorType) {
    case SENSOR_ULTRASONIC:
      Serial.println("Creando tarea de sensor ultrasónico");
      xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);
      break;
    case SENSOR_SINGLE_BUTTON:
    case SENSOR_DUAL_BUTTONS:
    case SENSOR_VIBRATION:
    case SENSOR_VIBRATION_BUTTON:
      Serial.println("Omitiendo tarea de sensor ultrasónico (tipo: " + String(deviceConfig.sensorType) + ")");
      break;
    default:
      Serial.println("Tipo de sensor desconocido, creando tarea ultrasónica por defecto");
      xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);
      break;
  }

  // Crear tareas comunes
  Serial.println("Creando tarea MQTT...");
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 10000, NULL, 1, NULL, 1);
  Serial.println("Creando tarea OTA...");
  xTaskCreatePinnedToCore(otaTask, "OTA Task", 10000, NULL, 2, NULL, 1);
  Serial.println("Tareas creadas exitosamente");  
}

void loop() {
  // Siempre revisar el botón de configuración
  checkConfigButton();

  if (bridgeMode) {
    checkBridgeTimeout();
    if (configServer) {
      configServer->handleClient();
    }
    delay(10);
    return;
  }

  if (hotspotMode) {
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

  readButtons();
  readVibrationSensor();


  // Operación normal - verificar modo de conexión y estado del sistema
  checkConnectionMode();
  updateSystemStatus();
  updateStatusLEDs();

  // Operación normal - reinicio programado
  if (millis() - lastSensorRestart >= sensorRestartInterval) {
    logSystemEvent("SCHEDULED_RESTART", "Reinicio programado del sistema");
    Serial.println("Reinicio programado del ESP32...");
    ESP.restart();
  }

  // Pequeño delay para no sobrecargar el CPU
  delay(50);
}

// Tarea para leer el sensor de ultrasonido (compatible con JSN-SR04T en modo por defecto)
void sensorTask(void *pvParameters) {
  long duration;
  int distance;
  int localSensorInterval = deviceConfig.sensorInterval; // Copia local para evitar accesos concurrentes

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
    
    // Esperar el intervalo configurado para la siguiente lectura
    vTaskDelay(localSensorInterval / portTICK_PERIOD_MS);
  }
}

// Funciones de lectura de sensores
void readButtons() {
  if (deviceConfig.sensorType == SENSOR_SINGLE_BUTTON || deviceConfig.sensorType == SENSOR_DUAL_BUTTONS || deviceConfig.sensorType == SENSOR_VIBRATION_BUTTON) {
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
      // Siempre mostrar para modo VIBRATION_BUTTON (para debugging)
      if (deviceConfig.sensorType == SENSOR_VIBRATION_BUTTON) {
        Serial.print("🔘 [BUTTON] Pulsador 1: ");
        Serial.println(button1State ? "PRESIONADO (1)" : "SUELTO (0)");
        Serial.print("   Topic: ");
        Serial.println(deviceConfig.button1Topic);
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
  if (deviceConfig.sensorType == SENSOR_VIBRATION || deviceConfig.sensorType == SENSOR_VIBRATION_BUTTON) {
    // SENSOR DIGITAL SW-420 - Dos modos: GOLPE (0) o VIBRACION (1)
    static unsigned long lastVibrationPublish = 0;
    static bool lastVibrationState = HIGH;

    pinMode(deviceConfig.vibrationPin, INPUT_PULLUP);
    bool currentPinState = digitalRead(deviceConfig.vibrationPin);

    // MODO 0: GOLPE - Solo publica 1 al detectar el inicio del golpe
    if (deviceConfig.vibrationMode == 0) {
      // Detectar flanco de HIGH a LOW (inicio de golpe)
      if (currentPinState == LOW && lastVibrationState == HIGH) {
        if (millis() - lastVibrationPublish > deviceConfig.vibrationThreshold) {
          lastVibrationPublish = millis();
          vibrationState = true;
          publishVibrationState(true);
          Serial.println("📳 [VIBRATION] ¡GOLPE DETECTADO! (1) - Publicando a MQTT...");
          Serial.println("📳 [VIBRATION] Topic: " + deviceConfig.vibrationTopic);
        }
      }

      // Actualizar estado interno (no publica 0)
      if (currentPinState == HIGH && lastVibrationState == LOW) {
        vibrationState = false;
        Serial.println("📳 [VIBRATION] Golpe finalizado - No se publica 0");
      }
    }
    // MODO 1: VIBRACION - Publica 1 cuando vibra, 0 cuando para
    else if (deviceConfig.vibrationMode == 1) {
      // Detectar flanco HIGH->LOW (inicio vibración)
      if (currentPinState == LOW && lastVibrationState == HIGH) {
        if (millis() - lastVibrationPublish > 50) {
          lastVibrationPublish = millis();
          vibrationState = true;
          publishVibrationState(true);
          Serial.println("📳 [VIBRATION] Vibración detectada (1) - Publicando a MQTT...");
        }
      }

      // Detectar flanco LOW->HIGH (fin vibración)
      if (currentPinState == HIGH && lastVibrationState == LOW) {
        vibrationState = false;
        publishVibrationState(false);
        Serial.println("📳 [VIBRATION] Vibración finalizada (0) - Publicando a MQTT...");
      }
    }

    lastVibrationState = currentPinState;
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

// Publicar mensaje de conexión al topic fijo
void publishConnectionStatus() {
  if (client.connected()) {
    StaticJsonDocument<512> doc;
    char jsonBuffer[512];

    // Información básica del dispositivo
    doc["device"] = deviceConfig.deviceName;
    doc["location"] = deviceConfig.location;
    doc["ip"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();

    // Información de configuración
    doc["sensor_type"] = deviceConfig.sensorType;
    doc["connection_mode"] = deviceConfig.connectionMode;
    doc["wifi_enabled"] = wifiConfig.enabled;
    doc["wifi_ssid"] = wifiConfig.ssid;

    // Información de MQTT
    doc["mqtt_server"] = mqttConfig.server;
    doc["mqtt_port"] = mqttConfig.port;

    // Estado actual
    doc["status"] = "online";
    doc["uptime"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["firmware_version"] = String(FW_VERSION);
    doc["timestamp"] = millis();

    serializeJson(doc, jsonBuffer);

    // Publicar al topic fijo (no configurable)
    bool result = client.publish("multi-sensor/status", jsonBuffer);

    Serial.println("=================================");
    Serial.println("MQTT > STATUS DE CONEXIÓN PUBLICADO");
    Serial.print("Topic: multi-sensor/status");
    Serial.print(" | Resultado: ");
    Serial.println(result ? "OK" : "FALLÓ");
    Serial.println("Mensaje:");
    Serial.println(jsonBuffer);
    Serial.println("=================================");
  }
}

// Tarea de MQTT con debug mejorado
void mqttTask(void *pvParameters) {
  unsigned long lastPublishTime = 0;
  Serial.println("🔄 MQTT Task iniciada");

  for (;;) {
    // Verificar conexión de red (Ethernet O WiFi)
    bool networkConnected = (eth_connected || WiFi.status() == WL_CONNECTED);

    if (!networkConnected) {
      Serial.println("❌ MQTT: Sin conexión de red (Ethernet: " + String(eth_connected ? "✅" : "❌") +
                     ", WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✅" : "❌") + "), esperando...");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Verificar conexión MQTT
    if (!client.connected()) {
      Serial.println("❌ MQTT: Desconectado, intentando reconectar...");
      if (reconectarMQTT()) {
        Serial.println("✅ MQTT: Reconexión exitosa");
      } else {
        Serial.println("❌ MQTT: Fallo en reconexión, reintentando en 5s");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
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


// Función para reconectar a MQTT con mensaje de status
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

    // Esperar un momento para asegurar conexión estable antes de publicar
    Serial.println("MQTT > Esperando conexión estable...");
    vTaskDelay(500 / portTICK_PERIOD_MS);

    // Publicar mensaje de status al conectar (topic fijo)
    publishConnectionStatus();

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
// Variables globales para control OTA
bool otaInProgress = false;
unsigned long lastOTAAttempt = 0;
int otaFailureCount = 0;
bool otaUpdatePending = false;  // Nueva: bandera para actualización pendiente
String pendingOtaUrl = "";      // Nueva: URL del firmware pendiente
String pendingOtaChecksum = ""; // Nueva: Checksum del firmware pendiente

void otaTask(void *pvParameters) {
  // RESET COMPLETO DE VARIABLES OTA (v1.6.) - Forzar actualización sin backoff
  otaInProgress = false;
  lastOTAAttempt = 0;  // IMPORTANTE: Resetear a 0 para evitar backoff
  otaFailureCount = 0; // IMPORTANTE: Resetear a 0 para evitar backoff
  otaUpdatePending = false;
  pendingOtaUrl = "";
  pendingOtaChecksum = "";
  Serial.println("🔄 [RESET OTA] Variables OTA reseteadas completamente - v1.6.");
  Serial.println("🔄 [RESET OTA] Backoff desactivado - verificará inmediatamente");
  Serial.println("=================================================");
  Serial.println("🚀 OTA > INICIANDO SISTEMA DE ACTUALIZACIONES v1.6. [SIN BACKOFF]");
  Serial.println("📊 OTA > Intervalo de verificación: " + String(ota_check_interval/1000) + " segundos");
  Serial.println("⏱️  OTA > Timeout de descarga: " + String(ota_timeout/1000) + " segundos");
  Serial.println("🔒 OTA > Modo seguro con reintentos exponenciales activado");
  Serial.println("📈 OTA > Contador de verificaciones activado");
  Serial.println("🔍 OTA > DEBUGGING DETALLADO DEL PROCESO OTA ACTIVADO");
  Serial.println("💡 OTA > INDICADOR LED DE ESTADO OTA ACTIVADO");
  Serial.println("🚀 OTA > VERIFICACIÓN INMEDIATA AL CONECTAR WIFI");
  Serial.println("🔄 OTA -> RESET BACKOFF PARA DESARROLLO");
  Serial.println("⚡ OTA -> SIN ESPERAR ENTRE INTENTOS");
  Serial.println("🎯 OTA -> VERSIÓN DEPURACIÓN DE BACKOFF");
  Serial.println("=================================================");

  unsigned long checkCount = 0;

  for (;;) {
    // Esperar el intervalo de verificación OTA
    vTaskDelay(ota_check_interval / portTICK_PERIOD_MS);

    // Incrementar contador
    checkCount++;

    // IMPORTANTE: NO verificar actualizaciones en modo bridge o hotspot
    if (bridgeMode || hotspotMode) {
      if (checkCount % 10 == 0) { // Mostrar mensaje cada 10 checks para no spammear
        Serial.println("🔒 OTA > Modo config activo (" + String(bridgeMode ? "Bridge" : "Hotspot") + "), OTA pausado");
      }
      continue;
    }

    // No verificar actualizaciones si ya hay una en curso o hay demasiados fallos recientes
    if (otaInProgress || otaFailureCount >= 3) {
      if (otaFailureCount >= 3) {
        vTaskDelay(300000 / portTICK_PERIOD_MS); // Esperar 5 minutos si hay muchos fallos
        Serial.println("OTA > Demasiados fallos, esperando 5 minutos...");
      }
      continue;
    }

    // Esperar a que haya conexión (Ethernet o WiFi) antes de verificar actualizaciones
    // IMPORTANTE: En modo hotspot, WiFi.status() == WL_CONNECTED es true pero es el AP, no cliente
    bool networkConnected = (eth_connected || (WiFi.status() == WL_CONNECTED && !hotspotMode));

    if (!networkConnected) {
      Serial.println("📶 OTA > [" + String(checkCount) + "] Sin conexión de red, esperando...");
      Serial.println("   ETH: " + String(eth_connected ? "✅" : "❌") +
                     " | WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✅" : "❌"));
    } else {
      // Indicador LED: Parpadeo azul cuando verifica OTA
      digitalWrite(CONFIG_LED_PIN, HIGH);
      delay(100);
      digitalWrite(CONFIG_LED_PIN, LOW);

      Serial.println("🌐 OTA > [" + String(checkCount) + "] Conexión detectada - Verificando actualizaciones...");
      Serial.println("   ETH: " + String(eth_connected ? "✅" : "❌") +
                     " | WiFi: " + String(WiFi.status() == WL_CONNECTED ? "✅" : "❌"));

      if (checkForUpdatesSafe()) {
        Serial.println("🎉 OTA > ¡ACTUALIZACIÓN EN PROGRESO! Reiniciando...");
        // checkForUpdatesSafe() ya maneja la actualización de forma segura
      } else {
        Serial.println("✅ OTA > Verificación completada - No hay actualizaciones disponibles");
      }
    }
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
  // vTaskSuspendAll(); // Comentado para evitar crash FreeRTOS (v1.6.)

  HTTPClient http;
  http.setTimeout(ota_timeout);

  if (!http.begin(firmwareInfo.url)) {
    Serial.println("OTA > No se pudo conectar al servidor de firmware");
    // xTaskResumeAll(); // Comentado para evitar crash FreeRTOS (v1.6.)
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
    // xTaskResumeAll(); // Comentado para evitar crash FreeRTOS (v1.6.)
    Serial.println("OTA > Actualización fallida, reanudando operaciones normales");
  }

  return success;
}

// Versión segura de performOTAUpdate con mejor manejo de FreeRTOS
bool performSafeOTAUpdate(const FirmwareInfo& firmwareInfo) {
  Serial.println("OTA > Iniciando actualización segura del firmware...");
  Serial.print("OTA > URL del firmware: ");
  Serial.println(firmwareInfo.url);

  if (firmwareInfo.checksum.length() > 0) {
    Serial.print("OTA > Checksum esperado: ");
    Serial.println(firmwareInfo.checksum);
  }

  bool success = false;

  try {
    // ACTUALIZACIÓN DIRECTA CON HTTPUpdate (v1.6.) - Sin banderas pendientes
    Serial.println("OTA > Ejecutando actualización directa con HTTPUpdate...");
    Serial.printf("OTA > Memoria libre antes de actualizar: %d bytes\n", ESP.getFreeHeap());

    // Detener MQTT si está conectado para evitar conflictos
    if (client.connected()) {
      Serial.println("OTA > Desconectando MQTT...");
      client.disconnect();
      delay(1000); // Esperar a que se complete la desconexión
    }

    // Detener servidor web si está activo
    if (configServer) {
      Serial.println("OTA > Deteniendo servidor web...");
      configServer->stop();
      delay(500); // Esperar a que se detenga completamente
    }

    // ACTUALIZACIÓN DIRECTA con Update (método ESP32 nativo) - v1.6.
    Serial.println("OTA > 🔄 Iniciando actualización directa con Update...");

    // Crear HTTPClient para descarga
    HTTPClient http;
    http.setTimeout(ota_timeout);

    if (!http.begin(firmwareInfo.url)) {
      Serial.println("OTA > ❌ Error: No se pudo iniciar conexión HTTP");
      otaFailureCount++;
      return false;
    }

    Serial.println("OTA > 🔄 Descargando firmware...");
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
      Serial.printf("OTA > ❌ Error HTTP %d: %s\n", httpCode, http.errorToString(httpCode).c_str());
      http.end();
      otaFailureCount++;
      return false;
    }

    int contentLength = http.getSize();
    if (contentLength <= 0) {
      Serial.println("OTA > ❌ Error: No se pudo obtener el tamaño del firmware");
      http.end();
      otaFailureCount++;
      return false;
    }

    Serial.printf("OTA > 📦 Tamaño del firmware: %d bytes\n", contentLength);

    // Verificar espacio disponible
    size_t sketchSpace = ESP.getFreeSketchSpace();
    if (contentLength > sketchSpace) {
      Serial.printf("OTA > ❌ Error: No hay espacio suficiente. Disponible: %d bytes\n", sketchSpace);
      http.end();
      otaFailureCount++;
      return false;
    }

    // Iniciar actualización
    if (!Update.begin(contentLength)) {
      Serial.printf("OTA > ❌ Error al iniciar actualización: %d\n", Update.getError());
      http.end();
      otaFailureCount++;
      return false;
    }

    // Configurar headers
    http.addHeader("User-Agent", "ESP32-OTA-Client/v1.6.");

    // Descargar y escribir firmware
    Serial.println("OTA > 🔄 Escribiendo firmware en partición OTA...");
    WiFiClient *stream = http.getStreamPtr();
    size_t written = Update.writeStream(*stream);

    if (written != contentLength) {
      Serial.printf("OTA > ❌ Error: Solo se escribieron %d de %d bytes\n", written, contentLength);
      Serial.printf("OTA > ❌ Error de actualización: %s\n", Update.errorString());
      http.end();
      Update.abort();
      otaFailureCount++;
      return false;
    }

    // Finalizar actualización
    if (!Update.end()) {
      Serial.printf("OTA > ❌ Error finalizando actualización: %s\n", Update.errorString());
      http.end();
      otaFailureCount++;
      return false;
    }

    http.end();

    Serial.println("OTA > ✅ Actualización completada exitosamente!");
    Serial.println("OTA > 🔄 Reiniciando dispositivo...");

    // Resetear contador de fallos en éxito
    otaFailureCount = 0;
    lastOTAAttempt = 0;

    delay(1000);
    ESP.restart();
    return true;

  } catch (...) {
    Serial.println("OTA > ❌ Excepción durante el proceso de actualización");
    otaFailureCount++;
    return false;
  }
}

// Versión segura de checkForUpdates con debugging intensivo
bool checkForUpdatesSafe() {
  Serial.println("🔍 [DEBUG] checkForUpdatesSafe() INICIADO");
  Serial.println("🔍 [DEBUG] Versión actual: " + String(FW_VERSION));

  // Prevenir múltiples intentos simultáneos
  if (otaInProgress) {
    Serial.println("🔍 [DEBUG] OTA ya en progreso, saliendo");
    return false;
  }

  // FORZAR VERIFICACIÓN INMEDIATA - SIN BACKOFF (v1.6.)
  unsigned long now = millis();
  otaInProgress = true;
  lastOTAAttempt = now;

  Serial.println("🔍 [DEBUG] Forzando verificación inmediata - sin backoff");
  Serial.println("🔍 [DEBUG] Fallos: " + String(otaFailureCount) + ", Reseteados para desarrollo");

  Serial.println("OTA > Iniciando verificación segura de actualizaciones...");
  Serial.println("🌐 [INFO] Versión remota se mostrará siempre para debugging");
  Serial.println("🔍 [DEBUG] WiFi status: " + String(WiFi.status()));
  Serial.println("🔍 [DEBUG] WiFi connected: " + String(WiFi.isConnected() ? "SÍ" : "NO"));
  Serial.println("🔍 [DEBUG] IP actual: " + WiFi.localIP().toString());

  bool success = false;

  // Usar contexto seguro para operaciones de red
  HTTPClient http;
  http.setTimeout(ota_timeout);

  Serial.println("🔍 [DEBUG] URL de versiones: " + String(ota_version_url));
  Serial.println("🔍 [DEBUG] Timeout HTTP: " + String(ota_timeout/1000) + "s");
  Serial.println("🔍 [DEBUG] Iniciando conexión HTTP...");

  try {
    // Obtener información de versión del servidor
    Serial.println("🔍 [DEBUG] Iniciando conexión HTTP...");
    if (!http.begin(ota_version_url)) {
      Serial.println("🔍 [ERROR] No se pudo iniciar conexión HTTP");
      otaFailureCount++;
      goto cleanup;
    }

    Serial.println("🔍 [DEBUG] Enviando request GET...");
    int httpCode = http.GET();
    Serial.println("🔍 [DEBUG] Código HTTP recibido: " + String(httpCode));

    if (httpCode != HTTP_CODE_OK) {
      Serial.print("OTA > Error al obtener información de versión, código HTTP: ");
      Serial.println(httpCode);
      otaFailureCount++;
      goto cleanup;
    }

    // Parsear JSON de versión
    Serial.println("🔍 [DEBUG] Obteniendo payload JSON...");
    String payload = http.getString();
    Serial.println("🔍 [DEBUG] Payload recibido (" + String(payload.length()) + " bytes):");
    Serial.println("🔍 [DEBUG] " + payload);
    http.end();

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println("🔍 [ERROR] Error al parsear JSON de versión: " + String(error.c_str()));
      Serial.println("🔍 [ERROR] Payload que falló el parseo:");
      Serial.println("🔍 [ERROR] " + payload);
      otaFailureCount++;
      goto cleanup;
    }

    // Extraer información del firmware y filesystem
    FirmwareInfo firmwareInfo;
    firmwareInfo.version = doc["version"].as<String>();

    // Soporte para formato antiguo (url) y nuevo (firmware/filesystem)
    if (doc.containsKey("firmware")) {
      // Formato nuevo: firmware y filesystem separados
      firmwareInfo.url = doc["firmware"].as<String>();
      firmwareInfo.filesystemUrl = doc["filesystem"].as<String>();
    } else {
      // Formato antiguo: solo url
      firmwareInfo.url = doc["url"].as<String>();
      firmwareInfo.filesystemUrl = "";
    }

    firmwareInfo.checksum = doc["checksum"].as<String>();
    firmwareInfo.mandatory = doc["mandatory"] | false;
    firmwareInfo.release_notes = doc["release_notes"].as<String>();

    // Obtener versión actual
    String currentVersion = getCurrentFirmwareVersion();
    Serial.println("🔍 [DEBUG] Versión actual: " + currentVersion);
    Serial.println("🌐 [REMOTE] Versión disponible en servidor: " + firmwareInfo.version);
    Serial.println("📊 [COMPARE] " + currentVersion + " vs " + firmwareInfo.version);

    // Mostrar info de filesystem si está disponible
    if (firmwareInfo.filesystemUrl.length() > 0) {
      Serial.println("📁 [FILESYSTEM] Actualización de filesystem disponible: " + firmwareInfo.filesystemUrl);
    }

    // Si hay notas de lanzamiento, mostrarlas
    if (firmwareInfo.release_notes.length() > 0) {
      Serial.print("OTA > Notas de la versión: ");
      Serial.println(firmwareInfo.release_notes);
    }

    // Comparar versiones
    Serial.println("🔍 [DEBUG] Comparando versiones...");
    bool updateNeeded = compareVersions(currentVersion, firmwareInfo.version);
    Serial.println("🔍 [DEBUG] ¿Se necesita actualización? " + String(updateNeeded ? "SÍ" : "NO"));

    if (updateNeeded) {
      Serial.println("🔍 [DEBUG] ¡ACTUALIZACIÓN DISPONIBLE! Iniciando descarga...");

      // Indicador LED: Parpadeo rápido azul cuando actualiza
      for(int i=0; i<5; i++) {
        digitalWrite(CONFIG_LED_PIN, HIGH);
        delay(50);
        digitalWrite(CONFIG_LED_PIN, LOW);
        delay(50);
      }

      // Para actualizaciones obligatorias, forzar la instalación
      if (firmwareInfo.mandatory) {
        Serial.println("🔍 [DEBUG] Actualización obligatoria, instalando...");
        success = performSafeOTAUpdate(firmwareInfo);
      } else {
        // Para actualizaciones opcionales, preguntar al usuario (o instalar automáticamente)
        Serial.println("🔍 [DEBUG] Actualización opcional, instalando automáticamente...");
        success = performSafeOTAUpdate(firmwareInfo);
      }
    } else {
      Serial.println("🔍 [DEBUG] Firmware actualizado, no se necesita actualización");
      otaFailureCount = 0; // Resetear contador de fallos
    lastOTAAttempt = 0; // Resetear timestamp para evitar backoff falso (v1.6.) en éxito
      lastOTAAttempt = 0; // Resetear timestamp para evitar backoff falso (v1.6.)
      success = false;
    }

  } catch (...) {
    Serial.println("OTA > Excepción durante la verificación de actualizaciones");
    otaFailureCount++;
    success = false;
  }

cleanup:
  otaInProgress = false;

  if (success) {
    Serial.println("OTA > Actualización completada exitosamente");
    otaFailureCount = 0; // Resetear contador de fallos
    lastOTAAttempt = 0; // Resetear timestamp para evitar backoff falso (v1.6.)

    // Esperar un momento antes de reiniciar
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP.restart();

    return true;
  } else {
    Serial.print("OTA > Verificación fallida, contador de fallos: ");
    Serial.println(otaFailureCount);

    // IMPORTANTE: Mostrar versión remota siempre, incluso si falló
    Serial.println("🌐 [FALLBACK] Intentando obtener versión remota para debugging...");
    try {
      HTTPClient fallbackHttp;
      fallbackHttp.setTimeout(10000); // 10 segundos timeout
      if (fallbackHttp.begin(ota_version_url)) {
        int fallbackCode = fallbackHttp.GET();
        if (fallbackCode == HTTP_CODE_OK) {
          String fallbackPayload = fallbackHttp.getString();
          DynamicJsonDocument fallbackDoc(512);
          if (deserializeJson(fallbackDoc, fallbackPayload) == DeserializationError::Ok) {
            String remoteVersion = fallbackDoc["version"].as<String>();
            Serial.println("🌐 [REMOTE] Versión en servidor (fallback): " + remoteVersion);
            Serial.println("📊 [COMPARE] Local: " + String(FW_VERSION) + " vs Remoto: " + remoteVersion);
          } else {
            Serial.println("🌐 [REMOTE] Error parseando JSON fallback");
          }
        } else {
          Serial.println("🌐 [REMOTE] Error HTTP fallback: " + String(fallbackCode));
        }
        fallbackHttp.end();
      } else {
        Serial.println("🌐 [REMOTE] Error conexión fallback");
      }
    } catch (...) {
      Serial.println("🌐 [REMOTE] Excepción en fallback");
    }

    return false;
  }
}

// Obtener versión actual del firmware
String getCurrentFirmwareVersion() {
  // Usar siempre nuestra versión definida en tiempo de compilación
  Serial.println("OTA > Usando versión FW_VERSION del firmware: " + String(FW_VERSION));
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

  // Debug: mostrar resultado de comparación
  bool needsUpdate = (available_major > current_major) ||
                      (available_major == current_major && available_minor > current_minor) ||
                      (available_major == current_major && available_minor == current_minor && available_patch > current_patch);

  Serial.printf("OTA > ¿Necesita actualización? %s\n", needsUpdate ? "SÍ" : "NO");

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
  Serial.println("Sistema de preferencias inicializado (persistente OTA)");
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
  deviceConfig.vibrationThreshold = preferences.getInt("vibrThresh", 250); // 250ms = ~4 publicaciones/segundo
  deviceConfig.vibrationMode = preferences.getInt("vibrationMode", 0); // 0=golpe, 1=vibracion

  // Cargar configuración WiFi
  wifiConfig.ssid = preferences.getString("wifiSSID", "");
  wifiConfig.password = preferences.getString("wifiPassword", "");
  wifiConfig.enabled = preferences.getBool("wifiEnabled", false);
  wifiConfig.channel = preferences.getInt("wifiChannel", 0);
  wifiConfig.hidden = preferences.getBool("wifiHidden", false);

  // Cargar modo de conexión (por defecto Ethernet)
  deviceConfig.connectionMode = preferences.getInt("connectionMode", MODE_ETHERNET);

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
  preferences.putInt("vibrThresh", deviceConfig.vibrationThreshold);
  preferences.putInt("vibrationMode", deviceConfig.vibrationMode);

  // Guardar configuración WiFi
  preferences.putString("wifiSSID", wifiConfig.ssid);
  preferences.putString("wifiPassword", wifiConfig.password);
  preferences.putBool("wifiEnabled", wifiConfig.enabled);
  preferences.putInt("wifiChannel", wifiConfig.channel);
  preferences.putBool("wifiHidden", wifiConfig.hidden);

  // Guardar modo de conexión
  preferences.putInt("connectionMode", deviceConfig.connectionMode);

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

    case SENSOR_VIBRATION_BUTTON:
      Serial.println("========================================");
      Serial.println("Configurando VIBRACIÓN + 1 PULSADOR");
      Serial.println("========================================");
      Serial.print("  Pin Vibración: GPIO ");
      Serial.println(deviceConfig.vibrationPin);
      Serial.print("  Pin Pulsador 1: GPIO ");
      Serial.println(deviceConfig.button1Pin);
      Serial.print("  Topic Vibración: ");
      Serial.println(deviceConfig.vibrationTopic);
      Serial.print("  Topic Pulsador: ");
      Serial.println(deviceConfig.button1Topic);
      pinMode(deviceConfig.vibrationPin, INPUT_PULLUP);
      pinMode(deviceConfig.button1Pin, INPUT_PULLUP);
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
  deviceConfig.vibrationThreshold = 250; // 250ms = ~4 publicaciones/segundo
  deviceConfig.vibrationMode = 0; // 0=golpe por defecto

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

  // Inicializar LittleFS para archivos web
  if (!LittleFS.begin(true)) {
    Serial.println("Error: No se pudo inicializar LittleFS");
    logSystemEvent("LITTLEFS_ERROR", "Error al inicializar LittleFS en modo hotspot");
  } else {
    Serial.println("LittleFS inicializado correctamente");
    logSystemEvent("LITTLEFS_OK", "LittleFS inicializado en modo hotspot");

    // DEBUG: Listar archivos en LittleFS
    Serial.println("=== Archivos en LittleFS ===");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    int fileCount = 0;
    while (file) {
      Serial.printf("Archivo: %s, Tamaño: %d bytes\n", file.name(), file.size());
      file = root.openNextFile();
      fileCount++;
    }
    if (fileCount == 0) {
      Serial.println("⚠️ LittleFS está VACIO - No hay archivos!");
    }
    Serial.println("===============================");
  }

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
  configServer->on("/exit", HTTP_ANY, [](){
    String method = (configServer->method() == HTTP_GET) ? "GET" : "POST";

    if (bridgeMode) {
      logSystemEvent("BRIDGE_EXIT", "Usuario solicitó salir del modo bridge (" + method + ")");

      String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Saliendo del Modo Configuración</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 600px; margin: 50px auto; background: white; padding: 30px; border-radius: 10px; text-align: center; }
        h1 { color: #4CAF50; margin-bottom: 20px; }
        .spinner { border: 4px solid #f3f3f3; border-top: 4px solid #4CAF50; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; margin: 20px auto; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .message { color: #666; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>✅ Configuración Guardada</h1>
        <div class="spinner"></div>
        <div class="message">
            <p><strong>Saliendo del modo configuración...</strong></p>
            <p>El dispositivo se reiniciará automáticamente en modo normal</p>
            <p>Por favor, espera unos segundos...</p>
        </div>
    </div>
</body>
</html>
      )";

      configServer->send(200, "text/html", html);
      delay(3000);
      exitBridgeMode();

    } else if (hotspotMode) {
      logSystemEvent("HOTSPOT_EXIT", "Usuario solicitó salir del modo hotspot (" + method + ")");

      String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Saliendo del Modo Configuración</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f0f0; }
        .container { max-width: 600px; margin: 50px auto; background: white; padding: 30px; border-radius: 10px; text-align: center; }
        h1 { color: #4CAF50; margin-bottom: 20px; }
        .spinner { border: 4px solid #f3f3f3; border-top: 4px solid #4CAF50; border-radius: 50%; width: 40px; height: 40px; animation: spin 1s linear infinite; margin: 20px auto; }
        @keyframes spin { 0% { transform: rotate(0deg); } 100% { transform: rotate(360deg); } }
        .message { color: #666; margin: 20px 0; }
    </style>
</head>
<body>
    <div class="container">
        <h1>✅ Configuración Guardada</h1>
        <div class="spinner"></div>
        <div class="message">
            <p><strong>Saliendo del modo configuración...</strong></p>
            <p>El dispositivo se reiniciará automáticamente en modo normal</p>
            <p>Por favor, espera unos segundos...</p>
        </div>
    </div>
</body>
</html>
      )";

      configServer->send(200, "text/html", html);
      delay(3000);
      exitHotspotMode();

    } else {
      configServer->send(200, "text/html",
        "<html><body><h2>El dispositivo ya está en modo normal</h2><p><a href='/'>Volver al panel</a></p></body></html>");
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
  Serial.println("=== PROCESANDO GUARDADO DE CONFIGURACIÓN ===");

  String message = "<html><body><div class='container'><h1>Guardando Configuración</h1>";

  // Depurar: mostrar todos los argumentos recibidos (tanto en Serial como en la página)
  String debugInfo = "<h3>Debug: Datos recibidos del formulario:</h3><ul style='background:#f0f0f0;padding:10px;border-radius:5px;font-family:monospace;font-size:12px;'>";
  Serial.println("Argumentos recibidos del formulario:");

  for (int i = 0; i < configServer->args(); i++) {
    String argName = configServer->argName(i);
    String argValue = configServer->arg(i);
    Serial.println("  " + argName + " = " + argValue);
    debugInfo += "<li><strong>" + argName + "</strong> = " + argValue + "</li>";
  }
  debugInfo += "</ul>";

  // Guardar configuración de red
  networkConfig.dhcpEnabled = configServer->hasArg("dhcpEnabled");
  networkConfig.staticIP = configServer->arg("staticIP");
  networkConfig.gateway = configServer->arg("gateway");
  networkConfig.subnet = configServer->arg("subnet");
  networkConfig.dns1 = configServer->arg("dns1");
  networkConfig.dns2 = configServer->arg("dns2");

  // Guardar configuración WiFi
  wifiConfig.enabled = configServer->hasArg("wifiEnabled");
  wifiConfig.ssid = configServer->arg("wifiSSID");
  wifiConfig.password = configServer->arg("wifiPassword");

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

  // Guardar modo de conexión
  deviceConfig.connectionMode = configServer->arg("connectionMode").toInt();

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
    // Sincronizar con el topic principal MQTT si está vacío o es el valor por defecto
    String mainTopic = configServer->arg("mainMqttTopic");
    if (mqttConfig.topic.equals("sensor/distance") || mqttConfig.topic.length() == 0) {
      mqttConfig.topic = mainTopic;
    }
  }

  // Umbral de vibración
  if (configServer->hasArg("vibrationThreshold")) {
    deviceConfig.vibrationThreshold = configServer->arg("vibrationThreshold").toInt();
  }

  if (configServer->hasArg("vibrationMode")) {
    deviceConfig.vibrationMode = configServer->arg("vibrationMode").toInt();
  }

  
  // Validar configuración
  bool configValid = true;
  String errorMessage = "";

  Serial.println("=== VALIDACIÓN DE CONFIGURACIÓN ===");
  Serial.print("MQTT Server: '");
  Serial.print(mqttConfig.server);
  Serial.println("' (length: " + String(mqttConfig.server.length()) + ")");
  Serial.print("MQTT Port: ");
  Serial.println(mqttConfig.port);
  Serial.print("DHCP Enabled: ");
  Serial.println(networkConfig.dhcpEnabled ? "true" : "false");

  if (mqttConfig.server.length() == 0) {
    configValid = false;
    errorMessage += "El servidor MQTT es requerido<br>";
    Serial.println("❌ MQTT server vacío");
  }

  if (!networkConfig.dhcpEnabled) {
    if (!validateNetworkConfig(networkConfig.staticIP, networkConfig.gateway, networkConfig.subnet)) {
      configValid = false;
      errorMessage += "La configuración de IP estática no es válida<br>";
      Serial.println("❌ Configuración IP estática inválida");
    }
  }

  if (!validateMQTTConfig(mqttConfig.server, mqttConfig.port)) {
    configValid = false;
    errorMessage += "La configuración MQTT no es válida<br>";
    Serial.println("❌ Validación MQTT falló");
  }

  Serial.print("Resultado validación: ");
  Serial.println(configValid ? "✅ VÁLIDA" : "❌ INVÁLIDA");
  if (!configValid) {
    Serial.print("Error message: ");
    Serial.println(errorMessage);
  }
  Serial.println("================================");

  // Añadir información de debug al mensaje
  message += debugInfo;
  message += "<hr>";

  if (configValid) {
    Serial.println("✅ Configuración válida, guardando...");
    saveConfiguration();
    logSystemEvent("CONFIG_SAVED", "Configuración guardada exitosamente desde panel web");
    message += "<div class='status success'>✅ Configuración guardada exitosamente!</div>";
    message += "<p>El dispositivo se reiniciará en modo normal.</p>";
    message += "<div style='margin: 20px 0;'>";
    message += "<p><strong>Configuración guardada:</strong></p>";
    message += "<ul>";
    message += "<li>WiFi: " + String(wifiConfig.enabled ? "Habilitado" : "Deshabilitado") + "</li>";
    message += "<li>SSID: " + wifiConfig.ssid + "</li>";
    message += "<li>Dispositivo: " + deviceConfig.deviceName + "</li>";
    message += "<li>Ubicación: " + deviceConfig.location + "</li>";
    message += "<li>MQTT Server: " + mqttConfig.server + ":" + mqttConfig.port + "</li>";
    message += "<li>Topic: " + mqttConfig.topic + "</li>";
    message += "</ul>";
    message += "</div>";
    message += "<button onclick='setTimeout(function(){ window.location.href=\"/exit\"; }, 2000);'>Continuar</button>";
  } else {
    Serial.println("❌ Error en la configuración: " + errorMessage);
    logSystemEvent("CONFIG_ERROR", "Error en validación: " + errorMessage);
    message += "<div class='status error'>❌ Error en la configuración:</div>";
    message += "<p>" + errorMessage + "</p>";
    message += "<button onclick='history.back()'>Volver</button>";
  }

  message += "</div></body></html>";
  configServer->send(200, "text/html", message);

  if (configValid) {
    Serial.println("Reiniciando dispositivo en 3 segundos...");
    delay(3000);
    ESP.restart();
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

  // Configuración de red
  json += "\"dhcpEnabled\":" + String(networkConfig.dhcpEnabled ? "true" : "false") + ",";
  json += "\"staticIP\":\"" + networkConfig.staticIP + "\",";
  json += "\"gateway\":\"" + networkConfig.gateway + "\",";
  json += "\"subnet\":\"" + networkConfig.subnet + "\",";
  json += "\"dns1\":\"" + networkConfig.dns1 + "\",";
  json += "\"dns2\":\"" + networkConfig.dns2 + "\",";

  // Configuración WiFi
  json += "\"wifiEnabled\":" + String(wifiConfig.enabled ? "true" : "false") + ",";
  json += "\"wifiSSID\":\"" + wifiConfig.ssid + "\",";
  json += "\"wifiPassword\":\"" + String(wifiConfig.password.length() > 0 ? "***PROTECTED***" : "") + "\",";

  // Configuración MQTT
  json += "\"mqttServer\":\"" + mqttConfig.server + "\",";
  json += "\"mqttPort\":" + String(mqttConfig.port) + ",";
  json += "\"mqttUsername\":\"" + mqttConfig.username + "\",";
  json += "\"mqttPassword\":\"" + String(mqttConfig.password.length() > 0 ? "***PROTECTED***" : "") + "\",";
  json += "\"mqttTopic\":\"" + mqttConfig.topic + "\",";
  json += "\"mqttClientId\":\"" + mqttConfig.clientId + "\",";
  json += "\"mqttKeepAlive\":" + String(mqttConfig.keepAlive) + ",";

  // Configuración de conexión
  json += "\"connectionMode\":" + String(deviceConfig.connectionMode) + ",";

  // Configuración de sensores
  json += "\"sensorType\":" + String(deviceConfig.sensorType) + ",";
  json += "\"button1Pin\":" + String(deviceConfig.button1Pin) + ",";
  json += "\"button2Pin\":" + String(deviceConfig.button2Pin) + ",";
  json += "\"vibrationPin\":" + String(deviceConfig.vibrationPin) + ",";
  json += "\"button1Invert\":" + String(deviceConfig.button1Invert ? "true" : "false") + ",";
  json += "\"button2Invert\":" + String(deviceConfig.button2Invert ? "true" : "false") + ",";
  json += "\"vibrationThreshold\":" + String(deviceConfig.vibrationThreshold) + ",";
  json += "\"vibrationMode\":" + String(deviceConfig.vibrationMode) + ",";
  json += "\"button1Topic\":\"" + deviceConfig.button1Topic + "\",";
  json += "\"button2Topic\":\"" + deviceConfig.button2Topic + "\",";
  json += "\"vibrationTopic\":\"" + deviceConfig.vibrationTopic + "\",";
  json += "\"mainMqttTopic\":\"" + deviceConfig.mainMqttTopic + "\",";

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

// -- NUEVAS FUNCIONES PARA CONTROL DE CONEXIÓN --

void initializeConnectionModules() {
  Serial.println("🔧 Inicializando módulos de conexión...");
  Serial.print("Modo de conexión configurado: ");

  switch(deviceConfig.connectionMode) {
    case MODE_ETHERNET:
      Serial.println("Ethernet (únicamente)");
      break;
    case MODE_WIFI:
      Serial.println("WiFi (únicamente)");
      setupWiFi();
      break;
    case MODE_DUAL_ETH_WIFI:
      Serial.println("Dual: Ethernet primario + WiFi backup");
      setupWiFi();
      break;
    default:
      Serial.println("No especificado - usando modo Ethernet por defecto");
      deviceConfig.connectionMode = MODE_ETHERNET;
      saveConfiguration();
      break;
  }

  currentConnectionMode = deviceConfig.connectionMode;
}

void setupWiFi() {
  if (!wifiConfig.enabled || wifiConfig.ssid.length() == 0) {
    Serial.println("❌ WiFi no está habilitado o no hay SSID configurado");
    return;
  }

  Serial.println("📶 Inicializando WiFi...");
  Serial.print("Conectando a: ");
  Serial.println(wifiConfig.ssid);

  // Configurar WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Mantener WiFi siempre activo

  // Nota: WiFi.setChannel() no está disponible en ESP32 con modo STA
  // Se usará el canal configurado en el AP si es necesario

  // Intentar conexión
  WiFi.begin(wifiConfig.ssid.c_str(), wifiConfig.password.c_str());

  int attempts = 0;
  const int maxAttempts = 20;

  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.println("✅ WiFi conectado exitosamente");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("Señal (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    logSystemEvent("WIFI_CONNECTED", "IP: " + WiFi.localIP().toString() + " | SSID: " + wifiConfig.ssid);

    // Sistema OTA con timer de 30 segundos - sin boot checks
  } else {
    wifiConnected = false;
    Serial.println();
    Serial.println("❌ Falló la conexión WiFi");
    logSystemEvent("WIFI_FAILED", "No se pudo conectar a: " + wifiConfig.ssid);
  }
}


void checkConnectionMode() {
  unsigned long currentTime = millis();

  // Revisar conexiones cada connectionCheckInterval
  if (currentTime - lastConnectionCheck < connectionCheckInterval) {
    return;
  }

  lastConnectionCheck = currentTime;

  bool currentEthStatus = eth_connected; 
  bool currentWifiStatus = (WiFi.status() == WL_CONNECTED);

  if (currentEthStatus != ethernetConnected) {
    ethernetConnected = currentEthStatus;
    if (ethernetConnected) {
      Serial.println("🌐 Ethernet conectado");
      logSystemEvent("ETH_CONNECTED", "Link activo");
    } else {
      Serial.println("📵 Ethernet desconectado");
      logSystemEvent("ETH_DISCONNECTED", "Link perdido");
    }
  }

  if (currentWifiStatus != wifiConnected) {
    wifiConnected = currentWifiStatus;
    if (wifiConnected) {
      Serial.println("📶 WiFi conectado");
      logSystemEvent("WIFI_CONNECTED", "IP: " + WiFi.localIP().toString());
    } else {
      Serial.println("📵 WiFi desconectado");
      logSystemEvent("WIFI_DISCONNECTED", "Señal perdida");
    }
  }

  switch(currentConnectionMode) {
    case MODE_DUAL_ETH_WIFI:
      if (!ethernetConnected && !wifiConnected) {
        Serial.println("⚠️ Ambas conexiones (Ethernet+WiFi) caídas");
        if (!ethernetConnected && !wifiConnected) {
          Serial.println("🔄 Intentando reconexión WiFi...");
          setupWiFi();
        }
      }
      break;
  }
}

void switchConnectionMode(int newMode) {
  if (newMode < MODE_ETHERNET || newMode > MODE_DUAL_ETH_WIFI) {
    Serial.println("❌ Modo de conexión inválido");
    return;
  }

  if (newMode == currentConnectionMode) {
    Serial.println("ℹ️ Ya está en el modo de conexión seleccionado");
    return;
  }

  Serial.print("🔄 Cambiando modo de conexión de ");
  Serial.print(currentConnectionMode);
  Serial.print(" a ");
  Serial.println(newMode);

  switch(currentConnectionMode) {
    case MODE_WIFI:
    case MODE_DUAL_ETH_WIFI:
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      wifiConnected = false;
      break;
  }

  deviceConfig.connectionMode = newMode;
  currentConnectionMode = newMode;
  saveConfiguration(); 

  initializeConnectionModules();

  Serial.println("✅ Modo de conexión cambiado exitosamente");
  logSystemEvent("CONNECTION_MODE_CHANGED", "Nuevo modo: " + String(newMode));
}


