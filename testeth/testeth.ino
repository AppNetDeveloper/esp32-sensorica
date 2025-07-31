#include <ETH.h>
#include <WiFi.h> // Se necesita para definir WiFiClient y el sistema de eventos, aunque no usemos el hardware WiFi.
#include <PubSubClient.h>
#include <ArduinoJson.h>

// -- INICIO: NUEVA SECCIÓN DE CONFIGURACIÓN DEL SENSOR --

// CAMBIO: Pines actualizados para ser compatibles con los conectores de la WT32-ETH01
#define TRIG_PIN 15
#define ECHO_PIN 14

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


// Configuración de MQTT
const char* mqtt_server = "192.168.3.154";
const char* mqtt_topic = "sensor/distance";

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

// Prototipos de tareas y funciones
void WiFiEvent(WiFiEvent_t event);
bool reconectarMQTT();
void sensorTask(void *pvParameters);
void mqttTask(void *pvParameters);


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
  Serial.println("\n\n--- INICIANDO DISPOSITIVO SENSOR (WT32-ETH01) ---");

  sensorMutex = xSemaphoreCreateMutex();

  // Configuración de pines para el sensor de ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // -- INICIO: INICIALIZACIÓN DE RED --
  
  Serial.println("Configurando conexión Ethernet...");
  WiFi.onEvent(WiFiEvent); 
  
  Serial.println("Intentando conectar a la red por RJ45...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
  
  // -- FIN: INICIALIZACIÓN DE RED --

  client.setServer(mqtt_server, 1883);

  Serial.println("Sensor de ultrasonido JSN-SR04T configurado.");
  Serial.println("------------------------------------");


  lastSensorRestart = millis();

  // Creación de tareas (sin cambios)
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  if (millis() - lastSensorRestart >= sensorRestartInterval) {
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
        
        if (client.publish(mqtt_topic, jsonBuffer)) {
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
  String clientId = generarClientId();
  Serial.print("MQTT > Intentando reconectar con ClientID: ");
  Serial.println(clientId);
  if (client.connect(clientId.c_str())) {
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

