#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Instancia para el sensor de distancia
VL53L1X sensor;

// Configuración de red WiFi
const char* ssid = "Lss";
const char* password = "cvlss2101281613";

// Configuración de MQTT
const char* mqtt_server = "arm1.appnet.dev";
const char* mqtt_topic = "sensor/distance2";
WiFiClient espClient;
PubSubClient client(espClient);

// Parámetros para el filtrado de lecturas (6 lecturas para calcular la mediana)
const int NUM_READINGS = 6;
volatile int readings[NUM_READINGS];   // Arreglo para almacenar las lecturas
volatile int readingIndex = 0;           // Índice actual
volatile int filteredValue = 0;          // Valor mediano calculado

// Bandera para indicar que hay nuevo dato listo para publicar
volatile bool newDataAvailable = false;
// Para llevar un control del último valor publicado (opcional)
volatile int lastPublishedValue = -1;

// Mutex para proteger variables compartidas entre tareas
SemaphoreHandle_t sensorMutex;

// Parámetros para reconexión MQTT con backoff exponencial
unsigned long lastReconnectAttempt = 0;
unsigned long baseReconnectInterval = 5000; // 5 segundos
int reconnectAttempts = 0;
const int maxReconnectAttempts = 6;         // Máximo (por ejemplo, 5 * 2^6 = 320 segundos de espera máximo)

// Para reinicio periódico del sensor
unsigned long lastSensorRestart = 0;
const unsigned long sensorRestartInterval = 1800000; // Cada 30 minutos

// Prototipos de tareas y funciones
void conectarWiFi();
bool reconectarMQTT();
void reinitSensor();
void sensorTask(void *pvParameters);
void mqttTask(void *pvParameters);

// Función auxiliar para ordenar un arreglo pequeño (burbuja)
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

// Función para calcular la mediana de NUM_READINGS elementos (para número par se promedian los dos centrales)
int calcularMediana(int arr[], int n) {
  int temp[n];
  for (int i = 0; i < n; i++) {
    temp[i] = arr[i];
  }
  sortArray(temp, n);
  return (temp[n/2 - 1] + temp[n/2]) / 2;
}

// Función para generar un clientId único basado en la MAC del ESP32
String generarClientId() {
  uint64_t mac = ESP.getEfuseMac();
  char id[25];
  sprintf(id, "ESP32Client-%llX", mac);
  return String(id);
}

void setup() {
  Serial.begin(115200);
  
  // Inicializamos el mutex
  sensorMutex = xSemaphoreCreateMutex();
  
  // Conexión a la red WiFi
  conectarWiFi();
  
  // Configurar el servidor MQTT
  client.setServer(mqtt_server, 1883);
  
  // Inicializar el sensor en modo continuo con 30 ms entre lecturas
  Wire.begin(21, 22);
  sensor.setTimeout(500);
  if (sensor.init()) {
    sensor.startContinuous(30); // Lectura cada 30 ms
    Serial.println("Sensor conectado.");
  } else {
    Serial.println("Sensor: No se ha detectado.");
  }
  
  lastSensorRestart = millis();
  
  // Crear tarea para la lectura del sensor en el núcleo 0
  xTaskCreatePinnedToCore(
    sensorTask,    // Función de la tarea
    "Sensor Task", // Nombre de la tarea
    10000,         // Tamaño del stack
    NULL,          // Parámetros
    1,             // Prioridad
    NULL,          // Handle (no se utiliza)
    0              // Ejecutar en el núcleo 0
  );
  
  // Crear tarea para el manejo de MQTT en el núcleo 1
  xTaskCreatePinnedToCore(
    mqttTask,
    "MQTT Task",
    10000,
    NULL,
    1,
    NULL,
    1              // Ejecutar en el núcleo 1
  );
}

void loop() {
  // Reinicio periódico del sensor cada 30 minutos
  if (millis() - lastSensorRestart >= sensorRestartInterval) {
    Serial.println("Reinicio programado del sensor...");
    reinitSensor();
    lastSensorRestart = millis();
  }
  delay(10);
}

// Tarea para la lectura del sensor (núcleo 0)
void sensorTask(void *pvParameters) {
  for (;;) {
    int lectura = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred()) {
      // Proteger acceso al arreglo y al índice
      if (xSemaphoreTake(sensorMutex, (TickType_t) 10) == pdTRUE) {
        readings[readingIndex] = lectura;
        readingIndex++;
        xSemaphoreGive(sensorMutex);
        
        Serial.print("Lectura: ");
        Serial.println(lectura);
        
        // Cuando se acumulen al menos NUM_READINGS lecturas, calcular la mediana
        if (readingIndex >= NUM_READINGS) {
          if (xSemaphoreTake(sensorMutex, (TickType_t) 10) == pdTRUE) {
            int med = calcularMediana((int*)readings, NUM_READINGS);
            filteredValue = med;          // Se asigna la nueva mediana
            newDataAvailable = true;        // Se activa la bandera para publicar
            Serial.print("Nueva mediana calculada: ");
            Serial.println(filteredValue);
            readingIndex = 0;  // Reiniciar el contador
            xSemaphoreGive(sensorMutex);
          }
        }
      }
    } else {
      Serial.println("Sensor: TIMEOUT");
    }
    // Espera 30 ms para la siguiente lectura
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

// Tarea para manejo de MQTT (núcleo 1)
void mqttTask(void *pvParameters) {
  unsigned long lastPublishTime = millis();
  for (;;) {
    // Procesamos la comunicación MQTT
    client.loop();
    
    // Cada 500 ms revisamos si hay dato pendiente para publicar
    if (millis() - lastPublishTime >= 500) {
      lastPublishTime = millis();
      
      if (newDataAvailable) {
        // Si no está conectado, intenta reconectar usando un clientId único
        if (!client.connected()) {
          if (!reconectarMQTT()) {
            Serial.println("No se pudo reconectar a MQTT.");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
          }
        }
        
        // Construimos el mensaje JSON con el valor filtrado
        StaticJsonDocument<200> doc;
        char jsonBuffer[512];
        doc["value"] = filteredValue;
        serializeJson(doc, jsonBuffer);
        
        Serial.print("Publicando valor: ");
        Serial.println(jsonBuffer);
        
        // Intentamos publicar
        if (client.publish(mqtt_topic, jsonBuffer)) {
          Serial.print("Datos enviados: ");
          Serial.println(jsonBuffer);
          lastPublishedValue = filteredValue;
          newDataAvailable = false;
        } else {
          Serial.println("Error al enviar datos, MQTT desconectado. Se reintentará...");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Función para conectar a la red WiFi
void conectarWiFi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("WiFi conectado.");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Función para reconectar el cliente MQTT (sin sesión persistente, usando client id único)
bool reconectarMQTT() {
  String clientId = generarClientId();
  Serial.print("Intentando reconectar a MQTT con ClientID: ");
  Serial.println(clientId);
  if (client.connect(clientId.c_str())) { // Conexión simple, clean session
    Serial.println("Conectado al broker MQTT");
    client.subscribe(mqtt_topic);
    return true;
  } else {
    Serial.print("Fallo la conexión, rc=");
    Serial.println(client.state());
    return false;
  }
}

// Función para reiniciar el sensor
void reinitSensor() {
  sensor.stopContinuous();
  delay(100);
  if (sensor.init()) {
    sensor.startContinuous(30);
    Serial.println("Sensor reiniciado con éxito.");
  } else {
    Serial.println("Error al reiniciar el sensor.");
  }
}
