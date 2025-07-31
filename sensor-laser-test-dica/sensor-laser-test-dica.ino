#include <ETH.h>
#include <WiFi.h> // Se necesita para definir WiFiClient y el sistema de eventos, aunque no usemos el hardware WiFi.
#include <PubSubClient.h>
#include <ArduinoJson.h>

// -- INICIO: NUEVA SECCIÓN DE CONFIGURACIÓN DEL SENSOR --

// Pines para el sensor de ultrasonido impermeable JSN-SR04T
#define TRIG_PIN 15
#define ECHO_PIN 14

// -- FIN: NUEVA SECCIÓN DE CONFIGURACIÓN DEL SENSOR --


// -- INICIO: SECCIÓN DE CONFIGURACIÓN ETHERNET PARA WT32-ETH01 --

// NUEVO: Configuración para IP Estática. ¡DEBES CAMBIAR ESTOS VALORES!
IPAddress staticIP(192, 168, 1, 124); // La IP que quieres para tu ESP32
IPAddress gateway(192, 168, 1, 1);    // La IP de tu router
IPAddress subnet(255, 255, 255, 0);  // La máscara de subred de tu red
IPAddress dns1(8, 8, 8, 8);            // Servidor DNS primario (Google)
IPAddress dns2(8, 8, 4, 4);            // Servidor DNS secundario (Google)

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
const char* mqtt_server = "192.168.1.29";
const char* mqtt_topic = "sensor/distance/corte/1"; //ojo se cambia topico

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

unsigned long startupTimestamp = 0; // Para el watchdog de conexión Ethernet
unsigned long lastSensorRestart = 0;
const unsigned long sensorRestartInterval = 1800000;

// Prototipos de tareas y funciones
void WiFiEvent(WiFiEvent_t event);
bool reconectarMQTT();
void sensorTask(void *pvParameters);
void mqttTask(void *pvParameters);
void reiniciarESP32();


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
  
  Serial.println("Configurando conexión Ethernet con IP estática...");
  WiFi.onEvent(WiFiEvent); 
  
  // NUEVO: Aplicamos la configuración de IP estática ANTES de iniciar la conexión.
  if (!ETH.config(staticIP, gateway, subnet, dns1, dns2)) {
    Serial.println("Error al configurar la IP estática para Ethernet.");
  }
  
  Serial.println("Intentando conectar a la red por RJ45...");
  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_MDC, ETH_PHY_MDIO, ETH_PHY_POWER, ETH_CLK_MODE);
  
  // -- FIN: INICIALIZACIÓN DE RED --

  client.setServer(mqtt_server, 1883);

  Serial.println("Sensor de ultrasonido JSN-SR04T configurado.");
  Serial.println("------------------------------------");

  startupTimestamp = millis(); // Guardamos el momento del arranque para el watchdog
  lastSensorRestart = millis();

  // Creación de tareas (sin cambios)
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 10000, NULL, 1, NULL, 1);
}

void loop() {
  // Watchdog para la conexión Ethernet. Si en 5 minutos no hay IP, reinicia.
  if (!eth_connected && startupTimestamp > 0 && (millis() - startupTimestamp > 300000)) { // 300000 ms = 5 minutos
      Serial.println("WATCHDOG: No se pudo obtener conexión Ethernet en 5 minutos. Reiniciando...");
      reiniciarESP32();
  }

  // Mantenemos el reinicio periódico general como medida de seguridad adicional
  if (millis() - lastSensorRestart >= sensorRestartInterval) { // 30 minutos
    Serial.println("WATCHDOG: Reinicio programado por tiempo (30 min).");
    reiniciarESP32();
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
    duration = pulseIn(ECHO_PIN, HIGH, 1000000); // Timeout de 1 segundo

    // Calcular la distancia en milímetros
    distance = duration * 0.343 / 2;
    
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
          readingIndex = 0;
          xSemaphoreGive(sensorMutex);
        }
      }
    }
    
    // Espera 60 ms para la siguiente lectura para asegurar estabilidad del sensor
    vTaskDelay(60 / portTICK_PERIOD_MS);
  }
}

// Tarea MQTT con lógica de reinicio por watchdog
void mqttTask(void *pvParameters) {
  unsigned long lastPublishTime = 0;
  unsigned long nextReconnectAttempt = 0;
  long reconnectInterval = 5000; // Intervalo inicial de 5 segundos
  const long maxReconnectInterval = 300000; // Intervalo máximo de 5 minutos

  for (;;) {
    // Primero, nos aseguramos de tener conexión a la red local.
    if (!eth_connected) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      continue;
    }

    // Si no estamos conectados al broker MQTT, gestionamos la reconexión.
    if (!client.connected()) {
      // Solo intentamos reconectar si ha pasado el tiempo de espera.
      if (millis() > nextReconnectAttempt) {
        Serial.println("MQTT > Conexión perdida. Intentando reconectar...");
        if (reconectarMQTT()) {
          // Éxito. Reiniciamos el intervalo para la próxima vez.
          reconnectInterval = 5000;
          nextReconnectAttempt = 0;
        } else {
          // Si el último intento fue con el intervalo máximo, reiniciamos.
          if (reconnectInterval >= maxReconnectInterval) {
            Serial.println("WATCHDOG: No se pudo reconectar a MQTT después de 5 minutos. Reiniciando...");
            reiniciarESP32();
          }

          // Fallo. Programamos el próximo intento con un intervalo mayor.
          nextReconnectAttempt = millis() + reconnectInterval;
          // Doblamos el intervalo para el siguiente intento (backoff)
          reconnectInterval *= 2;
          if (reconnectInterval > maxReconnectInterval) {
            reconnectInterval = maxReconnectInterval; // Limitamos el intervalo máximo
          }
          Serial.print("MQTT > El intento de reconexión falló. Próximo intento en ");
          Serial.print(reconnectInterval / 1000);
          Serial.println(" segundos.");
        }
      }
    } else {
      // Si estamos conectados, mantenemos la conexión activa.
      client.loop();
    }
    
    // La lógica de publicación solo se ejecuta si estamos conectados.
    if (client.connected() && millis() - lastPublishTime >= 500) {
      lastPublishTime = millis();
      
      if (newDataAvailable) {
        StaticJsonDocument<200> doc;
        char jsonBuffer[512];
        doc["value"] = filteredValue;
        serializeJson(doc, jsonBuffer);
        
        Serial.print("MQTT > Publicando valor: ");
        Serial.println(jsonBuffer);
        
        if (!client.publish(mqtt_topic, jsonBuffer)) {
          Serial.println("MQTT > Error al publicar. La conexión puede haberse perdido.");
        }
        newDataAvailable = false;
      }
    }
    
    // Pequeña pausa para no saturar el núcleo
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


// Función para reconectar a MQTT (ahora solo hace un intento)
bool reconectarMQTT() {
  String clientId = generarClientId();
  Serial.print("MQTT > Intentando conectar con ClientID: ");
  Serial.println(clientId);
  return client.connect(clientId.c_str());
}


void reiniciarESP32() {
  Serial.println("Reiniciando ESP32...");
  delay(1000);
  ESP.restart();
}

