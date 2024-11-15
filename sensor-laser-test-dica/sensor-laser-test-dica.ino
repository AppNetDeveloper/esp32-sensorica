#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Instancia para el sensor de distancia
VL53L1X sensor;

// Configuración de red WiFi
const char* ssid = "WALMACEN";                  // Nombre de la red WiFi
const char* password = "Dic20942323";   // Contraseña de la red WiFi

// Configuración de MQTT
const char* mqtt_server = "152.53.18.231";  // IP del servidor MQTT
const char* mqtt_topic = "sensor/distance"; // Tópico MQTT donde se publicarán los datos
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMillis = 0;  // Para controlar la frecuencia de las lecturas y publicaciones
int distance = 0;  // Variable para almacenar la distancia leída

void setup()
{
  Serial.begin(115200);

  // Conectar a la red WiFi
  conectarWiFi();

  // Configurar el cliente MQTT
  client.setServer(mqtt_server, 1883);

  // Inicializar el sensor
  Wire.begin(); // Inicializar I2C en pines por defecto
  sensor.setTimeout(500);
  if (sensor.init()) {
    sensor.startContinuous(100); // Iniciar medición continua con un periodo de 100 ms
    Serial.println("Sensor conectado.");
  } else {
    Serial.println("Sensor: No se ha detectado.");
  }

  // Crear una tarea para la lectura del sensor en Core 1
  xTaskCreatePinnedToCore(
    leerSensorTask,   // Función que se ejecutará en el segundo núcleo
    "Leer Sensor",    // Nombre de la tarea
    10000,            // Tamaño del stack en bytes
    NULL,             // Parámetro para pasar a la tarea
    1,                // Prioridad de la tarea
    NULL,             // No necesitamos manejar la tarea
    1                 // Ejecutar en Core 1
  );
}

void loop()
{
  // Asegurarse de que el cliente MQTT está conectado
  if (!client.connected()) {
    reconectarMQTT();
  }
  client.loop();

  // Controlar la frecuencia de envío de datos
  if (millis() - lastMillis >= 500) {  // Enviar cada 500 ms
    enviarDatosMQTT();
    lastMillis = millis();
  }
}

// Función para conectar a la red WiFi
void conectarWiFi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);

  // Iniciar conexión WiFi
  WiFi.begin(ssid, password);

  // Esperar hasta que se conecte
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando...");
  }

  Serial.println("Conexión WiFi exitosa");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

// Función para reconectar el cliente MQTT si se desconecta
void reconectarMQTT() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT... ");
    if (client.connect("ESP32Client")) {  // Cambia el nombre del cliente si lo deseas
      Serial.println("Conectado al broker MQTT");
      client.subscribe(mqtt_topic);  // Puedes suscribirte a otros tópicos si es necesario
    } else {
      Serial.print("Falló la conexión, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos...");
      delay(5000);
    }
  }
}

// Tarea que se ejecuta en el Core 1 para leer el sensor
void leerSensorTask(void *pvParameters) {
  while (true) {
    int nuevaDistancia = sensor.readRangeContinuousMillimeters();
    if (!sensor.timeoutOccurred()) {
      distance = nuevaDistancia;  // Actualizamos la variable global
      Serial.print("Distancia leída: ");
      Serial.print(distance);
      Serial.println(" mm");
    } else {
      Serial.println("Sensor: TIMEOUT");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Esperar 100 ms antes de la siguiente lectura
  }
}

// Función para enviar los datos por MQTT en formato JSON
void enviarDatosMQTT() {
  StaticJsonDocument<200> jsonDoc;
  char jsonBuffer[512];

  // Agregar el valor de la distancia al JSON
  jsonDoc["value"] = distance;

  // Convertir el JSON a una cadena (string)
  serializeJson(jsonDoc, jsonBuffer);

  // Publicar el mensaje en el tópico MQTT
  client.publish(mqtt_topic, jsonBuffer);

  // Mostrar en el monitor serial lo que se ha enviado
  Serial.print("Datos enviados por MQTT: ");
  Serial.println(jsonBuffer);
}
