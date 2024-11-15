#include <Wire.h>
#include <VL53L1X.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Instancia para el sensor de distancia
VL53L1X sensor;

// Configuración de red WiFi
const char* ssid = "Lss";                  // Nombre de la red WiFi
const char* password = "cvlss2101281613";   // Contraseña de la red WiFi

// Configuración de MQTT
const char* mqtt_server = "152.53.18.231";  // IP del servidor MQTT
const char* mqtt_topic = "sensor/distance"; // Tópico MQTT donde se publicarán los datos
WiFiClient espClient;
PubSubClient client(espClient);

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
    sensor.startContinuous(100); // Iniciar medición continua
    Serial.println("Sensor conectado.");
  } else {
    Serial.println("Sensor: No se ha detectado.");
  }
}

void loop()
{
  // Asegurarse de que el cliente MQTT está conectado
  if (!client.connected()) {
    reconectarMQTT();
  }
  client.loop();

  // Leer distancia del sensor y enviarla por MQTT
  leerSensorYEnviar();
  
  // Esperar antes de la siguiente medición
  delay(100);
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

// Función para leer el sensor y enviar los datos por MQTT en formato JSON
void leerSensorYEnviar() {
  StaticJsonDocument<200> jsonDoc;
  char jsonBuffer[512];

  // Leer el sensor
  int distance = sensor.readRangeContinuousMillimeters();
  
  if (!sensor.timeoutOccurred()) {
    // Agregar el valor al JSON
    jsonDoc["value"] = distance;
    Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.println(" mm");
  } else {
    // Si ocurre un timeout
    jsonDoc["value"] = "Timeout";
    Serial.println("Sensor: TIMEOUT");
  }

  // Convertir el JSON a una cadena (string)
  serializeJson(jsonDoc, jsonBuffer);

  // Publicar el mensaje en el tópico MQTT
  client.publish(mqtt_topic, jsonBuffer);

  // Mostrar en el monitor serial lo que se ha enviado
  Serial.print("Datos enviados por MQTT: ");
  Serial.println(jsonBuffer);
}
