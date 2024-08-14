#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Configuración del WiFi
const char* ssid = "AKOBOX";
const char* password = "12345678";

// Configuración del servidor MQTT
const char* mqtt_server = "152.53.18.231";
const int mqtt_port = 1883;
const char* mqtt_topic = "dicaproduct/sensorica/flujostato/token/635354772";

// Crear objeto WiFi y MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Crear objeto para el ADS1115
Adafruit_ADS1115 ads;

// Configuración del rango de medición del ADS1115
const float max_voltage = 5.0; // Voltaje máximo de entrada al ADS1115
const float ads_max_value = 32767.0; // Valor máximo que puede leer el ADS1115 (16 bits)

// Valores de calibración (a ajustar según el manual o calibración)
const float minFlow = 0;        // Flujo mínimo en L/min (podría ser diferente de 0 si hay zero-cut) Volver a poner 0 si te da algun fallo ....... en manual pone por debajo de 90 es 0
const float maxFlow = 12000.0;     // Flujo máximo en L/min (a ajustar según rango libre)
const float minVoltage = 1.0;      // Voltaje correspondiente al flujo mínimo (usualmente 1V)
const float maxVoltage = 5.0;      // Voltaje correspondiente al flujo máximo (a ajustar según rango libre)
const float accuracy = 0.03;       // Precisión de la salida analógica (±3.0%F.S.)

void setup() {
  Serial.begin(115200);

  // Iniciar comunicación con ADS1115
  if (!ads.begin()) {
    Serial.println("¡No se encontró el ADS1115!");
    while (1);
  }

  // Configurar la ganancia (ajusta según tu necesidad)
  ads.setGain(GAIN_ONE); // +/- 4.096V (1 bit = 0.125mV)

  // Conectar a la red Wi-Fi
  setup_wifi();

  // Configurar el servidor MQTT
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  // Reconnectar si se desconecta del servidor MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Promediar múltiples lecturas del flujostato para mayor estabilidad
  int16_t adc0_sum = 0;
  int samples = 30;

  for (int i = 0; i < samples; i++) {
    adc0_sum += ads.readADC_SingleEnded(0);
    delay(3); // Pequeño retraso entre lecturas.
  }

  int16_t adc0 = adc0_sum / samples;

  // Convertir el valor ADC a voltaje
  float voltaje = adc0 * (max_voltage / ads_max_value);

  // Calcular el flujo en L/min, considerando la precisión
  float flujo = calcularFlujo(voltaje);
  float error = flujo * accuracy; // Calcula el posible error en la lectura

  // Crear un documento JSON con tamaño especificado
  StaticJsonDocument<200> doc;  // 200 bytes capacity
  doc["Valor_ADC"] = adc0;
  doc["Voltaje"] = voltaje;
  doc["Flujo_L_min"] = flujo;
  doc["Error"] = error;
  doc["value"] = flujo;

  // Convertir el documento JSON a una cadena de caracteres
  char msg[150];
  serializeJson(doc, msg);

  // Publicar el valor en MQTT
  Serial.print("Enviando mensaje: ");
  Serial.println(msg);
  client.publish(mqtt_topic, msg);

  delay(900); // Enviar cada segundo, real lo he puesto a 900 mini segundos para que sea real 1 sec
}

float calcularFlujo(float voltaje) {
  // Asegurarse que el voltaje esté dentro del rango válido
  voltaje = constrain(voltaje, minVoltage, maxVoltage);

  // Utilizar los datos de calibración para convertir el voltaje a tasa de flujo
  return (voltaje - minVoltage) * (maxFlow - minFlow) / (maxVoltage - minVoltage) + minFlow;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  // Hacer un casting explícito de 'ssid' a 'char*'
  WiFi.begin((char*)ssid, (char*)password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");

  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop hasta que esté conectado
  while (!client.connected()) {
    Serial.print("Conectando al servidor MQTT...");

    // Cambia el "ESP32Client" a un char array para evitar el error de conversión
    char clientID[] = "ESP32Client";

    if (client.connect(clientID)) {
      Serial.println("Conectado");
    } else {
      Serial.print("Falló, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5 segundos");

      delay(5000);
    }
  }
}
