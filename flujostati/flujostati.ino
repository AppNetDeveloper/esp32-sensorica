#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Declaración de variables y objetos
Adafruit_ADS1115 ads;  // Crear objeto para el ADC
WiFiClient espClient;
PubSubClient client(espClient);

// Configuración del WiFi
const char* ssid = "AKOBOX";
const char* password = "12345678";

// Configuración del servidor MQTT
const char* mqtt_server = "172.26.0.97";
const int mqtt_port = 1883;
const char* mqtt_topic = "dicaproduct/sensorica/flujostato/token/635354772";

// Definir valores mínimos y máximos según las especificaciones del sensor
float minFlow =.0;   // Flujo mínimo en L/min
float maxFlow = 12000.0; // Flujo máximo en L/min
float minCurrent = 4.0; // Corriente mínima en mA según el fabricante
float maxCurrent = 20.0; // Corriente máxima en mA

// Variables para anti-ruido
const int numReadings = 10;
float readings[numReadings];  // Matriz para almacenar las lecturas
int readIndex = 0;
float total = 0.0;
float averageCurrent = 0.0;

// Umbral para el ruido
float noiseThreshold = 0.1; // mA, valor debajo del cual se considera ruido

// Variables de calibración
float currentOffset = 0.40;  // Ajuste de offset para eliminar el ruido (mA)
float currentScale = 1.0;   // Factor de escala

// Configuración inicial
void setup() {
    Serial.begin(115200);
    ads.begin();
    setup_wifi();
    client.setServer(mqtt_server, 1883);

    // Inicializar el filtro de promediado
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0.0;
    }
}

// Bucle principal
void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Leer la corriente promedio en mA con anti-ruido
    float current = leerCorrientePromedio(0, 10);
    
    // Aplicar el umbral para filtrar ruido
    if (current < noiseThreshold) {
        current = 0.0; // Si la corriente es menor que el umbral, considerarla como 0 (ruido)
    }

    // Aplicar la calibración (incluyendo el offset)
    current = (current - currentOffset) * currentScale;

    Serial.print("Corriente medida (calibrada): ");
    Serial.println(current);

    // Calcular flujo basado en la corriente
    float flujo_ma = calcularFlujoDesdeCorriente(current);
    float error_ma = flujo_ma * 0.05;

    // Enviar datos a través de MQTT
    enviarMQTT(current, flujo_ma, error_ma);

    delay(900); // Enviar cada segundo
}

// Función para leer la corriente promedio con anti-ruido
float leerCorrientePromedio(int channel, int samples) {
    int32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += ads.readADC_SingleEnded(channel);
        delay(1); // Pequeño retraso entre lecturas.
    }

    // Convertimos el promedio a corriente en mA
    float voltage = (sum / (float)samples) * (5.0 / 32767.0);
    float current = (voltage / 0.25);

    // Aplicar filtro de media móvil
    total = total - readings[readIndex];
    readings[readIndex] = current;
    total = total + readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    averageCurrent = total / numReadings;

    return averageCurrent;
}

// Función para calcular el flujo basado en la corriente en mA
float calcularFlujoDesdeCorriente(float corriente) {
    if (corriente < minCurrent) {
        return minFlow;
    } else if (corriente > maxCurrent) {
        return maxFlow;
    } else {
        return (corriente - minCurrent) * (maxFlow - minFlow) / (maxCurrent - minCurrent) + minFlow;
    }
}

// Función para enviar los datos a través de MQTT
void enviarMQTT(float current, float flujo_ma, float error_ma) {
    StaticJsonDocument<200> doc;
    doc["Corriente_mA"] = current;
    doc["Flujo_L_min_ma"] = flujo_ma;
    doc["Error_ma"] = error_ma;
    doc["value_ma"] = flujo_ma;

    char msg[150];
    serializeJson(doc, msg);
    client.publish(mqtt_topic, msg);
    Serial.print("Enviando mensaje: ");
    Serial.println(msg);
}

// Función para configurar la conexión Wi-Fi
void setup_wifi() {
    delay(10);
    Serial.println();
    Serial.print("Conectando a ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

// Función para reconectar al servidor MQTT en caso de desconexión
void reconnect() {
    int attempts = 0;
    while (!client.connected() && attempts < 10) {
        Serial.print("Conectando al servidor MQTT...");
        char clientID[] = "ESP32Client";

        if (client.connect(clientID)) {
            Serial.println("Conectado");
            return;
        } else {
            Serial.print("Falló, rc=");
            Serial.print(client.state());
            Serial.println(" reintentando en 5 segundos");

            delay(5000);
            attempts++;
        }
    }

    if (attempts >= 10) {
        Serial.println("No se pudo conectar al servidor MQTT. Reiniciando...");
        ESP.restart();
    }
}