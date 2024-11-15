
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
const char* mqtt_server = "152.53.18.231";
const int mqtt_port = 1883;
const char* mqtt_topic = "dicaproduct/sensorica/flujostato/token/635354772v2";

// Variables para compartir entre los núcleos
float currentMeasured = 0.0;
bool dataReady = false;
SemaphoreHandle_t xSemaphore;

// Funciones en Core 1 (Tarea de medición)
void TaskMeasure(void *pvParameters) {
    for (;;) {
        float current = leerCorrientePromedio(0, 10);

        Serial.print("Corriente medida (sin calibrar): ");
        Serial.println(current);

        // Proteger acceso compartido a variables
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        currentMeasured = current;
        dataReady = true;
        xSemaphoreGive(xSemaphore);

        delay(900); // Medir cada segundo
    }
}

// Funciones en Core 0 (Tarea de envío MQTT)
void TaskMQTT(void *pvParameters) {
    for (;;) {
        if (!client.connected()) {
            reconnect();
        }
        client.loop();

        // Verificar si los datos están listos para ser enviados
        xSemaphoreTake(xSemaphore, portMAX_DELAY);
        if (dataReady) {
            enviarMQTT(currentMeasured);
            dataReady = false;
        }
        xSemaphoreGive(xSemaphore);

        delay(100); // Revisar cada 100ms
    }
}

void setup() {
    Serial.begin(115200);
    ads.begin();
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);

    // Crear el semáforo para sincronización de núcleos
    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    // Iniciar las tareas en los diferentes núcleos
    xTaskCreatePinnedToCore(
        TaskMeasure,    // Función de la tarea
        "TaskMeasure",  // Nombre de la tarea
        4096,           // Tamaño del stack
        NULL,           // Parámetro de entrada
        1,              // Prioridad de la tarea
        NULL,           // Handle de la tarea
        1);             // Ejecutar en Core 1

    xTaskCreatePinnedToCore(
        TaskMQTT,       // Función de la tarea
        "TaskMQTT",     // Nombre de la tarea
        4096,           // Tamaño del stack
        NULL,           // Parámetro de entrada
        1,              // Prioridad de la tarea
        NULL,           // Handle de la tarea
        0);             // Ejecutar en Core 0
}

// Definir loop() vacío para evitar errores de compilación
void loop() {
    // No hacemos nada aquí porque estamos usando tareas de FreeRTOS
}

// Función para leer la corriente promedio con anti-ruido
float leerCorrientePromedio(int channel, int samples) {
    int32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += ads.readADC_SingleEnded(channel);
        delay(1); // Pequeño retraso entre lecturas.
    }

    // Configura la ganancia para un rango de ±6.144V
    ads.setGain(GAIN_TWOTHIRDS);

    // Convertir la lectura cruda del ADC a voltaje
    float voltage = (sum / (float)samples) * (6.144 / 32768.0);
    Serial.print("Voltaje calculado: ");
    Serial.println(voltage);

    // Convertir el voltaje a corriente en mA (asumiendo que 0V = 4mA y 5V = 20mA)
    float current = (voltage / 5.0) * 16.0;
    Serial.print("Corriente calculada (mA): ");
    Serial.println(current);

    return current; // Devolver la corriente en mA
}


// Función para enviar los datos a través de MQTT
void enviarMQTT(float current) {
    StaticJsonDocument<100> doc;
    doc["value_ma"] = current;

    char msg[100];
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
    while (!client.connected()) {
        Serial.print("Conectando al servidor MQTT...");

        // Obtener la dirección MAC del ESP32 y convertirla a un ID único
        String clientId = "ESP32Client-";
        clientId += String(WiFi.macAddress());

        if (client.connect(clientId.c_str())) {
            Serial.println("Conectado");
        } else {
            Serial.print("Falló, rc=");
            Serial.print(client.state());
            Serial.println(" reintentando en 5 segundos");
            delay(5000);
        }
    }
}
