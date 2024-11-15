#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <DNSServer.h>

#define DEFAULT_SSID "WALMACEN"
#define DEFAULT_PASSWORD "Dic20942323"

#define EEPROM_SIZE 512
#define EEPROM_TOPIC_ADDR 0
#define EEPROM_TOPIC_SIZE 100
#define EEPROM_MQTT_IP_ADDR (EEPROM_TOPIC_ADDR + 100)
#define EEPROM_MQTT_PORT_ADDR (EEPROM_MQTT_IP_ADDR + 16)
#define EEPROM_WIFI_SSID_ADDR (EEPROM_MQTT_PORT_ADDR + 2)
#define EEPROM_WIFI_SSID_SIZE 32
#define EEPROM_WIFI_PASS_ADDR (EEPROM_WIFI_SSID_ADDR + 32)
#define EEPROM_WIFI_PASS_SIZE 32

#define BUTTON_PIN 12
#define LONG_PRESS_TIME 5000

#define TRIG_PIN 14 // Pin para el Trigger del sensor ultrasónico
#define ECHO_PIN 13 // Pin para el Echo del sensor ultrasónico

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80);
DNSServer dnsServer;
VL53L1X laserSensor;
#define SDA_PIN 4
#define SCL_PIN 5

volatile uint16_t laserDistance = 0;
bool laserSensorConnected = true;

// Variables para el sensor ultrasónico
long ultrasonicDistance = 0;
bool ultrasonicSensorConnected = true;

const int numReadings = 20; // Aumentar el número de lecturas para promediar
long readings[numReadings];  // Arreglo para almacenar las lecturas
int readIndex = 0;           // Índice para la posición actual en el arreglo
long total = 0;              // Total de todas las lecturas
long average = 0;            // Promedio de las lecturas

char mqtt_topic[EEPROM_TOPIC_SIZE];
char mqtt_server[16];
int mqtt_port;
char wifi_ssid[EEPROM_WIFI_SSID_SIZE];
char wifi_password[EEPROM_WIFI_PASS_SIZE];
String lastMessage;

bool setupMode = false;
unsigned long buttonPressStartTime = 0;

SemaphoreHandle_t sensorDataMutex;

// Parámetros de filtrado




void saveStringToEEPROM(int addr, const char* data, int maxSize) {
    EEPROM.begin(EEPROM_SIZE);
    for (int i = 0; i < maxSize; ++i) {
        EEPROM.write(addr + i, (i < strlen(data)) ? data[i] : 0);
    }
    EEPROM.commit();
}

void loadStringFromEEPROM(int addr, char* data, int maxSize) {
    EEPROM.begin(EEPROM_SIZE);
    for (int i = 0; i < maxSize; ++i) {
        data[i] = EEPROM.read(addr + i);
    }
    data[maxSize - 1] = '\0';
}

void saveIntToEEPROM(int addr, int value) {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(addr, value & 0xFF);
    EEPROM.write(addr + 1, (value >> 8) & 0xFF);
    EEPROM.commit();
}

int loadIntFromEEPROM(int addr) {
    EEPROM.begin(EEPROM_SIZE);
    return EEPROM.read(addr) | (EEPROM.read(addr + 1) << 8);
}

void setup_wifi() {
    if (setupMode) {
        WiFi.softAP("ESP-Config", "123456789");
        Serial.println("Modo de configuración AP iniciado");
        Serial.print("IP del AP: ");
        Serial.println(WiFi.softAPIP());
    } else {
        delay(10);
        Serial.println();
        Serial.print("Conectando a WiFi: ");
        Serial.println(wifi_ssid);

        WiFi.begin(wifi_ssid, wifi_password);

        unsigned long startAttemptTime = millis();

        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
            delay(500);
            Serial.print(".");
            checkButton();
            if (setupMode) {
                return;
            }
        }

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nFallo al conectar al WiFi. Entrando en modo de configuración.");
            setupMode = true;
            setup_wifi();
            return;
        }

        Serial.println("\nConexión WiFi establecida!");
        Serial.print("Dirección IP: ");
        Serial.println(WiFi.localIP());

        String btName = "ESP-" + WiFi.localIP().toString();
        WiFi.setHostname(btName.c_str());
        if (MDNS.begin(btName.c_str())) {
            Serial.println("Servicio mDNS iniciado. Nombre Bluetooth: " + btName);
        } else {
            Serial.println("Error al iniciar el servicio mDNS!");
        }
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Conectando a MQTT...");
        if (client.connect("ESP32S3Client")) {
            Serial.println("Conectado a MQTT");
            if (lastMessage.length() > 0) {
                client.publish(mqtt_topic, lastMessage.c_str());
                Serial.println("Último mensaje enviado: " + lastMessage);
                lastMessage = "";
            }
        } else {
            Serial.print("Error al conectar a MQTT, rc=");
            Serial.print(client.state());
            Serial.println(" Intentando de nuevo en 1 segundo");
            delay(1000);
        }
        checkButton();
        if (setupMode) {
            return;
        }
    }
}

void checkButton() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (buttonPressStartTime == 0) {
            buttonPressStartTime = millis();
        } else if ((millis() - buttonPressStartTime) > LONG_PRESS_TIME) {
            setupMode = true;
            Serial.println("Entrando en modo de configuración");
            setup_wifi();
            buttonPressStartTime = 0;
        }
    } else {
        buttonPressStartTime = 0;
    }
}

const long maxSaltoPermitido = 2400;  // Máximo salto permitido entre lecturas consecutivas
const long minAlturaPalet = 200;  // Altura mínima esperada de un pallet
const long maxAlturaPalet = 3100;  // Altura máxima esperada (sin pallet)
long ultimaLecturaUltrasonido = maxAlturaPalet;  // Suponemos que la cinta está inicialmente vacía (3100 mm)

const float factorCalibracion = 1.0;  // Factor de calibración

const long tolerancia = 100;  // Tolerancia para lecturas de variaciones pequeñas
const int lecturasEstablesNecesarias = 5;  // Lecturas fallidas consecutivas necesarias para considerar error
const int numLecturasParaPromedio = 5;  // Número de lecturas consecutivas para el promedio móvil

int lecturasConsecutivasErroneas = 0;  // Contador de lecturas fallidas consecutivas
long lecturas[numLecturasParaPromedio];  // Arreglo para almacenar las lecturas recientes
int indiceLectura = 0;  // Índice actual en el arreglo de lecturas

long readUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout de 30ms

    // Si el sensor no detecta nada, aumentar el contador de lecturas fallidas
    if (duration == 0) {
        lecturasConsecutivasErroneas++;
        if (lecturasConsecutivasErroneas >= lecturasEstablesNecesarias) {
            ultrasonicSensorConnected = false;  // Considerar sensor desconectado tras varias lecturas fallidas
            return -1;  // Indicar error
        } else {
            return calcularPromedio();  // Devolver el promedio de las últimas lecturas válidas
        }
    } else {
        lecturasConsecutivasErroneas = 0;  // Resetear el contador si la lectura es válida
        ultrasonicSensorConnected = true;
    }

    // Calcular la distancia en milímetros
    long distance = duration * 0.34 / 2;

    // Aplicar el factor de calibración
    distance *= factorCalibracion;

    // Limitar la distancia a un rango esperado para pallets
    if (distance < minAlturaPalet || distance > maxAlturaPalet) {
        distance = calcularPromedio();  // Mantener el promedio si está fuera del rango esperado
    }

    // Guardar la lectura en el arreglo de lecturas recientes
    lecturas[indiceLectura] = distance;
    indiceLectura = (indiceLectura + 1) % numLecturasParaPromedio;  // Avanzar el índice circularmente

    // Retornar el promedio suavizado de las lecturas recientes
    return calcularPromedio();
}

long calcularPromedio() {
    long suma = 0;
    for (int i = 0; i < numLecturasParaPromedio; i++) {
        suma += lecturas[i];
    }
    return suma / numLecturasParaPromedio;
}

long filtrarLecturas(long* lecturas, int numLecturas) {
    long suma = 0;
    long promedio = 0;
    int lecturasValidas = 0;

    // Calcular el promedio de todas las lecturas
    for (int i = 0; i < numLecturas; i++) {
        suma += lecturas[i];
    }
    promedio = suma / numLecturas;

    // Volver a sumar solo las lecturas que están dentro de la tolerancia
    suma = 0;
    for (int i = 0; i < numLecturas; i++) {
        if (abs(lecturas[i] - promedio) <= tolerancia) {  // Filtrar valores fuera de la tolerancia
            suma += lecturas[i];
            lecturasValidas++;
        }
    }

    // Si hay lecturas válidas, calcular un nuevo promedio
    if (lecturasValidas > 0) {
        return suma / lecturasValidas;
    } else {
        return promedio;  // Si no hay lecturas válidas, devolver el promedio inicial
    }
}

void measureSensors(void* parameter) {
    while (true) {
        // Leer el sensor ultrasónico si está conectado
        long ultrasonicTemp = readUltrasonicDistance();

        xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
        ultrasonicDistance = ultrasonicTemp;
        xSemaphoreGive(sensorDataMutex);

        // Leer el sensor láser si está conectado
        if (laserSensorConnected) {
            uint16_t laserTemp = laserSensor.read();
            xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
            laserDistance = laserTemp;
            xSemaphoreGive(sensorDataMutex);
        }

        delay(50);  // Reducir el ruido tomando lecturas cada 50ms
    }
}

void mqttPublish(void* parameter) {
    while (true) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!client.connected()) {
                reconnect();
            }
            client.loop();

            StaticJsonDocument<200> jsonDoc;

            // Tomar los datos de los sensores con mutex para evitar conflictos de lectura
            xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
            int laserValue = (laserSensorConnected) ? laserDistance : -1;
            int ultrasonicValue = (ultrasonicSensorConnected) ? ultrasonicDistance : -1;
            xSemaphoreGive(sensorDataMutex);

            // Si ambos sensores devuelven -1, no se publica nada
            if (laserValue == -1 && ultrasonicValue == -1) {
                Serial.println("Ambos sensores desconectados. No se envían datos.");
            } else {
                // Crear el JSON con los valores de los sensores si alguno está conectado
                jsonDoc["medidor-laser"]["value"] = laserValue;
                jsonDoc["medidor-ultrasonido"]["value"] = ultrasonicValue;

                String jsonOutput;
                serializeJson(jsonDoc, jsonOutput);
                client.publish(mqtt_topic, jsonOutput.c_str());
                Serial.println("Datos enviados a MQTT: " + jsonOutput);
            }
        }

        delay(500);  // Publicar los datos cada 500 ms
    }
}


void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    sensorDataMutex = xSemaphoreCreateMutex();

    loadStringFromEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
    if (strlen(wifi_ssid) == 0) {
        strcpy(wifi_ssid, DEFAULT_SSID);
        saveStringToEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
        Serial.println("Configurando WiFi SSID por defecto: " + String(wifi_ssid));
    }

    loadStringFromEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
    if (strlen(wifi_password) == 0) {
        strcpy(wifi_password, DEFAULT_PASSWORD);
        saveStringToEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
        Serial.println("Configurando WiFi Password por defecto: " + String(wifi_password));
    }

    loadStringFromEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
    if (strlen(mqtt_topic) == 0) {
        sprintf(mqtt_topic, "sensorica/%08X", (uint32_t)ESP.getEfuseMac());
        saveStringToEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
        Serial.println("Configurando MQTT Topic por defecto: " + String(mqtt_topic));
    }

    loadStringFromEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
    if (strlen(mqtt_server) == 0) {
        strcpy(mqtt_server, "152.53.18.231");
        saveStringToEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
        Serial.println("Configurando MQTT Server por defecto: " + String(mqtt_server));
    }

    mqtt_port = loadIntFromEEPROM(EEPROM_MQTT_PORT_ADDR);
    if (mqtt_port == 0) {
        mqtt_port = 1883;
        saveIntToEEPROM(EEPROM_MQTT_PORT_ADDR, mqtt_port);
        Serial.println("Configurando MQTT Port por defecto: " + String(mqtt_port));
    }

    setup_wifi();

    client.setServer(mqtt_server, mqtt_port);

    // Inicializar el sensor láser
    Wire.begin(SDA_PIN, SCL_PIN);
    if (!laserSensor.init()) {
        Serial.println("Error al inicializar el sensor láser!");
        laserSensorConnected = false;
    } else {
        laserSensor.startContinuous(10);
        Serial.println("Sensor láser inicializado y en funcionamiento.");
    }

    // Inicializar el sensor ultrasónico
    ultrasonicSensorConnected = true; // Inicializar como conectado para comenzar la lectura
    Serial.println("Iniciando el sensor ultrasónico...");

    // Configuración del servidor web
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        StaticJsonDocument<200> doc; // Crear un documento JSON con tamaño 200 bytes
        if (laserSensorConnected) {
            doc["medidor-laser"]["value"] = laserDistance;  // Agregar la distancia del láser al JSON si el sensor está conectado
        } else {
            doc["medidor-laser"]["value"] = "Connection lost"; // Agregar mensaje de error si el sensor láser está desconectado
        }

        if (ultrasonicSensorConnected) {
            doc["medidor-ultrasonido"]["value"] = ultrasonicDistance;  // Agregar la distancia del ultrasonido al JSON si el sensor está conectado
        } else {
            doc["medidor-ultrasonido"]["value"] = "Connection lost"; // Agregar mensaje de error si el sensor ultrasonido está desconectado
        }

        String output;
        serializeJson(doc, output); // Convertir el JSON a una cadena de texto
        request->send(200, "application/json", output); // Enviar la respuesta JSON al cliente
    });

    // Página de configuración
    server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<html><body>";
        html += "<form action=\"/setup\" method=\"post\">";
        html += "WiFi SSID: <input type=\"text\" name=\"ssid\" value=\"" + String(wifi_ssid) + "\"><br>";
        html += "WiFi Password: <input type=\"text\" name=\"password\" value=\"" + String(wifi_password) + "\"><br>";
        html += "MQTT Topic: <input type=\"text\" name=\"topic\" value=\"" + String(mqtt_topic) + "\"><br>";
        html += "MQTT Server: <input type=\"text\" name=\"server\" value=\"" + String(mqtt_server) + "\"><br>";
        html += "MQTT Port: <input type=\"text\" name=\"port\" value=\"" + String(mqtt_port) + "\"><br>";
        html += "<input type=\"submit\" value=\"Save\">";
        html += "</form>";
        html += "</body></html>";
        request->send(200, "text/html", html);
    });

    server.on("/setup", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("ssid", true)) {
            String newSSID = request->getParam("ssid", true)->value();
            newSSID.toCharArray(wifi_ssid, EEPROM_WIFI_SSID_SIZE);
            saveStringToEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
            Serial.println("Nuevo WiFi SSID guardado: " + String(wifi_ssid));
        }
        if (request->hasParam("password", true)) {
            String newPassword = request->getParam("password", true)->value();
            newPassword.toCharArray(wifi_password, EEPROM_WIFI_PASS_SIZE);
            saveStringToEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
            Serial.println("Nuevo WiFi Password guardado: " + String(wifi_password));
        }
        if (request->hasParam("topic", true)) {
            String newTopic = request->getParam("topic", true)->value();
            newTopic.toCharArray(mqtt_topic, EEPROM_TOPIC_SIZE);
            saveStringToEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
            Serial.println("Nuevo MQTT Topic guardado: " + String(mqtt_topic));
        }
        if (request->hasParam("server", true)) {
            String newServer = request->getParam("server", true)->value();
            newServer.toCharArray(mqtt_server, 16);
            saveStringToEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
            Serial.println("Nuevo MQTT Server guardado: " + String(mqtt_server));
        }
        if (request->hasParam("port", true)) {
            String newPortStr = request->getParam("port", true)->value();
            mqtt_port = newPortStr.toInt();
            saveIntToEEPROM(EEPROM_MQTT_PORT_ADDR, mqtt_port);
            Serial.println("Nuevo MQTT Port guardado: " + String(mqtt_port));
        }
        request->send(200, "text/html", "<html><body>Configuration saved. <a href=\"/\">Return to main page</a></body></html>");
        ESP.restart();
    });

    server.begin();
    Serial.println("Servidor HTTP iniciado");

    // Iniciar tareas en diferentes núcleos
    xTaskCreatePinnedToCore(measureSensors, "MeasureSensors", 4096, NULL, 1, NULL, 0); // Núcleo 0
    xTaskCreatePinnedToCore(mqttPublish, "MQTTPublish", 4096, NULL, 1, NULL, 1); // Núcleo 1
}

void loop() {
    checkButton();  // Este loop sigue manejando el botón en el núcleo 1
}
