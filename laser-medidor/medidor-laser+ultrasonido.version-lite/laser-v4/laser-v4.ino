#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>

#define DEFAULT_SSID "WALMACEN"
#define DEFAULT_PASSWORD "Dic20942323"

#define EEPROM_SIZE 512
#define EEPROM_TOPIC_ADDR 0
#define EEPROM_TOPIC_SIZE 100
#define EEPROM_MQTT_IP_ADDR (EEPROM_TOPIC_ADDR + EEPROM_TOPIC_SIZE)
#define EEPROM_MQTT_PORT_ADDR (EEPROM_MQTT_IP_ADDR + 16)
#define EEPROM_WIFI_SSID_ADDR (EEPROM_MQTT_PORT_ADDR + 2)
#define EEPROM_WIFI_SSID_SIZE 32
#define EEPROM_WIFI_PASS_ADDR (EEPROM_WIFI_SSID_ADDR + EEPROM_WIFI_SSID_SIZE)
#define EEPROM_WIFI_PASS_SIZE 32

#define BUTTON_PIN 14
#define LONG_PRESS_TIME 5000

#define TRIG_PIN 12 // Pin para el Trigger del sensor ultrasónico
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

char mqtt_topic[EEPROM_TOPIC_SIZE];
char mqtt_server[16];
int mqtt_port;
char wifi_ssid[EEPROM_WIFI_SSID_SIZE];
char wifi_password[EEPROM_WIFI_PASS_SIZE];
String lastMessage;

bool setupMode = false;
unsigned long buttonPressStartTime = 0;

unsigned long previousMillis = 0; // Variable to store the previous time for sensor measurements
const long sensorInterval = 500; // Interval at which to perform measurements (milliseconds)
unsigned long previousMqttMillis = 0; // Variable to store the previous time for MQTT messages
const long mqttInterval = 500; // Interval at which to send MQTT messages (milliseconds)
bool measureLaser = true;

// Variables para las últimas distancias medidas
long lastLaserDistance = -1; 
long lastUltrasonicDistance = -1;

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
    } else {
        delay(10);
        WiFi.begin(wifi_ssid, wifi_password);

        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
            delay(500);
            checkButton();
            if (setupMode) {
                return;
            }
        }

        if (WiFi.status() != WL_CONNECTED) {
            setupMode = true;
            setup_wifi();
            return;
        }

        String btName = "ESP-" + WiFi.localIP().toString();
        WiFi.hostname(btName);
        MDNS.begin(btName.c_str());
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect("ESP8266Client")) {
            if (lastMessage.length() > 0) {
                client.publish(mqtt_topic, lastMessage.c_str());
                lastMessage = "";
            }
        } else {
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
            startWebServer();
            buttonPressStartTime = 0;
        }
    } else {
        buttonPressStartTime = 0;
    }
}

long readUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.34 / 2; // Convertir a mm
}

void startWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<200> doc;
        doc["medidor-laser"]["value"] = laserDistance;
        doc["medidor-ultrasonido"]["value"] = ultrasonicDistance;
        String output;
        serializeJson(doc, output);
        request->send(200, "application/json", output);
    });

    server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
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

    server.on("/setup", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (request->hasParam("ssid", true)) {
            String newSSID = request->getParam("ssid", true)->value();
            newSSID.toCharArray(wifi_ssid, EEPROM_WIFI_SSID_SIZE);
            saveStringToEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
        }
        if (request->hasParam("password", true)) {
            String newPassword = request->getParam("password", true)->value();
            newPassword.toCharArray(wifi_password, EEPROM_WIFI_PASS_SIZE);
            saveStringToEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
        }
        if (request->hasParam("topic", true)) {
            String newTopic = request->getParam("topic", true)->value();
            newTopic.toCharArray(mqtt_topic, EEPROM_TOPIC_SIZE);
            saveStringToEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
        }
        if (request->hasParam("server", true)) {
            String newServer = request->getParam("server", true)->value();
            newServer.toCharArray(mqtt_server, 16);
            saveStringToEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
        }
        if (request->hasParam("port", true)) {
            String newPortStr = request->getParam("port", true)->value();
            mqtt_port = newPortStr.toInt();
            saveIntToEEPROM(EEPROM_MQTT_PORT_ADDR, mqtt_port);
        }
        request->send(200, "text/html", "<html><body>Configuration saved. The device will restart.</body></html>");
        delay(3000);
        ESP.restart();
    });

    dnsServer.start(53, "*", WiFi.softAPIP());
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->redirect("/setup");
    });

    server.begin();
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

     // Borrar específicamente SSID y contraseña
  EEPROM.begin(EEPROM_SIZE);
  for (int i = EEPROM_WIFI_SSID_ADDR; i < EEPROM_WIFI_SSID_ADDR + EEPROM_WIFI_SSID_SIZE; i++) {
    EEPROM.write(i, 0); 
  }
  for (int i = EEPROM_WIFI_PASS_ADDR; i < EEPROM_WIFI_PASS_ADDR + EEPROM_WIFI_PASS_SIZE; i++) {
    EEPROM.write(i, 0); 
  }
  EEPROM.commit();
  Serial.println("SSID y contraseña borrados.");
  

    loadStringFromEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
    if (strlen(wifi_ssid) == 0) {
        strcpy(wifi_ssid, DEFAULT_SSID);
        saveStringToEEPROM(EEPROM_WIFI_SSID_ADDR, wifi_ssid, EEPROM_WIFI_SSID_SIZE);
    }

    loadStringFromEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
    if (strlen(wifi_password) == 0) {
        strcpy(wifi_password, DEFAULT_PASSWORD);
        saveStringToEEPROM(EEPROM_WIFI_PASS_ADDR, wifi_password, EEPROM_WIFI_PASS_SIZE);
    }

    loadStringFromEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
    if (strlen(mqtt_topic) == 0) {
        sprintf(mqtt_topic, "dicaproduct/sensorica/%08X", ESP.getChipId());
        saveStringToEEPROM(EEPROM_TOPIC_ADDR, mqtt_topic, EEPROM_TOPIC_SIZE);
    }

    loadStringFromEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
    if (strlen(mqtt_server) == 0) {
        strcpy(mqtt_server, "152.53.18.231");
        saveStringToEEPROM(EEPROM_MQTT_IP_ADDR, mqtt_server, 16);
    }

    mqtt_port = loadIntFromEEPROM(EEPROM_MQTT_PORT_ADDR);
    if (mqtt_port == 0) {
        mqtt_port = 1885;
        saveIntToEEPROM(EEPROM_MQTT_PORT_ADDR, mqtt_port);
    }

    setup_wifi();

    client.setServer(mqtt_server, mqtt_port);

    Wire.begin(SDA_PIN, SCL_PIN);
    if (!laserSensor.init()) {
        laserSensorConnected = false;
    } else {
        laserSensor.startContinuous(10);
    }
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= sensorInterval) {
        previousMillis = currentMillis;

        if (measureLaser) {
            if (laserSensorConnected) {
                laserDistance = laserSensor.read();
            } else {
                laserDistance = -1;
            }
            measureLaser = false;
        } else {
            if (ultrasonicSensorConnected) {
                ultrasonicDistance = readUltrasonicDistance();
            } else {
                ultrasonicDistance = -1;
            }
            measureLaser = true;
        }
    }

    if (currentMillis - previousMqttMillis >= mqttInterval) {
        previousMqttMillis = currentMillis;

        // Solo enviar MQTT si hay un cambio significativo
        bool sendMqtt = false;

        if (abs(laserDistance - lastLaserDistance) >= 5 || lastLaserDistance == -1) {
            sendMqtt = true;
            lastLaserDistance = laserDistance; // Actualizar la última distancia del láser
        }

        if (abs(ultrasonicDistance - lastUltrasonicDistance) >= 5 || lastUltrasonicDistance == -1) {
            sendMqtt = true;
            lastUltrasonicDistance = ultrasonicDistance; // Actualizar la última distancia ultrasónica
        }

        if (sendMqtt && WiFi.status() == WL_CONNECTED && client.connected()) {
            StaticJsonDocument<200> doc;
            doc["medidor-laser"]["value"] = laserDistance;
            doc["medidor-ultrasonido"]["value"] = ultrasonicDistance;
            String jsonOutput;
            serializeJson(doc, jsonOutput);
            
            if (client.connected()) {
                client.publish(mqtt_topic, jsonOutput.c_str());
            } else {
                lastMessage = jsonOutput;
            }
        }
    }

    checkButton();

    if (setupMode) {
        dnsServer.processNextRequest();
    } else {
        if (!client.connected()) {
            reconnect();
        }
        client.loop();
    }
}
