#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>

#define DEFAULT_SSID "Lss"         
#define DEFAULT_PASSWORD "cvlss2101281613" 

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

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80); 
DNSServer dnsServer;
VL53L1X sensor;           
#define SDA_PIN 4 
#define SCL_PIN 5 
volatile uint16_t distance = 0; 
bool sensorConnected = true;  

char mqtt_topic[EEPROM_TOPIC_SIZE];
char mqtt_server[16];
int mqtt_port;
char wifi_ssid[EEPROM_WIFI_SSID_SIZE];
char wifi_password[EEPROM_WIFI_PASS_SIZE];
String lastMessage;

bool setupMode = false;
unsigned long buttonPressStartTime = 0;

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
    WiFi.hostname(btName); 
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
    if (client.connect("ESP8266Client")) {
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

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  

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
    sprintf(mqtt_topic, "dicaproduct/sensorica/%08X", ESP.getChipId());
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

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!sensor.init()) {
    Serial.println("Error al inicializar el sensor!");
    sensorConnected = false;
  } else {
    sensor.startContinuous(10);
    Serial.println("Sensor VL53L1X inicializado y en funcionamiento.");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<100> doc;
    if (sensorConnected) {
      doc["value"] = distance;
    } else {
      doc["value"] = "Connection lost";
    }
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });

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

  if (setupMode) {
    dnsServer.start(53, "*", WiFi.softAPIP());
    server.onNotFound([](AsyncWebServerRequest *request){
      request->redirect("/setup");
    });
  }

  server.begin();
  Serial.println("Servidor HTTP iniciado");
}

void loop() {
  checkButton();
  
  if (setupMode) {
    dnsServer.processNextRequest();
  } else {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    if (sensorConnected) {
      distance = sensor.read();
      StaticJsonDocument<100> doc;
      doc["value"] = distance;
      String output;
      serializeJson(doc, output);
      
      if (client.connected()) {
        client.publish(mqtt_topic, output.c_str());
        Serial.println("Mensaje publicado: " + output);
      } else {
        lastMessage = output;
        Serial.println("No conectado a MQTT. Mensaje guardado para envío posterior: " + output);
      }
    }
  }

  delay(600);
}
