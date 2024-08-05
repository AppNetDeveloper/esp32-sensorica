#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <ESP8266mDNS.h> 

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

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80); 
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
String lastMessage; // Para almacenar el último mensaje enviado

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
  data[maxSize - 1] = '\0'; // Asegurarse de que esté terminado en null
}

void saveIntToEEPROM(int addr, int value) {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(addr, value & 0xFF);
  EEPROM.write(addr + 1, (value >> 8) & 0xFF);
  EEPROM.commit();
}

int loadIntFromEEPROM(int addr) {
  EEPROM.begin(EEPROM_SIZE);
  int value = EEPROM.read(addr) | (EEPROM.read(addr + 1) << 8);
  return value;
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a WiFi: ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConexión WiFi establecida!");  
  Serial.print("Dirección IP:\t");
  Serial.println(WiFi.localIP()); 
  
  // Configuración del nombre Bluetooth
  String btName = "ESP-" + WiFi.localIP().toString();
  WiFi.hostname(btName); 
  if (MDNS.begin(btName.c_str())) {
    Serial.println("Servicio mDNS iniciado. Nombre Bluetooth: " + btName);
  } else {
    Serial.println("Error al iniciar el servicio mDNS!");
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado a MQTT");
      
      // Si había un último mensaje guardado, enviarlo
      if (lastMessage.length() > 0) {
        client.publish(mqtt_topic, lastMessage.c_str());
        Serial.println("Último mensaje enviado: " + lastMessage);
        lastMessage = ""; // Limpiar el último mensaje después de enviarlo
      }
    } else {
      Serial.print("Error al conectar a MQTT, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 1 segundo");
      delay(1000); // Espera de 1 segundo antes de intentar de nuevo
    }
  }
}

void setup() {
  Serial.begin(115200); // Iniciar comunicación serial para depuración

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
  
  // Cargar valores de configuración desde la EEPROM
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

  setup_wifi(); // Conectar a WiFi

  // Cargar valores por defecto si no están definidos
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

  client.setServer(mqtt_server, mqtt_port); // Configurar el servidor MQTT

  // Inicialización del sensor VL53L1X
  Wire.begin(SDA_PIN, SCL_PIN);  // Iniciar comunicación I2C
  if (!sensor.init()) {
    Serial.println("Error al inicializar el sensor!");
    sensorConnected = false;    // Marcar el sensor como desconectado si falla la inicialización
  } else {
    sensor.startContinuous(10); // Iniciar mediciones continuas cada 10 ms
    Serial.println("Sensor VL53L1X inicializado y en funcionamiento.");
  }

  // Configuración del servidor web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<100> doc; // Crear un documento JSON con tamaño 100 bytes
    if (sensorConnected) {
      doc["value"] = distance;  // Agregar la distancia al JSON si el sensor está conectado
    } else {
      doc["value"] = "Connection lost"; // Agregar mensaje de error si el sensor está desconectado
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
}

void loop() {
  if (!client.connected()) {
    reconnect(); // Reconectar si se ha perdido la conexión
  }
  client.loop(); // Mantener la conexión activa

  StaticJsonDocument<100> doc;
  String output;

  if (sensorConnected) {
    distance = sensor.read();  // Leer la distancia si el sensor está conectado
    doc["value"] = distance;   // Crear JSON con la distancia
  } else {
    doc["value"] = "Connection lost"; // Crear JSON indicando pérdida de conexión
  }

  serializeJson(doc, output);
  
  if (client.connected()) {
    client.publish(mqtt_topic, output.c_str()); // Publicar el JSON al tópico MQTT
    Serial.println("Mensaje publicado: " + output);
  } else {
    lastMessage = output; // Guardar el último mensaje para enviarlo cuando se reconecte
    Serial.println("No conectado a MQTT. Mensaje guardado para envío posterior: " + output);
  }

  delay(500); // Publicar cada 0.5 segundos
}
