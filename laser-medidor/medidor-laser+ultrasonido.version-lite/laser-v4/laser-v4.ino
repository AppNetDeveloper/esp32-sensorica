#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define EEPROM_SIZE 512
#define FORMAT_FLAG_ADDRESS 0

struct ConfigData {
  char ssid[32];
  char password[32];
  char mqtt_server[32];
  int mqtt_port;
  char mqtt_topic[64];
};

ConfigData config;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  delay(100);

  // Leer flag de formato de la EEPROM
  byte formatFlag = EEPROM.read(FORMAT_FLAG_ADDRESS);

  if (formatFlag != 0xAA) {
    // EEPROM vacía o formateada, inicializamos con valores por defecto
    Serial.println("Inicializando configuración por defecto...");

    strcpy(config.ssid, "default_ssid");
    strcpy(config.password, "default_password");
    strcpy(config.mqtt_server, "152.53.18.231");
    config.mqtt_port = 1883;
    strcpy(config.mqtt_topic, "sensorica/meter/token/jhueugdh");

    // Guardar los valores por defecto en la EEPROM
    saveConfig();
  } else {
    // Leer configuración de la EEPROM
    loadConfig();
    Serial.println("Configuración cargada de EEPROM:");
    Serial.printf("SSID: %s\n", config.ssid);
    Serial.printf("MQTT Server: %s\n", config.mqtt_server);
    Serial.printf("MQTT Port: %d\n", config.mqtt_port);
    Serial.printf("MQTT Topic: %s\n", config.mqtt_topic);
  }

  // Conectar a WiFi
  connectToWiFi();
  
  // Configurar MQTT
  client.setServer(config.mqtt_server, config.mqtt_port);
}

void loop() {
  // Asegurarse de que está conectado a WiFi
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }

  // Mantener conexión MQTT
  if (!client.connected()) {
    reconnectMQTT();
  }

  client.loop();
}

// Función para conectar a WiFi
void connectToWiFi() {
  Serial.printf("Conectando a WiFi: %s\n", config.ssid);
  WiFi.begin(config.ssid, config.password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Conectado a WiFi.");
}

// Función para reconectar MQTT
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.printf("Conectando al servidor MQTT: %s\n", config.mqtt_server);
    if (client.connect("ESP8266Client")) {
      Serial.println("Conectado a MQTT.");
    } else {
      Serial.print("Fallo en conexión MQTT, estado=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

// Guardar configuración en EEPROM
void saveConfig() {
  EEPROM.write(FORMAT_FLAG_ADDRESS, 0xAA);  // Indicador de que la EEPROM está inicializada
  EEPROM.put(1, config);
  EEPROM.commit();
  Serial.println("Configuración guardada en EEPROM.");
}

// Cargar configuración desde EEPROM
void loadConfig() {
  EEPROM.get(1, config);
}

// Formatear EEPROM
void resetEEPROM() {
  EEPROM.write(FORMAT_FLAG_ADDRESS, 0x00);  // Resetea la marca de inicialización
  EEPROM.commit();
  Serial.println("EEPROM formateada.");
}

