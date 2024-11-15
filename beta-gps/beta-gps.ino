#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TinyGPSPlus.h>

// Configuración de red WiFi
const char* ssid = "Lss";
const char* password = "cvlss2101281613";

// Configuración de MQTT
const char* mqtt_server = "208.76.222.165";
const char* mqtt_topic = "sensor/gps/box/1";
WiFiClient espClient;
PubSubClient client(espClient);

// Configuración del GPS
TinyGPSPlus gps;
#define GPS_BAUD 9600
#define RXD1 16  // GPS RX
#define TXD1 17  // GPS TX

// Configuración del SIM8000L
#define SIM_RX 4
#define SIM_TX 2
#define SIM_RESET 5

// Configuración GPRS
const char* apn = "airtelwap.es";
const char* gprs_user = "wap@wap";
const char* gprs_pass = "wap125";

unsigned long lastMillis = 0;
bool usingGPRS = false;

// Crear objeto Serial para el SIM8000L
HardwareSerial simSerial(2);

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando sistema...");
  
  // Inicializar Serial1 para el GPS
  Serial1.begin(GPS_BAUD, SERIAL_8N1, RXD1, TXD1);
  
  // Inicializar Serial2 para el SIM8000L
  simSerial.begin(9600, SERIAL_8N1, SIM_RX, SIM_TX);
  
  // Configurar pin de reset del SIM8000L
  pinMode(SIM_RESET, OUTPUT);
  digitalWrite(SIM_RESET, HIGH);
  
  // Intentar conexión WiFi
  conectarWiFi();
  
  // Configurar MQTT
  client.setServer(mqtt_server, 1883);
  
  // Inicializar SIM8000L
  inicializarSIM8000L();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconectarMQTT();
    }
    client.loop();
    usingGPRS = false;
  } else if (!usingGPRS) {
    Serial.println("WiFi no disponible, cambiando a GPRS...");
    conectarGPRS();
    usingGPRS = true;
  }

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (millis() - lastMillis >= 5000) {
    if (usingGPRS) {
      enviarDatosGPRS();
    } else {
      enviarDatosMQTT();
    }
    lastMillis = millis();
  }
}

void inicializarSIM8000L() {
  Serial.println("Inicializando SIM8000L...");
  
  digitalWrite(SIM_RESET, LOW);
  delay(1000);
  digitalWrite(SIM_RESET, HIGH);
  delay(2000);
  
  enviarComandoAT("AT", 1000);
  enviarComandoAT("AT+CPIN?", 1000);
  enviarComandoAT("AT+CSQ", 1000);
  enviarComandoAT("AT+CREG?", 1000);
}

void conectarGPRS() {
  Serial.println("Conectando GPRS...");
  
  enviarComandoAT("AT+CIPSHUT", 1000);
  enviarComandoAT("AT+CGATT=1", 1000);
  
  char cmdBuffer[100];
  snprintf(cmdBuffer, sizeof(cmdBuffer), "AT+CSTT=\"%s\",\"%s\",\"%s\"", apn, gprs_user, gprs_pass);
  enviarComandoAT(cmdBuffer, 1000);
  
  enviarComandoAT("AT+CIICR", 1000);
  enviarComandoAT("AT+CIFSR", 1000);
}

void enviarDatosGPRS() {
  if (gps.location.isValid()) {
    StaticJsonDocument<200> jsonDoc;
    char jsonBuffer[512];
    
    jsonDoc["latitude"] = gps.location.lat();
    jsonDoc["longitude"] = gps.location.lng();
    
    if (gps.altitude.isValid()) {
      jsonDoc["altitude"] = gps.altitude.meters();
    }
    if (gps.speed.isValid()) {
      jsonDoc["speed"] = gps.speed.kmph();
    }
    
    serializeJson(jsonDoc, jsonBuffer);
    
    char cmdBuffer[100];
    snprintf(cmdBuffer, sizeof(cmdBuffer), "AT+CIPSTART=\"TCP\",\"%s\",\"1883\"", mqtt_server);
    enviarComandoAT(cmdBuffer, 2000);
    
    snprintf(cmdBuffer, sizeof(cmdBuffer), "AT+CIPSEND=%d", strlen(jsonBuffer));
    enviarComandoAT(cmdBuffer, 1000);
    enviarComandoAT(jsonBuffer, 1000);
    
    enviarComandoAT("AT+CIPCLOSE", 1000);
    
    Serial.print("Datos GPS enviados por GPRS: ");
    Serial.println(jsonBuffer);
  } else {
    Serial.println("No hay datos GPS válidos para enviar");
  }
}

bool enviarComandoAT(const char* comando, int timeout) {
  simSerial.println(comando);
  
  long tiempoInicio = millis();
  String respuesta = "";
  
  while (millis() - tiempoInicio < timeout) {
    if (simSerial.available()) {
      char c = simSerial.read();
      respuesta += c;
      
      if (respuesta.indexOf("OK") != -1) {
        Serial.println("Comando AT exitoso: ");
        Serial.println(comando);
        return true;
      }
    }
  }
  
  Serial.println("Timeout en comando AT: ");
  Serial.println(comando);
  return false;
}

void conectarWiFi() {
  Serial.print("Conectando a ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(1000);
    Serial.print(".");
    intentos++;
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi conectado");
    Serial.println("Dirección IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Falló la conexión WiFi");
  }
}

void reconectarMQTT() {
  int intentos = 0;
  while (!client.connected() && intentos < 3) {
    Serial.print("Conectando a MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("conectado");
      client.subscribe(mqtt_topic);
      break;
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
      intentos++;
    }
  }
}

void enviarDatosMQTT() {
  if (gps.location.isValid()) {
    StaticJsonDocument<200> jsonDoc;
    char jsonBuffer[512];
    
    jsonDoc["latitude"] = gps.location.lat();
    jsonDoc["longitude"] = gps.location.lng();
    
    if (gps.altitude.isValid()) {
      jsonDoc["altitude"] = gps.altitude.meters();
    }
    if (gps.speed.isValid()) {
      jsonDoc["speed"] = gps.speed.kmph();
    }
    if (gps.satellites.isValid()) {
      jsonDoc["satellites"] = gps.satellites.value();
    }
    
    jsonDoc["connection_type"] = "wifi";
    
    serializeJson(jsonDoc, jsonBuffer);
    
    if (client.publish(mqtt_topic, jsonBuffer)) {
      Serial.print("Datos GPS enviados por MQTT: ");
      Serial.println(jsonBuffer);
    } else {
      Serial.println("Error al publicar en MQTT");
    }
  } else {
    Serial.println("Error: No hay datos GPS válidos para enviar.");
  }
}
