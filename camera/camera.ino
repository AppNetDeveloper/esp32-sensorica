#include "esp_camera.h"
#include <WiFi.h>

// Reemplaza con tus credenciales de WiFi
const char* ssid = "Lss";
const char* password = "cvlss2101281613";

// Configuración de la cámara
#define CAMERA_MODEL_AI_THINKER  // Asegúrate de que esta línea coincida con tu modelo de cámara
#include "camera_pins.h"

void setup() {
  Serial.begin(115200);
  // Inicializar la cámara
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  // Puedes ajustar la resolución aquí
  config.frame_size = FRAMESIZE_QVGA; // QVGA(320x240)
  config.jpeg_quality = 10;
  config.fb_count = 1;

  // Iniciar la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Error al iniciar la cámara: 0x%x", err);
    return;
  }

  // Conectar a WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP:");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Tomar una foto
  camera_fb_t * fb = esp_camera_fb_get();  
  if (!fb) {
    Serial.println("Error al capturar la imagen");
    return;
  }

  // Generar la página web
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<title>ESP32-CAM</title>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Imagen desde ESP32-CAM</h1>";
  html += "<img src='data:image/jpeg;base64,";
  html += base64::encode(fb->buf, fb->len);
  html += "' />";
  html += "</body>";
  html += "</html>";

  // Liberar la memoria
  esp_camera_fb_return(fb); 

  // Esperar una conexión
  WiFiClient client = server.available();
  if (client) { 
    Serial.println("Nuevo cliente conectado");
    String currentLine = ""; 
    while (client.connected()) { 
      if (client.available()) { 
        char c = client.read(); 
        Serial.write(c); 
        currentLine += c;
        if (currentLine.endsWith("\r\n")) { 
          if (currentLine.startsWith("GET / ")) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            client.println(html); 
          }
          currentLine = ""; 
        }
      }
    }
    // Cerrar la conexión
    client.stop(); 
    Serial.println("Cliente desconectado");
  }
}
