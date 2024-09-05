#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include <WiFi.h>

// Configuración de los pines para RS485
#define TX_PIN 17
#define RX_PIN 16
#define DE_PIN 2 // Pin para habilitar el transmisor (Driver Enable)
#define RE_PIN 4 // Pin para habilitar el receptor (Receiver Enable)

// Configuración de la red WiFi
const char* ssid = "Lss";
const char* password = "cvlss2101281613";

HardwareSerial RS485Serial(2);
ModbusMaster node;

// Variables para gestionar los dispositivos activos
bool dispositivos_activos[247] = {false};  // Array para almacenar el estado de cada dispositivo (1-247)

// Mutex para sincronización del acceso al puerto RS485
SemaphoreHandle_t xMutex;

// Tareas de FreeRTOS
TaskHandle_t TaskSondeo;
TaskHandle_t TaskMonitoreo;

// Función para gestionar la transmisión Modbus
void preTransmission() {
    digitalWrite(DE_PIN, HIGH);
    digitalWrite(RE_PIN, LOW);
    delay(1); 
}

void postTransmission() {
    digitalWrite(DE_PIN, LOW);
    digitalWrite(RE_PIN, HIGH);
    delay(1);
}

// Función para gestionar la conexión WiFi
void conectarWiFi() {
    Serial.print("Conectando a WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("Conectado a la red WiFi.");
    Serial.print("Dirección IP: ");
    Serial.println(WiFi.localIP());
}

// Tarea para sondear los dispositivos en el Core 0 (cada minuto)
void taskSondeoDispositivos(void *pvParameters) {
    while (true) {
        Serial.println("Iniciando sondeo de dispositivos...");

        for (int id = 1; id <= 247; id++) {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                node.begin(id, RS485Serial);
                uint8_t result = node.readHoldingRegisters(0x00, 1);  // Intentar leer un registro básico

                if (result == node.ku8MBSuccess) {
                    Serial.print("Dispositivo encontrado con ID: ");
                    Serial.println(id);
                    dispositivos_activos[id] = true;  // Marcar dispositivo como activo
                } else {
                    dispositivos_activos[id] = false; // Marcar dispositivo como inactivo
                }
                xSemaphoreGive(xMutex);
            }
        }

        Serial.println("Sondeo completado.");
        vTaskDelay(60000 / portTICK_PERIOD_MS);  // Espera de 1 minuto
    }
}

// Tarea para monitorear dispositivos activos en el Core 1
void taskMonitoreoDispositivos(void *pvParameters) {
    while (true) {
        Serial.println("Monitoreando dispositivos activos...");

        for (int id = 1; id <= 247; id++) {
            if (dispositivos_activos[id]) {
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                    node.begin(id, RS485Serial);
                    uint8_t result = node.readHoldingRegisters(0x00, 1);  // Leer registro específico

                    if (result == node.ku8MBSuccess) {
                        Serial.print("Dispositivo con ID ");
                        Serial.print(id);
                        Serial.print(" respondió. Registro 0x00: ");
                        Serial.println(node.getResponseBuffer(0));  // Imprime el valor del registro

                        // Enviar datos a través de WiFi si es necesario
                        if (WiFi.status() == WL_CONNECTED) {
                            // Enviar los datos leídos a un servidor o hacer otra acción con WiFi
                        }
                    } else {
                        Serial.print("Error al leer el dispositivo con ID ");
                        Serial.println(id);
                    }
                    xSemaphoreGive(xMutex);
                }
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Espera de 2 segundos entre ciclos de monitoreo
    }
}

void setup() {
    // Inicializar puerto serie para depuración
    Serial.begin(115200);

    // Conectar a la red WiFi
    conectarWiFi();

    // Configuración de los pines para RS485
    pinMode(DE_PIN, OUTPUT);
    pinMode(RE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW); 
    digitalWrite(RE_PIN, HIGH);

    // Configurar el puerto RS485
    RS485Serial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    // Configurar ModbusMaster para la comunicación Modbus
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Crear mutex para sincronizar acceso al puerto RS485
    xMutex = xSemaphoreCreateMutex();

    // Crear las tareas en diferentes núcleos
    xTaskCreatePinnedToCore(
        taskSondeoDispositivos,   // Función de la tarea
        "SondeoDispositivos",     // Nombre de la tarea
        10000,                    // Tamaño del stack
        NULL,                     // Parámetros de entrada
        1,                        // Prioridad de la tarea
        &TaskSondeo,              // Handle de la tarea
        0                         // Núcleo donde se ejecutará (0)
    );

    xTaskCreatePinnedToCore(
        taskMonitoreoDispositivos, // Función de la tarea
        "MonitoreoDispositivos",   // Nombre de la tarea
        10000,                     // Tamaño del stack
        NULL,                      // Parámetros de entrada
        1,                         // Prioridad de la tarea
        &TaskMonitoreo,            // Handle de la tarea
        1                          // Núcleo donde se ejecutará (1)
    );
}

void loop() {
    // No se usa el loop() en este caso, todo se gestiona con FreeRTOS y las tareas
}



