#include <HardwareSerial.h>

HardwareSerial sim800l(2);

String obtenerRespuestaCompleta() {
  String respuesta = "";
  unsigned long tiempoInicio = millis();
  while (millis() - tiempoInicio < 5000) {  // Espera 5 segundos o hasta que haya una respuesta completa
    while (sim800l.available()) {
      char c = sim800l.read();
      respuesta += c;
    }
  }
  respuesta.trim();
  return respuesta;
}

bool esperarRegistroRed(int maximoIntentos = 60) {
  Serial.println("Esperando registro en la red (2G)...");
  int intentos = 0;

  while (intentos < maximoIntentos) {
    // Verifica la señal antes de cada intento
    sim800l.println("AT+CSQ");
    delay(1000);
    String respuesta = obtenerRespuestaCompleta();
    Serial.println("Intensidad de señal: " + respuesta);

    // Si la señal es demasiado baja, espera y reintenta
    if (respuesta.indexOf(":") > 0) {
      int nivelSenal = respuesta.substring(respuesta.indexOf(":") + 2, respuesta.indexOf(",")).toInt();
      if (nivelSenal < 10) {
        Serial.println("Señal débil, esperando...");
        delay(5000);
        continue;
      }
    }

    // Verifica el estado de registro en la red
    sim800l.println("AT+CREG?");
    delay(3000);
    respuesta = obtenerRespuestaCompleta();
    Serial.println("Estado de registro: " + respuesta);

    if (respuesta.indexOf(",1") > 0 || respuesta.indexOf(",5") > 0) { 
      Serial.println("Registrado en la red!");
      return true;
    }

    // Intento de registro manual después de 10 intentos fallidos
    if (intentos == 10) {
      Serial.println("Intentando registro manual en Vodafone España...");
      sim800l.println("AT+COPS=1,2,\"21401\"");
      delay(10000);
      respuesta = obtenerRespuestaCompleta();
      Serial.println("Respuesta de registro manual: " + respuesta);
    }

    Serial.print("Intento ");
    Serial.print(intentos + 1);
    Serial.println(" de " + String(maximoIntentos));
    intentos++;
    delay(2000);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  sim800l.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Iniciando...");
  delay(5000);

  // Resetea el módulo
  Serial.println("Reseteando módulo...");
  sim800l.println("AT+CFUN=1,1");
  delay(10000);
  String respuesta = obtenerRespuestaCompleta();
  Serial.println("Respuesta de reset: " + respuesta);

  // Verifica comunicación básica
  Serial.println("Verificando comunicación...");
  sim800l.println("AT");
  delay(1000);
  respuesta = obtenerRespuestaCompleta();
  Serial.println("Respuesta de comunicación AT: " + respuesta);

  // Verifica SIM
  Serial.println("Verificando SIM...");
  sim800l.println("AT+CPIN?");
  delay(1000);
  respuesta = obtenerRespuestaCompleta();
  Serial.println("Respuesta de verificación SIM: " + respuesta);

  // Espera registro en la red
  if (!esperarRegistroRed()) {
    Serial.println("Error: No se pudo registrar en la red");
    return;
  }

  // Configura GPRS
  Serial.println("Configurando GPRS...");
  // Aquí puedes continuar con la configuración GPRS según lo planeado
}

void loop() {
  // Verifica la conexión a internet (si es necesario)
  delay(10000);
}
