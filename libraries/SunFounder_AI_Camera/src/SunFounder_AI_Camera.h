#ifndef __SUNFOUNDER_AI_CAMERA_H__
#define __SUNFOUNDER_AI_CAMERA_H__

#include <Arduino.h>
#include <string.h>
#include <ArduinoJson.h>

/**
 * Use custom serial port
 */
// #define AI_CAM_DEBUG_CUSTOM
#ifdef ARDUINO_MINIMA
#define DataSerial Serial1
#define DebugSerial Serial
#else
#define DataSerial Serial
#define DebugSerial Serial
#endif

/**
 *  Set SERIAL_TIMEOUT & WS_BUFFER_SIZE
 */
#define SERIAL_TIMEOUT 100
#define WS_BUFFER_SIZE 200
#define CHAR_TIMEOUT 50

/**
 * Some keywords for communication with ESP32-CAM
 */
#define CHECK "SC"
#define OK_FLAG "[OK]"
#define ERROR_FLAG "[ERR]"
#define WS_HEADER "WS+"
#define WS_BIN_HEADER "WSB+"
#define CAM_INIT "[Init]"
#define WS_CONNECT "[CONNECTED]"
#define WS_DISCONNECT "[DISCONNECTED]"
#define APP_STOP "[APPSTOP]"

/**
 * Binary data defines
 */
#define WS_BIN_HEADER_LENGTH 4
#define BIN_START_BYTE 0xA0
#define BIN_END_BYTE 0xA1

/**
 * @name Set the print level of information received by esp32-cam
 *
 * @code {.cpp}
 * #define CAM_DEBUG_LEVEL CAM_DEBUG_LEVEL_INFO
 * @endcode
 *
 */
#define CAM_DEBUG_LEVEL CAM_DEBUG_LEVEL_ALL
#define CAM_DEBUG_LEVEL_OFF 0
#define CAM_DEBUG_LEVEL_ERROR 1
#define CAM_DEBUG_LEVEL_INFO 2
#define CAM_DEBUG_LEVEL_DEBUG 3
#define CAM_DEBUG_LEVEL_ALL 4

#define CAM_DEBUG_HEAD_ALL "[CAM_A]"
#define CAM_DEBUG_HEAD_ERROR "[CAM_E]"
#define CAM_DEBUG_HEAD_INFO "[CAM_I]"
#define CAM_DEBUG_HEAD_DEBUG "[CAM_D]"

/**
 * @name Define component-related values
 */
#define DPAD_STOP 0
#define DPAD_FORWARD 1
#define DPAD_BACKWARD 2
#define DPAD_LEFT 3
#define DPAD_RIGHT 4

#define JOYSTICK_X 0
#define JOYSTICK_Y 1
#define JOYSTICK_ANGLE 2
#define JOYSTICK_RADIUS 3

#define WIFI_MODE_NONE "0"
#define WIFI_MODE_STA "1"
#define WIFI_MODE_AP "2"

#define REGION_A 0
#define REGION_B 1
#define REGION_C 2
#define REGION_D 3
#define REGION_E 4
#define REGION_F 5
#define REGION_G 6
#define REGION_H 7
#define REGION_I 8
#define REGION_J 9
#define REGION_K 10
#define REGION_L 11
#define REGION_M 12
#define REGION_N 13
#define REGION_O 14
#define REGION_P 15
#define REGION_Q 16
#define REGION_R 17
#define REGION_S 18
#define REGION_T 19
#define REGION_U 20
#define REGION_V 21
#define REGION_W 22
#define REGION_X 23
#define REGION_Y 24
#define REGION_Z 25

#define MINIMAL_VERSION_MAJOR 1
#define MINIMAL_VERSION_MINOR 4
#define MINIMAL_VERSION_PATCH 0

#define WS_BUFFER_TYPE_NONE 0
#define WS_BUFFER_TYPE_TEXT 1
#define WS_BUFFER_TYPE_BINARY 2

class AiCamera
{
public:
  bool ws_connected = false;
  uint8_t recvBuffer[WS_BUFFER_SIZE];
  uint8_t recvBufferType = WS_BUFFER_TYPE_TEXT;
  uint8_t recvBufferLength = 0;
  StaticJsonDocument<200> sendDoc;

  AiCamera(const char *name, const char *type);
  void begin(const char *ssid, const char *password, const char *wsPort = "8765", bool autoSend = true);
  void begin(const char *ssid, const char *password, const char *wifiMode, const char *wsPort);

  void setOnReceived(void (*func)());
  void setOnReceivedBinary(void (*func)());
  void setCommandTimeout(uint32_t _timeout);
  void loop();

  void sendData();
  void sendBinaryData(uint8_t *data, size_t len);

  int16_t getSlider(uint8_t region);
  bool getButton(uint8_t region);
  bool getSwitch(uint8_t region);
  int16_t getJoystick(uint8_t region, uint8_t axis);
  uint8_t getDPad(uint8_t region);
  int16_t getThrottle(uint8_t region);
  void getSpeech(uint8_t region, char *result);

  void setMeter(uint8_t region, double value);
  void setRadar(uint8_t region, int16_t angle, double distance);
  void setGreyscale(uint8_t region, uint16_t value1, uint16_t value2, uint16_t value3);
  void setValue(uint8_t region, double value);

  void lamp_on(uint8_t level = 5);
  void lamp_off(void);

  void reset(bool wait = true);

private:
  bool autoSend = true;
  void readInto(char *buffer);
  void debug(char *msg);

  void command(const char *command, const char *value, char *result, bool wait = true);
  void set(const char *command, bool wait = true);
  void set(const char *command, const char *value, bool wait = true);
  void get(const char *command, char *result);
  void get(const char *command, const char *value, char *result);

  void subString(char *str, int16_t start, int16_t end = -1);
  void getStrOf(char *str, uint8_t index, char *result, char divider);
  void setStrOf(char *str, uint8_t index, String value, char divider = ';');
  int16_t getIntOf(char *str, uint8_t index, char divider = ';');
  bool getBoolOf(char *str, uint8_t index);
  double getDoubleOf(char *str, uint8_t index);

  bool checkFirmwareVersion(String version);
};

#endif // __SUNFOUNDER_AI_CAMERA_H__
