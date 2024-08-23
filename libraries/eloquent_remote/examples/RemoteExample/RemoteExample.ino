/**
 * Access your ESP32 camera stream from anywhere
 * in the world
 *
 *  > Requires WebSockets library
 * (NOT ArduinoWebsockets!)
 *
 *  > Requires EloquentEsp32Cam library
 */
#define WIFI_SSID "SSID"
#define WIFI_PASS "PASSWORD"

#include <WebSocketsClient.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/extra/esp32/wifi.h>
#include <eloquent_esp32cam_remote.h>

using eloq::camera;
using eloq::wifi;

Remote remote("replace-with-your-device-id");


/**
 *
 */
void setup() {
    delay(3000);
    Serial.begin(115200);
    Serial.println("___WEBSOCKETS___");

    // replace with your model!
    camera.pinout.freenove_s3();
    camera.brownout.disable();
    camera.resolution.qvga();
    camera.quality.high();

    // init camera
    while (!camera.begin().isOk())
        Serial.println(camera.exception.toString());

    // connect to wifi
    while (!wifi.connect().isOk())
        Serial.println(wifi.exception.toString());

    // connect to streaming platform
    while (!remote.begin().isOk())
        Serial.println(remote.exception.toString());

    Serial.println("Camera OK");
}

/**
 *
 */
void loop() {
    // stream data to cloud platform for remote access
    if (!remote.loop().isOk()) {
        Serial.println(remote.exception.toString());
        delay(1000);
    }
}