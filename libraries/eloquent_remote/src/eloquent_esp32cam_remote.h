#ifndef ELOQUENT_ESP32CAM_REMOTE_H
#define ELOQUENT_ESP32CAM_REMOTE_H

#include <WebSocketsClient.h>
#include <eloquent_esp32cam.h>


using Eloquent::Error::Exception;


class Remote {
public:
    Exception exception;

    Remote(String deviceId);

    Exception& begin();

    Exception& loop();

private:
    String _id;
    String _status;
    bool _serverIsAlive;
    size_t _dropTimeout;
    size_t _pingTimeout;
    float _latency;
    WebSocketsClient _client;


    void onConnected();

    void onDisconnected();

    void onError();

    void onText(char *message, size_t length);
};

#endif