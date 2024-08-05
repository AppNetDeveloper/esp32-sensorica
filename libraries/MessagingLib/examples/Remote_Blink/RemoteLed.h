#ifndef REMOTE_LED_H
#define REMOTE_LED_H

#include <MessagingLib.h>

// Message type strings must be the same for a given message that
// is serialized and deserialized.
const char * const RemoteLedMessageType = "LED";

// Sends message to control the state of a remote LED.
class RemoteLed : public SerializableMessage
{
    // It's not necessary to implement SerializableMessage and IDeserializableMessage
    // in the same class. These implementations can be split accross classes where
    // appropriate.
public:

    RemoteLed(Stream *outStream)
    {
        _outStream = outStream;
        _ledNumber = 0;
        _isOn = false;
    }

    // Set LED attached to remote Arduino.
    void setLed(uint8_t ledNumber, bool isOn)
    {
       _ledNumber = ledNumber;
       _isOn = isOn;

       serialize(_outStream);
    }

    // Gets unique message type.
    const char *getMessageType()
    {
        return RemoteLedMessageType;
    }

protected:

    uint8_t _ledNumber;
    bool _isOn;

    void onSerialize()
    {
        char numstr[4];

        serializeProperty(itoa(_ledNumber, numstr, 10));
        serializeProperty(_isOn ? "true" : "false");

        // Serialize additional properties here.
    }

private:
    Stream * _outStream;
};

#endif
