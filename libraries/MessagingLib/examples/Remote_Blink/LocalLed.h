#ifndef LOCAL_LED_H
#define LOCAL_LED_H

#include <MessagingLib.h>

// Message type strings must be the same for a given message that
// is serialized and deserialized.
const char * const LocalLedMessageType = "LED";

// Receives message to control locally attached LED.
class LocalLed : public IDeserializableMessage
{
public:

    LocalLed ()
    {
        _ledNumber_Received = 0;
        _isOn_Received = false;

        pinMode(LED_BUILTIN, OUTPUT);
    }

    // Gets unique message type.
    const char *getMessageType()
    {
        return LocalLedMessageType;
    }

protected:
    uint8_t _ledNumber_Received;
    bool _isOn_Received;

    void onBeginDeserialize()
    {
        // This method is optional. Can be removed.
    }

    void onDeserializeProperty(uint16_t propertyIndex, const char *propertyValue)
    {
        // Deserialize properties in the order they were serialized. Value of propertyIndex
        // determines order.

        switch (propertyIndex)
        {
            case 0:
            {
                _ledNumber_Received = atoi(propertyValue);
                break;
            }
            case 1:
            {
                _isOn_Received = (strcasecmp_P(propertyValue, PSTR("true")) == 0);
                break;
            }
        }
    }

    void onEndDeserialize(bool messageComplete)
    {
        if (messageComplete)
        {
            setLed(_ledNumber_Received, _isOn_Received);
        }
    }


    // Sets the state of a locally attached LED.
    void setLed(uint8_t ledNumber, bool isOn)
    {
        if (ledNumber == 0)
        {
            digitalWrite(LED_BUILTIN, isOn ? HIGH : LOW);
        }
    }
};

#endif