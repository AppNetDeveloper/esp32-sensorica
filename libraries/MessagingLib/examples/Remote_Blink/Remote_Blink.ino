/*

Deploy project to two Arduinos, then connect them via their serial pins.
Each Arduino will control the blinking of the built-in LED of the other
Arduino.

*/

#include "MyDeserializableMessageFactory.h"
#include "RemoteLed.h"

#if defined(__AVR_ATmega32U4__)
    #define MsgSerial  Serial1
#else
    #define MsgSerial  Serial
#endif

MyDeserializableMessageFactory factory = MyDeserializableMessageFactory();

// Buffer must be large enough to hold the largest property value of a message.
StringBuilder<24> msgBuffer;
MessageListener msgListener(&MsgSerial, &msgBuffer, &factory);

uint32_t lastMsgTime;
bool remoteLedState = false;

void setup()
{
    // put your setup code here, to run once:
#if defined(__AVR_ATmega32U4__)
    pinMode(0, INPUT_PULLUP);   // Initialize serial RX pin.
#endif
    MsgSerial.begin(115200);

    lastMsgTime = millis()-1000;
}


void loop()
{
    // Process incomming message stream and deserialize messages.
    msgListener.processMessageStream();

    // Check if blink interval expired.
    if (millis() - lastMsgTime >= 1000)
    {
        lastMsgTime = millis();

        remoteLedState = !remoteLedState;

        // Send message to set state of remote LED.
        RemoteLed remoteLed(&MsgSerial);
        remoteLed.setLed(0, remoteLedState);
    }
}
