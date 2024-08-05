#ifndef MESSAGE_LISTENER_H
#define MESSAGE_LISTENER_H

#include <Arduino.h>
#include <ctype.h>
#include <StringLib.h>
#include "IDeserializableMessage.h"
#include "IDeserializableMessageFactory.h"
#include "MessageDelimiters.h"

enum MsgListenerState : uint8_t
{
    MsgListenerState_Start = 0,
    MsgListenerState_MsgType,
    MsgListenerState_Property,
    MsgListenerState_Error
};

// Listens for incomming messages on a serial stream and deserializes to correct type.
class MessageListener
{
public:
    // Creates instance with serial stream, string builder and a message factory.
    //MessageListener(Stream *inStream, IStringBuilder *stringWriter, IDeserializableMessageFactory *messageDeserializerFactory);

    MessageListener(Stream *inStream, IStringBuilder *stringWriter, IDeserializableMessageFactory *messageDeserializerFactory,
                    char msg_start = MSG_START, char msg_end = MSG_END, char msg_property = MSG_PROPERTY);

    // Processes the message stream, listening for incomming messages.
    // Call in the loop of the main application.
    bool processMessageStream();

    // Returns true if currently deserializing a message.
    bool isDeserializingMessage() const;

    // Ends current deserialization and clears and resets the Message Listener instance.
    void reset();

private:

    // Deserializes the property of an incomming message.
    void deserializeProperty(IDeserializableMessage *msg, char *propertyEncValue);

    // Called when incomming message deserialization is complete or a new incomming message is detected.
    // Parameter messageComplete is set to true upon completion, and set to false if the
    // message 'end' delimiter was not encountered.
    void endDeserialize(IDeserializableMessage *msg, bool messageComplete);

    Stream * const _stream;
    IDeserializableMessageFactory * const _messageDeserializerFactory;
    MsgListenerState _listenerState;

    IDeserializableMessage *_currentMsg;
    IStringBuilder * const _currentMsgString;
    uint16_t _currentPropertyIndex;

    uint32_t _deserializeActivityTime;
    const uint32_t _timeOutMs = 1500;

    const char _msg_start;
    const char _msg_end;
    const char _msg_property;
};
#endif