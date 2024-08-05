#ifndef IDESERIALIZABLE_MESSAGE_FACTORY_H
#define IDESERIALIZABLE_MESSAGE_FACTORY_H


#include "IDeserializableMessage.h"

// Defines a factory for deserializable messages.
// For use by the MessageListener.
class IDeserializableMessageFactory
{
public:
    // Returns a message instance, given a message type string.
    // Returns NULL if message type is not recognised.
    virtual IDeserializableMessage *getMessageDeserializer(const char *messageType) = 0;
};


#endif
