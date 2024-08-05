#ifndef MY_DESERIALIZABLE_MESSAGE_FACTORY_H
#define MY_DESERIALIZABLE_MESSAGE_FACTORY_H

#include <MessagingLib.h>
#include "LocalLed.h"
#include <new>

// Determine amount of memory to allocate for messages.
static const size_t LARGEST_MESSAGE_SIZE =
    STATIC_MAX(sizeof(LocalLed),
    0);


// Pre-allocated memory for messages. Align union to 4 byte boundary.
union
{
    char Memory[LARGEST_MESSAGE_SIZE];
    uint32_t dummy;
} MessageUnion;

static char* Message = MessageUnion.Memory;

// Used by the MessageListener to construct the correct class instance for an
// incomming message.
class MyDeserializableMessageFactory : public IDeserializableMessageFactory
{
    IDeserializableMessage *getMessageDeserializer(const char *messageType)
    {
        // Instantiate message types based on the messageType string.
        if (strcmp(messageType, LocalLedMessageType) == 0)
        {
            return new (Message) LocalLed();
        }
        return NULL;
    }
};

#endif