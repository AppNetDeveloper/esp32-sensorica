#ifndef IDESERIALIZABLE_MESSAGE_H
#define IDESERIALIZABLE_MESSAGE_H

#include <stdint.h>

// Defines a deserializable message.
class IDeserializableMessage
{
public:
    // Returns the message type string. Must be unique for each implementation.
    virtual const char *getMessageType() =0 ;

    // Called when deserialization has begun.
    virtual void onBeginDeserialize() {}

    // Called when a message property has been deserialized.
    virtual void onDeserializeProperty(uint16_t propertyIndex, const char *propertyValue) =0 ;

    // Called when message serialization has ended.
    // Parameter messageComplete is set to true upon completion, and set to false if the
    // message 'end' delimiter was not encountered.
    virtual void onEndDeserialize(bool messageComplete) =0 ;
};

#endif