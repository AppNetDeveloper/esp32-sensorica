#ifndef SERIALIZABLE_MESSAGE_H
#define SERIALIZABLE_MESSAGE_H

#include <Arduino.h>

// An abstract class that supports serialization of simple properties.
class SerializableMessage
{
public:
    // Returns the message type string. Must be unique for each implementation.
    virtual const char *getMessageType() = 0;

    // Serializes object to the stream.
    void serialize(Print *outStream);

    // Returns true of the object is currently serializing.
    bool isSerializing() const;

    // Signal cancellation of serialization. Subclass implementation
    // must decide how to cancel serialization.
    void cancel();

    // Returns true if the cancel method has been invoked.
    bool isCancelled () const;

protected:

    // Called when serialization has begun.
    virtual void onBeginSerialize() {}

    // An abstract method that serializes the object.
    virtual void onSerialize() = 0;

    // Serializes a property to the stream. Set the propertyValue
    // to an empty string and use the underlying stream if more
    // formatting control of the property value is required.
    void serializeProperty(const char *propertyValue);

    // Set to false if successive calls to serialize method are required.
    // Set to true when serialization is complete. Use where serialization
    // is implemented as a FSM.
    void setSerializeCompleted(bool value);

    // Gets the underlying stream.
    Print *getStream() const;

protected:
    //void messageEncode(const char *str);
private:
    Print *stream;
    bool isSerializingInd = false;
    bool serializeCompletedInd = false;
    bool _cancel = false;
};

#endif