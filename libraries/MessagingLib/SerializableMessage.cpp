#include "SerializableMessage.h"
#include "MessageDelimiters.h"

void SerializableMessage::serialize(Print *outStream)
{
    this->stream = outStream;

    if (!isSerializingInd)
    {
        _cancel = false;
        isSerializingInd = true;
        serializeCompletedInd = true;

        onBeginSerialize();
        
        stream->write(MSG_START);
        stream->write(getMessageType());
    }

    onSerialize();

    if (serializeCompletedInd)
    {
        stream->write(MSG_END);
    }
    isSerializingInd = !serializeCompletedInd;
}

//---------------------------------------------------------------------------------------

bool SerializableMessage::isSerializing() const
{
    return isSerializingInd;
}

//---------------------------------------------------------------------------------------

void SerializableMessage::setSerializeCompleted(bool value)
{
    serializeCompletedInd = value;
}

//---------------------------------------------------------------------------------------

void SerializableMessage::serializeProperty(const char *propertyValue)
{
    stream->write(MSG_PROPERTY);
    stream->print(propertyValue);
    //Serial.print(F("Property:"));
    //Serial.println(propertyValue);
}

//---------------------------------------------------------------------------------------

Print *SerializableMessage::getStream() const
{
    return stream;
}

//---------------------------------------------------------------------------------------

void SerializableMessage::cancel()
{
    _cancel = true;
}

//---------------------------------------------------------------------------------------

bool SerializableMessage::isCancelled () const
{
    return _cancel;
}

//---------------------------------------------------------------------------------------
