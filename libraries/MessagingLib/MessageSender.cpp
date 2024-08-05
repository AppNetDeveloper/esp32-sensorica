#include "MessageSender.h"

SerializableMessageSender::SerializableMessageSender(SerializableMessage **msgArray, uint8_t msgArraySize, Print *outStream,
                    uint16_t msgIntervalMs, uint16_t msgChunkIntervalMs, bool enableTriggerTimeout)
{
    this->msgArray = msgArray;
    this->msgArraySize = msgArraySize;
    this->msgIntervalMs = msgIntervalMs;
    this->msgChunkIntervalMs = msgChunkIntervalMs;
    this->stream = outStream;
    this->triggerTimeoutEnabled = enableTriggerTimeout;
    clear();
}

//-----------------------------------------------------------------------------

void SerializableMessageSender::clear()
{
    msgIdxStart = 0;
    msgIdxEnd = 0;
    msgCount = 0;
    currentMsg = NULL;
    readyToSend = true;
    chunkTriggered = false;
}

void SerializableMessageSender::triggerChunkSend()
{
    chunkTriggered = true;
}
//-----------------------------------------------------------------------------

bool SerializableMessageSender::queueMessageIfNotQueued(SerializableMessage *msg)
{
    return queueMessage(msg, false);
}

bool SerializableMessageSender::queueMessage(SerializableMessage *msg, bool allowDuplicates)
{
    if (!hasCapacity())
    {
        return false;
    }
    if (messageQueued(msg) && !allowDuplicates)
    {
        return false;
    }

    //debugln(F("En-queuing"));

    msgArray[msgIdxEnd] = msg;

    msgIdxEnd++;
    msgCount++;
    if (msgIdxEnd >= msgArraySize)
    {
        msgIdxEnd = 0;
    }
    return true;
}

//-----------------------------------------------------------------------------

SerializableMessage *SerializableMessageSender::deQueueMessage()
{
    if (msgCount == 0)
    {
        return NULL;
    }
    SerializableMessage *val = msgArray[msgIdxStart];

    msgIdxStart++;
    msgCount--;

    if (msgIdxStart >= msgArraySize)
    {
        msgIdxStart = 0;
    }

    return val;
}

//-----------------------------------------------------------------------------

bool SerializableMessageSender::hasCapacity() const
{
    return msgCount < msgArraySize;
}

//-----------------------------------------------------------------------------

uint8_t SerializableMessageSender::count() const
{
    return msgCount;
}

//-----------------------------------------------------------------------------

SerializableMessage *SerializableMessageSender::peekMessage()
{
    if (msgCount == 0)
    {
        return NULL;
    }
    return msgArray[msgIdxStart];
}

//-----------------------------------------------------------------------------

bool SerializableMessageSender::messageQueued(SerializableMessage *msg) const
{
    for (uint8_t i=0; i < msgCount; i++)
    {
        if (getMessage(i) == msg)
        {
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------

SerializableMessage *SerializableMessageSender::getMessage(uint8_t msgIdx) const
{
    if (msgIdx >= msgCount)
    {
        return NULL;
    }

    uint8_t actualIdx = msgIdxStart + msgIdx;

    if (actualIdx >= msgArraySize)
    {
        actualIdx -= msgArraySize;
    }

    return msgArray[actualIdx];
}

//-----------------------------------------------------------------------------

void SerializableMessageSender::processMessageQueue()
{
    uint32_t currentTimeMs = millis();

    // Check interval
    if (!readyToSend)
    {
        // recalculate readyToSend
        if (currentTimeMs - msgLastSendTimeMs < (uint32_t)msgIntervalMs)
        {
            return;
        }
        readyToSend = true;
    }

    if (currentMsg == NULL)
    {
        currentMsg = peekMessage();
        if (currentMsg == NULL)
        {
            return;
        }
    }

    //uint32_t currentTimeUs = millis();//micros();

    bool chunkIntervalLapsed = currentTimeMs - msgLastChunkSendTimeMs >= (uint32_t)msgChunkIntervalMs;

    // Check serialization chunk interval
    if (currentMsg->isSerializing()) // && !chunkTriggered && !chunkIntervalLapsed)
    {
        if (chunkIntervalLapsed)
        {
            if (triggerTimeoutEnabled)
            {
                currentMsg->cancel();
            }
        }
        else if (!chunkTriggered)
        {
            return;
        }
    }

    chunkTriggered = false;
    
    currentMsg->serialize(stream);
    msgLastChunkSendTimeMs = currentTimeMs;

    if (!currentMsg->isSerializing())
    {
        deQueueMessage();
        currentMsg = NULL;
        msgLastSendTimeMs = currentTimeMs;

        readyToSend = false;
    }
}

//-----------------------------------------------------------------------------