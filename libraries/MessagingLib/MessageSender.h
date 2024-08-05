#ifndef MESSAGE_SENDER_H
#define MESSAGE_SENDER_H

#include "SerializableMessage.h"

// Manages the sending of FIFO queued messages.
class SerializableMessageSender
{
public:
    // Clears the message queue.
    void clear();
    
    // Returns true if there is capacity in the message queue.
    bool hasCapacity() const;

    // Queues the message. Returns true if message was queued.
    bool queueMessage(SerializableMessage *msg, bool allowDuplicates = false);

    // Queues the message if it is not already in the queue. Returns true if message was queued.
    bool queueMessageIfNotQueued(SerializableMessage *msg);

    // Returns true if the supplied message is already in the queue.
    bool messageQueued(SerializableMessage *msg) const;

    // Returns the message that is either currently being processed,
    // or is about to be processed.
    SerializableMessage *peekMessage();

    // Counts the number of queued messages.
    uint8_t count() const;

    // Sends the oldest queued message, when ready to send.
    // Call in the loop of the main application.
    void processMessageQueue();

    // Invoking this method triggers the next message chunk to be sent.
    // Use as part of flow control.
    void triggerChunkSend();

protected:
    SerializableMessageSender(SerializableMessage **msgArray, uint8_t msgArraySize, Print *outStream,
                uint16_t msgIntervalMs, uint16_t msgChunkIntervalMs, bool enableTriggerTimeout);
    
    // Removes the oldest queue in the message.
    SerializableMessage *deQueueMessage();

    // Gets a queued message by index. Returns NULL if the index is out of bounds.
    SerializableMessage *getMessage(uint8_t msgIdx) const;

private:
    uint16_t msgIntervalMs;         // should be const
    uint16_t msgChunkIntervalMs;    // should be const

    uint32_t msgLastSendTimeMs;
    uint32_t msgLastChunkSendTimeMs;

    // Enabling trigger timeout causes cancel() of the current
    // serialiable object to be invoked when the chunk interval
    // has lapsed.
    bool triggerTimeoutEnabled;
    bool chunkTriggered;
    bool readyToSend;

    SerializableMessage *currentMsg;
    Print *stream;

    uint8_t msgArraySize;           // should be const
    SerializableMessage **msgArray;

    uint8_t msgIdxStart;
    uint8_t msgIdxEnd;
    uint8_t msgCount;
};


// Implementation of SerializableMessageSender. L=size of message queue.
template <uint8_t L=4>
class MessageSender: public SerializableMessageSender
{
public:
    // Creates instance with a Stream, message sending interval, and message chunk interval.
    MessageSender(Print *outStream, uint16_t msgIntervalMs = 50, uint16_t msgChunkIntervalMs = 4, bool enableTriggerTimeout = false) :
        SerializableMessageSender(msgQueueBuffer, L, outStream, msgIntervalMs, msgChunkIntervalMs, enableTriggerTimeout) {}

private:
    SerializableMessage *msgQueueBuffer[L];
};

#endif
