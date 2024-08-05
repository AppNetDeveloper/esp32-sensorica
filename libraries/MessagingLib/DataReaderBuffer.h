#ifndef DATA_READER_BUFFER_H
#define DATA_READER_BUFFER_H

#include <stdint.h>

// This abstract class acts as a buffer between data that is being deserialized and
// its consumption by the DataReader class.
class DataReaderBuffer
{
public:
    DataReaderBuffer();
    
    // Resets the buffer to its initial state, and updates the SyncId.
    // Must call this base method if it is overridden.
    virtual void clear();

    // The SyncId is required to prevent buffer filling with out of date data.
    // Call this method in the constructor and deserialization methods of a class
    // that implements IDeserializableMessage. Compare the value recorded in the
    // constructor with the value obtained from subsequent calls to ensure it is the same.
    uint8_t getSyncId() const;

    // FIFO eviction of item from the buffer to make space for another item.
    virtual void evictItem() = 0;

    // Returns true if a response item (a data chunk) has been received from the server.
    bool itemAvailable() const;

    // Call this method from the onEndDeserialize method of a class that
    // implements IDeserializableMessage, and where messageComplete is false.
    void cancel();

    // Returns true if the response from the server ended unexpectedly.
    bool cancelled () const;
    
    // Call this method from the onEndDeserialize method of a class that
    // implements IDeserializableMessage, and where messageComplete is true.
    void setComplete();

    // Returns true if there are no more items to receive from the server.
    bool completed() const;

    // Adds item (FIFO) to the buffer (if space is available).
    // Item is parsed into an instance where necessary.
    // Call this method in the onDeserializeProperty method of a class
    // that implements IDeserializableMessage.
    virtual void addItem (const char *value) = 0;

protected:

    void setAvailable(bool val);
    
private:

    bool _cancelled;
    bool _available;
    bool _completed;
    uint8_t _syncId;
};

#endif
