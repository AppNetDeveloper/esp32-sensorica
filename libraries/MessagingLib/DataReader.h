#ifndef DATA_READER_H
#define DATA_READER_H

#include "DataReaderBuffer.h"

// Use this class to consume data from a DataReaderBuffer implementation.
// Example usage:
/*
    if (dataReader->connected())
    {
        if (dataReader->itemAvailable())
        {
            // TODO get data item from DataReaderBuffer.
            // TODO consume data item.
            dataReader->moveNext();
        }
        // TODO return or yield.
    }
    // TODO if necessary, return or yield.
*/
class DataReader
{
public:

    // Instance is considered to be connected until either all items
    // have been received and processed, or cancellation has been requested.
    virtual bool connected();

    // Returns true if an item is available.
    bool itemAvailable() const;

    // Move to the next item.
    void moveNext();

protected:
    DataReader(DataReaderBuffer *buffer);

    DataReaderBuffer *getBuffer() const;
private:

    DataReaderBuffer *_buffer;

};


#endif
