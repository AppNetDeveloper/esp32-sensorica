#include "DataReader.h"

DataReader::DataReader(DataReaderBuffer *buffer)
{
    _buffer = buffer;
    _buffer->clear();
}

//-----------------------------------------------------------------------------

DataReaderBuffer *DataReader::getBuffer() const
{
    return _buffer;
}

//-----------------------------------------------------------------------------

bool DataReader::connected()
{
    if (!_buffer->itemAvailable() && (_buffer->cancelled() || _buffer->completed()))
    {
        _buffer->clear();
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------

bool DataReader::itemAvailable() const
{
    return _buffer->itemAvailable();
}

//-----------------------------------------------------------------------------

void DataReader::moveNext()
{
    // Evicting the current item enables space additional
    // items to be written to the buffer.
    _buffer->evictItem();
}