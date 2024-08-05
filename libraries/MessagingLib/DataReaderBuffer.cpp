#include "DataReaderBuffer.h"

DataReaderBuffer::DataReaderBuffer()
{
    _syncId = 0;
    clear();
}

//-----------------------------------------------------------------------------

void DataReaderBuffer::clear()
{
    _available = false;
    _completed = false;
    _cancelled = false;
    _syncId++;
}

//-----------------------------------------------------------------------------

uint8_t DataReaderBuffer::getSyncId() const
{
    return _syncId;
}


// void DataReaderBuffer::evictItem()
// {
//     _available = false;
// }

//-----------------------------------------------------------------------------

bool DataReaderBuffer::itemAvailable() const
{
    return _available;
}

//-----------------------------------------------------------------------------

void DataReaderBuffer::cancel()
{
    _cancelled = true;
}

//-----------------------------------------------------------------------------

bool DataReaderBuffer::cancelled () const
{
    return _cancelled;
}

//-----------------------------------------------------------------------------

void DataReaderBuffer::setComplete()
{
    _completed = true;
}

//-----------------------------------------------------------------------------

bool DataReaderBuffer::completed() const
{
    return _completed;
}

//-----------------------------------------------------------------------------

void DataReaderBuffer::setAvailable(bool val)
{
    _available = val;
}