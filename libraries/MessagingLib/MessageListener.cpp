#include "MessageListener.h"
// #include "MessageDelimiters.h"

MessageListener::MessageListener(Stream *inStream, IStringBuilder *stringWriter, IDeserializableMessageFactory *messageDeserializerFactory,
    char msg_start, char msg_end, char msg_property) :
    _stream (inStream), _currentMsgString (stringWriter), _messageDeserializerFactory (messageDeserializerFactory),
    _msg_start(msg_start), _msg_end(msg_end), _msg_property(msg_property)
{
    //_stream = inStream;
    //_messageDeserializerFactory = messageDeserializerFactory;
    _currentMsg = NULL;
    //_currentMsgString = stringWriter;
    _currentPropertyIndex = 0;

}

//-----------------------------------------------------------------------------

void MessageListener::reset()
{
    if (isDeserializingMessage())
    {
        endDeserialize(_currentMsg, false);
    }
    _currentMsg = NULL;
    _currentPropertyIndex = 0;
    _currentMsgString->clear();
    _listenerState = MsgListenerState_Start;
}

//-----------------------------------------------------------------------------

void MessageListener::deserializeProperty(IDeserializableMessage *msg, char *propertyEncValue)
{
    if (msg == NULL)
        return;

    msg->onDeserializeProperty(_currentPropertyIndex, propertyEncValue);
    _currentPropertyIndex++;
}

//-----------------------------------------------------------------------------

void MessageListener::endDeserialize(IDeserializableMessage *msg, bool messageComplete)
{
    if (msg != NULL) msg->onEndDeserialize(messageComplete);
}

//-----------------------------------------------------------------------------

bool MessageListener::isDeserializingMessage() const
{
    return (_listenerState == MsgListenerState_MsgType || _listenerState == MsgListenerState_Property);
}

//-----------------------------------------------------------------------------

bool MessageListener::processMessageStream()
{
    bool streamAvailable = false;
    bool breakLoop = false;

    while (_stream->available() && !breakLoop)
    {
        streamAvailable = true;
        const char chr = _stream->read();

        switch (_listenerState)
        {
            case MsgListenerState_Error:
            case MsgListenerState_Start:
            {
                if (chr == _msg_start)
                {
                    _currentMsgString->clear();
                    _listenerState = MsgListenerState_MsgType;
                }
                else
                {
                    _listenerState = MsgListenerState_Start;
                }
                break;
            }
            case MsgListenerState_MsgType:
            {
                if (isalnum(chr))
                {
                    _currentMsgString->append(chr);
                    _listenerState = MsgListenerState_MsgType;
                }
                else
                {
                    _currentPropertyIndex = 0;
                    _currentMsg = _messageDeserializerFactory->getMessageDeserializer(_currentMsgString->toString());
                    if (_currentMsg != NULL)
                        _currentMsg->onBeginDeserialize();

                    if (chr == _msg_property)
                    {
                        _listenerState = MsgListenerState_Property;
                    }
                    else if (chr == _msg_end)
                    {
                        endDeserialize(_currentMsg, true);
                        _listenerState = MsgListenerState_Start;
                        breakLoop = true;
                    }
                    else if (chr == _msg_start)
                    {
                        //Serial.println(F("Cancelled 1"));
                        endDeserialize(_currentMsg, false);
                        _listenerState = MsgListenerState_MsgType;
                        breakLoop = true;
                    }
                    else
                    {
                        //Serial.println(F("Cancelled 2"));
                        endDeserialize(_currentMsg, false);
                        _listenerState = MsgListenerState_Error;
                        breakLoop = true;
                    }
                    _currentMsgString->clear();
                }
                break;
            }
            case MsgListenerState_Property:
            {
                if (chr == _msg_property)
                {
                    deserializeProperty(_currentMsg, _currentMsgString->toString());
                    _listenerState = MsgListenerState_Property;
                    breakLoop = true;
                }
                else if (chr == _msg_end)
                {
                    deserializeProperty(_currentMsg, _currentMsgString->toString());
                    endDeserialize(_currentMsg, true);
                    _listenerState = MsgListenerState_Start;
                    breakLoop = true;
                }
                else if (chr == _msg_start)
                {
                    //Serial.println(F("Cancelled 3"));
                    deserializeProperty(_currentMsg, _currentMsgString->toString());
                    endDeserialize(_currentMsg, false);
                    _listenerState = MsgListenerState_MsgType;
                    breakLoop = true;
                }
                else
                {
                    _currentMsgString->append(chr);
                    _listenerState = MsgListenerState_Property;
                }

                if (chr == _msg_property || chr == _msg_end || chr == _msg_start)
                {
                    _currentMsgString->clear();
                }
                break;
            }
        }
    }

    //Serial.print(F("."));
    if (streamAvailable)
    {
        _deserializeActivityTime = millis();
    }
    else if (isDeserializingMessage() && millis() - _deserializeActivityTime > _timeOutMs)
    {
        // reset the listener.
        //Serial.println(F("reset the listener"));
        reset();
    }
    return streamAvailable;
}

//-----------------------------------------------------------------------------
