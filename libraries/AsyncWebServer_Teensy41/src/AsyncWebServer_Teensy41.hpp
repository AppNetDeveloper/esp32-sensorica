/****************************************************************************************************************************
  AsyncWebServer_Teensy41.hpp - Dead simple AsyncWebServer for Teensy41 QNEthernet
  
  For Teensy41 with QNEthernet
   
  AsyncWebServer_Teensy41 is a library for the Teensy41 with QNEthernet
  
  Based on and modified from ESPAsyncWebServer (https://github.com/me-no-dev/ESPAsyncWebServer)
  Built by Khoi Hoang https://github.com/khoih-prog/AsyncWebServer_Teensy41
  
  Copyright (c) 2016 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.
  This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License 
  as published bythe Free Software Foundation, either version 3 of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
  You should have received a copy of the GNU General Public License along with this program.
  If not, see <https://www.gnu.org/licenses/>.  
 
  Version: 1.7.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.4.1   K Hoang      18/03/2022 Initial coding for Teensy 4.1 using built-in QNEthernet.
                                  Bump up version to v1.4.1 to sync with AsyncWebServer_STM32 v1.4.1
  1.5.0   K Hoang      01/10/2022 Fix issue with slow browsers or network. Add function and example to support favicon.ico
  1.6.0   K Hoang      06/10/2022 Option to use non-destroyed cString instead of String to save Heap
  1.6.1   K Hoang      10/11/2022 Add examples to demo how to use beginChunkedResponse() to send in chunks
  1.6.2   K Hoang      16/01/2023 Add examples Async_WebSocketsServer
  1.7.0   K Hoang      29/01/2024 Fix file upload handler by adjusting function signatures
 *****************************************************************************************************************************/

#ifndef _ASYNC_WEBSERVER_TEENSY41_HPP_
#define _ASYNC_WEBSERVER_TEENSY41_HPP_

/////////////////////////////////////////////////

#define ASYNC_WEBSERVER_TEENSY41_VERSION           "AsyncWebServer_Teensy41 v1.7.0"

#define ASYNC_WEBSERVER_TEENSY41_VERSION_MAJOR     1
#define ASYNC_WEBSERVER_TEENSY41_VERSION_MINOR     7
#define ASYNC_WEBSERVER_TEENSY41_VERSION_PATCH     0

#define ASYNC_WEBSERVER_TEENSY41_VERSION_INT       1007000

/////////////////////////////////////////////////

#ifndef BOARD_NAME
	#define BOARD_NAME			"Teensy 4.1"
#endif

/////////////////////////////////////////////////

#ifndef AWS_TEENSY41_UNUSED
  #define AWS_TEENSY41_UNUSED(x) (void)(x)
#endif

/////////////////////////////////////////////////

#include "Arduino.h"
#include <functional>

#include <Teensy41_AsyncTCP.hpp>

#include "AsyncWebServer_Teensy41_Debug.h"
#include "StringArray_Teensy41.h"

/////////////////////////////////////////////////

#ifdef ASYNCWEBSERVER_REGEX
  #if(_AWS_TEENSY41_LOGLEVEL_>3)
    #warning Using ASYNCWEBSERVER_REGEX
  #endif
  
  #define ASYNCWEBSERVER_REGEX_ATTRIBUTE
#else
  #define ASYNCWEBSERVER_REGEX_ATTRIBUTE __attribute__((warning("ASYNCWEBSERVER_REGEX not defined")))
#endif

#define DEBUGF(...) Serial.printf(__VA_ARGS__)

static const String SharedEmptyString = String();

/////////////////////////////////////////////////

class AsyncWebServer;
class AsyncWebServerRequest;
class AsyncWebServerResponse;
class AsyncWebHeader;
class AsyncWebParameter;
class AsyncWebRewrite;
class AsyncWebHandler;
class AsyncStaticWebHandler;
class AsyncCallbackWebHandler;
class AsyncResponseStream;

/////////////////////////////////////////////////

#ifndef WEBSERVER_H
  typedef enum
  {
    HTTP_GET     = 0b00000001,
    HTTP_POST    = 0b00000010,
    HTTP_DELETE  = 0b00000100,
    HTTP_PUT     = 0b00001000,
    HTTP_PATCH   = 0b00010000,
    HTTP_HEAD    = 0b00100000,
    HTTP_OPTIONS = 0b01000000,
    HTTP_ANY     = 0b01111111,
  } WebRequestMethod;
#endif

//if this value is returned when asked for data, packet will not be sent and you will be asked for data again
#define RESPONSE_TRY_AGAIN 0xFFFFFFFF

typedef uint8_t WebRequestMethodComposite;
typedef std::function<void()> ArDisconnectHandler;

/////////////////////////////////////////////////
/////////////////////////////////////////////////

/*
   PARAMETER :: Chainable object to hold GET/POST and FILE parameters
 * */

class AsyncWebParameter
{
  private:
    String _name;
    String _value;
    size_t _size;
    bool _isForm;
    bool _isFile;

  public:

    AsyncWebParameter(const String& name, const String& value, bool form = false, bool file = false, size_t size = 0): _name(name), _value(value), _size(size), _isForm(form), _isFile(file)  {}

    /////////////////////////////////////////////////
    
    inline const String& name() const 
    {
      return _name;
    }

    /////////////////////////////////////////////////
    
    inline const String& value() const 
    {
      return _value;
    }

    /////////////////////////////////////////////////
    
    inline size_t size() const 
    {
      return _size;
    }

    /////////////////////////////////////////////////
    
    inline bool isPost() const 
    {
      return _isForm;
    }

    /////////////////////////////////////////////////
    
    inline bool isFile() const 
    {
      return _isFile;
    }

    /////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


/*
   HEADER :: Chainable object to hold the headers
 * */

class AsyncWebHeader
{
  private:
    String _name;
    String _value;

  public:
    AsyncWebHeader(const String& name, const String& value): _name(name), _value(value) {}

    /////////////////////////////////////////////////
    
    AsyncWebHeader(const String& data): _name(), _value() 
    {
      if (!data) 
        return;
        
      int index = data.indexOf(':');
      
      if (index < 0) 
        return;
        
      _name = data.substring(0, index);
      _value = data.substring(index + 2);
    }

    /////////////////////////////////////////////////
    
    ~AsyncWebHeader() {}

    /////////////////////////////////////////////////
    
    inline const String& name() const 
    {
      return _name;
    }

    /////////////////////////////////////////////////
    
    inline const String& value() const 
    {
      return _value;
    }

    /////////////////////////////////////////////////
    
    inline String toString() const 
    {
      return String(_name + ": " + _value + "\r\n");
    }

    /////////////////////////////////////////////////
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   REQUEST :: Each incoming Client is wrapped inside a Request and both live together until disconnect
 * */

typedef enum { RCT_NOT_USED = -1, RCT_DEFAULT = 0, RCT_HTTP, RCT_WS, RCT_EVENT, RCT_MAX } RequestedConnectionType;

typedef std::function<size_t(uint8_t*, size_t, size_t)> AwsResponseFiller;
typedef std::function<String(const String&)> AwsTemplateProcessor;

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

class AsyncWebServerRequest
{
    friend class AsyncWebServer;
    friend class AsyncCallbackWebHandler;

  private:
    AsyncClient* _client;
    AsyncWebServer* _server;
    AsyncWebHandler* _handler;
    AsyncWebServerResponse* _response;
    StringArray _interestingHeaders;
    ArDisconnectHandler _onDisconnectfn;

    String    _temp;
    uint8_t   _parseState;
    uint8_t   _version;

    WebRequestMethodComposite _method;

    String    _url;
    String    _host;
    String    _contentType;
    String    _boundary;
    String    _authorization;

    RequestedConnectionType _reqconntype;

    void _removeNotInterestingHeaders();

    bool      _isDigest;
    bool      _isMultipart;
    bool      _isPlainPost;
    bool      _expectingContinue;
    size_t    _contentLength;
    size_t    _parsedLength;

    LinkedList<AsyncWebHeader *> _headers;
    LinkedList<AsyncWebParameter *> _params;
    LinkedList<String *> _pathParams;

    uint8_t   _multiParseState;
    uint8_t   _boundaryPosition;
    size_t    _itemStartIndex;
    size_t    _itemSize;
    String    _itemName;
    String    _itemFilename;
    String    _itemType;
    String    _itemValue;
    uint8_t*  _itemBuffer;
    size_t    _itemBufferIndex;
    bool      _itemIsFile;

    void _onPoll();
    void _onAck(size_t len, uint32_t time);
    void _onError(int8_t error);
    void _onTimeout(uint32_t time);
    void _onDisconnect();
    void _onData(void *buf, size_t len);

    void _addParam(AsyncWebParameter*);
    void _addPathParam(const char *param);

    bool _parseReqHead();
    bool _parseReqHeader();
    void _parseLine();
    void _parsePlainPostChar(uint8_t data);
    void _parseMultipartPostByte(uint8_t data, bool last);
    void _addGetParams(const String& params);

    void _handleUploadStart();
    void _handleUploadByte(uint8_t data, bool last);
    void _handleUploadEnd();

  public:
    void *_tempObject;

    AsyncWebServerRequest(AsyncWebServer*, AsyncClient*);
    ~AsyncWebServerRequest();

    /////////////////////////////////////////////////

    inline AsyncClient* client() 
    {
      return _client;
    }

    /////////////////////////////////////////////////
    
    inline uint8_t version() const 
    {
      return _version;
    }

    /////////////////////////////////////////////////
    
    inline WebRequestMethodComposite method() const 
    {
      return _method;
    }

    /////////////////////////////////////////////////
    
    inline const String& url() const 
    {
      return _url;
    }

    /////////////////////////////////////////////////
    
    inline const String& host() const 
    {
      return _host;
    }

    /////////////////////////////////////////////////
    
    inline const String& contentType() const 
    {
      return _contentType;
    }

    /////////////////////////////////////////////////
    
    inline size_t contentLength() const 
    {
      return _contentLength;
    }

    /////////////////////////////////////////////////
    
    inline bool multipart() const 
    {
      return _isMultipart;
    }

    /////////////////////////////////////////////////
    
    const char * methodToString() const;
    const char * requestedConnTypeToString() const;

    /////////////////////////////////////////////////
    
    inline RequestedConnectionType requestedConnType() const 
    {
      return _reqconntype;
    }

    /////////////////////////////////////////////////

    bool isExpectedRequestedConnType(RequestedConnectionType erct1, RequestedConnectionType erct2 = RCT_NOT_USED, RequestedConnectionType erct3 = RCT_NOT_USED);
    void onDisconnect (ArDisconnectHandler fn);

    //hash is the string representation of:
    // base64(user:pass) for basic or
    // user:realm:md5(user:realm:pass) for digest
    bool authenticate(const char * hash);
    bool authenticate(const char * username, const char * password, const char * realm = NULL, bool passwordIsHash = false);
    void requestAuthentication(const char * realm = NULL, bool isDigest = true);

    /////////////////////////////////////////////////

    inline void setHandler(AsyncWebHandler *handler) 
    {
      _handler = handler;
    }

    /////////////////////////////////////////////////

    void addInterestingHeader(const String& name);

    void redirect(const String& url);

    void send(AsyncWebServerResponse *response);
    void send(int code, const String& contentType = String(), const String& content = String());
    void send(int code, const String& contentType, const char *content, bool nonCopyingSend = true);    // RSMOD

    void send(Stream &stream, const String& contentType, size_t len, AwsTemplateProcessor callback = nullptr);
    void send(const String& contentType, size_t len, AwsResponseFiller callback, AwsTemplateProcessor templateCallback = nullptr);
    void sendChunked(const String& contentType, AwsResponseFiller callback, AwsTemplateProcessor templateCallback = nullptr);

    AsyncWebServerResponse *beginResponse(int code, const String& contentType = String(), const String& content = String());
    AsyncWebServerResponse *beginResponse(int code, const String& contentType, const char * content = nullptr); // RSMOD
    
    // KH add
    AsyncWebServerResponse *beginResponse(int code, const String& contentType, const uint8_t * content, size_t len, 
                                          AwsTemplateProcessor callback = nullptr);
    //////
    
    AsyncWebServerResponse *beginResponse(Stream &stream, const String& contentType, size_t len, AwsTemplateProcessor callback = nullptr);
    AsyncWebServerResponse *beginResponse(const String& contentType, size_t len, AwsResponseFiller callback, 
                                          AwsTemplateProcessor templateCallback = nullptr);
    AsyncWebServerResponse *beginResponse_P(int code, const String& contentType, const uint8_t * content, size_t len, 
                                            AwsTemplateProcessor callback=nullptr);
    AsyncWebServerResponse *beginResponse_P(int code, const String& contentType, PGM_P content, 
                                            AwsTemplateProcessor callback=nullptr);

    AsyncWebServerResponse *beginChunkedResponse(const String& contentType, AwsResponseFiller callback, 
                                                  AwsTemplateProcessor templateCallback = nullptr);
    AsyncResponseStream *beginResponseStream(const String& contentType, size_t bufferSize = 1460);

    size_t headers() const;                     // get header count
    bool hasHeader(const String& name) const;   // check if header exists

    AsyncWebHeader* getHeader(const String& name) const;
    AsyncWebHeader* getHeader(size_t num) const;

    size_t params() const;                      // get arguments count
    bool hasParam(const String& name, bool post = false , bool file = false) const;

    AsyncWebParameter* getParam(const String& name, bool post = false, bool file = false) const;
    AsyncWebParameter* getParam(size_t num) const;

    /////////////////////////////////////////////////

    inline size_t args() const 
    {
      return params();  // get arguments count
    }

    /////////////////////////////////////////////////

    const String& arg(const String& name) const; // get request argument value by name
    const String& arg(size_t i) const;           // get request argument value by number
    const String& argName(size_t i) const;       // get request argument name by number
    bool hasArg(const char* name) const;         // check if argument exists

    const String& ASYNCWEBSERVER_REGEX_ATTRIBUTE pathArg(size_t i) const;

    const String& header(const char* name) const;// get request header value by name
    const String& header(size_t i) const;        // get request header value by number
    const String& headerName(size_t i) const;    // get request header name by number
    String urlDecode(const String& text) const;
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   FILTER :: Callback to filter AsyncWebRewrite and AsyncWebHandler (done by the Server)
 * */

typedef std::function<bool(AsyncWebServerRequest *request)> ArRequestFilterFunction;

bool ON_STA_FILTER(AsyncWebServerRequest *request);

bool ON_AP_FILTER(AsyncWebServerRequest *request);

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   REWRITE :: One instance can be handle any Request (done by the Server)
 * */

class AsyncWebRewrite
{
  protected:
    String _from;
    String _toUrl;
    String _params;
    ArRequestFilterFunction _filter;

  public:
  
    /////////////////////////////////////////////////
  
    AsyncWebRewrite(const char* from, const char* to): _from(from), _toUrl(to), _params(String()), _filter(NULL) 
    {
      int index = _toUrl.indexOf('?');
      
      if (index > 0) 
      {
        _params = _toUrl.substring(index + 1);
        _toUrl = _toUrl.substring(0, index);
      }
    }

    /////////////////////////////////////////////////
    
    virtual ~AsyncWebRewrite() {}

    /////////////////////////////////////////////////
    
    inline AsyncWebRewrite& setFilter(ArRequestFilterFunction fn) 
    {
      _filter = fn;
      return *this;
    }

    /////////////////////////////////////////////////
     
    inline bool filter(AsyncWebServerRequest *request) const 
    {
      return _filter == NULL || _filter(request);
    }

    /////////////////////////////////////////////////
     
    inline const String& from() const 
    {
      return _from;
    }

    /////////////////////////////////////////////////
     
    inline const String& toUrl() const 
    {
      return _toUrl;
    }

    /////////////////////////////////////////////////
     
    inline const String& params() const 
    {
      return _params;
    }

    /////////////////////////////////////////////////
     
    virtual bool match(AsyncWebServerRequest *request) 
    {
      return from() == request->url() && filter(request);
    }
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   HANDLER :: One instance can be attached to any Request (done by the Server)
 * */

class AsyncWebHandler
{
  protected:
    ArRequestFilterFunction _filter;
    String _username;
    String _password;

  public:
    AsyncWebHandler(): _username(""), _password("") {}

    /////////////////////////////////////////////////
     
    inline AsyncWebHandler& setFilter(ArRequestFilterFunction fn) 
    {
      _filter = fn;
      return *this;
    }

    /////////////////////////////////////////////////
     
    inline AsyncWebHandler& setAuthentication(const char *username, const char *password) 
    {
      _username = String(username);
      _password = String(password);
      return *this;
    };

    /////////////////////////////////////////////////
     
    inline bool filter(AsyncWebServerRequest *request) 
    {
      return _filter == NULL || _filter(request);
    }

    /////////////////////////////////////////////////
     
    virtual ~AsyncWebHandler() {}

    /////////////////////////////////////////////////
     
    virtual bool canHandle(AsyncWebServerRequest *request __attribute__((unused))) 
    {
      return false;
    }

    /////////////////////////////////////////////////

    virtual void handleRequest(AsyncWebServerRequest *request __attribute__((unused))) {}
    virtual void handleUpload(AsyncWebServerRequest *request  __attribute__((unused)), String filename __attribute__((unused)), 
                 size_t index __attribute__((unused)), uint8_t *data __attribute__((unused)), size_t len __attribute__((unused)), 
                 bool final  __attribute__((unused))) {}
    virtual void handleBody(AsyncWebServerRequest *request __attribute__((unused)), uint8_t *data __attribute__((unused)), 
                 size_t len __attribute__((unused)), size_t index __attribute__((unused)), size_t total __attribute__((unused))) {}

    /////////////////////////////////////////////////
     
    virtual bool isRequestHandlerTrivial() 
    {
      return true;
    }
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   RESPONSE :: One instance is created for each Request (attached by the Handler)
 * */

typedef enum
{
  RESPONSE_SETUP, RESPONSE_HEADERS, RESPONSE_CONTENT, RESPONSE_WAIT_ACK, RESPONSE_END, RESPONSE_FAILED
} WebResponseState;

/////////////////////////////////////////////////////////

class AsyncWebServerResponse
{
  protected:
    int _code;
    LinkedList<AsyncWebHeader *> _headers;
    String _contentType;
    size_t _contentLength;
    bool _sendContentLength;
    bool _chunked;
    size_t _headLength;
    size_t _sentLength;
    size_t _ackedLength;
    size_t _writtenLength;
    WebResponseState _state;
    const char* _responseCodeToString(int code);

  public:
    AsyncWebServerResponse();
    virtual ~AsyncWebServerResponse();
    virtual void setCode(int code);
    virtual void setContentLength(size_t len);
    virtual void setContentType(const String& type);
    virtual void addHeader(const String& name, const String& value);
    virtual String _assembleHead(uint8_t version);
    virtual bool _started() const;
    virtual bool _finished() const;
    virtual bool _failed() const;
    virtual bool _sourceValid() const;
    virtual void _respond(AsyncWebServerRequest *request);
    virtual size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t time);
};

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/*
   SERVER :: One instance
 * */

typedef std::function<void(AsyncWebServerRequest *request)> ArRequestHandlerFunction;
typedef std::function<void(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)> ArUploadHandlerFunction;
typedef std::function<void(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)> ArBodyHandlerFunction;

/////////////////////////////////////////////////////////

class AsyncWebServer
{
  protected:
    AsyncServer _server;
    LinkedList<AsyncWebRewrite*> _rewrites;
    LinkedList<AsyncWebHandler*> _handlers;
    AsyncCallbackWebHandler* _catchAllHandler;

  public:
    AsyncWebServer(uint16_t port);
    ~AsyncWebServer();

    void begin();
    void end();

#if ASYNC_TCP_SSL_ENABLED
    //void onSslFileRequest(AcSSlFileHandler cb, void* arg);
    //void beginSecure(const char *cert, const char *private_key_file, const char *password);
#endif

    AsyncWebRewrite& addRewrite(AsyncWebRewrite* rewrite);
    bool removeRewrite(AsyncWebRewrite* rewrite);
    AsyncWebRewrite& rewrite(const char* from, const char* to);

    AsyncWebHandler& addHandler(AsyncWebHandler* handler);
    bool removeHandler(AsyncWebHandler* handler);

    AsyncCallbackWebHandler& on(const char* uri, ArRequestHandlerFunction onRequest);
    AsyncCallbackWebHandler& on(const char* uri, WebRequestMethodComposite method, ArRequestHandlerFunction onRequest);
    AsyncCallbackWebHandler& on(const char* uri, WebRequestMethodComposite method, ArRequestHandlerFunction onRequest, ArUploadHandlerFunction onUpload);
    AsyncCallbackWebHandler& on(const char* uri, WebRequestMethodComposite method, ArRequestHandlerFunction onRequest, ArUploadHandlerFunction onUpload, ArBodyHandlerFunction onBody);

    void onNotFound(ArRequestHandlerFunction fn);  //called when handler is not assigned
    void onRequestBody(ArBodyHandlerFunction fn); //handle posts with plain body content (JSON often transmitted this way as a request)

    void reset(); //remove all writers and handlers, with onNotFound/onFileUpload/onRequestBody

    void _handleDisconnect(AsyncWebServerRequest *request);
    void _attachHandler(AsyncWebServerRequest *request);
    void _rewriteRequest(AsyncWebServerRequest *request);
};

/////////////////////////////////////////////////////////

class DefaultHeaders 
{
    using headers_t = LinkedList<AsyncWebHeader *>;
    headers_t _headers;

    /////////////////////////////////////////////////

    DefaultHeaders()
      : _headers(headers_t([](AsyncWebHeader * h) 
    {
      delete h;
    }))
    {}

    /////////////////////////////////////////////////
    
  public:
    using ConstIterator = headers_t::ConstIterator;

    /////////////////////////////////////////////////

    void addHeader(const String& name, const String& value) 
    {
      _headers.add(new AsyncWebHeader(name, value));
    }

    /////////////////////////////////////////////////

    inline ConstIterator begin() const 
    {
      return _headers.begin();
    }

    /////////////////////////////////////////////////
    
    inline ConstIterator end() const 
    {
      return _headers.end();
    }

    /////////////////////////////////////////////////

    DefaultHeaders(DefaultHeaders const &) = delete;
    DefaultHeaders &operator=(DefaultHeaders const &) = delete;

    /////////////////////////////////////////////////
    
    static DefaultHeaders &Instance() 
    {
      static DefaultHeaders instance;
      return instance;
    }
};

/////////////////////////////////////////////////////////

#include "AsyncWebResponseImpl_Teensy41.h"
#include "AsyncWebHandlerImpl_Teensy41.h"
#include "AsyncWebSocket_Teensy41.h"
#include "AsyncEventSource_Teensy41.h"

#endif /* _ASYNC_WEBSERVER_TEENSY41_HPP_ */