/****************************************************************************************************************************
  defines.h
  
  For ESP8266 with lwIP_5100, lwIP_5500 or lwIP_enc28j60 library
  
  AsyncUDP_Ethernet is a Async UDP library for the ESP8266 with lwIP_5100, lwIP_5500 or lwIP_enc28j60 library
  
  Based on and modified from ESPAsyncUDP Library (https://github.com/me-no-dev/ESPAsyncUDP)
  Built by Khoi Hoang https://github.com/khoih-prog/ASYNC_UDP_Ethernet
 *****************************************************************************************************************************/

#ifndef defines_h
#define defines_h

#if defined(ESP8266)
  #define LED_ON      LOW
  #define LED_OFF     HIGH
#else
  #error Only ESP8266
#endif  

#define _AWS_ETHERNET_LOGLEVEL_               1

//////////////////////////////////////////////////////////

#define USING_W5500         true
#define USING_W5100         false
#define USING_ENC28J60      false

#include <SPI.h>

#define CSPIN       16      // 5

#if USING_W5500
  #include "W5500lwIP.h"
  #define SHIELD_TYPE       "ESP8266_W5500 Ethernet"
  
  Wiznet5500lwIP eth(CSPIN); 
   
#elif USING_W5100
  #include <W5100lwIP.h>
  #define SHIELD_TYPE       "ESP8266_W5100 Ethernet"
  
  Wiznet5100lwIP eth(CSPIN);

#elif USING_ENC28J60
  #include <ENC28J60lwIP.h>
  #define SHIELD_TYPE       "ESP8266_ENC28J60 Ethernet"
  
  ENC28J60lwIP eth(CSPIN);
#else
  // default if none selected
  #include "W5500lwIP.h"

  Wiznet5500lwIP eth(CSPIN);
#endif

#include <WiFiClient.h> // WiFiClient (-> TCPClient)

using TCPClient = WiFiClient;

//////////////////////////////////////////////////////////

#define USING_DHCP        true

#if !USING_DHCP
  IPAddress localIP(192, 168, 2, 222);
  IPAddress gateway(192, 168, 2, 1);
  IPAddress netMask(255, 255, 255, 0);
#endif

#include <AsyncUDP_Ethernet.h>        // https://github.com/khoih-prog/AsyncUDP_Ethernet

#endif    //defines_h
