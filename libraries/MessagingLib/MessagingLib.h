#ifndef MESSAGING_LIB_H
#define MESSAGING_LIB_H

#include <Arduino.h>

#if defined(__AVR__)
  #include <avr/pgmspace.h>

#elif defined(ESP8266)

#endif


#include "IDeserializableMessageFactory.h"
#include "SerializableMessage.h"
#include "MessageListener.h"
#include "DataReader.h"
#include "MessageSender.h"

#ifndef STATIC_MAX
  #define STATIC_MAX(a, b)  ((a) > (b) ? (a) : (b))
#endif


// #ifndef MESSAGING_DEBUG_ENABLED
//   #define MESSAGING_DEBUG_ENABLED 0
// #endif


// #if not MESSAGING_DEBUG_ENABLED
//   #ifndef debugInit
//     #define debugInit(m)  ;
//     #define debugln(m) ;
//     #define debuglnbin(m) ;
//     #define debughex(m) ;
//     #define debug(m)  ;
//     #define debugbuf(m,i) ;
//   #endif
// #else
//   #define debugInit(m)  Serial.begin(m); while (!Serial) ;
//   #define debugln(m)  Serial.println(m);
//   #define debuglnbin(m)  Serial.println(m, BIN);
//   #define debughex(m)  Serial.print(m, HEX);
//   #define debug(m)    Serial.print(m);
//   #define debugbuf(m,i)  Serial.write(m,(int)i);
// #endif

#endif