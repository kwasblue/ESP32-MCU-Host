#pragma once

// Toggle this per env or per build.
#ifndef ENABLE_DEBUG_LOGS
#define ENABLE_DEBUG_LOGS 1
#endif

#if ENABLE_DEBUG_LOGS
  #define DBG_PRINT(x)           Serial.print(x)
  #define DBG_PRINTLN(x)         Serial.println(x)
  #define DBG_PRINTF(fmt, ...)   Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(fmt, ...)
#endif
