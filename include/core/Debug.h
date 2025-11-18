#pragma once

// Toggle this per env or per build.
// Default: OFF (0) so logs compile out unless explicitly enabled.
#ifndef ENABLE_DEBUG_LOGS
#define ENABLE_DEBUG_LOGS 0
#endif

#if ENABLE_DEBUG_LOGS
  #define DBG_PRINT(x)           Serial.print(x)
  #define DBG_PRINTLN(x)         Serial.println(x)
  #define DBG_PRINTF(fmt, ...)   Serial.printf(fmt, ##__VA_ARGS__)
#else
  // compile-time no-ops so strings/calls get stripped
  #define DBG_PRINT(x)           do {} while (0)
  #define DBG_PRINTLN(x)         do {} while (0)
  #define DBG_PRINTF(fmt, ...)   do {} while (0)
#endif
