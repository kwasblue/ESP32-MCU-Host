#pragma once
#include <cstdint>

// ---- minimal millis() stub ----
static uint32_t __test_millis = 0;
inline uint32_t millis() { return __test_millis; }
inline void test_set_millis(uint32_t v) { __test_millis = v; }
inline void test_advance_millis(uint32_t d) { __test_millis += d; }

// ---- debug macro stubs (so DBG_PRINT compiles on native) ----
#ifndef DBG_PRINT
  #define DBG_PRINT(x)      do { (void)(x); } while(0)
#endif
#ifndef DBG_PRINTLN
  #define DBG_PRINTLN(x)    do { (void)(x); } while(0)
#endif
#ifndef DBG_PRINTF
  #define DBG_PRINTF(...)   do { } while(0)
#endif
