#pragma once

// Serial output subsystem
// Toggle SERIAL_ENABLED to 0 to disable INFO output globally.
// ERROR output is always enabled regardless of this setting.

#define SERIAL_ENABLED 1

// ERROR_PRINT/LN: Always prints, regardless of SERIAL_ENABLED.
// Prefixed with "[ERROR] " for visibility.
#define ERROR_PRINT(...)              \
    do {                              \
        Serial.print(F("[ERROR] ")); \
        Serial.print(__VA_ARGS__);    \
    } while (0)
#define ERROR_PRINTLN(...)            \
    do {                              \
        Serial.print(F("[ERROR] ")); \
        Serial.println(__VA_ARGS__);  \
    } while (0)

// INFO_PRINT/LN: Only prints when SERIAL_ENABLED is 1.
// Use for debug/status messages that can be disabled.
#if SERIAL_ENABLED
#define INFO_PRINT(...) Serial.print(__VA_ARGS__)
#define INFO_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define INFO_PRINT(...) ((void)0)
#define INFO_PRINTLN(...) ((void)0)
#endif

// Legacy aliases
#define SERIAL_PRINT(...) INFO_PRINT(__VA_ARGS__)
#define SERIAL_PRINTLN(...) INFO_PRINTLN(__VA_ARGS__)

namespace serial {

    void init() {
#if SERIAL_ENABLED
        Serial.begin(115200);
#endif
    }

} // namespace serial
