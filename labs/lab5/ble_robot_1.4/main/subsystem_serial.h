#pragma once

// Serial output subsystem
// Toggle SERIAL_ENABLED to 0 to disable all serial output globally.

#define SERIAL_ENABLED 1

#if SERIAL_ENABLED
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...) ((void)0)
#define SERIAL_PRINTLN(...) ((void)0)
#endif

namespace serial {

    void init() {
#if SERIAL_ENABLED
        Serial.begin(115200);
#endif
    }

} // namespace serial
