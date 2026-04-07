#pragma once

// Lightweight error reporting for BLE command handlers.
// Provides macros to check read results and send error responses.

#include "subsystem_serial.h"

// Check a BLERequest::read() call. If it fails, log the error,
// send an error response back to the client, and return from the handler.
//
// Usage:
//   int32_t left, right;
//   BLE_CHECK_READ(req, req.read(left), "left");
//   BLE_CHECK_READ(req, req.read(right), "right");
//
#define BLE_CHECK_READ(req, expr, field_name)                          \
    do {                                                               \
        if (!(expr)) {                                                 \
            ERROR_PRINTLN(                                             \
                F("cmd " #field_name ": failed to read parameter")     \
            );                                                         \
            BLEResponse _err = (req).new_response();                   \
            _err.add_error("Failed to read param: " field_name);       \
            _err.end();                                                \
            return;                                                    \
        }                                                              \
    } while (0)

// Same as BLE_CHECK_READ but for read_str (checks >= 0 instead of bool).
#define BLE_CHECK_READ_STR(req, len_expr, field_name)                  \
    do {                                                               \
        if ((len_expr) < 0) {                                          \
            ERROR_PRINTLN(                                             \
                F("cmd " #field_name ": failed to read string")        \
            );                                                         \
            BLEResponse _err = (req).new_response();                   \
            _err.add_error("Failed to read string: " field_name);      \
            _err.end();                                                \
            return;                                                    \
        }                                                              \
    } while (0)
