#pragma once

// Binary BLE request/response protocol.
// Self-describing fields: each value is prefixed with a DataType tag byte.
// Packet format: [u16 req_id] [DT tag, value]... [DT_END]

#include <cstring>

#define MAX_MSG_SIZE 151

enum DataType : uint8_t {
    // Strings
    DT_STR = 0x00,

    // Signed Integers
    DT_I8 = 0x10,
    DT_I16 = 0x11,
    DT_I32 = 0x12,

    // Unsigned Integers
    DT_U8 = 0x20,
    DT_U16 = 0x21,
    DT_U32 = 0x22,

    // Floats
    DT_F32 = 0x30,

    // Special
    DT_END = 0xFF,
};

template <typename T> class DataTypeToID {};

#define DEFINE_TYPE_MAPPING(type, id)                                          \
    template <> struct DataTypeToID<type> {                                    \
        static const DataType value = id;                                      \
    };

DEFINE_TYPE_MAPPING(int8_t, DT_I8)
DEFINE_TYPE_MAPPING(int16_t, DT_I16)
DEFINE_TYPE_MAPPING(int32_t, DT_I32)
DEFINE_TYPE_MAPPING(uint8_t, DT_U8)
DEFINE_TYPE_MAPPING(uint16_t, DT_U16)
DEFINE_TYPE_MAPPING(uint32_t, DT_U32)
DEFINE_TYPE_MAPPING(float, DT_F32)

#undef DEFINE_TYPE_MAPPING

// Defined in subsystem_ble.h. Sends raw bytes over the TX characteristic.
void ble_send_raw(const uint8_t *data, uint8_t len);

// TX: builds and sends self-describing binary packets.
struct BLEResponse {
    uint8_t len;
    uint8_t buf[MAX_MSG_SIZE];

    // Numeric types: tag is resolved via DataTypeToID<T>.
    template <typename T, DataType id = DataTypeToID<T>::value>
    void add(const T &v) {
        buf[len++] = id;
        memcpy(buf + len, &v, sizeof(T));
        len += sizeof(T);
    }

    // String specialization: DT_STR + u8 length + chars.
    void add(const char *s) {
        uint8_t slen = strlen(s);
        buf[len++] = DT_STR;
        buf[len++] = slen;
        memcpy(buf + len, s, slen);
        len += slen;
    }

    // Send the current packet and reset for the next one (same req_id).
    void flush() {
        ble_send_raw(buf, len);
        len = 2; // keep req_id prefix
    }

    // Send a DT_END marker and flush. Call once at the end of a response.
    void end() {
        buf[len++] = DT_END;
        flush();
    }

  private:
    friend struct BLERequest;

    BLEResponse(uint16_t req_id) : len(0) {
        memcpy(buf, &req_id, 2);
        len = 2;
    }
};

template <typename T> T read_from_buffer(const uint8_t *data) {
    T v;
    memcpy(&v, data, sizeof(T));
    return v;
}

// RX: reads sequential typed values from a raw byte buffer.
// Constructor parses the header (req_id + cmd_id). Then call read<T>()
// for each argument.
struct BLERequest {
    const uint8_t *const data;
    const int total_len;
    const uint16_t req_id;
    const uint8_t cmd;
    mutable int pos;

    BLERequest(const uint8_t *d, int l)
        : data(d), total_len(l), req_id(read_from_buffer<uint16_t>(data)),
          cmd(d[2]), pos(3) {}

    // Create a response packet bound to this request's req_id.
    BLEResponse new_response() const {
        return BLEResponse(req_id);
    }

    // Numeric types: verify the type tag matches, then read sizeof(T) bytes.
    // Returns false on type mismatch or insufficient data.
    template <typename T, DataType id = DataTypeToID<T>::value>
    bool read(T &out) const {
        if (pos + 1 + (int)sizeof(T) > total_len)
            return false;
        if ((DataType)data[pos++] != id)
            return false;
        memcpy(&out, data + pos, sizeof(T));
        pos += sizeof(T);
        return true;
    }

    // String: verify DT_STR tag, read u8 length, copy chars.
    // Returns length of string, or -1 on failure.
    int read_str(char *out) const {
        if (pos + 2 > total_len)
            return -1;
        if ((DataType)data[pos++] != DT_STR)
            return -1;
        uint8_t slen = data[pos++];
        if (pos + slen > total_len)
            return -1;
        memcpy(out, data + pos, slen);
        out[slen] = '\0';
        pos += slen;
        return slen;
    }
};
