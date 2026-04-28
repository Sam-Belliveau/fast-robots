#pragma once

// BLE subsystem
// Owns: BLE service/characteristics, command registry, core commands,
//       binary packet TX/RX, recording state, misc buffers.

#include "subsystem_serial.h"
#include "lib_BLEPacket.h"
#include "lib_Exceptions.h"
#include "lib_CircularBuffer.h"
#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE "1785129f-3b3a-4cf5-a01f-03668e8b12e9"
#define BLE_UUID_RX_CMD "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_DATA "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define MAX_COMMANDS 64

// CMD enum -- keep in sync with ble_python/cmd_types.py
enum CommandTypes {
    PING = 0,
    SEND_TWO_INTS = 1,
    SEND_THREE_FLOATS = 2,
    GET_TIME_MILLIS = 3,
    ECHO = 4,
    DANCE = 5,
    SET_VEL = 6,
    STORE_TIME_MILLIS = 7,
    SEND_TIME_MILLIS = 8,
    SEND_IMU_DATA = 9,
    SEND_TOF_DATA = 10,
    TOF_MODE = 11,
    TOF_LONG = 12, // unused
    TOF_STATS = 13,
    START_RECORDING = 14,
    STOP_RECORDING = 15,
    MOTOR_CMD = 16,
    MOTOR_STOP = 17,
    MOTOR_CAL = 18,
    MOTOR_TIMEOUT = 19,
    PID_START = 20,
    PID_STOP = 21,
    PID_SETPOINT = 22,
    PID_GAINS = 23,
    PID_PARAMS = 24,
    SEND_PID_DATA = 25,
    MOTOR_TEST = 26,
    KF_PARAMS = 27,
    KF_RESET = 28,
    SEND_KF_DATA = 29,
    ANGLE_PID_START = 30,
    ANGLE_PID_STOP = 31,
    ANGLE_PID_SETPOINT = 32,
    ANGLE_PID_GAINS = 33,
    ANGLE_PID_PARAMS = 34,
    SEND_ANGLE_PID_DATA = 35,
    STEP_RESPONSE = 36,
    SEND_STEP_DATA = 37,
    GET_AVG_HZ = 38,
    ANGLE_PID_SETPOINT_REL = 39,
    TOF_AUTO = 40, // unused
    STUNT_DRIFT = 41,
    STUNT_FLIP = 42,
    STUNT_STOP = 43,
    STUNT_PARAMS = 44,
    STUNT_YAW_GAINS = 45,
    SEND_STUNT_DATA = 46,
    MAP_START = 47,
    MAP_STOP = 48,
    MAP_PARAMS = 49,
    SEND_MAP_DATA = 50,
    MAP_STATUS = 51,
    DRIVE_UPDATE = 52,
    SET_HEADING = 53,
};

typedef void (*CommandHandler)(BLERequest &);

namespace ble {

    // Variables

    BLEService testService(BLE_UUID_TEST_SERVICE);

    BLECharacteristic rx_characteristic(
        BLE_UUID_RX_CMD, BLEWriteWithoutResponse, MAX_MSG_SIZE
    );

    BLECharacteristic
        tx_characteristic(BLE_UUID_TX_DATA, BLERead | BLENotify, MAX_MSG_SIZE);

    bool recording = false;

    CircularBuffer<int, 0x200> time_millis_buffer;
    CircularBuffer<float, 0x200> temp_buffer;

    CommandHandler command_handlers[MAX_COMMANDS] = {nullptr};
    int num_commands = 0;

    // Methods

    namespace methods {

        void register_command(int cmd_type, CommandHandler handler) {
            if (cmd_type >= 0 && cmd_type < MAX_COMMANDS) {
                command_handlers[cmd_type] = handler;
                if (cmd_type >= num_commands) {
                    num_commands = cmd_type + 1;
                }
            }
        }

        void dispatch_command(int cmd_type, BLERequest &req) {
            if (cmd_type < 0 || cmd_type >= MAX_COMMANDS) {
                ERROR_PRINT(F("Unknown command type: "));
                ERROR_PRINTLN(cmd_type);
                BLEResponse err = req.new_response();
                err.add_error("Unknown command type");
                err.end();
                return;
            }
            if (command_handlers[cmd_type] == nullptr) {
                ERROR_PRINT(F("Unregistered command type: "));
                ERROR_PRINTLN(cmd_type);
                BLEResponse err = req.new_response();
                err.add_error("Unregistered command");
                err.end();
                return;
            }
            command_handlers[cmd_type](req);
        }

        void handle_command() {
            int len = rx_characteristic.valueLength();
            if (len < 3) {
                ERROR_PRINT(F("Packet too short: "));
                ERROR_PRINT(len);
                ERROR_PRINTLN(F(" bytes (need at least 3)"));
                return;
            }
            BLERequest req(rx_characteristic.value(), len);
            INFO_PRINT(F("CMD "));
            INFO_PRINT(req.cmd);
            INFO_PRINT(F(" (req_id="));
            INFO_PRINT(req.req_id);
            INFO_PRINT(F(", len="));
            INFO_PRINT(len);
            INFO_PRINTLN(F(")"));
            dispatch_command(req.cmd, req);
        }

    } // namespace methods

    // Commands

    namespace commands {

        void get_avg_hz(BLERequest &req) {
            float hz = timer::methods::avg_hz();
            BLEResponse res = req.new_response();
            res.add(hz);
            res.end();
            INFO_PRINT(F("Avg Hz: "));
            INFO_PRINTLN(hz);
        }

        void ping(BLERequest &req) {
            BLEResponse res = req.new_response();
            res.add("PONG");
            res.end();
            INFO_PRINTLN(F("PING -> PONG"));
        }

        void send_two_ints(BLERequest &req) {
            int32_t int_a, int_b;
            BLE_CHECK_READ(req, req.read(int_a), "int_a");
            BLE_CHECK_READ(req, req.read(int_b), "int_b");
            INFO_PRINT(F("Two Integers: "));
            INFO_PRINT(int_a);
            INFO_PRINT(F(", "));
            INFO_PRINTLN(int_b);
            req.new_response().end();
        }

        void send_three_floats(BLERequest &req) {
            float float_a, float_b, float_c;
            BLE_CHECK_READ(req, req.read(float_a), "float_a");
            BLE_CHECK_READ(req, req.read(float_b), "float_b");
            BLE_CHECK_READ(req, req.read(float_c), "float_c");
            INFO_PRINT(F("Three Floats: "));
            INFO_PRINT(float_a);
            INFO_PRINT(F(", "));
            INFO_PRINT(float_b);
            INFO_PRINT(F(", "));
            INFO_PRINTLN(float_c);
            req.new_response().end();
        }

        void get_time_millis(BLERequest &req) {
            BLEResponse res = req.new_response();
            res.add((int32_t)millis());
            res.add(getTempDegF());
            res.end();
        }

        void echo(BLERequest &req) {
            char char_arr[MAX_MSG_SIZE];
            BLE_CHECK_READ_STR(req, req.read_str(char_arr), "msg");
            BLEResponse res = req.new_response();
            res.add("Robot Says -> ");
            res.add(char_arr);
            res.end();
            INFO_PRINT(F("Echo: "));
            INFO_PRINTLN(char_arr);
        }

        void dance(BLERequest &req) {
            INFO_PRINTLN(F("Look Ma, I'm Dancin'!"));
            req.new_response().end();
        }

        void set_vel(BLERequest &req) {
            req.new_response().end();
        }

        void store_time_millis(BLERequest &req) {
            int32_t count;
            BLE_CHECK_READ(req, req.read(count), "count");
            for (int i = 0; i < count; ++i) {
                time_millis_buffer.push((int)millis());
                temp_buffer.push(getTempDegF());
            }
            req.new_response().end();
        }

        void send_time_millis(BLERequest &req) {
            BLEResponse res = req.new_response();
            int time_millis;
            float temp;
            while (time_millis_buffer.pop(time_millis) &&
                   temp_buffer.pop(temp)) {
                res.add((int32_t)time_millis);
                res.add(temp);
            }
            res.end();
        }

        void start_recording(BLERequest &req) {
            recording = true;
            INFO_PRINTLN(F("Recording started"));
            req.new_response().end();
        }

        void stop_recording(BLERequest &req) {
            recording = false;
            INFO_PRINTLN(F("Recording stopped"));
            req.new_response().end();
        }

    } // namespace commands

    // Init

    void init() {
        BLE.begin();

        BLE.setDeviceName("Artemis BLE");
        BLE.setLocalName("Artemis BLE");
        BLE.setAdvertisedService(testService);

        testService.addCharacteristic(tx_characteristic);
        testService.addCharacteristic(rx_characteristic);

        BLE.addService(testService);

        INFO_PRINT(F("Advertising BLE with MAC: "));
        INFO_PRINTLN(BLE.address());

        BLE.advertise();

        // Register core commands
        methods::register_command(PING, commands::ping);
        methods::register_command(SEND_TWO_INTS, commands::send_two_ints);
        methods::register_command(
            SEND_THREE_FLOATS, commands::send_three_floats
        );
        methods::register_command(GET_TIME_MILLIS, commands::get_time_millis);
        methods::register_command(ECHO, commands::echo);
        methods::register_command(DANCE, commands::dance);
        methods::register_command(SET_VEL, commands::set_vel);
        methods::register_command(
            STORE_TIME_MILLIS, commands::store_time_millis
        );
        methods::register_command(SEND_TIME_MILLIS, commands::send_time_millis);
        methods::register_command(START_RECORDING, commands::start_recording);
        methods::register_command(STOP_RECORDING, commands::stop_recording);
        methods::register_command(GET_AVG_HZ, commands::get_avg_hz);
    }

    // Periodic

    void periodic() {
        if (rx_characteristic.written()) {
            methods::handle_command();
        }
    }

} // namespace ble

// Definition of ble_send_raw (declared in lib_BLEPacket.h).
void ble_send_raw(const uint8_t *data, uint8_t len) {
    ble::tx_characteristic.writeValue(data, len);
}
