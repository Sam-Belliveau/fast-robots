#pragma once

// BLE subsystem
// Owns: BLE service/characteristics, command registry, core commands,
//       robot_cmd, tx_estring_value, recording state, misc buffers.

#include "subsystem_serial.h"
#include "cornell_BLECStringCharacteristic.h"
#include "cornell_EString.h"
#include "cornell_RobotCommand.h"
#include "lib_CircularBuffer.h"
#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE "1785129f-3b3a-4cf5-a01f-03668e8b12e9"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define MAX_COMMANDS 32

// CMD enum — keep in sync with ble_python/cmd_types.py
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
    TOF_SHORT = 11,
    TOF_LONG = 12,
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
};

typedef void (*CommandHandler)();

namespace ble {

    // Variables

    BLEService testService(BLE_UUID_TEST_SERVICE);

    BLECStringCharacteristic
        rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

    BLEFloatCharacteristic
        tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);

    BLECStringCharacteristic tx_characteristic_string(
        BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE
    );

    RobotCommand robot_cmd(":|");
    EString tx_estring_value;
    float tx_float_value = 0.0;

    long interval = 500;
    static long previousMillis = 0;
    unsigned long currentMillis = 0;

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

        void dispatch_command(int cmd_type) {
            if (cmd_type >= 0 && cmd_type < MAX_COMMANDS &&
                command_handlers[cmd_type] != nullptr) {
                command_handlers[cmd_type]();
            } else {
                SERIAL_PRINT(F("Invalid Command Type: "));
                SERIAL_PRINTLN(cmd_type);
            }
        }

        void handle_command() {
            robot_cmd.set_cmd_string(
                rx_characteristic_string.value(),
                rx_characteristic_string.valueLength()
            );
            int cmd_type = -1;
            if (!robot_cmd.get_command_type(cmd_type))
                return;
            dispatch_command(cmd_type);
        }

    } // namespace methods

    // Commands

    namespace commands {

        void ping() {
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            SERIAL_PRINT(F("Sent back: "));
            SERIAL_PRINTLN(tx_estring_value.c_str());
        }

        void send_two_ints() {
            int int_a, int_b;
            if (!robot_cmd.get_next_value(int_a))
                return;
            if (!robot_cmd.get_next_value(int_b))
                return;
            SERIAL_PRINT(F("Two Integers: "));
            SERIAL_PRINT(int_a);
            SERIAL_PRINT(F(", "));
            SERIAL_PRINTLN(int_b);
        }

        void send_three_floats() {
            float float_a, float_b, float_c;
            if (!robot_cmd.get_next_value(float_a))
                return;
            if (!robot_cmd.get_next_value(float_b))
                return;
            if (!robot_cmd.get_next_value(float_c))
                return;
            SERIAL_PRINT(F("Three Floats: "));
            SERIAL_PRINT(float_a);
            SERIAL_PRINT(F(", "));
            SERIAL_PRINT(float_b);
            SERIAL_PRINT(F(", "));
            SERIAL_PRINTLN(float_c);
        }

        void get_time_millis() {
            int time_millis = millis();
            float temp = getTempDegF();
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(time_millis);
            tx_estring_value.append("|");
            tx_estring_value.append(temp);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            SERIAL_PRINT(F("Sent back: "));
            SERIAL_PRINTLN(tx_estring_value.c_str());
        }

        void echo() {
            char char_arr[MAX_MSG_SIZE];
            if (!robot_cmd.get_next_value(char_arr))
                return;
            tx_estring_value.clear();
            tx_estring_value.append("Robot Says -> ");
            tx_estring_value.append(char_arr);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            SERIAL_PRINT(F("Sent back: "));
            SERIAL_PRINTLN(tx_estring_value.c_str());
        }

        void dance() {
            SERIAL_PRINTLN(F("Look Ma, I'm Dancin'!"));
        }

        void set_vel() { /* placeholder */ }

        void store_time_millis() {
            int count;
            if (!robot_cmd.get_next_value(count))
                return;
            for (int i = 0; i < count; ++i) {
                time_millis_buffer.push((int)millis());
                temp_buffer.push(getTempDegF());
            }
        }

        void send_time_millis() {
            int time_millis;
            float temp;
            while (time_millis_buffer.pop(time_millis) &&
                   temp_buffer.pop(temp)) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(time_millis);
                tx_estring_value.append("|");
                tx_estring_value.append(temp);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                SERIAL_PRINT(F("Sent back: "));
                SERIAL_PRINTLN(tx_estring_value.c_str());
            }
        }

        void start_recording() {
            recording = true;
            SERIAL_PRINTLN(F("Recording started"));
        }

        void stop_recording() {
            recording = false;
            SERIAL_PRINTLN(F("Recording stopped"));
        }

    } // namespace commands

    // Init

    void init() {
        BLE.begin();

        BLE.setDeviceName("Artemis BLE");
        BLE.setLocalName("Artemis BLE");
        BLE.setAdvertisedService(testService);

        testService.addCharacteristic(tx_characteristic_float);
        testService.addCharacteristic(tx_characteristic_string);
        testService.addCharacteristic(rx_characteristic_string);

        BLE.addService(testService);

        tx_characteristic_float.writeValue(0.0);

        tx_estring_value.clear();
        tx_estring_value.append("[->");
        tx_estring_value.append(9.0);
        tx_estring_value.append("<-]");
        tx_characteristic_string.writeValue(tx_estring_value.c_str());

        SERIAL_PRINT(F("Advertising BLE with MAC: "));
        SERIAL_PRINTLN(BLE.address());

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
    }

    // Periodic

    void periodic() {
        // Write float data
        currentMillis = millis();
        if (currentMillis - previousMillis > interval) {
            tx_float_value = tx_float_value + 0.5;
            tx_characteristic_float.writeValue(tx_float_value);
            if (tx_float_value > 10000) {
                tx_float_value = 0;
            }
            previousMillis = currentMillis;
        }

        // Read and dispatch commands
        if (rx_characteristic_string.written()) {
            methods::handle_command();
        }
    }

} // namespace ble
