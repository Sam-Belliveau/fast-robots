
// Subsystem enable flags. Set to false to skip a subsystem's init/periodic.
// serial / timer / ble are mandatory and always run.
constexpr bool ENABLE_IMU       = true;
constexpr bool ENABLE_TOF       = true;
constexpr bool ENABLE_MOTORS    = true;
constexpr bool ENABLE_KALMAN    = false;
constexpr bool ENABLE_GAMEPAD   = false;
constexpr bool ENABLE_PID       = false;
constexpr bool ENABLE_ANGLE_PID = false;
constexpr bool ENABLE_STEP      = false;
constexpr bool ENABLE_STUNTS    = false;
constexpr bool ENABLE_MAPPING   = false;
constexpr bool ENABLE_DRIVE     = true;

#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "subsystem_kalman.h"
#include "subsystem_gamepad.h"
#include "subsystem_pid.h"
#include "subsystem_angle_pid.h"
#include "subsystem_step.h"
#include "subsystem_stunts.h"
#include "subsystem_mapping.h"
#include "subsystem_drive.h"

void setup() {
    serial::init();
    ble::init();

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    timer::init();
    if (ENABLE_IMU)       imu::init();
    if (ENABLE_TOF)       tof::init();
    if (ENABLE_MOTORS)    motors::init();
    if (ENABLE_KALMAN)    kalman::init();
    if (ENABLE_GAMEPAD)   gamepad::init();
    if (ENABLE_PID)       pid::init();
    if (ENABLE_ANGLE_PID) angle_pid::init();
    if (ENABLE_STEP)      step::init();
    if (ENABLE_STUNTS)    stunts::init();
    if (ENABLE_MAPPING)   mapping::init();
    if (ENABLE_DRIVE)     drive::init();

    INFO_PRINT(F("Commands registered: "));
    INFO_PRINTLN(ble::num_commands);
}

void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        INFO_PRINT(F("Connected to: "));
        INFO_PRINTLN(central.address());

        while (central.connected()) {
            if (ENABLE_IMU)       imu::periodic();
            if (ENABLE_TOF)       tof::periodic();
            if (ENABLE_KALMAN)    kalman::periodic();
            if (ENABLE_GAMEPAD)   gamepad::periodic();
            if (ENABLE_PID)       pid::periodic();
            if (ENABLE_ANGLE_PID) angle_pid::periodic();
            if (ENABLE_STEP)      step::periodic();
            if (ENABLE_STUNTS)    stunts::periodic();
            if (ENABLE_MAPPING)   mapping::periodic();
            if (ENABLE_DRIVE)     drive::periodic();
            if (ENABLE_MOTORS)    motors::periodic(); // last: sums all sources
            ble::periodic();
            timer::periodic();
        }

        if (ENABLE_MOTORS) motors::methods::stop();
        INFO_PRINTLN(F("Disconnected"));
    }
}
