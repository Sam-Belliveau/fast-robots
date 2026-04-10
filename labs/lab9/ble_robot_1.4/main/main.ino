
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

void setup() {
    serial::init();
    ble::init();

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    timer::init();
    imu::init();
    tof::init();
    motors::init();
    kalman::init();
    gamepad::init();
    pid::init();
    angle_pid::init();
    step::init();
    stunts::init();
    mapping::init();

    INFO_PRINT(F("Commands registered: "));
    INFO_PRINTLN(ble::num_commands);
}

void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        INFO_PRINT(F("Connected to: "));
        INFO_PRINTLN(central.address());

        while (central.connected()) {
            imu::periodic();
            tof::periodic();
            kalman::periodic();
            gamepad::periodic();
            pid::periodic();
            angle_pid::periodic();
            step::periodic();
            stunts::periodic();
            mapping::periodic();
            motors::periodic(); // last: sums all sources
            ble::periodic();
            timer::periodic();
        }

        motors::methods::stop();
        INFO_PRINTLN(F("Disconnected"));
    }
}
