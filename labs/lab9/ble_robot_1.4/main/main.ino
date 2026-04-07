
#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "subsystem_kalman.h"
#include "subsystem_angle_pid.h"
#include "subsystem_pid.h"
#include "subsystem_step.h"

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
    pid::init();
    step::init();
    angle_pid::init();

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
            motors::periodic();
            kalman::periodic();
            pid::periodic();
            step::periodic();
            angle_pid::periodic();
            ble::periodic();
            timer::periodic();
        }

        motors::methods::stop();
        INFO_PRINTLN(F("Disconnected"));
    }
}
