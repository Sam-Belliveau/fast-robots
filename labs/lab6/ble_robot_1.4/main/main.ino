
#include "subsystem_serial.h"
#include "subsystem_timer.h"
#include "subsystem_ble.h"
#include "subsystem_imu.h"
#include "subsystem_tof.h"
#include "subsystem_motors.h"
#include "subsystem_pid.h"
#include "subsystem_kalman.h"
#include "subsystem_angle_pid.h"

void setup() {
    serial::init();
    ble::init();

    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);

    timer::init();
    imu::init();
    tof::init();
    motors::init();
    pid::init();
    angle_pid::init();
    kalman::init();

    SERIAL_PRINT(F("Commands registered: "));
    SERIAL_PRINTLN(ble::num_commands);
}

void loop() {
    BLEDevice central = BLE.central();

    if (central) {
        SERIAL_PRINT(F("Connected to: "));
        SERIAL_PRINTLN(central.address());

        while (central.connected()) {
            imu::periodic();
            tof::periodic();
            motors::periodic();
            pid::periodic();
            kalman::periodic();
            angle_pid::periodic();
            ble::periodic();
            timer::periodic();
        }

        motors::methods::stop();
        SERIAL_PRINTLN(F("Disconnected"));
    }
}
