#pragma once

// IMU subsystem
// Owns: ICM-20948 sensor, IMU data buffers, init and periodic read.

#include "subsystem_serial.h"
#include "subsystem_ble.h"
#include "ICM_20948.h"
#include "lib_CircularBuffer.h"
#include "lib_Zip.h"

#define WIRE_PORT Wire
#define AD0_VAL 1
#define SPI_PORT SPI
#define CS_PIN 2

namespace imu {

    // Variables

    ICM_20948_I2C myICM;
    bool ok = false;

    CircularBuffer<int, 0x100> times;
    CircularBuffer<float, 0x100> acc_x;
    CircularBuffer<float, 0x100> acc_y;
    CircularBuffer<float, 0x100> acc_z;
    CircularBuffer<float, 0x100> gyr_x;
    CircularBuffer<float, 0x100> gyr_y;
    CircularBuffer<float, 0x100> gyr_z;
    CircularBuffer<float, 0x100> mag_x;
    CircularBuffer<float, 0x100> mag_y;
    CircularBuffer<float, 0x100> mag_z;

    // Methods

    namespace methods {

        void read() {
            if (!ok)
                return;
            const int time = micros();
            const int last_time = times.is_empty() ? 0 : times.bottom();
            if (time - last_time > 20000) {
                if (myICM.dataReady()) {
                    myICM.getAGMT();
                    times.push(time);
                    acc_x.push(myICM.accX());
                    acc_y.push(myICM.accY());
                    acc_z.push(myICM.accZ());
                    gyr_x.push(myICM.gyrX());
                    gyr_y.push(myICM.gyrY());
                    gyr_z.push(myICM.gyrZ());
                    mag_x.push(myICM.magX());
                    mag_y.push(myICM.magY());
                    mag_z.push(myICM.magZ());
                }
            }
        }

    } // namespace methods

    // Commands

    namespace commands {

        void send_data() {
            zip(
                [&](int t,
                    float ax,
                    float ay,
                    float az,
                    float gx,
                    float gy,
                    float gz,
                    float mx,
                    float my,
                    float mz) {
                    ble::tx_estring_value.clear();
                    ble::tx_estring_value.append("I:");
                    ble::tx_estring_value.append(t);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(ax);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(ay);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(az);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(gx);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(gy);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(gz);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(mx);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(my);
                    ble::tx_estring_value.append("|");
                    ble::tx_estring_value.append(mz);
                    ble::tx_characteristic_string.writeValue(ble::tx_estring_value.c_str());
                    delay(1);
                },
                times.begin(),
                times.end(),
                acc_x.begin(),
                acc_y.begin(),
                acc_z.begin(),
                gyr_x.begin(),
                gyr_y.begin(),
                gyr_z.begin(),
                mag_x.begin(),
                mag_y.begin(),
                mag_z.begin()
            );

            ble::tx_estring_value.clear();
            ble::tx_estring_value.append("END");
            ble::tx_characteristic_string.writeValue(ble::tx_estring_value.c_str());

            SERIAL_PRINT(F("SEND_IMU_DATA: "));
            SERIAL_PRINT(times.size());
            SERIAL_PRINTLN(F(" samples"));
        }

    } // namespace commands

    // Init

    void init() {
        bool initialized = false;
        while (!initialized) {
            myICM.begin(WIRE_PORT, AD0_VAL);
            SERIAL_PRINT(F("Initialization of the sensor returned: "));
            SERIAL_PRINTLN(myICM.statusString());
            if (myICM.status != ICM_20948_Stat_Ok) {
                SERIAL_PRINTLN(F("Trying again..."));
                delay(500);
            } else {
                initialized = true;
                ok = true;
            }
        }

        ble::methods::register_command(SEND_IMU_DATA, commands::send_data);
    }

    // Periodic

    void periodic() {
        methods::read();
    }

} // namespace imu
