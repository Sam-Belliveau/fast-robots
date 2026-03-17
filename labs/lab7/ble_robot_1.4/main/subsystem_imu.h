#pragma once

// IMU subsystem
// Owns: ICM-20948 sensor, DMP, IMU data buffers, init and periodic read.

#include "subsystem_serial.h"
#include "subsystem_ble.h"

// ICM_20948_USE_DMP is enabled in the library's ICM_20948_C.h
#include "ICM_20948.h"

#include "lib_CircularBuffer.h"
#include "lib_Zip.h"
#include "math.h"

#define WIRE_PORT Wire
#define AD0_VAL 1
#define SPI_PORT SPI
#define CS_PIN 2

namespace imu {

    // Variables

    ICM_20948_I2C myICM;
    bool ok = false;
    bool dmp_ok = false;

    float yaw = 0;         // current DMP yaw in degrees
    bool yaw_valid = false; // true when a new DMP reading arrived

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
    CircularBuffer<float, 0x100> yaw_buf;

    // Methods

    namespace methods {

        void read_dmp() {
            if (!dmp_ok)
                return;

            icm_20948_DMP_data_t data;
            myICM.readDMPdataFromFIFO(&data);

            if ((myICM.status == ICM_20948_Stat_Ok ||
                 myICM.status ==
                     ICM_20948_Stat_FIFOMoreDataAvail) &&
                (data.header & DMP_header_bitmap_Quat6) > 0) {

                float q1 = (float)data.Quat6.Data.Q1 / 1073741824.0f;
                float q2 = (float)data.Quat6.Data.Q2 / 1073741824.0f;
                float q3 = (float)data.Quat6.Data.Q3 / 1073741824.0f;
                float q0 =
                    sqrtf(1.0f - (q1 * q1) - (q2 * q2) - (q3 * q3));

                float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
                float cosy_cosp =
                    1.0f - 2.0f * (q2 * q2 + q3 * q3);
                yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f /
                      (float)M_PI;
                yaw_valid = true;
            } else {
                yaw_valid = false;
            }
        }

        void read() {
            if (!ok)
                return;

            if (myICM.dataReady()) {
                myICM.getAGMT();
                times.push(timer::methods::time_us());
                acc_x.push(myICM.accX());
                acc_y.push(myICM.accY());
                acc_z.push(myICM.accZ());
                gyr_x.push(myICM.gyrX());
                gyr_y.push(myICM.gyrY());
                gyr_z.push(myICM.gyrZ());
                mag_x.push(myICM.magX());
                mag_y.push(myICM.magY());
                mag_z.push(myICM.magZ());
                yaw_buf.push(yaw);
            }
        }

        void init_dmp() {
            ICM_20948_Status_e st;

            st = myICM.initializeDMP();
            SERIAL_PRINT(F("DMP initializeDMP: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.enableDMPSensor(
                INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR
            );
            SERIAL_PRINT(F("DMP enableSensor: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);
            SERIAL_PRINT(F("DMP setODR: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.enableFIFO();
            SERIAL_PRINT(F("DMP enableFIFO: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.enableDMP();
            SERIAL_PRINT(F("DMP enableDMP: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.resetDMP();
            SERIAL_PRINT(F("DMP resetDMP: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.resetFIFO();
            SERIAL_PRINT(F("DMP resetFIFO: "));
            SERIAL_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            dmp_ok = true;
            SERIAL_PRINTLN(F("DMP initialized OK"));
        }

    } // namespace methods

    // Commands

    namespace commands {

        void send_data(BLERequest &req) {
            BLEResponse res = req.new_response();
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
                    float mz,
                    float y) {
                    res.add((int32_t)t);
                    res.add(ax);
                    res.add(ay);
                    res.add(az);
                    res.add(gx);
                    res.add(gy);
                    res.add(gz);
                    res.add(mx);
                    res.add(my);
                    res.add(mz);
                    res.add(y);
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
                mag_z.begin(),
                yaw_buf.begin()
            );

            res.end();

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

        methods::init_dmp();

        ble::methods::register_command(SEND_IMU_DATA, commands::send_data);
    }

    // Periodic

    void periodic() {
        methods::read_dmp();
        methods::read();
    }

} // namespace imu
