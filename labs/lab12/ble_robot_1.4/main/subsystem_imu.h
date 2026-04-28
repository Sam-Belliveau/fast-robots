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

    float yaw = 0;          // current DMP yaw in degrees, post-offset
    float raw_yaw = 0;      // raw quaternion-derived yaw before offset
    float yaw_offset = 0;   // added to raw_yaw to align with world frame
    bool yaw_valid = false; // true when a new DMP reading arrived

    // Orientation from accelerometer Z with hysteresis
    enum Orientation { RIGHT_SIDE_UP, IN_BETWEEN, UPSIDE_DOWN };
    Orientation orientation = RIGHT_SIDE_UP;

    // Thresholds in mg (ICM-20948 accZ is in mg)
    constexpr float UP_THRESH = 500.0f;
    constexpr float DOWN_THRESH = -500.0f;
    constexpr float HYST = 200.0f;

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
                 myICM.status == ICM_20948_Stat_FIFOMoreDataAvail) &&
                (data.header & DMP_header_bitmap_Quat6) > 0) {

                float q1 = (float)data.Quat6.Data.Q1 / 1073741824.0f;
                float q2 = (float)data.Quat6.Data.Q2 / 1073741824.0f;
                float q3 = (float)data.Quat6.Data.Q3 / 1073741824.0f;
                float q0 = sqrtf(1.0f - (q1 * q1) - (q2 * q2) - (q3 * q3));

                float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
                float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
                raw_yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / (float)M_PI;
                float y = raw_yaw + yaw_offset;
                y -= 360.0f * roundf(y / 360.0f);
                yaw = y;
                yaw_valid = true;
            } else {
                yaw_valid = false;
            }
        }

        void update_orientation(float az) {
            switch (orientation) {
            case RIGHT_SIDE_UP:
                if (az < UP_THRESH - HYST)
                    orientation = IN_BETWEEN;
                break;
            case IN_BETWEEN:
                if (az > UP_THRESH + HYST)
                    orientation = RIGHT_SIDE_UP;
                else if (az < DOWN_THRESH - HYST)
                    orientation = UPSIDE_DOWN;
                break;
            case UPSIDE_DOWN:
                if (az > DOWN_THRESH + HYST)
                    orientation = IN_BETWEEN;
                break;
            }
        }

        void read() {
            if (!ok)
                return;

            if (myICM.dataReady()) {
                myICM.getAGMT();
                update_orientation(myICM.accZ());
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
            INFO_PRINT(F("DMP initializeDMP: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st =
                myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
            INFO_PRINT(F("DMP enableSensor: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);
            INFO_PRINT(F("DMP setODR: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.enableFIFO();
            INFO_PRINT(F("DMP enableFIFO: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.enableDMP();
            INFO_PRINT(F("DMP enableDMP: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.resetDMP();
            INFO_PRINT(F("DMP resetDMP: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            st = myICM.resetFIFO();
            INFO_PRINT(F("DMP resetFIFO: "));
            INFO_PRINTLN(st);
            if (st != ICM_20948_Stat_Ok) {
                dmp_ok = false;
                return;
            }

            dmp_ok = true;
            INFO_PRINTLN(F("DMP initialized OK"));
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

            INFO_PRINT(F("SEND_IMU_DATA: "));
            INFO_PRINT(times.size());
            INFO_PRINTLN(F(" samples"));
        }

        void set_heading(BLERequest &req) {
            float desired;
            BLE_CHECK_READ(req, req.read(desired), "desired");
            yaw_offset = desired - raw_yaw;
            float y = raw_yaw + yaw_offset;
            y -= 360.0f * roundf(y / 360.0f);
            yaw = y;
        }

    } // namespace commands

    // Init

    void init() {
        bool initialized = false;
        while (!initialized) {
            myICM.begin(WIRE_PORT, AD0_VAL);
            INFO_PRINT(F("Initialization of the sensor returned: "));
            INFO_PRINTLN(myICM.statusString());
            if (myICM.status != ICM_20948_Stat_Ok) {
                INFO_PRINTLN(F("Trying again..."));
                delay(500);
            } else {
                initialized = true;
                ok = true;
            }
        }

        methods::init_dmp();

        ble::methods::register_command(SEND_IMU_DATA, commands::send_data);
        ble::methods::register_command(SET_HEADING, commands::set_heading);
    }

    // Periodic

    void periodic() {
        methods::read_dmp();
        methods::read();
    }

} // namespace imu
