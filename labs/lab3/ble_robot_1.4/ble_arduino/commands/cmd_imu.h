#ifndef CMD_IMU_H
#define CMD_IMU_H

// IMU circular buffers, periodic read, and command handlers
// Accesses globals: myICM, tx_estring_value, tx_characteristic_string

//////////// IMU Circular Buffers ////////////

CircularBuffer<int, 0x100> imu_times;
CircularBuffer<float, 0x100> imu_acc_x;
CircularBuffer<float, 0x100> imu_acc_y;
CircularBuffer<float, 0x100> imu_acc_z;
CircularBuffer<float, 0x100> imu_gyr_x;
CircularBuffer<float, 0x100> imu_gyr_y;
CircularBuffer<float, 0x100> imu_gyr_z;
CircularBuffer<float, 0x100> imu_mag_x;
CircularBuffer<float, 0x100> imu_mag_y;
CircularBuffer<float, 0x100> imu_mag_z;

//////////// IMU Periodic Read ////////////

void read_imu() {
  if (!imu_ok) return;
  const int time = micros();
  const int last_time = imu_times.is_empty() ? 0 : imu_times.bottom();
  if (time - last_time > 20000) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      imu_times.push(time);
      imu_acc_x.push(myICM.accX());
      imu_acc_y.push(myICM.accY());
      imu_acc_z.push(myICM.accZ());
      imu_gyr_x.push(myICM.gyrX());
      imu_gyr_y.push(myICM.gyrY());
      imu_gyr_z.push(myICM.gyrZ());
      imu_mag_x.push(myICM.magX());
      imu_mag_y.push(myICM.magY());
      imu_mag_z.push(myICM.magZ());
    }
  }
}

//////////// IMU Command Handlers ////////////

void cmd_send_imu_data() {
  zip(
    [&](
      int t,
      float ax, float ay, float az,
      float gx, float gy, float gz,
      float mx, float my, float mz
    ) {
      tx_estring_value.clear();
      tx_estring_value.append("I:");
      tx_estring_value.append(t);
      tx_estring_value.append("|");
      tx_estring_value.append(ax);
      tx_estring_value.append("|");
      tx_estring_value.append(ay);
      tx_estring_value.append("|");
      tx_estring_value.append(az);
      tx_estring_value.append("|");
      tx_estring_value.append(gx);
      tx_estring_value.append("|");
      tx_estring_value.append(gy);
      tx_estring_value.append("|");
      tx_estring_value.append(gz);
      tx_estring_value.append("|");
      tx_estring_value.append(mx);
      tx_estring_value.append("|");
      tx_estring_value.append(my);
      tx_estring_value.append("|");
      tx_estring_value.append(mz);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      delay(1);
    },
    imu_times.begin(),
    imu_times.end(),
    imu_acc_x.begin(),
    imu_acc_y.begin(),
    imu_acc_z.begin(),
    imu_gyr_x.begin(),
    imu_gyr_y.begin(),
    imu_gyr_z.begin(),
    imu_mag_x.begin(),
    imu_mag_y.begin(),
    imu_mag_z.begin()
  );

  tx_estring_value.clear();
  tx_estring_value.append("END");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print(F("SEND_IMU_DATA: "));
  Serial.print(imu_times.size());
  Serial.println(F(" samples"));
}

void register_imu_commands() {
  register_command(SEND_IMU_DATA, cmd_send_imu_data);
}

#endif // CMD_IMU_H
