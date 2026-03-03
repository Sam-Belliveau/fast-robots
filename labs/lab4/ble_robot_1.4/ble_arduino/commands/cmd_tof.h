#ifndef CMD_TOF_H
#define CMD_TOF_H

// ToF circular buffers, periodic read, and command handlers
// Accesses globals: distanceSensor1, distanceSensor2,
//                   tx_estring_value, tx_characteristic_string

//////////// ToF Circular Buffers ////////////

CircularBuffer<int, 0x100> tof1_times;
CircularBuffer<int, 0x100> tof1_dist;

CircularBuffer<int, 0x100> tof2_times;
CircularBuffer<int, 0x100> tof2_dist;

//////////// ToF Periodic Read ////////////

void read_tof() {
  if (distanceSensor1.checkForDataReady()) {
    const int time = micros();
    int distance = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    tof1_times.push(time);
    tof1_dist.push(distance);
  }

  if (distanceSensor2.checkForDataReady()) {
    const int time = micros();
    int distance = distanceSensor2.getDistance();
    distanceSensor2.clearInterrupt();
    tof2_times.push(time);
    tof2_dist.push(distance);
  }
}

//////////// ToF Command Handlers ////////////

void cmd_send_tof_data() {
  zip(
      [&](int t, int d) {
        tx_estring_value.clear();
        tx_estring_value.append("D1:");
        tx_estring_value.append(t);
        tx_estring_value.append("|");
        tx_estring_value.append(d);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
      },
      tof1_times.begin(), tof1_times.end(), tof1_dist.begin());

  zip(
      [&](int t, int d) {
        tx_estring_value.clear();
        tx_estring_value.append("D2:");
        tx_estring_value.append(t);
        tx_estring_value.append("|");
        tx_estring_value.append(d);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
      },
      tof2_times.begin(), tof2_times.end(), tof2_dist.begin());

  tx_estring_value.clear();
  tx_estring_value.append("END");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print(F("SEND_TOF_DATA: "));
  Serial.print(tof1_dist.size());
  Serial.print(F(" S1 samples, "));
  Serial.print(tof2_dist.size());
  Serial.println(F(" S2 samples"));
}

void cmd_tof_short() {
  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();
  Serial.println(F("ToF: short mode"));
}

void cmd_tof_long() {
  distanceSensor1.setDistanceModeLong();
  distanceSensor2.setDistanceModeLong();
  Serial.println(F("ToF: long mode"));
}

void cmd_tof_stats() {
  // Sensor 1 stats
  {
    const int n = tof1_dist.size();
    float mean = 0, std_dev = 0;
    if (n > 0) {
      float sum = 0;
      for (auto d : tof1_dist)
        sum += d;
      mean = sum / n;
      float sq_sum = 0;
      for (auto d : tof1_dist) {
        float diff = d - mean;
        sq_sum += diff * diff;
      }
      std_dev = sqrt(sq_sum / n);
    }
    tx_estring_value.clear();
    tx_estring_value.append("S1:");
    tx_estring_value.append(n);
    tx_estring_value.append("|");
    tx_estring_value.append(mean);
    tx_estring_value.append("|");
    tx_estring_value.append(std_dev);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }

  // Sensor 2 stats
  {
    const int n = tof2_dist.size();
    float mean = 0, std_dev = 0;
    if (n > 0) {
      float sum = 0;
      for (auto d : tof2_dist)
        sum += d;
      mean = sum / n;
      float sq_sum = 0;
      for (auto d : tof2_dist) {
        float diff = d - mean;
        sq_sum += diff * diff;
      }
      std_dev = sqrt(sq_sum / n);
    }
    tx_estring_value.clear();
    tx_estring_value.append("S2:");
    tx_estring_value.append(n);
    tx_estring_value.append("|");
    tx_estring_value.append(mean);
    tx_estring_value.append("|");
    tx_estring_value.append(std_dev);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
  }

  tx_estring_value.clear();
  tx_estring_value.append("END");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  Serial.print(F("TOF_STATS: S1 n="));
  Serial.print(tof1_dist.size());
  Serial.print(F(", S2 n="));
  Serial.println(tof2_dist.size());
}

void register_tof_commands() {
  register_command(SEND_TOF_DATA, cmd_send_tof_data);
  register_command(TOF_SHORT, cmd_tof_short);
  register_command(TOF_LONG, cmd_tof_long);
  register_command(TOF_STATS, cmd_tof_stats);
}

#endif // CMD_TOF_H
