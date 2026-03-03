
#include "BLECStringCharacteristic.h"
#include "CircularBuffer.h"
#include "EString.h"
#include "RobotCommand.h"
#include "Zip.h"
#include <ArduinoBLE.h>

//////////// Accelerometer ////////////
#include "ICM_20948.h"

//////////// Time of Flight ////////////
#include "SparkFun_VL53L1X.h"

// #define USE_SPI

#define SERIAL_PORT Serial
#define SPI_PORT SPI
#define CS_PIN 2
#define WIRE_PORT Wire
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM;
#else
ICM_20948_I2C myICM;
#endif

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2;
#define TOF1_XSHUT_PIN A5
bool imu_ok = false;

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "1785129f-3b3a-4cf5-a01f-03668e8b12e9"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);

BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

RobotCommand robot_cmd(":|");

EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

bool recording = false;

//////////// Misc Circular Buffers ////////////

CircularBuffer<int, 0x200> time_millis_buffer;
CircularBuffer<float, 0x200> temp_buffer;

// === COMMAND ENUM ===
// Keep in sync with ble_python/cmd_types.py
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
};

//////////// Command Modules ////////////

#include "commands/CommandRegistry.h"
#include "commands/cmd_imu.h"
#include "commands/cmd_tof.h"

// Command dispatch table storage
CommandHandler command_handlers[MAX_COMMANDS] = {nullptr};
int num_commands = 0;

//////////// Core Command Handlers ////////////

void cmd_ping() {
  tx_estring_value.clear();
  tx_estring_value.append("PONG");
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.print(F("Sent back: "));
  Serial.println(tx_estring_value.c_str());
}

void cmd_send_two_ints() {
  int int_a, int_b;
  if (!robot_cmd.get_next_value(int_a))
    return;
  if (!robot_cmd.get_next_value(int_b))
    return;
  Serial.print(F("Two Integers: "));
  Serial.print(int_a);
  Serial.print(F(", "));
  Serial.println(int_b);
}

void cmd_send_three_floats() {
  float float_a, float_b, float_c;
  if (!robot_cmd.get_next_value(float_a))
    return;
  if (!robot_cmd.get_next_value(float_b))
    return;
  if (!robot_cmd.get_next_value(float_c))
    return;
  Serial.print(F("Three Floats: "));
  Serial.print(float_a);
  Serial.print(F(", "));
  Serial.print(float_b);
  Serial.print(F(", "));
  Serial.println(float_c);
}

void cmd_get_time_millis() {
  int time_millis = millis();
  float temp = getTempDegF();
  tx_estring_value.clear();
  tx_estring_value.append("T:");
  tx_estring_value.append(time_millis);
  tx_estring_value.append("|");
  tx_estring_value.append(temp);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.print(F("Sent back: "));
  Serial.println(tx_estring_value.c_str());
}

void cmd_echo() {
  char char_arr[MAX_MSG_SIZE];
  if (!robot_cmd.get_next_value(char_arr))
    return;
  tx_estring_value.clear();
  tx_estring_value.append("Robot Says -> ");
  tx_estring_value.append(char_arr);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
  Serial.print(F("Sent back: "));
  Serial.println(tx_estring_value.c_str());
}

void cmd_dance() { Serial.println(F("Look Ma, I'm Dancin'!")); }

void cmd_set_vel() { /* placeholder */ }

void cmd_store_time_millis() {
  int count;
  if (!robot_cmd.get_next_value(count))
    return;
  for (int i = 0; i < count; ++i) {
    time_millis_buffer.push((int)millis());
    temp_buffer.push(getTempDegF());
  }
}

void cmd_send_time_millis() {
  int time_millis;
  float temp;
  while (time_millis_buffer.pop(time_millis) && temp_buffer.pop(temp)) {
    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(time_millis);
    tx_estring_value.append("|");
    tx_estring_value.append(temp);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print(F("Sent back: "));
    Serial.println(tx_estring_value.c_str());
  }
}

void cmd_start_recording() {
  recording = true;
  Serial.println(F("Recording started"));
}

void cmd_stop_recording() {
  recording = false;
  Serial.println(F("Recording stopped"));
}

void register_core_commands() {
  register_command(PING, cmd_ping);
  register_command(SEND_TWO_INTS, cmd_send_two_ints);
  register_command(SEND_THREE_FLOATS, cmd_send_three_floats);
  register_command(GET_TIME_MILLIS, cmd_get_time_millis);
  register_command(ECHO, cmd_echo);
  register_command(DANCE, cmd_dance);
  register_command(SET_VEL, cmd_set_vel);
  register_command(STORE_TIME_MILLIS, cmd_store_time_millis);
  register_command(SEND_TIME_MILLIS, cmd_send_time_millis);
  register_command(START_RECORDING, cmd_start_recording);
  register_command(STOP_RECORDING, cmd_stop_recording);
}

//////////// Command Dispatch ////////////

void handle_command() {
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());
  int cmd_type = -1;
  if (!robot_cmd.get_command_type(cmd_type))
    return;
  dispatch_command(cmd_type);
}

//////////// Setup ////////////

#define RESOLUTION_BITS (16)

void setup() {
  Serial.begin(115200);

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

  Serial.print(F("Advertising BLE with MAC: "));
  Serial.println(BLE.address());

  BLE.advertise();

  analogReadResolution(RESOLUTION_BITS);
  analogWriteResolution(RESOLUTION_BITS);

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  bool initialized = false;
  while (!initialized) {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    } else {
      initialized = true;
      imu_ok = true;
    }
  }

  // Let sensors fully boot before init
  delay(500);

  // Initialize ToF sensor(s)
  // Both sensors start at default address 0x29.
  // Shut down sensor 1 (XSHUT), init sensor 2, change its address,
  // then bring sensor 1 back and init at default.
  pinMode(TOF1_XSHUT_PIN, OUTPUT);

  // Shut down sensor 1 via XSHUT
  digitalWrite(TOF1_XSHUT_PIN, LOW);
  delay(100);

  // First begin() call tends to fail (Pololu level shifter bus issue),
  // but it wakes the bus up for subsequent calls.
  distanceSensor2.begin();
  delay(100);

  // Now retry for real
  if (distanceSensor2.begin() == 0) {
    distanceSensor2.setI2CAddress(0x54);
    SERIAL_PORT.println(F("ToF sensor 2 online at address 0x54!"));
    distanceSensor2.startRanging();
  } else {
    SERIAL_PORT.println(F("WARNING: ToF sensor 2 failed to begin."));
  }

  // Bring sensor 1 back up
  digitalWrite(TOF1_XSHUT_PIN, HIGH);
  delay(100);

  if (distanceSensor1.begin() == 0) {
    SERIAL_PORT.println(F("ToF sensor 1 online!"));
    distanceSensor1.startRanging();
  } else {
    SERIAL_PORT.println(F("WARNING: ToF sensor 1 failed to begin."));
  }

  // Register all command handlers
  register_core_commands();
  register_imu_commands();
  register_tof_commands();

  Serial.print(F("Commands registered: "));
  Serial.println(num_commands);
}

//////////// BLE Periodic ////////////

void write_data() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);
    if (tx_float_value > 10000) {
      tx_float_value = 0;
    }
    previousMillis = currentMillis;
  }
}

void read_data() {
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

//////////// Main Loop ////////////

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print(F("Connected to: "));
    Serial.println(central.address());

    while (central.connected()) {
      read_imu();
      read_tof();
      write_data();
      read_data();
    }

    Serial.println(F("Disconnected"));
  }
}
