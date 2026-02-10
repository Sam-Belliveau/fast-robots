
#include "BLECStringCharacteristic.h"
#include "CircularBuffer.h"
#include "EString.h"
#include "RobotCommand.h"
#include "Zip.h"
#include <ArduinoBLE.h>

//////////// Accelerometer ////////////
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//////////// Time of Flight ////////////
#include "SparkFun_VL53L1X.h" // Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

// #define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT                                                               \
  SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN                                                                 \
  2 // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT                                                              \
  Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper
// is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

SFEVL53L1X distanceSensor;

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "1785129f-3b3a-4cf5-a01f-03668e8b12e9"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite,
                                                  MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT,
                                               BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING,
                                                  BLERead | BLENotify,
                                                  MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//////////// Global Variables ////////////

CircularBuffer<int, 0x200> time_millis_buffer;
CircularBuffer<float, 0x200> temp_buffer;

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

CircularBuffer<int, 0x40> tof_times;
CircularBuffer<int, 0x40> tof_dist;

enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  GET_TIME_MILLIS,
  ECHO,
  DANCE,
  SET_VEL,
  STORE_TIME_MILLIS,
  SEND_TIME_MILLIS,
  SEND_IMU_DATA,
  SEND_TOF_DATA,
  TOF_SHORT,
  TOF_LONG,
  TOF_STATS,
};

void handle_command() {
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success) {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type) {
  /*
   * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
   */
  case PING:
    tx_estring_value.clear();
    tx_estring_value.append("PONG");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print(F("Sent back: "));
    Serial.println(tx_estring_value.c_str());

    break;
  /*
   * Extract two integers from the command string
   */
  case SEND_TWO_INTS:
    int int_a, int_b;

    // Extract the next value from the command string as an integer
    success = robot_cmd.get_next_value(int_a);
    if (!success)
      return;

    // Extract the next value from the command string as an integer
    success = robot_cmd.get_next_value(int_b);
    if (!success)
      return;

    Serial.print(F("Two Integers: "));
    Serial.print(int_a);
    Serial.print(F(", "));
    Serial.println(int_b);

    break;
  /*
   * Extract three floats from the command string
   */
  case SEND_THREE_FLOATS:
    float float_a, float_b, float_c;

    success = robot_cmd.get_next_value(float_a);
    if (!success)
      return;

    success = robot_cmd.get_next_value(float_b);
    if (!success)
      return;

    success = robot_cmd.get_next_value(float_c);
    if (!success)
      return;

    Serial.print(F("Three Floats: "));
    Serial.print(float_a);
    Serial.print(F(", "));
    Serial.print(float_b);
    Serial.print(F(", "));
    Serial.println(float_c);

    break;

  case GET_TIME_MILLIS:
    int time_millis;
    float temp;

    time_millis = millis();
    temp = getTempDegF();

    tx_estring_value.clear();
    tx_estring_value.append("T:");
    tx_estring_value.append(time_millis);
    tx_estring_value.append("|");
    tx_estring_value.append(temp);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print(F("Sent back: "));
    Serial.println(tx_estring_value.c_str());

    break;

  case STORE_TIME_MILLIS:

    int count, i;

    // Extract the next value from the command string as an integer
    success = robot_cmd.get_next_value(count);
    if (!success)
      return;

    for (i = 0; i < count; ++i) {
      time_millis = millis();
      temp = getTempDegF();
      time_millis_buffer.push(time_millis);
      temp_buffer.push(temp);
    }

    break;

  case SEND_TIME_MILLIS:
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

    break;

  /*
   * Add a prefix and postfix to the string value extracted from the command
   * string
   */
  case ECHO:
    char char_arr[MAX_MSG_SIZE];

    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);
    if (!success)
      return;

    tx_estring_value.clear();
    tx_estring_value.append("Robot Says -> ");
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print(F("Sent back: "));
    Serial.println(tx_estring_value.c_str());

    break;
  /*
   * DANCE
   */
  case DANCE:
    Serial.println(F("Look Ma, I'm Dancin'!"));

    break;

  /*
   * SET_VEL
   */
  case SET_VEL:

    break;

  /*
   * Send the entire IMU circular buffer over BLE
   */
  case SEND_IMU_DATA: {
    zip(
        [&](int t, float ax, float ay, float az, float gx, float gy, float gz,
            float mx, float my, float mz) {
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
        },
        imu_times.begin(), imu_times.end(), imu_acc_x.begin(),
        imu_acc_y.begin(), imu_acc_z.begin(), imu_gyr_x.begin(),
        imu_gyr_y.begin(), imu_gyr_z.begin(), imu_mag_x.begin(),
        imu_mag_y.begin(), imu_mag_z.begin());

    // Send end marker
    tx_estring_value.clear();
    tx_estring_value.append("END");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.println(F("Sent IMU data"));
    break;
  }

  /*
   * Send the entire ToF circular buffer over BLE
   */
  case SEND_TOF_DATA: {
    zip(
        [&](int t, int d) {
          tx_estring_value.clear();
          tx_estring_value.append("D:");
          tx_estring_value.append(t);
          tx_estring_value.append("|");
          tx_estring_value.append(d);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        },
        tof_times.begin(), tof_times.end(), tof_dist.begin());

    // Send end marker
    tx_estring_value.clear();
    tx_estring_value.append("END");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.println(F("Sent ToF data"));
    break;
  }

  /*
   * Switch ToF sensor to short distance mode
   */
  case TOF_SHORT:
    distanceSensor.setDistanceModeShort();
    Serial.println(F("ToF: short mode"));
    break;

  /*
   * Switch ToF sensor to long distance mode
   */
  case TOF_LONG:
    distanceSensor.setDistanceModeLong();
    Serial.println(F("ToF: long mode"));
    break;

  /*
   * Compute and send mean and std of ToF distance buffer
   */
  case TOF_STATS: {
    const int n = tof_dist.size();
    if (n == 0) {
      tx_estring_value.clear();
      tx_estring_value.append("S:0|0|0");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      break;
    }

    float sum = 0;
    for (auto d : tof_dist)
      sum += d;
    float mean = sum / n;

    float sq_sum = 0;
    for (auto d : tof_dist) {
      float diff = d - mean;
      sq_sum += diff * diff;
    }
    float std_dev = sqrt(sq_sum / n);

    tx_estring_value.clear();
    tx_estring_value.append("S:");
    tx_estring_value.append(n);
    tx_estring_value.append("|");
    tx_estring_value.append(mean);
    tx_estring_value.append("|");
    tx_estring_value.append(std_dev);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.println(F("Sent ToF stats"));
    break;
  }

  /*
   * The default case may not capture all types of invalid commands.
   * It is safer to validate the command string on the central device (in
   * python) before writing to the characteristic.
   */
  default:
    Serial.print(F("Invalid Command Type: "));
    Serial.println(cmd_type);
    break;
  }
}

#define RESOLUTION_BITS (16) // choose resolution (explained in depth below)

#ifdef ADCPIN
#define EXTERNAL_ADC_PIN                                                       \
  ADCPIN // ADCPIN is the lowest analog capable pin exposed on the variant
#endif   // - if no exposed pins are analog capable this will be undefined
         // - to use another pin provide an analog capable pin number such as:
         //   - A0 -> A9 (when analog pins are named sequentially from 0)
         //   - A11 -> A13, A16, A29, A31 -> A35 (when pins are named after
         //   Apollo3 pads)
         //   - A variant-specific pin number (when none of the above apply)

void setup() {
  Serial.begin(115200);

  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on
  // central devices
  tx_characteristic_float.writeValue(0.0);

  /*
   * An example using the EString
   */
  // Clear the contents of the EString before using it
  tx_estring_value.clear();

  // Append the string literal "[->"
  tx_estring_value.append("[->");

  // Append the float value
  tx_estring_value.append(9.0);

  // Append the string literal "<-]"
  tx_estring_value.append("<-]");

  // Write the value to the characteristic
  tx_characteristic_string.writeValue(tx_estring_value.c_str());

  // Output MAC Address
  Serial.print(F("Advertising BLE with MAC: "));
  Serial.println(BLE.address());

  BLE.advertise();

  analogReadResolution(
      RESOLUTION_BITS); // set the resolution of analogRead results
                        //  - maximum: 16 bits (padded with trailing zeroes)
                        //  - ADC:     14 bits (maximum ADC resolution)
                        //  - default: 10 bits (standard Arduino setting)
                        //  - minimum:  1 bit

  analogWriteResolution(RESOLUTION_BITS); // match resolution for analogWrite

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
    }
  }

  // Initialize ToF sensor
  if (distanceSensor.begin() != 0) {
    SERIAL_PORT.println(F("ToF sensor failed to begin. Check wiring."));
  } else {
    SERIAL_PORT.println(F("ToF sensor online!"));
    distanceSensor.startRanging();
  }
}

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
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}

void read_imu() {
  const int time = micros();

  const int last_time = imu_times.bottom();

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

void read_tof() {
  if (distanceSensor.checkForDataReady()) {
    const int time = micros();
    int distance = distanceSensor.getDistance();
    distanceSensor.clearInterrupt();

    tof_times.push(time);
    tof_dist.push(distance);
  }
}

void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print(F("Connected to: "));
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      // Read sensors
      read_imu();
      read_tof();

      // Send data
      write_data();

      // Read data
      read_data();
    }

    Serial.println(F("Disconnected"));
  }
}
