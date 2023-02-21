#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define SHUTDOWN_PIN 8

SFEVL53L1X distanceSensor1;

SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN);

//////////// For Temperature Reading ////////////
#define RESOLUTION_BITS (16)
#ifdef ADCPIN
#define EXTERNAL_ADC_PIN ADCPIN
#endif
//////////// For Temperature Reading ////////////



//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "c1135480-10fc-47dc-8021-8d6f8c21a39c"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

static long prevMillis = 0;
unsigned long currMillis = 0;

static long prevMillisTOF = 0;
unsigned long currMillisTOF = 0;
//////////// Global Variables ////////////



enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  SEND_THREE_FLOATS,
  ECHO,
  DANCE,
  SET_VEL,
  GET_TIME_MILLIS,
  GET_TEMP_5s,
  GET_TEMP_5s_RAPID,
  GET_TOF_5s,
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

      Serial.print("Sent back: ");
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

      Serial.print("Two Integers: ");
      Serial.print(int_a);
      Serial.print(", ");
      Serial.println(int_b);

      break;
    /*
         * Extract three floats from the command string
         */
    case SEND_THREE_FLOATS:
      /*
             * Your code goes here.
             */

      break;
    /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
    case ECHO:

      char char_arr[MAX_MSG_SIZE];

      // Extract the next value from the command string as a character array
      success = robot_cmd.get_next_value(char_arr);
      if (!success)
        return;

      tx_estring_value.clear();
      tx_estring_value.append("Artemis Says: ");
      tx_estring_value.append(char_arr);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());


      break;
    /*
         * DANCE
         */
    case DANCE:
      Serial.println("Look Ma, I'm Dancin'!");

      break;

    /*
         * SET_VEL
         */
    case SET_VEL:

      break;



    case GET_TIME_MILLIS:
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append((int)millis());
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;

    case GET_TEMP_5s:
      
      int count;
      count = 0;
      while (count < 5) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append((int)millis());
        tx_estring_value.append(" C:");
        tx_estring_value.append(getTempDegC());
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        if (count != 4)
          delay(999);
        count++;
      }


      tx_estring_value.clear();
      tx_estring_value.append("END");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());
      break;


    case GET_TEMP_5s_RAPID:


      prevMillis = millis();
      currMillis = prevMillis;

      while (currMillis - prevMillis <= 5000) {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append((int)millis());
        tx_estring_value.append(" C:");
        tx_estring_value.append(getTempDegC());

        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        Serial.print("Sent back: ");
        Serial.println(tx_estring_value.c_str());
        currMillis = millis();
      }
      tx_estring_value.clear();
      tx_estring_value.append("END");
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());

      break;

      case GET_TOF_5s:
        prevMillisTOF = millis();
        currMillisTOF = prevMillisTOF;

        while (currMillisTOF - prevMillisTOF <= 5000) {
          tx_estring_value.clear();
          tx_estring_value.append("T:");
          tx_estring_value.append((int)millis());

          if (distanceSensor1.checkForDataReady()) {
            int distance1 = distanceSensor1.getDistance();
            tx_estring_value.append("|D1:");
            tx_estring_value.append(distance1);
          }
          if (distanceSensor2.checkForDataReady()) {
            int distance2 = distanceSensor2.getDistance();
            tx_estring_value.append("|D2:");
            tx_estring_value.append(distance2);
          }

          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.print("Sent back: ");
          Serial.println(tx_estring_value.c_str());
          currMillisTOF = millis();
        }

        break;




    /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
    default:
      Serial.print("Invalid Command Type: ");
      Serial.println(cmd_type);
      break;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);

  if (distanceSensor1.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");
  distanceSensor1.setI2CAddress(0x02);
  Serial.println(distanceSensor1.getI2CAddress());

  digitalWrite(SHUTDOWN_PIN, HIGH);

  if (distanceSensor2.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 2 online!");

  distanceSensor1.setDistanceModeShort();
  distanceSensor2.setDistanceModeShort();


  distanceSensor1.startRanging();
  distanceSensor2.startRanging();

  analogReadResolution(RESOLUTION_BITS);
  analogWriteResolution(RESOLUTION_BITS);

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
  // Set initial values to prevent errors when reading for the first time on central devices
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
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
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

void loop() {
  // Listen for connections
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected()) {
      // Send data
      write_data();

      // Read data
      read_data();
    }

    Serial.println("Disconnected");
  }
}
