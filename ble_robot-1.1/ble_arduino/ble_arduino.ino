#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include "ICM_20948.h"         // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;



#define SERIAL_PORT Serial
#define AD0_VAL 1  // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

ICM_20948_I2C myICM;


SFEVL53L1X distanceSensor1;



/////////////////Motor Drivers//////////////////////
#define motorL1 A3
#define motorL2 A2
#define motorR1 4
#define motorR2 A5
/////////////////Motor Drivers//////////////////////

////////////////Data Arrays/////////////////////////
int time_array[600];
int TOF_array[600];
int PWM_array[600];
int xkf_array[600];
int angV_array[600];
////////////////Data Arrays/////////////////////////


//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "c1135480-10fc-47dc-8021-8d6f8c21a39c"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
#define BLE_UUID_TX_STRING2 "6ffc30b7-807f-42e0-994c-4c209f85e9e2"
//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);
BLECStringCharacteristic tx_characteristic_string2(BLE_UUID_TX_STRING2, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
EString tx_estring2_value;
EString tx_motorPWM_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

static long prevMillis = 0;
unsigned long currMillis = 0;

static long prevMillisIMU = 0;
unsigned long currMillisIMU = 0;

static long prev_time = 0;
unsigned long curr_time = 0;

//////////// Global Variables ////////////



void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  /////////////////Motor Drivers//////////////////////
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);

  /////////////////Motor Drivers//////////////////////


  //////////////////////////IMU////////////////////////////////////////
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  //////////////////////////IMU////////////////////////////////////////


  //////////////////////////TOF////////////////////////////////////////

  if (distanceSensor1.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");

  distanceSensor1.setDistanceModeLong();

  distanceSensor1.startRanging();

  //////////////////////////TOF////////////////////////////////////////


  //////////////////////////BLE////////////////////////////////////////
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(tx_characteristic_string2);
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
  //////////////////////////BLE////////////////////////////////////////


  for (int i = 0; i < 3; i++) {
    digitalWrite(blinkPin, HIGH);
    delay(500);
    digitalWrite(blinkPin, LOW);
    delay(200);
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
    stop();

    Serial.println("Disconnected");
  }
}


////////BLE//////////////
enum CommandTypes {
  PING,
  SEND_TWO_INTS,
  START,
  STOP,
  GO_FORWARD,
  TURN_CW,
  TURN_CCW

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
      tx_estring2_value.clear();
      tx_estring2_value.append("PONG");
      tx_characteristic_string2.writeValue(tx_estring2_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring2_value.c_str());

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


    case START:
      forward(80);
      break;

    case STOP:
      stop();
      break;

    case GO_FORWARD:
      {
        stop();
        int currDist;
        int forwardDist;
        success = robot_cmd.get_next_value(forwardDist);
        if (!success) {
          return;
        }
        prevMillis = millis();
        currMillis = prevMillis;
        while (currMillis - prevMillis < 5000) {
          if (distanceSensor1.checkForDataReady()) {
            currDist = distanceSensor1.getDistance();
          }
          pid(currDist, forwardDist);
          Serial.println(currDist);

          currMillis = millis();
        }
        stop();
      }
      break;



    case TURN_CW:
      {
        stop();
        float dt = 0;
        float curr_ang = 0;
        float err;

        float target_ang1;
        success = robot_cmd.get_next_value(target_ang1);
        if (!success) {
          return;
        }
        prevMillisIMU = millis();
        currMillisIMU = prevMillisIMU;
        while (currMillisIMU - prevMillisIMU < 5000) {
          if (myICM.dataReady()) {
            myICM.getAGMT();
            curr_time = millis();
            dt = curr_time - prev_time;
            prev_time = curr_time;
            curr_ang = curr_ang + myICM.gyrZ() * dt / 1000;
            err = target_ang1 - curr_ang;

            float pid_ang = 0.9 * err;
            pid_ang = speed_adjusted(pid_ang);
            if (pid_ang > 1.5) {
              turnCCW(pid_ang);
            } else if (pid_ang < 1.5) {
              turnCW(pid_ang);
            } else {
              stop();
            }
          }
          currMillisIMU = millis();
        }
        curr_ang = 0;
        err = 0;
        stop();
      }
      break;

    case TURN_CCW:
      {
        float turnAngle;
        success = robot_cmd.get_next_value(turnAngle);
        if (!success) {
          return;
        }
        turnCCW(110);
        delay(turnAngle);
        stop();
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

int pid(int dist, int targetDist) {
  int currDist = dist;
  float error = (float)(targetDist - currDist);
  float speed = 0.04 * error;
  Serial.println(speed);
  int PWM = speed;


  if (PWM < 0) {
    Serial.println("forward");
    PWM = max(PWM, -255);
    PWM = min(PWM, -40);
    PWM = -PWM;
    analogWrite(motorL2, PWM);
    analogWrite(motorL1, 0);
    analogWrite(motorR2, PWM);
    analogWrite(motorR1, 0);

  } else if (PWM > 0) {
    Serial.println("backward");

    PWM = min(PWM, 255);
    PWM = max(PWM, 40);

    analogWrite(motorL2, 0);
    analogWrite(motorL1, PWM);
    analogWrite(motorR2, 0);
    analogWrite(motorR1, PWM);
  } else {
    stop();
  }
  return speed;
}

int speed_adjusted(int PWM) {
  if (PWM < 0) {
    PWM = max(PWM, -255);
    PWM = min(PWM, -90);
  } else if (PWM > 0) {
    PWM = min(PWM, 255);
    PWM = max(PWM, 90);
  }
  return PWM;
}


void forward(int PWM) {
  analogWrite(motorR2, PWM);
  analogWrite(motorL2, PWM);
  analogWrite(motorL1, 0);
  analogWrite(motorR1, 0);
}

void backward(int PWM) {
  analogWrite(motorR1, PWM);
  analogWrite(motorL1, PWM);
  analogWrite(motorL2, 0);
  analogWrite(motorR2, 0);
}

void turnCCW(int PWM) {
  analogWrite(motorR2, PWM);
  analogWrite(motorL1, PWM);
  analogWrite(motorL2, 0);
  analogWrite(motorR1, 0);
}

void turnCW(int PWM) {
  analogWrite(motorR1, PWM);
  analogWrite(motorL2, PWM);
  analogWrite(motorL1, 0);
  analogWrite(motorR2, 0);
}

void stop() {
  analogWrite(motorL2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorR1, 0);
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

////////////////BLE///////////////////////////


////////IMU HELPER FUNCTIONS//////////////////////////

void printPaddedInt16b(int16_t val) {
  if (val > 0) {
    SERIAL_PORT.print(" ");
    if (val < 10000) { SERIAL_PORT.print("0"); }
    if (val < 1000) { SERIAL_PORT.print("0"); }
    if (val < 100) { SERIAL_PORT.print("0"); }
    if (val < 10) { SERIAL_PORT.print("0"); }
  } else {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000) { SERIAL_PORT.print("0"); }
    if (abs(val) < 1000) { SERIAL_PORT.print("0"); }
    if (abs(val) < 100) { SERIAL_PORT.print("0"); }
    if (abs(val) < 10) { SERIAL_PORT.print("0"); }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}


void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_AGMT_t agmt) {
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(myICM.accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(myICM.gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(myICM.magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(myICM.magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(myICM.temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

////////IMU HELPER FUNCTIONS//////////////////////////
