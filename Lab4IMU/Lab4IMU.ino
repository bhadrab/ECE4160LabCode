/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo 
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 * 
 * This code is beerware; if you see me (or any other SparkFun employee) at the
 * local, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

#define SERIAL_PORT Serial
#define AD0_VAL 1  // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN


ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

void setup() {

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
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
  delay(3000);
}

void loop() {

  /* Computation variables */
  float pitch_a = 0, roll_a = 0, pitch_g = 0, roll_g = 0, yaw_g = 0, dt = 0, pitch = 0, roll = 0, yaw = 0, comp_roll = 0, comp_pitch=0;
  float Xm = 0, Ym = 0, Zm = 0, x = 0, y = 0;
  unsigned long last_time = millis();
  double pitch_a_LPF[] = { 0, 0 };
  double roll_a_LPF[] = { 0, 0 };
  const int n = 1;
  int curr_ang = 0;

  while (1) {
    if (myICM.dataReady()) {
      myICM.getAGMT();  // The values are only updated when you call 'getAGMT'

      dt = (micros()-last_time)/1000000.;
      last_time = micros();
      pitch_g = pitch_g + myICM.gyrY()*dt;
      roll_g = roll_g + myICM.gyrX()*dt;
      yaw_g = yaw_g + myICM.gyrZ()*dt;
      const float alpha = 0.2;
      pitch_a = -atan2(myICM.accX(),myICM.accZ())*180/M_PI;
      roll_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
      pitch_a_LPF[n] = alpha * pitch_a + (1 - alpha) * pitch_a_LPF[n - 1];
      pitch_a_LPF[n - 1] = pitch_a_LPF[n];
      roll_a_LPF[n] = alpha * roll_a + (1 - alpha) * roll_a_LPF[n - 1];
      roll_a_LPF[n - 1] = roll_a_LPF[n];

      comp_pitch = (comp_pitch - myICM.gyrY()*dt)*0.9 + pitch_a_LPF[n]*0.1;
      comp_roll = (comp_roll - myICM.gyrZ()*dt)*0.9 + roll_a_LPF[n]*0.1;
      curr_ang = curr_ang + myICM.gyrZ()*dt;
      
      Serial.print("acc:");
      Serial.print(myICM.gyrZ());
      Serial.print("pitch:");
      Serial.print(pitch_a);
      Serial.print(", "); 
      Serial.print("roll:");
      Serial.println(pitch_g);
      
    }
  }
}

void blink(unsigned char no) {
  //Indicate success
  for (char i = 0; i <= no - 1; i++) {
    digitalWrite(blinkPin, HIGH);
    delay(1000);
    digitalWrite(blinkPin, LOW);
    delay(1000);
  }
}

// Below here are some helper functions to print the data nicely!

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