#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define SHUTDOWN_PIN 8

SFEVL53L1X distanceSensor1;

SFEVL53L1X distanceSensor2(Wire, SHUTDOWN_PIN);

unsigned long start = 0;
unsigned long end = 0;


void setup(void) {
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

  // if (distanceSensor2.begin() != 0)  //Begin returns 0 on a good init
  // {
  //   Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
  //   while (1)
  //     ;
  // }
  // Serial.println("Sensor 2 online!");

  distanceSensor1.setDistanceModeShort();
  // distanceSensor2.setDistanceModeShort();


  distanceSensor1.startRanging();
  // distanceSensor2.startRanging();
}

void loop(void) {

  if (distanceSensor1.checkForDataReady()) {
    int distance1 = distanceSensor1.getDistance();
    Serial.print("\tDistance 1(mm): ");
    Serial.print(distance1);
  }

  // if (distanceSensor2.checkForDataReady()) {
  //   int distance2 = distanceSensor2.getDistance();
  //   Serial.print("\tDistance 2(mm): ");
  //   Serial.print(distance2);
  // }
  Serial.println();
}





