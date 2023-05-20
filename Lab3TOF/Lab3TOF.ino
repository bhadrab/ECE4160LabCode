#include <Wire.h>
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#define SHUTDOWN_PIN 8

SFEVL53L1X distanceSensor1;



unsigned long start = 0;
unsigned long end = 0;


void setup(void) {
  Wire.begin();
  Serial.begin(115200);


  if (distanceSensor1.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");
  distanceSensor1.setI2CAddress(0x02);
  Serial.println(distanceSensor1.getI2CAddress());

  

  distanceSensor1.setDistanceModeShort();


  distanceSensor1.startRanging();
}

void loop(void) {

  if (distanceSensor1.checkForDataReady()) {
    int distance1 = distanceSensor1.getDistance();
    Serial.print("\tDistance 1(mm): ");
    Serial.print(distance1);
  }


  Serial.println();
}
