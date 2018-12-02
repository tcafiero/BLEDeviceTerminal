#include <stdbool.h>
#include <stdint.h>

#include <pca10040.h>

#define HERE(condition) if(condition){\
  Serial.print("Reached ");\
  Serial.print(__FILE__);\
  Serial.print(" line: ");\
  Serial.println(__LINE__);\
  };

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include <bluefruit.h>


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"
#include <MicroShell.h>
#include "TimeStampManager.h"
#include "CyclicBuffer.h"
#include "Trigger.h"

//#define DEBUG 1
#define GREEN true
#define RED false
#define PRINTRAW
#define IMU_SAMPLING_PERIOD 12 // ms
#define IMU_SENDING_PERIOD 2500 // ms

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

extern TimestampManager ts;
extern CyclicBuffer cb;
extern Trigger tr;


//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
extern LSM9DS1 imu;


void setup() {
  Serial.begin(115200);
  configureBLE();
  delay(2000);
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  if (!initLSM9DS1())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while(1);
  }
  else Serial.println("LSM9DS1 connected");
  cb.begin();
  ts.begin();
  tr.begin();
  configureTimers();
  configureTasks();
  configureInterrupts();
  InitMicroShell();
  //suspendLoop();
  //SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  /* Start FreeRTOS scheduler. In this case useless because Arduino environment yet started it*/
  //vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  while ( bleuart.available() )
  {
    MicroShell();
  }
  // Request CPU to enter low-power mode until an event/interrupt occurs
    vTaskDelay(2000);
  //waitForEvent();
}

