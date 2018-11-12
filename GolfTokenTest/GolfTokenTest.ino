#include <stdbool.h>
#include <stdint.h>

#include <pca10040.h>

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

//#define DEBUG 1
#define GREEN true
#define RED false
#define PRINT_CALCULATED
#define IMU_SAMPLING_PERIOD 20 // ms
#define IMU_SENDING_PERIOD 250 // ms

// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;

extern TimestampManager ts;
extern CyclicBuffer cb;


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
  };
  cb.begin();
  ts.begin();
  configureTimers();
#if 0
  configureTasks();
#endif
  InitMicroShell();
  //suspendLoop();
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
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
  vTaskDelay(200);
  waitForEvent();
}

