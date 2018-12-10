TaskHandle_t  samplingData_task_handle;
TaskHandle_t  led2_toggle_task_handle;
TaskHandle_t  thresholdAccelGyro_task_handle;
TaskHandle_t  SendDataRead_task_handle;
TaskHandle_t  IoTloop_task_handle;

static void IoTloop (void *);
//static void BLEloop (void *);


static void thresholdAccelGyro_task_function (void * pvParameter)
{
  while (1)
  {
    if (thresholdAccelGyro_flag)
    {
      thresholdAccelGyro_flag = false;
      cb.sendCapturedRecords();
    }
    WAIT(10);
  }
}


static void samplingData_task_function (void * pvParameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xSamplingPeriod = IMU_SAMPLING_PERIOD * 1000 / 30;
  // Initialise the xLastWakeTime variable with the current time.
  //xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    //vTaskDelayUntil( &xLastWakeTime, xSamplingPeriod );
    //xLastWakeTime = xTaskGetTickCount();
    // Perform action here.
    digitalWrite(D6, !digitalRead(D6));
    imuRead();
    if (tr.zeroCrossing(imu.ay))
    {
      cb.sendCapturedRecords();
    }
    imuDataSampling(ts.get());
    vTaskDelay(xSamplingPeriod);
    /* Tasks must be implemented to never return... */
  }
}

static void led2_toggle_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  TickType_t xLastWakeTime;
  const TickType_t xSamplingPeriod = 500 * 1000 / 30;
  // Initialise the xLastWakeTime variable with the current time.
  //xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    //vTaskDelayUntil( &xLastWakeTime, xSamplingPeriod );
    //xLastWakeTime = xTaskGetTickCount();
    digitalWrite(D7, !digitalRead(D7));
    vTaskDelay(xSamplingPeriod);
  }
}

static void SendDataRead_task_function (void * pvParameter)
{
  while (1)
  {
    if (DoSend)
    {
#ifdef PRINTRAW
      printRaw(xTaskGetTickCount(), imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz);
#else
      printAttitude(imu.ax, imu.ay, imu.az,
                    -imu.my, -imu.mx, imu.mz);
#endif
    }
    WAIT(IMU_SENDING_PERIOD);
  }
}


void configureTasks()
{
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  UNUSED_VARIABLE(xTaskCreate(samplingData_task_function, "LED1", configMINIMAL_STACK_SIZE + 1500, NULL, 2, &samplingData_task_handle));
  UNUSED_VARIABLE(xTaskCreate(led2_toggle_task_function, "LED2", configMINIMAL_STACK_SIZE + 1500, NULL, 2, &led2_toggle_task_handle));
  //UNUSED_VARIABLE(xTaskCreate(thresholdAccelGyro_task_function, "thresholdAccel", configMINIMAL_STACK_SIZE + 200, NULL, 2, &thresholdAccelGyro_task_handle));
  UNUSED_VARIABLE(xTaskCreate(SendDataRead_task_function, "SendDataRead", configMINIMAL_STACK_SIZE + 1500, NULL, 2, &SendDataRead_task_handle));
  UNUSED_VARIABLE(xTaskCreate(IoTloop, "IoTloop", configMINIMAL_STACK_SIZE + 1500, NULL, 2, &IoTloop_task_handle));
}

