TaskHandle_t  led_toggle_task1_handle;
TaskHandle_t  led_toggle_task2_handle;
TaskHandle_t  thresholdAccelGyro_task_handle;
TaskHandle_t  SendDataRead_task_handle;

static void thresholdAccelGyro_task_function (void * pvParameter)
{
  while (1)
  {
    if (thresholdAccelGyro_flag)
    {
      thresholdAccelGyro_flag = false;
      cb.sendCapturedRecords();
    }
    vTaskDelay(10);
  }
}


static void led_toggle_task1_function (void * pvParameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xSamplingPeriod = IMU_SAMPLING_PERIOD;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil( &xLastWakeTime, xSamplingPeriod );
    xLastWakeTime = xTaskGetTickCount();
    // Perform action here.
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    imuRead();
    if (tr.zeroCrossing(imu.ay))
    {
      cb.sendCapturedRecords();
    }
    imuDataSampling(xLastWakeTime);
    /* Tasks must be implemented to never return... */
  }
}

static void led_toggle_task2_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  TickType_t xLastWakeTime;
  const TickType_t xSamplingPeriod = 500;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil( &xLastWakeTime, xSamplingPeriod );
    xLastWakeTime = xTaskGetTickCount();
    digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    //printAttitude(imu.ax, imu.ay, imu.az,
    //                  -imu.my, -imu.mx, imu.mz);
    /* Delay a task for a given number of ticks */
    vTaskDelay(500);
    /* Tasks must be implemented to never return... */
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
#elif
      printAttitude(imu.ax, imu.ay, imu.az,
                    -imu.my, -imu.mx, imu.mz);
#endif
    }
    vTaskDelay(IMU_SENDING_PERIOD);
  }
}


void configureTasks()
{
  UNUSED_VARIABLE(xTaskCreate(led_toggle_task1_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task1_handle));
  UNUSED_VARIABLE(xTaskCreate(led_toggle_task2_function, "LED1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task2_handle));
  //UNUSED_VARIABLE(xTaskCreate(thresholdAccelGyro_task_function, "LED1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &thresholdAccelGyro_task_handle));
  UNUSED_VARIABLE(xTaskCreate(SendDataRead_task_function, "LED1", configMINIMAL_STACK_SIZE + 200, NULL, 2, &SendDataRead_task_handle));
}

