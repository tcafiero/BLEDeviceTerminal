TaskHandle_t  led_toggle_task1_handle;
TaskHandle_t  led_toggle_task2_handle;

static void led_toggle_task1_function (void * pvParameter)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    // Perform action here.
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    imuRead();
    /* Tasks must be implemented to never return... */
  }
}

static void led_toggle_task2_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);
  while (true)
  {
    //digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    //printAttitude(imu.ax, imu.ay, imu.az,
    //                  -imu.my, -imu.mx, imu.mz);
    /* Delay a task for a given number of ticks */
    vTaskDelay(500);

    /* Tasks must be implemented to never return... */
  }
}

void configureTasks()
{
  UNUSED_VARIABLE(xTaskCreate(led_toggle_task1_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task1_handle));
  UNUSED_VARIABLE(xTaskCreate(led_toggle_task2_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task2_handle));
}

