#include "TimeStampManager.h"

extern TimestampManager ts;

TimerHandle_t Timer1;
TimerHandle_t Timer2;
TimerHandle_t Timer3;

void tsTimer_callback()
{
  ts.inc();
}

void ImuReadCallback( TimerHandle_t xTimer )
{
  record_t* record;
  imuRead();
  if(cb.getSemaphore() == RED) return;
  digitalToggle(PIN_LED1);
  record = cb.getRecord();
  record->a.x = imu.ax;
  record->a.y = imu.ay;
  record->a.z = imu.az;
  record->g.x = imu.gx;
  record->g.y = imu.gy;
  record->g.z = imu.gz;
  record->m.x = imu.mx;
  record->m.y = imu.my;
  record->m.z = imu.mz;
  record->ts = ts.get();
}

void SendDataReadCallback( TimerHandle_t xTimer )
{
  if (DoSend)
  {
    digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
    printRaw(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz);
#if 0
    printAttitude(imu.ax, imu.ay, imu.az,
                  -imu.my, -imu.mx, imu.mz);
#endif
  }
}

void configureTimers()
{
  Timer1 = xTimerCreate
           ( /* Just a text name, not used by the RTOS
                     kernel. */
             "Timer1",
             /* The timer period in ticks, must be
               greater than 0. */
             IMU_SAMPLING_PERIOD,
             /* The timers will auto-reload themselves
               when they expire. */
             pdTRUE,
             /* The ID is used to store a count of the
               number of times the timer has expired, which
               is initialised to 0. */
             ( void * ) 0,
             /* Each timer calls the same callback when
               it expires. */
             ImuReadCallback
           );
  Timer2 = xTimerCreate
           ( /* Just a text name, not used by the RTOS
                     kernel. */
             "Timer2",
             /* The timer period in ticks, must be
               greater than 0. */
             IMU_SENDING_PERIOD,
             /* The timers will auto-reload themselves
               when they expire. */
             pdTRUE,
             /* The ID is used to store a count of the
               number of times the timer has expired, which
               is initialised to 0. */
             ( void * ) 0,
             /* Each timer calls the same callback when
               it expires. */
             SendDataReadCallback
           );
  Timer3 = xTimerCreate
           ( /* Just a text name, not used by the RTOS
                     kernel. */
             "Timer3",
             /* The timer period in ticks, must be
               greater than 0. */
             10,
             /* The timers will auto-reload themselves
               when they expire. */
             pdTRUE,
             /* The ID is used to store a count of the
               number of times the timer has expired, which
               is initialised to 0. */
             ( void * ) 0,
             /* Each timer calls the same callback when
               it expires. */
             tsTimer_callback
           );
  xTimerStart( Timer1, 0 );
  xTimerStart( Timer2, 0 );
  xTimerStart( Timer3, 0 );
}


