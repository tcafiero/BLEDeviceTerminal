#include "TimeStampManager.h"

extern TimestampManager ts;

TimerHandle_t Timer1;

void tsTimer_callback()
{
  ts.inc();
}

void configureTimers()
{
  Timer1 = xTimerCreate
           ( /* Just a text name, not used by the RTOS
                     kernel. */
             "Timer1",
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
}


