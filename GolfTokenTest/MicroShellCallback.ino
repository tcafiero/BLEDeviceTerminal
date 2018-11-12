#include "TimeStampManager.h"
bool DoSend;
bool thresholdAccelGyro_flag;

extern TimestampManager ts;

char* TemplateFunc(int a, char* b)
{
  static char buf[50];
  sprintf(buf, "a+1=%d and b=%s\n", a + 1, b);
  bleuart.write( buf, strlen(buf) );
  return buf;
}

char* Send()
{
  DoSend = true;
  thresholdAccelGyro_flag = true;
  return "ok";
}

char* StopSend()
{
  DoSend = false;
  return "ok";
}

char* SendFastBuffer()
{
  /* to be implemented */
  return "ok";
}

char* BatteryStatus()
{
  /* to be implemented */
  char buf[] = "100\%";
  bleuart.write( buf, strlen(buf) );
  return "ok";
}

char* ResetTimestamp()
{
  /* to be implemented */
  ts.begin();
  char buf[] = "Timestamp cleared.\n";
  bleuart.write( buf, strlen(buf) );
  return "ok";
}


