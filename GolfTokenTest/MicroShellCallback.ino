#include "TimeStampManager.h"
bool DoSend;

extern TimestampManager ts;

char* TemplateFunc(int a, char* b)
{
  static char buf[50];
  sprintf(buf, "a+1=%d and b=%s\n", a + 1, b);
  //bleSerial.write( buf, strlen(buf) );
  bleSerial.print(buf);
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
  //char buf[] = "100\%";
  //bleSerial.write( buf, strlen(buf) );
  bleSerial.println("100\%");

  return "ok";
}

char* ResetTimestamp()
{
  /* to be implemented */
  ts.begin();
  //char buf[] = "Timestamp cleared.\n";
  //bleSerial.write( buf, strlen(buf) );
  bleSerial.print("Timestamp cleared.\n");
  return "ok";
}


