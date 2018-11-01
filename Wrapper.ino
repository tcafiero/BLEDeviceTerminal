#include <MicroShell.h>

// Put here function prototype to expose with micro shell
char* TemplateFunc(int a, char* b);
char* Battery();
char* Send();
char* StopSend();
char* SendFastBuffer();
char* SendFastBuffer();
char* ResetTimestamp();

// Put here function wrapper
void TemplateFunc_wrapper(int argc, char **argv)
{
  int a;
  char s[20];
  argvscanf("%d %s", &a, s);
  #ifdef DEBUG
//  result("Result: %s\n", TemplateFunc(a, s));
  Serial.printf("Result: %s\n", TemplateFunc(a, s));
  #else
  TemplateFunc(a, s);  
  #endif
}

void ResetTimestamp_wrapper(int argc, char **argv)
{
  #ifdef DEBUG
  Serial.printf("Result: %s\n", ResetTimestamp());
  #else
  ResetTimestamp();  
  #endif
}

void Send_wrapper(int argc, char **argv)
{
  #ifdef DEBUG
  Serial.printf("Result: %s\n", Send());
  #else
  Send();  
  #endif
}

void StopSend_wrapper(int argc, char **argv)
{
  #ifdef DEBUG
  Serial.printf("Result: %s\n", StopSend());
  #else
  StopSend();  
  #endif
}

void Battery_wrapper(int argc, char **argv)
{
  #ifdef DEBUG
  Serial.printf("Result: %s\n", Battery());
  #else
  Battery();  
  #endif
}


//Put here Publish function table
PublishFunctionStruct PublishFunction[] =
{
  {"TemplateFunc", TemplateFunc_wrapper},
  {"Send", Send_wrapper},
  {"StopSend", StopSend_wrapper},
  {"BatteryStatus", Battery_wrapper},
  {"ResetTimestamp", ResetTimestamp_wrapper},
  {"", 0}
};

