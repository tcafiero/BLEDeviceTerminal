#include "TimeStampManager.h"

TimestampManager ts;

void TimestampManager::begin()
{
  Timestamp = xTaskGetTickCount();
}

unsigned long int TimestampManager::get()
{
  return xTaskGetTickCount()-Timestamp;
}

