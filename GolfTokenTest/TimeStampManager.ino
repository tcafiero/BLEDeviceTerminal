#include "TimeStampManager.h"

TimestampManager ts;

void TimestampManager::begin()
{
  Timestamp = 0;
}
void TimestampManager::inc()
{
  Timestamp += 10;
}
unsigned long int TimestampManager::get()
{
  return Timestamp;
}

