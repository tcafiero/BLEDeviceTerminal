#include "Trigger.h"
Trigger tr;

void Trigger::begin()
{
  previous=16000;
}

bool Trigger::zeroCrossing(int16_t value)
{
  if(value > 0)
  {
    if(previous < 0)
    {
      previous=value;
      return true;
    }
    else
    {
      previous=value;
      return false;
    };
  }
  else
  {
    previous=value;
    return false;
  };
}

