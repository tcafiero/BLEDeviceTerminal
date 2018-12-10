#include "CyclicBuffer.h"
CyclicBuffer cb;


void CyclicBuffer::begin()
{
  head = 0;
  n = 0;
  Semaphore = GREEN;
  trigger_flag = RED;
}

void CyclicBuffer::setSemaphore(bool value)
{
  Semaphore = value;
}

bool CyclicBuffer::getSemaphore()
{
  return Semaphore;
}

void CyclicBuffer::trigger(int samples)
{
  if (samples != 0)
  {
    trigger_flag = GREEN;
    trigger_samples = samples;
  }
  else
  {
    if (trigger_flag == GREEN)
    {
      if (--trigger_samples <= 0)
      {
        trigger_flag = RED;
        sendCapturedRecords();
      }
    }
  }
}


int CyclicBuffer::getHead()
{
  if (n == 0) return BUFFERSIZE;
  else return (head + 1) % n;
}

record_t* CyclicBuffer::getRecord()
{
  record_t* record_ptr;
  n++;
  if (n > BUFFERSIZE) n = BUFFERSIZE;
  if (n == 1)
  {
    record_ptr = &record[0];
    n = 1;
  }
  else {
    record_ptr = &record[head];
  }
  head++;
  head %= BUFFERSIZE;
  return record_ptr;
}

void CyclicBuffer::sendCapturedRecords()
{
  char b[100];
  if (n == 0) return;
  setSemaphore(RED);
  int i = getHead();
  for (int j = 0; j < n-1; j++)
  {
    #ifdef PRINTRAW
    printRaw( record[i].ts, record[i].a.x, record[i].a.y, record[i].a.z, record[i].g.x, record[i].g.y, record[i].g.z, record[i].m.x, record[i].m.y, record[i].m.z);
    #else
    printAttitude( (float)record[i].a.x, (float)record[i].a.y, (float)record[i].a.z, (float)record[i].m.x, (float)record[i].m.y, (float)record[i].m.z);
    #endif
    i++;
    i %= BUFFERSIZE;
  }
  setSemaphore(GREEN);
}



