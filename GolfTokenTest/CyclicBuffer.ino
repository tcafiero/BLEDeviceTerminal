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
    #ifdef PRINT_RAW
    sprintf(b, "{\"a\":[%05d,%05d,%05d]\n", record[i].a.x, record[i].a.y, record[i].a.z);
    #elif defined PRINT_CALCULATED
    // g
    sprintf(b, "{\"a\":[%6.2f,%6.2f,%6.2f]\n", imu.calcAccel(record[i].a.x), imu.calcAccel(record[i].a.y), imu.calcAccel(record[i].a.z));
    #endif
    bleout( b, strlen(b), 15 );
    #ifdef PRINT_RAW
    sprintf(b, "\"g\":[%05d,%05d,%05d]\n", record[i].g.x, record[i].g.y, record[i].g.z);
    #elif defined PRINT_CALCULATED
    // deg/s
    sprintf(b, "{\"g\":[%6.2f,%6.2f,%6.2f]\n", imu.calcGyro(record[i].g.x), imu.calcGyro(record[i].g.y), imu.calcGyro(record[i].g.z));
    #endif
    bleout( b, strlen(b), 15);
    #ifdef PRINT_RAW
    sprintf(b, "\"m\":[%05d,%05d,%05d]\n", record[i].m.x, record[i].m.y, record[i].m.z);
    #elif defined PRINT_CALCULATED
    // gauss
    sprintf(b, "{\"m\":[%6.2f,%6.2f,%6.2f]\n", imu.calcMag(record[i].m.x), imu.calcMag(record[i].m.y), imu.calcMag(record[i].m.z));
    #endif
    bleout( b, strlen(b), 15);
    sprintf(b, "\"t\":%lu}\n", record[i].ts);
    bleout( b, strlen(b), 15);
    i++;
    i %= BUFFERSIZE;
  }
  setSemaphore(GREEN);
}



