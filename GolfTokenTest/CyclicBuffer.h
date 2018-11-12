#ifndef CYCLICBUFFER_H
#define CYCLICBUFFER_H
#define BUFFERSIZE 100
#include <SparkFunLSM9DS1.h>

typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} vector_t;

typedef struct {
  vector_t a;
  vector_t g;
  vector_t m;
  uint16_t t;
  unsigned long int ts;
} record_t;


class CyclicBuffer
{
  public:
    bool getSemaphore();
    void setSemaphore(bool value);
    void begin();
    record_t* getRecord();
    void sendCapturedRecords();
    void trigger(int samples);
  private:
    bool Semaphore;
    record_t record[BUFFERSIZE + 10];
    int head;   // index for the top of the buffer
    int n;
    int getHead();
    bool trigger_flag;
    int trigger_samples;
};
#endif
