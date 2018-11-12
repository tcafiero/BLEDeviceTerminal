#ifndef TIMESTAMPMANAGER
#define TIMESTAMPMANAGER
class TimestampManager
{
  public:
    void begin();
    void inc();
    unsigned long int get();
  private:
    unsigned long int Timestamp;
};
#endif