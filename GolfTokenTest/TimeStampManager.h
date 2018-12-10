#ifndef TIMESTAMPMANAGER
#define TIMESTAMPMANAGER
class TimestampManager
{
  public:
    void begin();
    unsigned long int get();
  private:
    unsigned long int Timestamp;
};
#endif
