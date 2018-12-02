#ifndef TRIGGER_H
#define TRIGGER_H

class Trigger
{
  public:
    void begin();
    bool zeroCrossing(int16_t value);
  private:
    int16_t previous;
};
#endif
