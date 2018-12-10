void thresholdAccelGyro_isr()
{
  thresholdAccelGyro_flag = true;
}

void configureInterrupts()
{
  //attachInterrupt(INT1_PIN_THS, thresholdAccelGyro_isr, FALLING);
}

