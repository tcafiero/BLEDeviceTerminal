void printRaw( int32_t ts, int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
  char b[100];
  sprintf(b, "{\"ts\":%lu, \"a\":[%05d,%05d,%05d],", ts, ax, ay, az);
  bleout( b, strlen(b), 15 );
  sprintf(b, "\"g\":[%05d,%05d,%05d],", gx, gy, gz);
  bleout( b, strlen(b), 15 );
  sprintf(b, "\"m\":[%05d,%05d,%05d]}\n", mx, my, mz);
  bleout( b, strlen(b), 15 );
}

