#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <MicroShell.h>
#include <bluefruit.h>
// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;


#define BUFFERSIZE 2

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor


typedef struct {
  float x;
  float y;
  float z;
} vector_t;

typedef struct {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t m;
  unsigned short int t;
  unsigned short int ts;
} record_t;



class CyclicBuffer
{
  public:
    record_t record[BUFFERSIZE];
    void clearRecords();
    //void setRecord(int ax, int ay, int az, int gx, int gy, int gz, int mx, int my, int mz, int t, int ts);
    record_t* CyclicBuffer::getRecord();
    void transmitRecords();
    char  getHead();
  private:
    char head;   // index for the top of the buffer
};


void CyclicBuffer::clearRecords()
{
  head = BUFFERSIZE - 1;
  for (char  i = 0; i < BUFFERSIZE; i++)
  {
    record[i].a.acceleration.x = 0xcafe;
  }
}

char CyclicBuffer::getHead()
{
  return head;
}

record_t* CyclicBuffer::getRecord()
{
  head++;
  head %= BUFFERSIZE;
  record[(head + 1) % BUFFERSIZE].a.acceleration.x = 0xcafe;
  return &record[head];
}

#if 0
void CyclicBuffer::setRecord(int ax, int ay, int az, int gx, int gy, int gz, int mx, int my, int mz, int t, int ts)
{
  head++;
  head %= BUFFERSIZE;
  record[head].a.x = ax;
  record[head].a.y = ay;
  record[head].a.z = az;
  record[head].g.x = gx;
  record[head].g.y = gy;
  record[head].g.z = gz;
  record[head].m.x = mx;
  record[head].m.y = my;
  record[head].m.z = mz;
  record[head].t = t;
  record[head].ts = ts;
  record[(head + 1) % BUFFERSIZE].a.x = 0xcafe;
}
#endif


void CyclicBuffer::transmitRecords()
{
  char i = head;
  char b[100];
  do
  {
    i++;
    i %= BUFFERSIZE;
  }
  while (record[i].a.acceleration.x == 0xcafe);
  while (record[i].a.acceleration.x != 0xcafe);
  {
    sprintf(b, "{\"a:\"[%5.2f,%5.2f,%5.2f],\n", record[head].a.acceleration.x, record[head].a.acceleration.y, record[head].a.acceleration.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"g\":[%5.2f,%5.2f,%5.2f],\n", record[head].g.gyro.x, record[head].g.gyro.y, record[head].g.gyro.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"m\":[%5.2f,%5.2f,%5.2f],\n", record[head].m.magnetic.x, record[head].m.magnetic.y, record[head].m.magnetic.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"t\":%d,\n", record[head].a.timestamp);
    bleuart.write( b, strlen(b) );
    delay(2);
    i++;
    i %= BUFFERSIZE;
  }
}

CyclicBuffer cb;




/*********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/
bool IsConnect, DoSend;
unsigned short int TestValue;



// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;
SoftwareTimer highFrequencySampling;
SoftwareTimer lowFrequencyTransmitting;

char* TemplateFunc(int a, char* b)
{
  static char buf[50];
  sprintf(buf, "a+1=%d and b=%s\n", a + 1, b);
  bleuart.write( buf, strlen(buf) );
  return buf;
}

char* Send()
{
  DoSend = true;
  return "ok";
}

char* StopSend()
{
  DoSend = false;
  return "ok";
}

char* SendFastBuffer()
{
  /* to be implemented */
  return "ok";
}

char* Battery()
{
  /* to be implemented */
  char buf[] = "100\%";
  bleuart.write( buf, strlen(buf) );
  return "ok";
}

char* ResetTimestamp()
{
  /* to be implemented */
  char buf[] = "ok";
  bleuart.write( buf, strlen(buf) );
  return "ok";
}

void highFrequencySampling_callback(/*TimerHandle_t xTimerID*/)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
   // must be implemented reading from imu component
  // following only for testing purpose
  record_t* record;
  record = cb.getRecord();
  lsm.read();
  sensors_event_t temp;
  //sensors_event_t a, m, g, temp;
  lsm.getEvent(&(record->a), &(record->m), &(record->g), &temp);
  Serial.print("Accel X: "); Serial.print(record->a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(record->a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(record->a.acceleration.z);     Serial.println(" m/s^2 ");

  //cb.setRecord(a.acceleration.x, a.acceleration.y, a.acceleration.z, m.magnetic.x, m.magnetic.y, m.magnetic.z, g.gyro.x, g.gyro.y, g.gyro.z, 0, 0);
  delay(2000);              // wait for 2 ms 

}


void lowFrequencyTransmitting_callback(/*TimerHandle_t xTimerID*/)
{
  char b[100];
  int head = cb.getHead();
  if (IsConnect == true && DoSend == true)
  {
    
    sprintf(b, "{\"a:\"[%5.2f,%5.2f,%5.2f],\n", cb.record[head].a.acceleration.x, cb.record[head].a.acceleration.y, cb.record[head].a.acceleration.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"g\":[%5.2f,%5.2f,%5.2f],\n", cb.record[head].g.gyro.x, cb.record[head].g.gyro.y, cb.record[head].g.gyro.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"m\":[%5.2f,%5.2f,%5.2f],\n", cb.record[head].m.magnetic.x, cb.record[head].m.magnetic.y, cb.record[head].m.magnetic.z);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    sprintf(b, "\"t\":%d,\n", cb.record[head].a.timestamp);
    bleuart.write( b, strlen(b) );
    delay(2);
    //Serial.print(b);
    //sprintf(b, "\"ts\":%04x}\n", cb.record[head].ts);
    //bleuart.write( b, strlen(b) );
    
    //Serial.print(b);
  }
  delay(2000);              // wait for 20 ms
}


void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("TopView Easy Stroke");
  Serial.println("---------------------\n");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();
#if 1
  if (!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS1 ... check your connections */
    Serial.println("Error: No LSM9DS1 detected. Check wiring");
    delay(200);
    while (1);
  }
#endif
  Serial.println("LSM9DS1 9DOF connected.");
  delay(200);
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  
  
  
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("TopViewEasyStroke");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);

  // Set up and start advertising
  startAdv();

  Serial.println("Please use a BLE Terminal applet to connect in UART mode");
  Serial.println("Once connected, enter commands(s) that you wish to send");
  InitMicroShell();
  IsConnect = false;
  DoSend = false;
  TestValue = 0;
  //highFrequencySampling.begin(20, highFrequencySampling_callback);
  //highFrequencySampling.start();
  //lowFrequencyTransmitting.begin(20, lowFrequencyTransmitting_callback);
  //lowFrequencyTransmitting.start();

  Scheduler.startLoop(lowFrequencyTransmitting_callback);
  Scheduler.startLoop(highFrequencySampling_callback);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_myIndex.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}

void loop()
{

  if ( bleuart.available() )
  {
    MicroShell();
  }

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);
  IsConnect = true;
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  IsConnect = false;
  Serial.println();
  Serial.println("Disconnected");
}

/**
   Software Timer callback is invoked via a built-in FreeRTOS thread with
   minimal stack size. Therefore it should be as simple as possible. If
   a periodically heavy task is needed, please use Scheduler.startLoop() to
   create a dedicated task for it.

   More information http://www.freertos.org/RTOS-software-timer.html
*/
void blink_timer_callback(TimerHandle_t xTimerID)
{
  (void) xTimerID;
  digitalToggle(LED_RED);
}

/**
   RTOS Idle callback is automatically invoked by FreeRTOS
   when there are no active threads. E.g when loop() calls delay() and
   there is no bluetooth or hw event. This is the ideal place to handle
   background data.

   NOTE: FreeRTOS is configured as tickless idle mode. After this callback
   is executed, if there is time, freeRTOS kernel will go into low power mode.
   Therefore waitForEvent() should not be called in this callback.
   http://www.freertos.org/low-power-tickless-rtos.html

   WARNING: This function MUST NOT call any blocking FreeRTOS API
   such as delay(), xSemaphoreTake() etc ... for more information
   http://www.freertos.org/a00016.html
*/


void rtos_idle_callback(void)
{
}

