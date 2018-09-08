#include <MicroShell.h>
#include <bluefruit.h>
// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;


#define BUFFERSIZE 30
typedef struct {
  unsigned short int x;
  unsigned short int y;
  unsigned short int z;
} vector_t;

typedef struct {
  vector_t a;
  vector_t g;
  vector_t m;
  unsigned short int t;
  unsigned short int ts;
} record_t;

class CyclicBuffer
{
  public:
    record_t record[BUFFERSIZE];
    void clearRecords();
    void setRecord(int ax, int ay, int az, int gx, int gy, int gz, int mx, int my, int mz, int t, int ts);
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
    record[i].a.x = 0xcafe;
  }
}

char CyclicBuffer::getHead()
{
  return head;
}

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

void CyclicBuffer::transmitRecords()
{
  char i = head;
  char b[100];
  do
  {
    i++;
    i %= BUFFERSIZE;
  }
  while (record[i].a.x == 0xcafe);
  while (record[i].a.x != 0xcafe);
  {
    sprintf(b, "{\"a:\"[%x,%x,%x],\n", record[i].a.x, record[i].a.y, record[i].a.z);
    bleuart.write( b, strlen(b) );
    Serial.print(b);
    sprintf(b, "\"g\":[%x,%x,%x],\n", record[i].g.x, record[i].g.y, record[i].g.z);
    bleuart.write( b, strlen(b) );
    Serial.print(b);
    sprintf(b, "\"m\":[%x,%x,%x],\n", record[i].m.x, record[i].m.y, record[i].m.z);
    bleuart.write( b, strlen(b) );
    Serial.print(b);
    sprintf(b, "\"t\":%x,\n", record[i].t);
    bleuart.write( b, strlen(b) );
    Serial.print(b);
    sprintf(b, "\"ts\":%x}\n", record[i].ts);
    bleuart.write( b, strlen(b) );
    Serial.print(b);
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

void HighFrequencySampling(void)
{
  // must be implemented reading from imu component
  // following only for testing purpose
  cb.setRecord(TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++, TestValue++);
  delay(2);              // wait for 2 ms
}


void LowFrequencyTransmission(void)
{
  char b[100];
  int head=cb.getHead();
  if (IsConnect == true && DoSend == true)
  {
    sprintf(b, "{\"a:\"[%04x,%04x,%04x],\n", cb.record[head].a.x, cb.record[head].a.y, cb.record[head].a.z);
    bleuart.write( b, strlen(b) );
    //Serial.print(b);
    sprintf(b, "\"g\":[%04x,%04x,%04x],\n", cb.record[head].g.x, cb.record[head].g.y, cb.record[head].g.z);
    bleuart.write( b, strlen(b) );
    //Serial.print(b);
    sprintf(b, "\"m\":[%04x,%04x,%04x],\n", cb.record[head].m.x, cb.record[head].m.y, cb.record[head].m.z);
    bleuart.write( b, strlen(b) );
    //Serial.print(b);
    sprintf(b, "\"t\":%04x,\n", cb.record[head].t);
    bleuart.write( b, strlen(b) );
    //Serial.print(b);
    sprintf(b, "\"ts\":%04x}\n", cb.record[head].ts);
    bleuart.write( b, strlen(b) );
    //Serial.print(b);
  }
  delay(20);              // wait for 20 ms
}


void setup()
{
  Serial.begin(115200);
  Serial.println("TopView Easy Stroke");
  Serial.println("---------------------\n");

  // Initialize blinkTimer for 1000 ms and start it
  blinkTimer.begin(1000, blink_timer_callback);
  blinkTimer.start();

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
  TestValue=0;
  Scheduler.startLoop(LowFrequencyTransmission);
  Scheduler.startLoop(HighFrequencySampling);
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
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}

