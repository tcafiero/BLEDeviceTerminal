#include <MicroShell.h>
#include <bluefruit.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

volatile bool thresholdAccelGyro_flag = false;
volatile bool dataRdyAccelGyro_flag = false;


LSM9DS1 imu; // Create an LSM9DS1 object to use from here on.

///////////////////////////////
// Interrupt Pin Definitions //
///////////////////////////////
// These can be swapped to any available digital pin:
const int INT1_PIN_THS = ARDUINO_3_PIN; //3 INT1 pin to D3 - will be attached to gyro
const int INT2_PIN_DRDY = ARDUINO_4_PIN; //4 INT2 pin to D4 - attached to accel
const int INTM_PIN_THS = ARDUINO_5_PIN;  //5 INTM_PIN_THS pin to D5
const int RDYM_PIN = ARDUINO_8_PIN;  // RDY pin to D8



// BLE Service
BLEDis  bledis;
BLEUart bleuart;
BLEBas  blebas;


#define BUFFERSIZE 100

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
  uint16_t ts;
} record_t;

void bleout( char* b, int len, int blocklen )
{
  int block;
  for (block = 0; block < len / blocklen; block++)
  {
    bleuart.write(&b[block * blocklen], blocklen);
  }
  bleuart.write(&b[block * blocklen], len % blocklen);
}


class CyclicBuffer
{
  public:
    record_t record[BUFFERSIZE];
    void begin();
    record_t* getRecord();
    void sendCapturedRecords();
    char  getHead();
  private:
    char head;   // index for the top of the buffer
};


void CyclicBuffer::begin()
{
  head = BUFFERSIZE - 1;
  for (char  i = 0; i < BUFFERSIZE; i++)
  {
    record[i].a.x = 32768;
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
  record[(head + 1) % BUFFERSIZE].a.x = 1792;
  return &record[head];
}

void CyclicBuffer::sendCapturedRecords()
{
  char i = head;
  char j = 0;
  char b[100];
  do
  {
    if ((j++) >= BUFFERSIZE) return;
    i++;
    i %= BUFFERSIZE;
  }
  while (record[i].a.x == 32768);
  while (record[i].a.x != 32768)
  {
    sprintf(b, "{\"a\":[%05d,%05d,%05d],\n", record[i].a.x, record[i].a.y, record[i].a.z);
    bleout( b, strlen(b), 15 );
    sprintf(b, "\"g\":[%05d,%05d,%05d],\n", record[i].g.x, record[i].g.y, record[i].g.z);
    bleout( b, strlen(b), 15);
    sprintf(b, "\"m\":[%05d,%05d,%05d],\n", record[i].m.x, record[i].m.y, record[i].m.z);
    bleout( b, strlen(b), 15);
    sprintf(b, "\"t\":%d}\n", record[i].ts);
    bleout( b, strlen(b), 15);
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

// Software Timer for blinking RED LED
SoftwareTimer blinkTimer;

// Software Timer for IMU data sampling
SoftwareTimer imuDataSampling;

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

void imuDataSampling_callback()
{
  record_t* record;
  record = cb.getRecord();
  record->a.x = imu.ax;
  record->a.y = imu.ay;
  record->a.z = imu.az;
  record->g.x = imu.gx;
  record->g.y = imu.gy;
  record->g.z = imu.gz;
  record->m.x = imu.mx;
  record->m.y = imu.my;
  record->m.z = imu.mz;
#if 0
  Serial.println();
  Serial.print("A: ");
  Serial.print(imu.ax); Serial.print(", ");
  Serial.print(imu.ay); Serial.print(", ");
  Serial.println(imu.az);
  Serial.print("G: ");
  Serial.print(imu.gx); Serial.print(", ");
  Serial.print(imu.gy); Serial.print(", ");
  Serial.println(imu.gz);
  Serial.print("M: ");
  Serial.print(imu.mx); Serial.print(", ");
  Serial.print(imu.my); Serial.print(", ");
  Serial.println(imu.mz);
#endif
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
  // must be implemented reading from imu component
  // following only for testing purpose
  delay(20);
}


void triggerEventManager_callback()
{
  char b[100];
  int head = cb.getHead();
  if (IsConnect == true && DoSend == true)
  {
    sprintf(b, "{\"a:\"[%05d,%05d,%05d],\n", cb.record[head].a.x, cb.record[head].a.y, cb.record[head].a.z);
    bleuart.write( b, strlen(b) );
    sprintf(b, "\"g\":[%05d,%05d,%05d],\n", cb.record[head].g.x, cb.record[head].g.y, cb.record[head].g.z);
    bleuart.write( b, strlen(b) );
    sprintf(b, "\"m\":[%05d,%05d,%05d],\n", cb.record[head].m.x, cb.record[head].m.y, cb.record[head].m.z);
    bleuart.write( b, strlen(b) );
    sprintf(b, "\"t\":%05d,\n", cb.record[head].ts);
    bleuart.write( b, strlen(b) );
  }
}

// configureIMU sets up our LSM9DS1 interface, sensor scales
// and sample rates.
uint16_t configureIMU()
{
  // Set up Device Mode (I2C) and I2C addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.agAddress = LSM9DS1_AG_ADDR(1);
  imu.settings.device.mAddress = LSM9DS1_M_ADDR(1);

  // gyro.latchInterrupt controls the latching of the
  // gyro and accelerometer interrupts (INT1 and INT2).
  // false = no latching
  imu.settings.gyro.latchInterrupt = false
  ;

  // Set gyroscope scale to +/-245 dps:
  imu.settings.gyro.scale = 245;
  // Set gyroscope (and accel) sample rate to 14.9 Hz
  imu.settings.gyro.sampleRate = 1;
  // Set accelerometer scale to +/-2g
  imu.settings.accel.scale = 2;
  // Set magnetometer scale to +/- 4g
  imu.settings.mag.scale = 4;
  // Set magnetometer sample rate to 0.625 Hz
  imu.settings.mag.sampleRate = 0;

  // Call imu.begin() to initialize the sensor and instill
  // it with our new settings.
  return imu.begin();
}

void configureLSM9DS1Interrupts()
{
  /////////////////////////////////////////////
  // Configure INT1 - Gyro & Accel Threshold //
  /////////////////////////////////////////////
  // For more information on setting gyro interrupt, threshold,
  // and configuring the intterup, see the datasheet.
  // We'll configure INT_GEN_CFG_G, INT_GEN_THS_??_G,
  // INT_GEN_DUR_G, and INT1_CTRL.
  // 1. Configure the gyro interrupt generator:
  //  - ZHIE_G: Z-axis high event (more can be or'd together)
  //  - false: and/or (false = OR) (not applicable)
  //  - false: latch interrupt (false = not latched)
  imu.configGyroInt(ZHIE_G, false, false);
  // 2. Configure the gyro threshold
  //   - 500: Threshold (raw value from gyro)
  //   - Z_AXIS: Z-axis threshold
  //   - 10: duration (based on ODR)
  //   - true: wait (wait duration before interrupt goes low)
  imu.configGyroThs(500, Z_AXIS, 10, true);
  // 3. Configure accelerometer interrupt generator:
  //   - XHIE_XL: x-axis high event
  //     More axis events can be or'd together
  //   - false: OR interrupts (N/A, since we only have 1)
  imu.configAccelInt(XHIE_XL, false);
  // 4. Configure accelerometer threshold:
  //   - 20: Threshold (raw value from accel)
  //     Multiply this value by 128 to get threshold value.
  //     (20 = 2600 raw accel value)
  //   - X_AXIS: Write to X-axis threshold
  //   - 10: duration (based on ODR)
  //   - false: wait (wait [duration] before interrupt goes low)
  imu.configAccelThs(20, X_AXIS, 1, false);
  // 5. Configure INT1 - assign it to gyro interrupt
  //   - XG_INT1: Says we're configuring INT1
  //   - INT1_IG_G | INT1_IG_XL: Sets interrupt source to
  //     both gyro interrupt and accel
  //   - INT_ACTIVE_LOW: Sets interrupt to active low.
  //         (Can otherwise be set to INT_ACTIVE_HIGH.)
  //   - INT_PUSH_PULL: Sets interrupt to a push-pull.
  //         (Can otherwise be set to INT_OPEN_DRAIN.)
  imu.configInt(XG_INT1, INT1_IG_G | INT_IG_XL, INT_ACTIVE_LOW, INT_PUSH_PULL);

  ////////////////////////////////////////////////
  // Configure INT2 - Gyro and Accel Data Ready //
  ////////////////////////////////////////////////
  // Configure interrupt 2 to fire whenever new accelerometer
  // or gyroscope data is available.
  // Note XG_INT2 means configuring interrupt 2.
  // INT_DRDY_XL is OR'd with INT_DRDY_G
  imu.configInt(XG_INT2, INT_DRDY_XL | INT_DRDY_G, INT_ACTIVE_LOW, INT_PUSH_PULL);

  //////////////////////////////////////
  // Configure Magnetometer Interrupt //
  //////////////////////////////////////
  // 1. Configure magnetometer interrupt:
  //   - XIEN: axis to be monitored. Can be an or'd combination
  //     of XIEN, YIEN, or ZIEN.
  //   - INT_ACTIVE_LOW: Interrupt goes low when active.
  //   - true: Latch interrupt
  imu.configMagInt(XIEN, INT_ACTIVE_LOW, true);
  // 2. Configure magnetometer threshold.
  //   There's only one threshold value for all 3 mag axes.
  //   This is the raw mag value that must be exceeded to
  //   generate an interrupt.
  imu.configMagThs(10000);
}


void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("TopView Easy Stroke");
  Serial.println("---------------------\n");
  cb.begin();
  // Set up our Arduino pins connected to interrupts.
  // We configured all of these interrupts in the LSM9DS1
  // to be active-low.
  pinMode(INT2_PIN_DRDY, INPUT_PULLUP);
  pinMode(INT1_PIN_THS, INPUT_PULLUP);
  pinMode(INTM_PIN_THS, INPUT_PULLUP);

  // The magnetometer DRDY pin (RDY) is not configurable.
  // It is active high and always turned on.
  pinMode(RDYM_PIN, INPUT);

  // Turn on the IMU with configureIMU() (defined above)
  // check the return status of imu.begin() to make sure
  // it's connected.
  uint16_t status = configureIMU();
  if (!status)
  {
    Serial.print("Failed to connect to IMU - Status: 0x");
    Serial.println(status, HEX);
    while (1) ;
  }
  Serial.println("LSM9DS1 9DOF connected.");


  // After turning the IMU on, configure the interrupts:
  configureLSM9DS1Interrupts();


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

  // Initialize blinkTimer for 1000 ms and start it
  //blinkTimer.begin(1000, blink_timer_callback);
  //blinkTimer.start();
  Scheduler.startLoop(blink_timer_callback);
  Scheduler.startLoop(imuDataSampling_callback);
  Scheduler.startLoop(dataRdyAccelGyro_callback);
  Scheduler.startLoop(thresholdAccelGyro_callback);
  
  attachInterrupt(digitalPinToInterrupt(INT1_PIN_THS), thresholdAccelGyro_isr, FALLING);
  attachInterrupt(digitalPinToInterrupt(INT2_PIN_DRDY), dataRdyAccelGyro_isr, LOW);
  //attachInterrupt(INT1_PIN_THS, thresholdAccelGyro_isr, FALLING);
  //attachInterrupt(INT2_PIN_DRDY, dataRdyAccelGyro_isr, FALLING);

  IsConnect = false;
  DoSend = false;
}

void thresholdAccelGyro_isr()
{
  thresholdAccelGyro_flag = true;
}

void dataRdyAccelGyro_isr()
{
  dataRdyAccelGyro_flag = true;
}

void dataRdyAccelGyro_callback()
{
  if (digitalRead(INT2_PIN_DRDY) == LOW)
  //if (digitalRead(dataRdyAccelGyro_flag) == true)
  {
    dataRdyAccelGyro_flag = false;
    Serial.println("dataRdyAccelGyro_flag=true");
    if (imu.accelAvailable())
      imu.readAccel();
    if (imu.gyroAvailable())
      imu.readGyro();
  } //else Serial.println("dataRdyAccelGyro_flag=false");

  delay(2);
  waitForEvent();
}

void thresholdAccelGyro_callback()
{
  if (thresholdAccelGyro_flag)
  {
    thresholdAccelGyro_flag = false;
    Serial.println("it's me!");
    cb.sendCapturedRecords();
  }
  delay(2);
  waitForEvent();
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
void blink_timer_callback()
{
  digitalToggle(LED_RED);
  delay(1000);
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

