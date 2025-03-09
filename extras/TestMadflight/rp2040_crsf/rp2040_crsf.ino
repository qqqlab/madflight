//==========================================================================================
// WORKS - rcin+imu
//==========================================================================================

#define RCIN_USE RCIN_USE_CRSF
#define IMU_USE IMU_USE_SPI_MPU6500
#define MF_TEST MF_TEST_RCIN | MF_TEST_IMU
//#define HW_RP2040_SYS_CLK_KHZ 266000

#define IMU_EXEC IMU_EXEC_FREERTOS //IMU_EXEC_IRQ does not work on RP2040

#include <madflight.h>

//in mf
//uint8_t rcin_txbuf[256];
//uint8_t rcin_rxbuf[1024];
//SerialIRQ *rcin_Serial = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf)); //uart0 or uart1

volatile uint32_t ts;
volatile uint32_t dt;

/*
void __not_in_flash_func(imu_loop)() {
  volatile uint32_t ts2 = micros();
  dt = ts2 - ts;
  ts = ts2;

  //nothing imurt=110

  delayMicroseconds(100); //does not work

  //while(micros() - ts2 < 100);  //works imurt=209

  //for(int i=0;i<10000;i++) ts2+=i; //works imurt=448
}
*/

void imu_loop() {
  volatile uint32_t ts2 = micros();
  dt = ts2 - ts;
  ts = ts2;

  //nothing imurt=110

  delayMicroseconds(100); //does not work in IRQ

  //while(micros() - ts2 < 100);    //works imurt=207

  //for(int i=0;i<10000;i++) ts2+=i;  //works imurt=451
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");
  hal_setup();
  imu.setup(1000);
  imu.onUpdate = imu_loop;
  rcin.setup();
}

void loop() {
  //Serial.print(rcin_Serial->available());
  //Serial.print(" ");
  Serial.print(" upd=");
  Serial.print(rcin.update());
  Serial.print(" pwm=");
  Serial.print(rcin.pwm[0]);
  Serial.print(" az=");
  Serial.print(imu.az); 
  Serial.print(" dt=");
  Serial.print(dt);
  Serial.print(" imurt=");
  Serial.print(imu.stat_runtime_max);          
  Serial.println();
  delay(100);
}




/*
//==========================================================================================
// WORKS - rcin
//==========================================================================================

#define RCIN_USE RCIN_USE_CRSF
#define MF_TEST MF_TEST_RCIN
#include <madflight.h>

//in mf
//uint8_t rcin_txbuf[256];
//uint8_t rcin_rxbuf[1024];
//SerialIRQ *rcin_Serial = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf)); //uart0 or uart1


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");
  rcin.setup();
}

void loop() {
  //Serial.print(rcin_Serial->available());
  //Serial.print(" ");
  Serial.print(rcin.update());
  Serial.print(" ");
  Serial.print(rcin.pwm[0]);
  Serial.println();
  delay(100);
}
*/

/*
//==========================================================================================
// WORKS - simple SerialIRQ with MF defines
//==========================================================================================



#define MF_TEST 0
#include <madflight.h>

//in mf
//uint8_t rcin_txbuf[256];
//uint8_t rcin_rxbuf[1024];
//SerialIRQ *rcin_Serial = new SerialIRQ(uart0, HW_PIN_RCIN_TX, rcin_txbuf, sizeof(rcin_txbuf), HW_PIN_RCIN_RX, rcin_rxbuf, sizeof(rcin_rxbuf)); //uart0 or uart1


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  rcin_Serial->begin(420000);  
}

void loop() {
  Serial.println(rcin_Serial->available());
  while(rcin_Serial->available()) rcin_Serial->read();
  delay(10);
}
*/

/*
//==========================================================================================
// NOPE - simple SerialPIO
//==========================================================================================

//RC Receiver:
#define HW_PIN_RCIN_RX            1 //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
#define HW_PIN_RCIN_TX            0 //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

SerialPIO uart(HW_PIN_RCIN_TX, HW_PIN_RCIN_RX, 32);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  uart.begin(420000);  
}

void loop() {
  Serial.println(uart.available());
  while(uart.available()) uart.read();
  delay(1000);
}
*/

/*
//==========================================================================================
// NOPE - simple SerialUART
//==========================================================================================

//RC Receiver:
#define HW_PIN_RCIN_RX            1 //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
#define HW_PIN_RCIN_TX            0 //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

SerialUART uart(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  uart.begin(420000);  
}

void loop() {
  Serial.println(uart.available());
  while(uart.available()) uart.read();
  delay(1000);
}
*/


/*
//==========================================================================================
// WORKS - simple SerialIRQ
//==========================================================================================

//RC Receiver:
#define HW_PIN_RCIN_RX            1 //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default) , this pin is also used as PPM input
#define HW_PIN_RCIN_TX            0 //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

#define MF_TEST 0
#include <madflight.h>

uint8_t txbuf[256];
uint8_t rxbuf[1024];
SerialIRQ uart(uart0, HW_PIN_RCIN_TX, txbuf, sizeof(txbuf), HW_PIN_RCIN_RX, rxbuf, sizeof(rxbuf));

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  uart.begin(420000);  
}

void loop() {
  Serial.println(uart.available());
  while(uart.available()) uart.read();
  delay(10);
}
*/



/*

//==========================================================================================
//RP2040 CRSF/ELRS receiver demo program
//
//Connect receiver TX to HW_PIN_RCIN_RX, receiver RX to HW_PIN_RCIN_TX
//==========================================================================================

#define MF_TEST MF_TEST_RCIN | MF_TEST_IMU

#define RCIN_USE RCIN_USE_CRSF

//#define HW_PIN_RCIN_RX  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
//#define HW_PIN_RCIN_TX  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)

#define IMU_USE IMU_USE_SPI_MPU6500

#define IMU_EXEC_ON_SECOND_CORE //QQQQ


#include <madflight.h>



#ifdef IMU_EXEC_ON_SECOND_CORE
volatile bool imu_setup_exec = false;
void setup1() {
  while(!imu_setup_exec);
  imu_setup();
  imu_setup_exec = false;
}
void imu_setup_core_select() {
  imu_setup_exec = true;
  Serial.print("IMU: Starting on second core: ");
  while(imu_setup_exec);  
}
#else
void imu_setup_core_select() {
  Serial.print("IMU: Starting on first core: ");
  imu_setup();
}
#endif

void imu_setup() {
  //IMU: keep on trying until no error is returned (some sensors need a couple of tries...)
  while(true) {
    int rv = imu.setup(1000); //request 1000 Hz sample rate, returns 0 on success, positive on error, negative on warning
    if(rv<=0) break;
    if(rv<=0) break;
    Serial.print("IMU: init failed rv= " + String(rv) + ". Retrying...\n");
  }
  Serial.println("IMU Started");
}

volatile int cnt_imu_loop=0;

void imu_loop() {
  cnt_imu_loop++;
  //simulate imu_loop()
  delayMicroseconds(200); 
}


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  //imu_setup_core_select();

  rcin.setup();

  //start IMU update handler
  //imu.onUpdate = imu_loop;
  //if(!imu.waitNewSample()) Serial.println("IMU interrupt not firing.");

  Serial.println("Starting...");
}

uint32_t ts = 0;
int msg_cnt = 0;
int loopno = 0;

void loop() {
  if(rcin.update()) msg_cnt++;

  if(millis() - ts > 1000) {
    loopno++;
    ts = millis();

    //Serial.printf(" baud:%d ", (int)uart.baud_actual); 
    Serial.printf(" cnt_imu_loop:%d", (int)cnt_imu_loop); cnt_imu_loop=0;
    Serial.printf(" az:%f", imu.az); 
    Serial.printf(" stat_runtime_max:%d", (int)imu.stat_runtime_max); imu.stat_runtime_max=0;
    
    //Serial.printf(" bytes:%d", (int)cnt_rxbytes); cnt_rxbytes=0;
    Serial.printf(" avail:%d", (int)rcin_Serial->available());
    Serial.printf(" msg_cnt:%d", (int)msg_cnt); msg_cnt=0;

    Serial.printf("\n");

    String s = String(loopno) + "=loop_b";  
    rcin_telemetry_flight_mode(s.c_str());

  }
}
*/









/*

CRSF crsf;

uint8_t txbuf[256];
uint8_t rxbuf[256];
SerialIRQ uart(uart0, HW_PIN_RCIN_TX, txbuf, sizeof(txbuf), HW_PIN_RCIN_RX, rxbuf, sizeof(rxbuf));

//SerialUART uart(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX);


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("CRSF DEMO");

  imu_setup_core_select();

  uart.begin(420000);

  //start IMU update handler
  imu.onUpdate = imu_loop;
  if(!imu.waitNewSample()) Serial.println("IMU interrupt not firing.");

  Serial.println("Starting...");
}

int loopno =0;

void loop() {
  loopno++;
  //SerialRingBuf *rbuf = &_rxbuf[0]; 
  //Serial.printf(" rbuf:%d ", (int)rbuf->len()); 
  //rbuf->push(1);
  

  //Serial.printf(" baud:%d ", (int)uart.baud_actual); 
  Serial.printf(" cnt_imu_loop:%d", (int)cnt_imu_loop); cnt_imu_loop=0;
  Serial.printf(" az:%f", imu.az); 
  Serial.printf(" stat_runtime_max:%d", (int)imu.stat_runtime_max); imu.stat_runtime_max=0;
  
  //Serial.printf(" bytes:%d", (int)cnt_rxbytes); cnt_rxbytes=0;
  Serial.printf(" avail:%d", (int)uart.available());

  int cnt_msg = 0;
  while(uart.available() > 0) {
    uint8_t c = uart.read();
    if(crsf.update(c)) cnt_msg++;
  }

  Serial.printf(" msg:%d", (int)cnt_msg); cnt_msg=0;
  Serial.printf(" ris:0x%X", (int)uart0_hw->ris );
  Serial.printf(" txbuf:%d", (int)_uart_txbuf[0].len());
   
  //Serial.printf(" fr:%X", (int)uart_get_hw(UART_ID)->fr);
  //Serial.printf(" ifls:%X", (int)uart_get_hw(UART_ID)->ifls);
   
  uint8_t msgbuf[2000];
  int msglen = 0;
  for(int i=0;i<20;i++) {
    String s = String(loopno*100 + i) + "=loop_aaaaaa";  
    msglen += CRSF_Telemetry::telemetry_flight_mode(msgbuf+msglen, s.c_str());
  }
  uint32_t dt = micros();
  uart.write(msgbuf,msglen);
  dt = micros() - dt;

  Serial.printf(" txbuf2:%d", (int)_uart_txbuf[0].len());
  Serial.printf(" msglen:%d", (int)msglen);
  Serial.printf(" dt:%d", (int)dt);
  Serial.printf(" txbaud:%d", (int)(10*1000000*msglen/dt));

  Serial.printf("\n"); 

  delay(1000);
}
*/
