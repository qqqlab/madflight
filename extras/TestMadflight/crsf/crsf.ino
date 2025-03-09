/*==========================================================================================
CRSF/ELRS receiver demo program

Connect receiver TX to HW_PIN_RCIN_RX, receiver RX to HW_PIN_RCIN_TX
==========================================================================================*/

#define MF_TEST MF_TEST_RCIN

#define RCIN_USE RCIN_USE_CRSF

//#define IMU_USE IMU_USE_SPI_MPU6500

#include <madflight.h>

CRSF crsf;
/*
#if defined ARDUINO_ARCH_ESP32
    const int HW_PIN_RCIN_RX  = 35;
    const int HW_PIN_RCIN_TX  = 32;
    HardwareSerial *rcin_Serial = &Serial1; //&Serial1 or &Serial2 (&Serial is used for debugging)
    void hal_setup()
    {
      rcin_Serial->setPins(HW_PIN_RCIN_RX, HW_PIN_RCIN_TX);
    }
#elif defined ARDUINO_ARCH_RP2040
    const int HW_PIN_RCIN_RX  = 1; //uart0: 1(default), 5, 13, 17   uart1: 5, 9(default)
    const int HW_PIN_RCIN_TX  = 0; //uart0: 0(default), 4, 12, 16   uart1: 4, 8(default)
    SerialUART *rcin_Serial = new SerialUART(uart0, HW_PIN_RCIN_TX, HW_PIN_RCIN_RX); //uart0 or uart1
    void hal_setup() 
    { 
    }
#else 
    #error "Unknown hardware architecture"
#endif
*/


//#define uart0 ((uart_inst_t *)uart0_hw) ///< Identifier for UART instance 0
//#define uart1 ((uart_inst_t *)uart1_hw) ///< Identifier for UART instance 1


#define UART_ID uart0

volatile uint32_t cnt = 0;
volatile uint32_t cnt2 = 0;
volatile uint32_t cnt3 = 0;
volatile uint32_t fr = 0;

volatile uint32_t cnt_rt = 0;
volatile uint32_t cnt_rx = 0;


uint32_t baud_actual;

static void __not_in_flash_func(_uart0IRQ)() {
  //fr = uart_get_hw(UART_ID)->fr;

  cnt++;

  if(uart_get_hw(uart0)->ris && UART_UARTRIS_RTRIS_BITS) cnt_rt++;
  if(uart_get_hw(uart0)->ris && UART_UARTRIS_RXRIS_BITS) cnt_rx++;

  // ICR is write-to-clear
  uart_get_hw(uart0)->icr = UART_UARTICR_RTIC_BITS | UART_UARTICR_RXIC_BITS;
  while (uart_is_readable(uart0)) {
      uint32_t raw = uart_get_hw(uart0)->dr;

/* ignore flags, this might insert some additional bytes
      if (raw & 0x400) {
          // break!
          continue;
          //NOTE: keep bad framing char, so that CRSF will resync quicker
//        } else if (raw & 0x300) { 
          // Framing, Parity Error.  Ignore this bad char
//            continue;
      }
*/
      uint8_t val = raw & 0xff;
      cnt2++;

      if(crsf.update(val)) cnt3++;
  }

}

void uart0_setup()  {
  //deinit
  irq_set_enabled(UART0_IRQ, false); //disable interrupt
  uart_set_irq_enables(UART_ID, false, false); // disable the UART interrupts
  uart_deinit(UART_ID);

  //set pins
  gpio_set_function(HW_PIN_RCIN_RX, GPIO_FUNC_UART);
  gpio_set_function(HW_PIN_RCIN_TX, GPIO_FUNC_UART);

  //properties
  baud_actual = uart_init(uart0, 420000); //The call will return the actual baud rate selected
  uart_set_format(uart0, 8, 1, UART_PARITY_NONE); //data format
  uart_set_hw_flow(uart0, false, false); //no hardware flow
  uart_set_fifo_enabled(UART_ID, true); //enable FIFOs

  //interrupts
  irq_set_exclusive_handler(UART0_IRQ, _uart0IRQ); //interrupt handler
  irq_set_enabled(UART0_IRQ, true); //enable interrupt
  uart_set_irq_enables(uart0, true, false); // enable the UART to send interrupts - RX only

  //FIFO sizes, do this last to make it stick - 0b000=4, 0b001=8, 0b010=16, 0b011=24, 0b100=28 bytes
  //uart_get_hw(UART_ID)->ifls = 0b010010; // interrupt rxfifo[5:3] & txfifo[2:0] level 1/2 full = 16 bytes  
  uart_get_hw(UART_ID)->ifls = 0b000010; // interrupt rxfifo[5:3] = 0 (4 bytes) & txfifo[2:0] = 2 (16 bytes)

}




void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("CRSF DEMO");
   
   uart0_setup();
}

void loop() {
  fr = uart_get_hw(UART_ID)->fr;

  Serial.printf(" baud:%d ", (int)baud_actual); 
  Serial.printf(" intr:%d", (int)cnt); cnt=0;
  Serial.printf(" bytes:%d", (int)cnt2); cnt2=0;
  Serial.printf(" msg:%d", (int)cnt3); cnt3=0;
  Serial.printf(" rx:%d", (int)cnt_rx); cnt_rx=0;
  Serial.printf(" rt:%d", (int)cnt_rt); cnt_rt=0;    
  Serial.printf(" fr:%X", (int)uart_get_hw(UART_ID)->fr);
  Serial.printf(" ifls:%X", (int)uart_get_hw(UART_ID)->ifls);
  Serial.printf("\n");    


  delay(1000);

  //simulate imu_loop()
  //delayMicroseconds(500);     
  //uint32_t ts = micros();
  //while(micros()-ts<500);
}



/*

uint32_t print_ts;


//does not work with delay
void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.println("CRSF DEMO");
   
    hal_setup();
    rcin_Serial->setFIFOSize(256);
    rcin_Serial->begin(CRSF_BAUD);


    print_ts = millis();
}

void loop() {
    //simulate imu_loop()
    //delayMicroseconds(100);     
    uint32_t ts = micros();
    //while(micros()-ts<500);

    if(rcin_Serial->available()) {
        int c = rcin_Serial->read();
        //print received data
        //if(c == CRSF_ADDRESS_FLIGHT_CONTROLLER) Serial.printf("\nreceived: ");
        //Serial.printf("%02x ",c);
        if(crsf.update(c)) {
            //print decoded rc data
            Serial.print(" decoded RC: ");
            for(int i=0;i<16;i++) Serial.printf("ch%d:%d ",i,crsf.channel[i]);
            Serial.println();
            print_ts = millis();
        }
    }

    //print connection status if nothing received for 1 second
    if(millis() - print_ts > 1000) {
       Serial.printf("is_connected:%d ",crsf.is_connected());
       for(int i=0;i<16;i++) Serial.printf("ch%d:%d ",i,crsf.channel[i]);
       Serial.println();
       print_ts = millis();
    }
}

*/