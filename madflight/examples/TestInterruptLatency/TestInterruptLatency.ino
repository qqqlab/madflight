//program to test Interupt and Task Latency for ESP32 and RP2040

/*==============================================================================================================================
2023-12-11 Results:

ESP32  tick_MHz:240   test_toggle_fast:50 ns   test_toggle:334 ns   
ESP32 TEST_NOTIFY("official")      HigherPriorityTaskWoken:1    tick_MHz:240     pin_set:58 ns       ISR_latency:1662 ns         ISR_Task_Latency:10558 ns 		
ESP32 TEST_NOTIFY("shortcut")      HigherPriorityTaskWoken:-1   tick_MHz:240     pin_set:54 ns       ISR_latency:2129 ns         ISR_Task_Latency:17025 ns 
ESP32 TEST_SEMAPHORE("official")   HigherPriorityTaskWoken:1    tick_MHz:240     pin_set:58 ns       ISR_latency:2120 ns         ISR_Task_Latency:12283 ns
ESP32 TEST_SEMAPHORE("shortcut")   HigherPriorityTaskWoken:-1   tick_MHz:240     pin_set:54 ns       ISR_latency:2125 ns         ISR_Task_Latency:18312 ns 		

RP2040  tick_MHz:133   test_toggle_fast:10-13 ns   test_toggle:695-701 ns 
RP2040 TEST_NOTIFY("official")     HigherPriorityTaskWoken:0    tick_MHz:133     pin_set:1330-1436   ISR_latency:20428-22571 ns  ISR_Task_Latency:49451-60022 ns
RP2040 TEST_NOTIFY("shortcut")     HigherPriorityTaskWoken:-1   tick_MHz:133     pin_set:1330 ns     ISR_latency:16218-19827 ns  ISR_Task_Latency:49804-54766 ns 	
RP2040 TEST_SEMAPHORE("official")  HigherPriorityTaskWoken:0    tick_MHz:133     pin_set:1330 ns     ISR_latency:17067-24165 ns  ISR_Task_Latency:-999999 ns  -- need reset to reprogram 
RP2040 TEST_SEMAPHORE("shortcut")  HigherPriorityTaskWoken:-1   tick_MHz:133     pin_set:1330 ns     ISR_latency:10962-15518 ns  ISR_Task_Latency:-999999 ns  -- need reset to reprogram 

===============================================================================================================================*/

//------------------------------------------
// configuration
//------------------------------------------
#define PIN 12

//uncomment one
//#define TEST_SEMAPHORE
#define TEST_NOTIFY
#define TEST_PINTOGGLE
//------------------------------------------

#if defined ARDUINO_ARCH_ESP32
  #define ARCH "ESP32"
  #define RTOS_HIGHEST_PRIORITY 31
  #define PIN_MASK (1<<PIN)
  #define PIN_HI() GPIO.out_w1ts = PIN_MASK
  #define PIN_LO() GPIO.out_w1tc = PIN_MASK
  #define TICKS() ESP.getCycleCount()
  #define TICKS_MHZ ESP.getCpuFreqMHz()
#elif defined ARDUINO_ARCH_RP2040
  #define ARCH "RP2040"
  #include <FreeRTOS.h>
  #include "semphr.h"
  #define RTOS_HIGHEST_PRIORITY 7
  #define PIN_MASK (1<<PIN)  
  #define PIN_HI() gpio_set_mask(PIN_MASK)
  #define PIN_LO() gpio_clr_mask(PIN_MASK)
  #define TICKS() rp2040.getCycleCount()
  #define TICKS_MHZ (rp2040.f_cpu()/1000000)
  //#define IRAM_ATTR __not_in_flash("my_group_name")
#else
  #define ARCH "ARCH_UNKNOWN"
  #define RTOS_HIGHEST_PRIORITY 31
  #define PIN_HI() digitalWrite(PIN, 1)
  #define PIN_LO() digitalWrite(PIN, 0)
  #define TICKS() micros()
  #define TICKS_MHZ 1
#endif

uint32_t t1;
volatile uint32_t t2;
volatile uint32_t t2b;
volatile uint32_t t3;
int HigherPriorityTaskWoken = -1;

#if defined TEST_SEMAPHORE
//=======================================================================================
//  TEST_SEMAPHORE
//=======================================================================================
#define TEST_TYPE "TEST_SEMAPHORE"
SemaphoreHandle_t IsrSemaphore;

void task_setup() {
  xTaskCreate(IsrTask, "IsrTask", 4096, NULL, RTOS_HIGHEST_PRIORITY /*priority 0=lowest*/, NULL);
  //vTaskCoreAffinitySet(IsrTaskHandle, 0);
  IsrSemaphore = xSemaphoreCreateBinary();
  if(!IsrSemaphore) {Serial.println("xSemaphoreCreateBinary FAILED"); delay(100);}
  attachInterrupt(digitalPinToInterrupt(PIN), IsrHandler, RISING); 
}

void IsrTask(void*) {
  for(;;) {
    if(xSemaphoreTake(IsrSemaphore, portMAX_DELAY) == pdPASS) {
      t3 = TICKS();
    }
  }
}

void IsrHandler() {
  t2 = TICKS();
  PIN_LO();
  t2b = TICKS();

  // "official"
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // xSemaphoreGiveFromISR() will set *pxHigherPriorityTaskWoken to pdTRUE if giving the semaphore caused a task to unblock, 
  // and the unblocked task has a priority higher than the currently running task. If xSemaphoreGiveFromISR() sets this value
  // to pdTRUE then a context switch should be requested before the interrupt is exited.
  xSemaphoreGiveFromISR(IsrSemaphore, &xHigherPriorityTaskWoken);
  HigherPriorityTaskWoken = xHigherPriorityTaskWoken;
  // Exit to context switch if necessary
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  // "shortcut"
  //xSemaphoreGiveFromISR(IsrSemaphore, NULL);
}

#elif defined TEST_NOTIFY
//=======================================================================================
//  TEST_NOTIFY
//=======================================================================================
#define TEST_TYPE "TEST_NOTIFY"
TaskHandle_t IsrTaskHandle;

void task_setup() {
  xTaskCreate(IsrTask, "IsrTask", 4096, NULL, RTOS_HIGHEST_PRIORITY /*priority 0=lowest*/, &IsrTaskHandle);
  //vTaskCoreAffinitySet(IsrTaskHandle, 0);
  attachInterrupt(digitalPinToInterrupt(PIN), IsrHandler, RISING); 
}

void IsrTask(void*) {
  for(;;) {
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    t3 = TICKS();
  }
}

void IsrHandler() {
  t2 = TICKS();
  PIN_LO();
  t2b = TICKS();

  // "official"
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(IsrTaskHandle, &xHigherPriorityTaskWoken);
  HigherPriorityTaskWoken = xHigherPriorityTaskWoken;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

  // "shortcut"
  //vTaskNotifyGiveFromISR(IsrTaskHandle, NULL);
}
#endif

//returns toggle time in nanoseconds
uint32_t test_toggle_fast() {
  uint32_t t1 = TICKS();
  for(int i=0;i<1000;i++) { //10k toggles
    PIN_HI();
    PIN_LO();
    PIN_HI();
    PIN_LO();
    PIN_HI();
    PIN_LO();
    PIN_HI();
    PIN_LO();
    PIN_HI();
    PIN_LO();
  }
  uint32_t t2 = TICKS();
  return (t2-t1)/TICKS_MHZ/10;
}

//returns toggle time in nanoseconds
uint32_t test_toggle() {
  uint32_t t1 = TICKS();
  for(int i=0;i<1000;i++) { //10k toggles
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
    digitalWrite(PIN, HIGH);
    digitalWrite(PIN, LOW);
  }
  uint32_t t2 = TICKS();
  return (t2-t1)/TICKS_MHZ/10;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  pinMode(PIN,OUTPUT);
  PIN_LO();

  for(int i=0;i<10;i++) {
    Serial.print(ARCH "  ");
    Serial.printf("tick_MHz:%d   ", TICKS_MHZ);    
    Serial.printf("test_toggle_fast:%d ns   ", (int)test_toggle_fast());    
    Serial.printf("test_toggle:%d ns   ", (int)test_toggle());
    Serial.println();
  }

  task_setup();
}


void loop() {
  t3 = 0;  
  t2 = 0;
  t1 = TICKS();
  PIN_HI();
  delay(100);
  uint32_t dt_pin_set = t2b - t2;
  t1 += dt_pin_set; //adjust start time to after pin_set
  Serial.print(ARCH " " TEST_TYPE "  ");
  Serial.printf("HigherPriorityTaskWoken:%d   ", HigherPriorityTaskWoken);  
  //Serial.printf("configTEST_PREEMPTION:%d   ", configTEST_PREEMPTION);
  //Serial.printf("configNUM_CORES:%d   ", configNUM_CORES);  
  Serial.printf("tick_MHz:%d   ", TICKS_MHZ);
  Serial.printf("pin_set:%d ns   ",  (int)(t2==0 ? -999999 : (1000*dt_pin_set)/TICKS_MHZ) );
  Serial.printf("ISR_latency:%d ns   ", (int)(t2==0 ? -999999 : (1000*(t2-t1))/TICKS_MHZ) );
  Serial.printf("ISR_Task_Latency:%d ns   ", (int)(t3==0 ? -999999 : (1000*(t3-t1))/TICKS_MHZ) );
  Serial.println();
  PIN_LO();
}
