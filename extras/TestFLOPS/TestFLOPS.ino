
//ESP32     240MHz 1:dt=  2515 e=2.721910715 2:dt= 1682 e=2.721910715  -> 108.05 MLOOPS,  62.89 MFLOPS
//ESP32S3   240MHz 1:dt=  2100 e=2.721910715 2:dt= 1675 e=2.721910715  -> 211.77 MLOOPS,  61.43 MFLOPS

//RP2040    133MHz 1:dt= 54372 e=2.721910715 2:dt=50233 e=2.721910715  ->  21.74 MLOOPS,   2.00 MFLOPS
//RP2350    150MHz 1:dt=  2677 e=2.721910715 2:dt= 1421 e=2.721910715  ->  71.66 MLOOPS,  78.00 MFLOPS
//RP2350    300MHz 1:dt=  1336 e=2.721910715 2:dt=  709 e=2.721910715  -> 143.55 MLOOPS, 156.25 MFLOPS

//STM32F103  72MHz 1:dt=100353 e=2.721910715 2:dt=91298 e=2.721910715  ->   9.93 MLOOPS,   1.10 MFLOPS
//STM32F405 168MHz 1:dt=  2382 e=2.721910715 2:dt= 1328 e=2.721910715  ->  85.39 MLOOPS,  82.57 MFLOPS
//STM32F411 100MHz 1:dt=  4170 e=2.721910715 2:dt= 2301 e=2.721910715  ->  48.15 MLOOPS,  47.75 MFLOPS
//STM32G431 170MHz 1:dt=  2358 e=2.721910715 2:dt= 1306 e=2.721910715  ->  85.55 MLOOPS,  84.03 MFLOPS

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("TestFLOPS");
}

void loop() {
  int dt1 = run1();
  int dt2 = run2();
  int t_1M_loop = 1000 * (dt1-dt2) / 90;
  int t_1M_float = 10 * (dt2 - (dt1-dt2)/9);
  float MLOOPS = 1.0e6 / t_1M_loop;
  float MFLOPS = 1.0e6 / t_1M_float;
  Serial.printf(" -> %.2f MLOOPS, %.2f MFLOPS\n",MLOOPS,MFLOPS);
  delay(1000);
}

//100k loops + 100k mults
int run1() {
  int iter = 100000;
  float f = 1+1.0/iter;
  float e = 1;
  uint32_t t1=micros();
  for(int i=0;i<iter;i++) {
    e *= f;
  }
  uint32_t t2=micros();
  int dt = (int)(t2-t1);
  Serial.printf("1:dt=%d e=%.9f ",dt,e);
  return dt;
}

//10k loops + 100k mults
int run2() {
  int iter = 10000;
  float f = 1+0.1/iter;
  float e = 1;
  uint32_t t1=micros();
  for(int i=0;i<iter;i++) {
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
    e *= f;
  }
  uint32_t t2=micros();
  int dt = (int)(t2-t1);
  Serial.printf("2:dt=%d e=%.9f ",dt,e);
  return dt;
}