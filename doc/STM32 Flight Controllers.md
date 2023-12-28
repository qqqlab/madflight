# STM32 Flight Controllers

These are common microcontollers used in flight controllers, in order of power, from least to most:

| MCU | Clock | Data RAM | Flash (max) | Core | Flight Controller Software |
|-|-|-|-|-|-|
STM32F411 | 100MHz | 512K | 128K | M4 FPU | -B
STM32G473 | 170MHz | 512K | 128K | M4 FPU | -B
STM32F722 | 216MHz | 512K | 256K | M7 FPU | -B
STM32F745 | 216MHz | 1M | 256K | M7 FPU | -B
STM32F765 | 216MHz | 2M | 512K | M7 FPU | A-
STM32F405 | 168MHz | 1M | 128K | M4 FPU | AB
STM32F427 | 180MHz | 2M | 256K | M4 FPU | A-
STM32H743 | 480MHz | 2M | 512K | M7 FPU | AB
STM32H753 | 480MHz | 2M | 1024K | M7 FPU | A-

Flight Comntroller Software  
A: ArduPilot  
B: BetaFlight  
