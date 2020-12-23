# AD594x - Arduino#
## About
This repo contains code to run electrochemical measurements with the AD5940 or AD5941 potentiostat controlled by an Adafruit Feather M0 Wifi (MCU).
The code compiles with the Arduino IDE and can be used with other compatible MCUs.
## Hardware

## How to use the code
###AD5940/AD5941 library
* Download the library files (ad5940.c and ad5940.h) from https://github.com/analogdevicesinc/ad5940lib
* Make sure that c++ compiler is supported:
#ifdef __cplusplus
extern "C" {
#endif
< content of .h file >
#ifdef __cplusplus
}
#endif
* select the AD5940/AD5941 chip by uncommenting the following line in the ad5940.h file:
#define CHIPSEL_594X      /**< AD5940 or AD5941 */ 
* if you want to see debug info from the .c files (e.g. RampTest.c) uncomment the following line in the ad5940.h file:
#define ADI_DEBUG   /**< Comment this line to remove debug info. */