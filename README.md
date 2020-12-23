# FreiStat - AD5941 Potentiostat
## About
This repo contains code to run electrochemical measurements with the AD5940 or AD5941 potentiostat controlled by an Adafruit Feather M0 Wifi (MCU).
The code compiles with the Arduino IDE and can be used with other compatible MCUs.
## Hardware
The following hardware was used:
* Adafruit Feather M0 Wifi
* "AD5941 FeatherWing"

Code is compatible with the AD5940!

## Library Requirements
### AD5940/AD5941 library
* Download the library files (ad5940.c and ad5940.h) from https://github.com/analogdevicesinc/ad5940lib
* Place the files into the project folder or create a folder "ad5940" in your local Arduino libraries directory
* Make sure that c++ compiler is supported. The ad5940.h file should contain the following:
```c++
#ifdef __cplusplus
extern "C" {
#endif
< content of .h file >
#ifdef __cplusplus
}
#endif
```
* Select the AD5940/AD5941 chip by uncommenting the following line in the ad5940.h file:
```c++
#define CHIPSEL_594X      /**< AD5940 or AD5941 */
 ```
* If you want to see debug info from the .c files (e.g. RampTest.c) uncomment the following line in the ad5940.h file:
```c++
#define ADI_DEBUG   /**< Comment this line to remove debug info. */
```

### LibPrintf
* Download the Arduino Printf library from https://github.com/embeddedartistry/arduino-printf to add support for the `printf()` function

### ArduinoPort.cpp
This file is the bridge between the Adafruit/Arduino MCU and the AD5940/AD5941 library. It provides all functions (SPI communication and interrupts)
to make the library work.
* Place the file into the project folder or create a folder "ad5940" in your local Arduino libraries directory
Include it in the project folder or in the 
## How to use the code
* 
* Ensure that no debug infos are output by commenting the following lines:
1) ad5940.h file:
```c++
//#define ADI_DEBUG   /**< Comment this line to remove debug info. */
```
2) .ino file:
```c++
//#define DEBUG
```
* Compile and upload the code using the Arduino IDE or other compatible IDEs (e.g. Visual Studio Code with Arduino extension)
* Set the correct COM port in the python script
* Run the python script