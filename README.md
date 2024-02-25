# Adressable LEDs Controller: User Manual
### Work In Progress

## PWM reading:
#### with current configuration, we can read upto 7 codes using PWM.
### How to Use PWM reading?(RoboRio side)
* The RoboRio's PWM frequency must be at max(150kHz).

code|duty range|optimal duty
:--:|:--------:|:----------:
0   |0% - 15%  |0%
1   |15% - 30% |22.5%
2   |30% - 45% |37.5%
3   |45% - 60% |52.5%
4   |60% - 75% |67.5%
5   |75% - 90% |82.5%
6*  |90% - 100%|97.5%
##### *DO NOT SET DUTY TO 100%!
##### *might be less stable

## Flashing MCU from src:
1) obtain all files
2) compile: `avr-gcc -Os -DF_CPU=8000000L -mmcu=attiny85 main.c -o main`
3) convert to hex: `avr-objcopy -R .eeprom -O ihex main main.hex`
4) flash: `sudo avrdude -F -v -c stk500v1 -p attiny85 -U flash:w:main.hex -P /dev/ttyACM0 -b 19200`
##### Note: instructions for flashing using Arduino as ISP with Uno board on a linux machine.
