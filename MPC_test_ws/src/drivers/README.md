# Drivers
The driver package contains all necessary drivers for the robot. The drivers to send voltages to the steering and driving motors (DAC communication), drivers for the encoders (Counterboard communication) and a driver for the IMU are provided.

## DAC
The [uldaq](https://www.mccdaq.com/PDFs/Manuals/UL-Linux/c/index.html) library of the company for Ubuntu is used to communicate with the DAC. 16 Channels are used (direction and speed / frequency) for the motors.

## Counterboard
The counterboard commmunication is also done with the [uldaq](https://www.mccdaq.com/PDFs/Manuals/UL-Linux/c/index.html) library of the company. The pulses are counted and then the angles and speed of the motors are calculated and published.

## IMU driver
The IMU driver for the RT-USB-9AXIS-00 is modified version of the driver in [github](https://github.com/rt-net/RT-USB-9AXIS-00). The driver in github can only read Serial data in the modified version the driver reads also ASCII (which we use with the IMU).
