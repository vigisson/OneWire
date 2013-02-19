1-Wire Communication Library for STM32Fxxx
===========

Author:    Vojtěch Vigner <vojtech.vigner@gmail.com>
Version:   V1.0.5
Date:      12-February-2013
Copyright: 2013, The BSD 3-Clause License.

This library provides 1-Wire bus support for STM32Fxxx devices.

Additional Information
-----------
This library provides functions to manage the following 
functionalities of the 1-Wire bus from MAXIM:           
    - Initialization and configuration.
    - Low level communication functions.
    - 1-Wire specific CRC calculation.
    - Device address operations.
    - Parasite powered device support.
    - 1-Wire advanced device search, based on MAXIM App. Note 126.

Library requires one USART for communication with 1-Wire devices. 
Current implementation used Half Duplex USART mode. This means that
only one pin is used for communication.  

Currently supports and has been tested on STM32F2xx and STM32F4xx
devices. 

How to use this library
-----------
1. Modify hardware specific section in OneWire.h file according to
your HW. Decide if you will be using parasite powered device/s and
enable or disable this support.

2. Initialize bus using OW_Init().

3. Now you can use all communication functions. 

4. See Example_OneWire.c for simple example. 

