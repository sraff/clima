/*-----------------------------------------------------------------------------
  Project:  home automation controller
  Module:   1-WIRE functions
  File:     owFuncs.ino
  Version:  1
  Author:   Stefano Raffaglio
  NoKeywords:

  Description: function and macro to access 1-WIRE clients

------------------------------------------------------------------------------

Change History:

Date	      Initials    Version 	Change Details
--------	  --------    -------	--------------
29/07/16      sraff       0         creation

-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Compiler option
-----------------------------------------------------------------------------*/
//#define CONSOLE_LOG
//#define TEMP_TUNING

/*-----------------------------------------------------------------------------
// System include files.
-----------------------------------------------------------------------------*/
#include <Arduino.h>
#ifdef ARDUINO_AVR_YUN
	#include <Bridge.h>
#endif
#ifdef CONSOLE_LOG
  #include <Console.h>
#endif

/*-----------------------------------------------------------------------------
// Project include files.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Module include files.
-----------------------------------------------------------------------------*/
#include "owFuncs.h"

/*-----------------------------------------------------------------------------
// MACRO
-----------------------------------------------------------------------------*/
#ifdef ARDUINO_AVR_YUN
	#define PRINTLN Console.println
	#define PRINT Console.print
#else
	#define PRINTLN Serial.println
	#define PRINT Serial.print
#endif

/*-----------------------------------------------------------------------------
// Constant definitions
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Variable definitions. (global scope)
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Function prototype declarations. (global scope)
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Function definitions. (global scope)
-----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
bool ResetAct(
    OneWire* ow,
    const byte addr[8]) // Input: SLAVE address
/*
 Output:  none
 Return:  TRUE in case of error

 function description - OW: reset activity register
-----------------------------------------------------------------------------*/
{
  ow->reset();
  ow->select(addr); // issue Match ROM command
  ow->write(DS2408_RESET_CMD);
  if (ow->read() != 0xAA)
    return true; // error
  return false; // ok
}

/*---------------------------------------------------------------------------*/
bool SetPio(
    OneWire* ow,
    const byte addr[8],  // Input: SLAVE address
    byte* value)         // In/Out: pointer to value
/*
 Return: TRUE in case of error

 function description - Set PIO to *x and read the value back
-----------------------------------------------------------------------------*/
{
  ow->reset();
  ow->select(addr); // issue Match ROM command
  ow->write(DS2408_CHANNEL_WRITE_CMD);
  ow->write(*value);
  ow->write(~(*value));
  if (ow->read() == 0xAA) {
    *value = ow->read();
    return false; // ok
  }
  return true; // error
}

/*---------------------------------------------------------------------------*/
bool GetPio(
    OneWire* ow,
    const byte addr[8],  // Input: SLAVE address
    byte* value)         // Output: registry value
/*
 Return: TRUE in case of error

 function description - Get a value from PIO register
-----------------------------------------------------------------------------*/
{
  byte buf[13];  // Put everything in the buffer so we can compute CRC easily.
  buf[0] = DS2408_PIO_READ_CMD;    // Read PIO Registers
  buf[1] = 0x88;    // LSB address
  buf[2] = 0x00;    // MSB address
  
  ow->reset();
  ow->select(addr); // issue Match ROM command
  ow->write_bytes(buf, 3, 0);
  ow->read_bytes(buf + 3, 10);   // 3 cmd bytes, 6 data bytes, 2 0xFF, 2 CRC16
  
#ifdef CONSOLE_LOG
  PrintBytes(addr, 8, false);
  PRINT(" = ");
#endif
  
  if ( OneWire::check_crc16(buf, 11, &buf[11]) ) {
    *value = buf[3];

#ifdef CONSOLE_LOG
    // First 3 bytes contain command, register address.
    PRINT(buf[3], BIN); // Current state  
    PRINT(", last=");
    PRINT(buf[4], BIN); // Last write
    PRINT(", activity=");
    PRINT(buf[5], BIN); // state change
    PRINT(", mask=");
    PRINT(buf[6], BIN); // mask
    PRINT(", control = ");
    PRINTLN(buf[7], BIN); // control
#endif
  
    return true;
  }

#ifdef CONSOLE_LOG
  PRINTLN("PIO CRC16 ERROR");
#endif

  return false; // CRC error  
}

#ifdef TEMP_TUNING
/*---------------------------------------------------------------------------*/
bool setResolution(
    OneWire* ow,
    const byte addr[8],
    byte newResolution)
/*
 Input:   none
 Output:  none
 Return:  none

 function description - set resolution of a device to 9, 10, 11, or 12 bits
 if new resolution is out of range, 9 bits is used.
-----------------------------------------------------------------------------*/
{
#ifdef CONSOLE_LOG
  PrintBytes(addr, 8, true);
#endif
  ow->reset();
  ow->select(addr);
  ow->write(WRITESCRATCH);
  ow->write(50);           // sets the high alarm temperature 
  ow->write(-50);          // sets the low alarm temperature
  ow->write(newResolution);
  ow->reset();
  return false;
}
#endif

/*---------------------------------------------------------------------------*/
bool GetTempC(
    OneWire* ow,
    const byte addr[8],
    double* value)
/*
 Input:   none
 Output:  none
 Return:  none

 function description - 
-----------------------------------------------------------------------------*/
{
  byte data[9];
  
  if ( !ow->reset() ) {
 #ifdef CONSOLE_LOG
    PRINTLN("RESET1 FAIL");
#endif
    return false;
  }
  ow->select(addr);
  ow->write(READSCRATCH);
  ow->read_bytes(data, 9);
  // new temp request
  if ( !ow->reset() ) {
 #ifdef CONSOLE_LOG
    PRINTLN("RESET2 FAIL");
#endif
    return false;
  }
  ow->select(addr);
  ow->write(STARTCONVO);
  if ( OneWire::crc8(data, 8) == data[8] ) {
    int raw = (data[1] << 8) | data[0]; // shifting bits up to form a 12 bit value
    switch ( data[4] ) {
    case TEMP_12_BIT:
      *value = (double)raw * 0.0625;
      break;
    case TEMP_11_BIT:
      *value = (double)(raw >> 1) * 0.125;
      break;
    case TEMP_10_BIT:
      *value = (double)(raw >> 2) * 0.25;
      break;
    case TEMP_9_BIT:
      *value = (double)( raw >> 3 ) * 0.5; // result is in Celsius with 0.1 degree accuracy
      break;
    }
    if ( *value < 85.0 ) return true;
    else return false;
  }
  return false;
}
