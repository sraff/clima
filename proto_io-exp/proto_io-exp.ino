/*-----------------------------------------------------------------------------
  Project:  home automation controller
  Module:   main controller
  File:     proto_io-exp.ino
  Version:  1
  Author:   Stefano Raffaglio
  NoKeywords:

  Description:
  
  HW init, main continuosly loops for:
    - production and cunsumpion calc
    - temperature monitoring
    - waiting for new msg from uC
    - watch dog on critical timer

------------------------------------------------------------------------------

Change History:

Date	      Initials    Version 	Change Details
--------	  --------    -------	  --------------
29/07/16    sraff       0         creation

-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Compiler option
-----------------------------------------------------------------------------*/
//#define CONSOLE_LOG

/*-----------------------------------------------------------------------------
// System include files.
-----------------------------------------------------------------------------*/
#ifdef ARDUINO_AVR_YUN
	#include <Bridge.h>
#endif
#ifdef CONSOLE_LOG
  #include <Console.h>
#endif

/*-----------------------------------------------------------------------------
// Project include files.
-----------------------------------------------------------------------------*/
//#include <EEPROM.h> // TODO: move OW addr to EEPROM
#include <EmonLib.h>
#include <Wire.h>
#include <OneWire.h> // custom version from https://github.com/sraff/OneWire
#include <Adafruit_MCP23017.h>

/*-----------------------------------------------------------------------------
// Module include files.
-----------------------------------------------------------------------------*/
#include "proto_io-exp.h"
#include "owFuncs.h"

/*-----------------------------------------------------------------------------
// Constant definitions
-----------------------------------------------------------------------------*/
const Msg MSG_NULL = {O_NULL, O_NULL};

/*-----------------------------------------------------------------------------
// Variable definitions. (global scope)
-----------------------------------------------------------------------------*/
Adafruit_MCP23017 mcp;
OneWire ow1(I2C_ONEW, true); // init 1-wire with DS2482 d0=d1=0
OneWire ow2(10, false); // init 1-wire with Data into D10
EnergyMonitor emonPv, emonLo;
unsigned long tapSeMilli, tapSoMilli, tapNoMilli, lastMilli;
double pv, load, maxPv, maxLd;
int loopCnt;
unsigned int error;

/*-----------------------------------------------------------------------------
// Function prototype declarations. (global scope)
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Function definitions. (global scope)
-----------------------------------------------------------------------------*/

#ifdef CONSOLE_LOG
/*---------------------------------------------------------------------------*/
void PrintBytes(
  const byte* addr,     // Input: buffer addr
  uint8_t count,        // Input: num of byte to print to console
  bool newline = true)
/*
 Output:  none
 Return:  none

 function description - print count number of byte
-----------------------------------------------------------------------------*/
{
  if (newline)
    PRINTLN();
  for (uint8_t i = 0; i < count; i++) {
    PRINT(addr[i] >> 4, HEX);
    PRINT(addr[i] & 0x0f, HEX);
  }
}
#endif // CONSOLE_LOG

/*---------------------------------------------------------------------------*/
byte potGet(
  uint8_t addr, // Input: I2C address
  byte select)  // Input: potentiometer to be read
/*
 Return:  actual value

 function description - read a digital potentiometer
-----------------------------------------------------------------------------*/
{
  int c=0;

  Wire.requestFrom(addr, select);   // request 2 bytes from slave device #89
  while (Wire.available())   // slave may send less than requested
  {
    c = Wire.read(); // receive a byte
  }
  return c;
}

/*---------------------------------------------------------------------------*/
byte potSet(
  uint8_t addr,     // Input: I2C address
  byte x,           // Input: value to write
  byte select = 3)  // Input: potentiometer to be read
/*
 Return:  read back value

 function description - set and read back a digital potentiometer
-----------------------------------------------------------------------------*/
{
  Wire.beginTransmission(addr); // device address is specified in datasheet
  switch (select) {
    case 1:
      Wire.write(byte(0xa9));   // sends instruction byte (WRITE POT0: 1010 1001)
      break;
    case 2:
      Wire.write(byte(0xaa));   // sends instruction byte (WRITE POT1: 1010 1010)
      break;
    case 3:
      Wire.write(byte(0xaf));   // sends instruction byte (WRITE POT0/1: 1010 1111 - same value)
      break;
    default:
      return 0;
  }
  Wire.write(x);            // sends potentiometer value byte
  Wire.endTransmission();   // stop transmitting

  return potGet(addr, select);  // request up to 2 bytes from slave device #89
}

/*---------------------------------------------------------------------------*/
void setup()
/*

 function description - board and peripheral init
-----------------------------------------------------------------------------*/
{
  byte status;

  error=0; // error is set to 0 only at setup, linino is responsable to reset it
  
#ifdef ARDUINO_AVR_YUN
  Bridge.begin();
  Bridge.put("E", "0");
#else
  Serial.begin(57600);
#endif
  
#ifdef CONSOLE_LOG  
  PRINTLN(F("\n-- SETUP START -- "));
#endif
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on
  
  // set pin D9 to input (with internal pullup), active LOW: Booster switch status,
  pinMode(9, INPUT_PULLUP);  

  // Init the MCP: set I/O and ena/dis pullupp (address 001, Wire.begin() already called inside...)
  mcp.begin(I2C_MCP1, MCP23017_GPA_IO, MCP23017_GPB_IO);

#ifdef ARDUINO_AVR_YUN
  Bridge.put("A", String(mcp.readGPIO(0)));
  Bridge.put("B", String(mcp.readGPIO(1)));
  Bridge.put("D9", String(digitalRead(9)));
#endif
  
#ifdef CONSOLE_LOG
  PRINT(F("MPC GPA: "));
  PRINTLN(mcp.readGPIO(0), BIN); // read GPA
  PRINT(F("MPC GPB: "));
  PRINTLN(mcp.readGPIO(1), BIN); // read GPB
  PRINT(F("D9: "));
  PRINTLN(digitalRead(9)); // read D9 and print
#endif

  // init 1-wire TAPPA to 0 (all I/O channel active HIGH) and reset all activity (state change) register
  status = T_INIT_STATE;
  if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE], &status) || ResetAct(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE]) ) error |= ERROR_PIO_TAPPA_SET;
#ifdef ARDUINO_AVR_YUN
  else Bridge.put("T", String(status));
#endif  

  // init 1-wire ALARM to 1 (all I/O channel active LOW) and reset the activity (state change) register
  status = A_INIT_STATE;
  if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME], &status) || ResetAct(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME]) ) error |= ERROR_PIO_ALARM_SET;
#ifdef ARDUINO_AVR_YUN
  else Bridge.put("I", String(status));
#endif

  // initialise all digital POTs to middle value
#ifdef ARDUINO_AVR_YUN
  Bridge.put("P", String(potSet(I2C_POT100K, 252, 1)));
  Bridge.put("Q", String(potSet(I2C_POT100K, 252, 2)));
  Bridge.put("R", String(potSet(I2C_POT10K, 127, 1)));
  Bridge.put("S", String(potSet(I2C_POT10K, 127, 2)));
#endif
#ifdef CONSOLE_LOG
  PRINT(F("POT100K-A: "));
  PRINTLN(potSet(I2C_POT100K, 252, 1));
  PRINT(F("POT100K-B: "));
  PRINTLN(potSet(I2C_POT100K, 252, 2));
  PRINT(F("POT10K-A: "));
  PRINTLN(potSet(I2C_POT10K, 127, 1));
  PRINT(F("POT10K-B: "));
  PRINTLN(potSet(I2C_POT10K, 127, 2));
#endif
	
  // Init Current sensors: input pin, calibration, calculate Irms PV and LOAD
  emonPv.current(A0, CURRENT_CONSTANT);
  emonLo.current(A1, CURRENT_CONSTANT);

  pv = maxPv = load = maxLd = 0.0;
  loopCnt = 0;

#ifdef ARDUINO_AVR_YUN
  // initialise with NULL command
  //Bridge.put("C", "0");
  Bridge.put("C", MSG_NULL.cw);
  Bridge.put("E", String(error));
#endif
#ifdef CONSOLE_LOG
  PRINT(F("ERROR CODE = "));
  PRINT(error, BIN);
  PRINTLN(F("\n-- SETUP END --"));
#endif
  
#ifdef TEMP_TUNING
  for ( int i = 0; i<9; i++ ) setResolution(&ow2, ONE_WIRE_ADD[i], TEMP_10_BIT);
  for ( int i = 9; i<13; i++ ) setResolution(&ow1, ONE_WIRE_ADD[i], TEMP_10_BIT);
#endif

  lastMilli = millis();

  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
}

/*---------------------------------------------------------------------------*/
Msg getMsg(void)
/*
 Return:  Msg from LININO

 function description - get new command from LININO/SERIAL: k=C, value=MSG
    MSG[0] -> OPCODE (1 byte) [0-255]
    MSG[1] -> DATA (1 byte) [0-255]
    
    e.g.:
    URL: clima.local/data/put/C/K0
    LININO: bridge.put("C", chr(75)+chr(48)) or bridge.put('C', 'K'+chr(48))
    MCU -> Bridge.get("C", buff, 2) -> OPCODE = 75, DATA = 48

    SERIAL MONITOR: E0 + CR (3 x char)
    MCU -> 2 x Serial.read() -> OPCODE = 69, DATA = 48

-----------------------------------------------------------------------------*/
{
  Msg msg = MSG_NULL;

#ifndef ARDUINO_AVR_YUN
  // 
  if ( Serial.available() == sizeof(Msg)+1 ) {
    msg.cw[0] = Serial.read();
    msg.cw[1] = Serial.read();
    Serial.read(); // flush \n
  }
  else  {
    while (Serial.available() > 0) {
      Serial.read(); // flush and print everything
    }
  }
#else
  Bridge.get("C", msg.uw, sizeof(Msg));
  Bridge.put("C", MSG_NULL.cw); // reset to NULL command
#endif
  return msg;
}

/*---------------------------------------------------------------------------*/  
void loop()
/*
-----------------------------------------------------------------------------*/
{
  byte status;
  unsigned long milli = millis();
  Msg msg;

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // turn the LED toggle

#ifdef CONSOLE_LOG
  PRINT("\nloop: ");
  PRINTLN(loopCnt);
#endif
  
  // Calculate Irms only PV and LOAD
  double temp = emonPv.calcIrms(1480);
  pv += temp;
  
  if ( temp > maxPv) maxPv = temp;

#ifdef ARDUINO_AVR_YUN
  Bridge.put("PV", String(temp));
#endif

#ifdef CONSOLE_LOG
  PRINT(F("PV: "));
  PRINTLN(temp);
#endif

  temp = emonLo.calcIrms(1480);  // Calculate Irms only LOAD
  load += temp;
  
  if ( temp > maxLd) maxLd = temp;

#ifdef ARDUINO_AVR_YUN
  Bridge.put("LOAD", String(temp));
#endif

#ifdef CONSOLE_LOG
  PRINT(F("LOAD: "));
  PRINTLN(temp);
#endif

  // every minute actions (reset loopCnt to 0)
  if ( (unsigned long)(milli - lastMilli) >= TIME_CUR_AVG_MSEC ) {

#ifdef ARDUINO_AVR_YUN
    Bridge.put("PV_AVG", String(pv/loopCnt));
    Bridge.put("LOAD_AVG", String(load/loopCnt));
    Bridge.put("PV_MAX", String(maxPv));
    Bridge.put("LD_MAX", String(maxLd));
#endif

#ifdef CONSOLE_LOG
    PRINT(F("\nPV_AVG: "));
    PRINTLN(pv/loopCnt);
    PRINT(F("LOAD_AVG: "));
    PRINTLN(load/loopCnt);
#endif

    pv = maxPv = load = maxLd = 0.0;
    loopCnt = 0;
    lastMilli = milli;
  }
  else loopCnt++;
  
  msg = getMsg();

#ifdef CONSOLE_LOG
  PRINT(F(" -->  opcode="));
  PRINT(msg.f.opcode);
  PRINT(F(" data="));
  PRINT(msg.f.data);
  PRINT(F(" -> "));
  PRINTLN(msg.f.data, BIN);
#endif

  switch ( msg.f.opcode )
  {
    case O_ERROR:

      error = msg.f.data;
      break;
    
    case O_TUNE_PV_SENSOR:

      emonPv.current(A0, msg.f.data / 10);
      break;
      
    case O_SET_POT100K_A:

      status = potSet(I2C_POT100K, msg.f.data, 1);
#ifdef ARDUINO_AVR_YUN
      Bridge.put("P", String(status));
#endif
#ifdef CONSOLE_LOG
      PRINT(F("POT100K-A: "));
      PRINTLN(status);
#endif
      break;
    
    case O_SET_POT100K_B:

      // 3K@25 --> 0(-5), 54(-4), 141(-3), 183(-2), 196(-1), 207(0), 218(1), 222(2), 226(3),
      //          229(4), 233(5), 237(6), 243(7), 247(8), 248(9), 249(11), 250(12), 251(15),
      //          252(20), 253(30), 254(40),
      status = potSet(I2C_POT100K, msg.f.data, 2);
#ifdef ARDUINO_AVR_YUN
      Bridge.put("Q", String(status));
#endif
#ifdef CONSOLE_LOG  
      PRINT(F("POT100K-B: "));
      PRINTLN(status);
#endif
      break;

    case O_SET_POT10K_A:
      
      status = potSet(I2C_POT10K, msg.f.data, 1);
#ifdef ARDUINO_AVR_YUN
      Bridge.put("R", String(status));
#endif
#ifdef CONSOLE_LOG 
      PRINT(F("POT10K-A: "));
      PRINTLN(status);
#endif
      break;

    case O_SET_POT10K_B:
      // read/write POT10K-B
      status = potSet(I2C_POT10K, msg.f.data, 2);
#ifdef ARDUINO_AVR_YUN
      Bridge.put("S", String(status));
#endif
#ifdef CONSOLE_LOG
      PRINT(F("POT10K-B: "));
      PRINTLN(status);
#endif
      break;

    case O_MCP_GPA:
      // set MCP GPA (ignore the 7 input and handle 3 PDC output)
      status = mcp.readGPIO(0);
      if ( (status & ~MCP23017_GPA_IO) != (msg.f.data & ~MCP23017_GPA_IO) ) { // update output and read back
        mcp.writeGPIOA((status & MCP23017_GPA_IO ) | (msg.f.data & ~MCP23017_GPA_IO ));
      }
      break;

    case O_MCP_GPB:
      // set MCP GPB: all outout (pompe, caldaia, R, boost)
      mcp.writeGPIOB(msg.f.data);
      status = mcp.readGPIO(1);

#ifdef ARDUINO_AVR_YUN
      Bridge.put("B", String(status));
#endif
#ifdef CONSOLE_LOG
      PRINT(F("MPC GPB: "));
      PRINTLN(status, BIN); // read GPB
#endif
      break;
  
    case O_PIO_TAPPA:
	    // take action if the command is different from actual status
	    if ( GetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE], &status) ) {

        status ^= msg.f.data & T_I_MASK;
         
        // check for new TAPPARELLE valid command (msg.f.data)
        if ( (status & SE_MASK)==SE_MASK || (status & NO_MASK)==NO_MASK || (status & SO_MASK)==SO_MASK )
          break;
      
        if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE], &status) )
          error |= ERROR_PIO_TAPPA_SET;
        else
          error &= ~ERROR_PIO_TAPPA_SET; // now set is OK: clear previous error report
        
        // msg.f.data OK --> start watch-dog
      	if ( msg.f.data & SE_MASK ) tapSeMilli = milli;
      	if ( msg.f.data & NO_MASK ) tapNoMilli = milli;
      	if ( msg.f.data & SO_MASK ) tapSoMilli = milli;

#ifdef ARDUINO_AVR_YUN
        Bridge.put("T", String(status));
#endif
#ifdef CONSOLE_LOG
	     PRINT(F("status="));
	     PRINTLN(status, BIN);
#endif
         error &= ~ERROR_PIO_TAPPA_GET; // now get is OK: clear previous error report
      }
      else error |= ERROR_PIO_TAPPA_GET;
      break;
    
    case O_PIO_ALARM:
      // now handle the other PIO: active low (alarm and/or gates control)
      if( GetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME], &status) ) {
         
         // now handle the new command
         if ( status != (msg.f.data | A_I_MASK) ) {
	        
	        status &= (msg.f.data | A_I_MASK); // avoid reset to the IN pins...
	        
	        if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME], &status) ) error |= ERROR_PIO_ALARM_SET;
	        else error &= ~ERROR_PIO_ALARM_SET; // now set is OK: clear previous error report
         }
#ifdef ARDUINO_AVR_YUN
         Bridge.put("I", String(status));
#endif
         error &= ~ERROR_PIO_ALARM_GET; // now get is OK: clear previous error report
      }
      else error |= ERROR_PIO_ALARM_GET;
      break;
  
    case O_NULL:
#ifdef CONSOLE_LOG
      PRINTLN(F("O_NULL"));
#endif
      // check tapparelle timeout (TODO: instantiate timer interrupt...)
      if ( GetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE], &status) ) {
  	
        // check watch-dogs (tapparelle still moving?): client should always takes action before timeout...
        if ( (status & SE_MASK) && (unsigned long)(milli - tapSeMilli) >= TIME_TAP_SE_MSEC ) status &= ~SE_MASK;
        if ( (status & NO_MASK) && (unsigned long)(milli - tapNoMilli) >= TIME_TAP_NO_MSEC ) status &= ~NO_MASK;
        if ( (status & SO_MASK) && (unsigned long)(milli - tapSoMilli) >= TIME_TAP_SO_MSEC ) status &= ~SO_MASK;

        if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_TAPPARELLE], &status) ) error |= ERROR_PIO_TAPPA_SET;
        else error &= ~ERROR_PIO_TAPPA_SET; // now set is OK: clear previous error report
  
#ifdef ARDUINO_AVR_YUN
        Bridge.put("T", String(status));
#endif  
        error &= ~ERROR_PIO_TAPPA_GET; // now get is OK: clear previous error report
      }
      else error |= ERROR_PIO_TAPPA_GET;
      
      // now the other PIO: active low (alarm and/or gates control)
      if( GetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME], &status) ) {
  	
  	    // stop any relays if still active since the previos loop (about 500 ms later)
  	    if ( (status | A_I_MASK ) != A_INIT_STATE ) {
    	    
    	    status = A_INIT_STATE;
    	    
    	    if ( SetPio(&ow1, ONE_WIRE_ADD[OW1_PIO_ALLARME], &status) ) error |= ERROR_PIO_ALARM_SET;
    	    else error &= ~ERROR_PIO_ALARM_SET; // now set is OK: clear previous error report
#ifdef ARDUINO_AVR_YUN
         Bridge.put("I", String(status));
#endif  	     
  	    }
  	    error &= ~ERROR_PIO_ALARM_GET; // now get is OK: clear previous error report
      }
      else error |= ERROR_PIO_ALARM_GET;

      // ciclically read and print all available Temperature sensors (d10)
      if ( GetTempC(&ow2, ONE_WIRE_ADD[loopCnt%9], &temp) ) {
#ifdef ARDUINO_AVR_YUN
         Bridge.put(String(loopCnt%9), String(temp));
         //error &= ~ERROR_OW2_GET_TEMP; // now get is OK: clear previous error report
#endif
      } else error |= ERROR_OW2_GET_TEMP;

#ifdef CONSOLE_LOG
      PRINT(F("T"));
      PRINT(loopCnt%9);
      PRINT(F(" = "));
      PRINTLN(temp);
#endif
   
      // ciclically read and print all available Temperature sensors (i2c)
      if ( GetTempC(&ow1, ONE_WIRE_ADD[9+loopCnt%4], &temp) ) {
#ifdef ARDUINO_AVR_YUN
         Bridge.put(String(9+loopCnt%4), String(temp));
         //error &= ~ERROR_OW1_GET_TEMP; // now get is OK: clear previous error report
#endif
      } else error |= ERROR_OW1_GET_TEMP;

#ifdef CONSOLE_LOG
      PRINT(F("T"));
      PRINT(9+loopCnt%4);
      PRINT(F(" = "));
      PRINTLN(temp);
#endif
      break;
      
    default:
#ifdef CONSOLE_LOG
      PRINTLN(F("ERROR_INVALID_KEY"));
#endif
      error |= ERROR_INVALID_KEY;
      break;
  }

  // always read MCP GPA: 7 input and 3 output
  status = mcp.readGPIO(0);
#ifdef ARDUINO_AVR_YUN
  Bridge.put("A", String(status));
  
  // always read D9: Booster switch status, active LOW
  Bridge.put("D9", String(digitalRead(9)));
#endif
#ifdef CONSOLE_LOG
  PRINT(F("MPC GPA: "));
  PRINTLN(status, BIN);
  PRINT(F("D9: "));
  PRINTLN(digitalRead(9));
#endif

#ifdef ARDUINO_AVR_YUN
  // update error code
  Bridge.put("E", String(error));
#endif
#ifdef CONSOLE_LOG
  PRINT(F("ERROR CODE = "));
  PRINTLN(error, BIN);
  PRINT("SEC elapsed: ");
  PRINTLN( (millis() - milli)/1000);
#endif
}
