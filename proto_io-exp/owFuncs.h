#ifndef OW_FUNCS_H
#define OW_FUNCS_H
/*-----------------------------------------------------------------------------
  Project:  home automation controller
  Module:   main controller
  File:     owFuncs.h
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
//#define TEMP_TUNING

/*-----------------------------------------------------------------------------
// System include files.
-----------------------------------------------------------------------------*/
#include <Arduino.h>

/*-----------------------------------------------------------------------------
// Project include files.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Module include files.
-----------------------------------------------------------------------------*/
#include "OneWire.h"

/*-----------------------------------------------------------------------------
// Constant definitions. (external scope)
-----------------------------------------------------------------------------*/
#define OW_T_OUT_ADDR           { 0x28, 0x74, 0x49, 0xD9, 0x05, 0x00, 0x00, 0xE7 }
#define OW_T_STUDIO_ADDR        { 0x28, 0x65, 0xBD, 0x49, 0x03, 0x00, 0x00, 0x02 }
#define OW_T_CUCINA_ADDR        { 0x28, 0x79, 0x7E, 0xDA, 0x05, 0x00, 0x00, 0xF0 }
#define OW_T_GIOCHI_ADDR        { 0x28, 0xEC, 0x14, 0xDA, 0x05, 0x00, 0x00, 0x06 }
#define OW_T_BIMBE_ADDR         { 0x28, 0x6B, 0x5B, 0xDA, 0x05, 0x00, 0x00, 0xEF }
#define OW_T_GENITORI_ADDR      { 0x28, 0xF4, 0x90, 0xDA, 0x05, 0x00, 0x00, 0x28 }
#define OW_T_M_COLL_PT_ADDR     { 0x28, 0x9F, 0x0D, 0xA1, 0x05, 0x00, 0x00, 0x74 }
#define OW_T_M_COLL_P1_ADDR     { 0x28, 0xD1, 0x55, 0xA0, 0x05, 0x00, 0x00, 0x36 }
#define OW_T_MANSARDA_ADDR      { 0x28, 0x78, 0xEF, 0xB1, 0x06, 0x00, 0x00, 0x5D }
#define OW_T_RITORNO_ADDR       { 0x28, 0x53, 0xB8, 0xB2, 0x06, 0x00, 0x00, 0x5F }
#define OW_T_SOGGIORNO_ADDR     { 0x28, 0x48, 0x82, 0xD9, 0x05, 0x00, 0x00, 0x25 }
#define OW_T_ACS_ADDR           { 0x28, 0xAA, 0x4B, 0xB2, 0x06, 0x00, 0x00, 0x53 }
#define OW_T_MANDATA_ADDR       { 0x28, 0x0B, 0x4A, 0xA1, 0x05, 0x00, 0x00, 0xA1 }
#define OW_PIO_TAPPARELLE_ADDR  { 0x29, 0xE2, 0xBA, 0x13, 0x00, 0x00, 0x00, 0x26 }
#define OW_PIO_ALLARME_ADDR     { 0x29, 0x43, 0xA2, 0x13, 0x00, 0x00, 0x00, 0x0F }

const byte ONE_WIRE_ADD[][8] = { 
  OW_T_OUT_ADDR,        // 0: fuori
  OW_T_STUDIO_ADDR,     // 1: Studio
  OW_T_CUCINA_ADDR,     // 2: Cucina
  OW_T_GIOCHI_ADDR,     // 3: Giochi
  OW_T_BIMBE_ADDR,      // 4: Bimbe
  OW_T_GENITORI_ADDR,   // 5: Genitori
  OW_T_M_COLL_PT_ADDR,  // 6: collettore PT
  OW_T_M_COLL_P1_ADDR,  // 7: collettore P1
  OW_T_MANSARDA_ADDR,   // 8: mansarda
  
  OW_T_RITORNO_ADDR,    // 9: ritorno 
  OW_T_SOGGIORNO_ADDR,  // 10: soggiorno
  OW_T_ACS_ADDR,        // 11: ACS
  OW_T_MANDATA_ADDR,    // 12: mandata
  
  OW_PIO_TAPPARELLE_ADDR,   // 13: PIO tapparelle: all I/O channel active HIGH
  OW_PIO_ALLARME_ADDR       // 14: PIO bentel: : all I/O channel active LOW
};

// 1-wire DS2408 Commands
#define DS2408_PIO_READ_CMD      0xF0
#define DS2408_CHANNEL_READ_CMD  0xF5
#define DS2408_CHANNEL_WRITE_CMD 0x5A
#define DS2408_SEARCH_CMD        0xCC
#define DS2408_RESET_CMD         0xC3

// 1-wire DS18x20 commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// 1-wire Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

// 1-wire PIO Tapparelle Commands - Active HIGH
#define xx1			  (1<<0) // unused: 1
#define T_NO_S		(1<<1) // Tapparella Nord-ovest (lavanderia+tinello) SU: 2
#define T_SO_S		(1<<2) // Tapparella Sud-Ovest (bagno+studio) SU: 4
#define T_SO_G		(1<<3) // Tapparella Sud-Ovest (bagno+studio) GIU: 8
#define T_SE_S		(1<<4) // Tapparella Sud-Est (soggiorno+cucina) SU: 16
#define T_SE_G		(1<<5) // Tapparella Sud-Est (soggiorno+cucina) GIU: 32
#define xx2			  (1<<6) // unused: 64
#define T_NO_G		(1<<7) // Tapparella Nord-ovest (lavanderia+tinello) GIU: 128

// mask to check invalid commands: e.g. SU+GIU on the same tapparella
#define SO_MASK			  (T_SO_S | T_SO_G)   /* 00001100: tapparelle a SO */
#define SE_MASK			  (T_SE_S | T_SE_G)   /* 00110000: tapparelle a SE */
#define NO_MASK			  (T_NO_S | T_NO_G)   /* 10000010: tapparelle a NO */
#define T_I_MASK		  (SO_MASK | SE_MASK | NO_MASK) /* 10111110 */
#define T_INIT_STATE	(0)

// 1-wire PIO ALARM/GATE Commands - Active LOW
#define A_RELE1  		  (1<<0) // Alarm Relay 1: cancelletto
#define A_RELE2			  (1<<1) // Alarm Relay 2: cancello / OC5 (supertasto 2) *
#define A_OC4			    (1<<2) // Alarm OC4: P1 on
#define A_OC3			    (1<<3) // Alarm OC3: PT on 
#define A_OC2			    (1<<4) // Alarm OC2: FINESTRE on
#define A_ALARM			  (1<<5) // Alarm relay: SIRENA on
#define A_RELE3			  (1<<6) // Alarm Relay 2: ALL ON (zona 7)
#define A_RELE4			  (1<<7) // Alarm Relay 3: B ON (zona 8) *
#define A_I_MASK	    (A_OC4 | A_OC3 | A_OC2 | A_ALARM) // input signal mask: 0x3C
#define A_INIT_STATE	(0xFF)

/*-----------------------------------------------------------------------------
// Type definitions (including Enumerations). (external scope)
-----------------------------------------------------------------------------*/
typedef enum {
    OW2_T_OUT            = 0,
    OW2_T_STUDIO         = 1,
    OW2_T_CUCINA         = 2,
    OW2_T_GIOCHI         = 3,
    OW2_T_BIMBE          = 4,
    OW2_T_GENITORI       = 5,
    OW2_T_MANDATA_PT     = 6,
    OW2_T_MANDATA_P1     = 7,
    OW2_T_MANSARDA       = 8,
    
    OW1_T_RITORNO        = 9,
    OW1_T_SOGGIORNO      = 10,
    OW1_T_ACS            = 11,
    OW1_T_MANDATA        = 12,
    
    OW1_PIO_TAPPARELLE   = 13,
    OW1_PIO_ALLARME      = 14
} OwIdx;

/*-----------------------------------------------------------------------------
// Variable definitions. (external scope)
-----------------------------------------------------------------------------*/

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
// Function prototype declarations (external scope)
-----------------------------------------------------------------------------*/
bool ResetAct(OneWire*, const byte addr[8]);
bool SetPio(OneWire*, const byte addr[8], byte*);
bool GetPio(OneWire*, const byte addr[8], byte*);
#ifdef TEMP_TUNING
    bool setResolution(OneWire*, const byte addr[8], byte);
#endif
bool GetTempC(OneWire*, const byte addr[8], double*);

#endif