/*-----------------------------------------------------------------------------
  Project:  home automation controller
  Module:   main controller
  File:     proto_ioexp.h
  Version:  1
  Author:   Stefano Raffaglio
  NoKeywords:

  Description: header for proto_ioexp.c

------------------------------------------------------------------------------

Change History:

Date	      Initials    Version 	Change Details
--------	  --------    -------	  --------------
29/07/16    sraff       0         creation

-----------------------------------------------------------------------------*/
#ifndef PROTO_H
#define PROTO_H

/*-----------------------------------------------------------------------------
// System include files.
-----------------------------------------------------------------------------*/
#include <stdint.h>

/*-----------------------------------------------------------------------------
// Project include files.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Module include files.
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Constant definitions. (external scope)
-----------------------------------------------------------------------------*/
#define MCP23017_GPA_IO		(0xF8) // 11111000: GPA0-GPA2 (ILQ5 A2/A3/A4)-> OUTPUT=0, GPA3-GPA7 -> INPUT=1
#define MCP23017_GPB_IO		(0x00) // 00000000: GPB0 (ILQ5 A1), GPB1-GPB7 (ULN 7/6/5/4/3/2/1) -> ALL OUTPUT=0

#define MCP_GPA0_PDC_FRE	(1<<0) //   1: A2: PDC FREQ OFF/ON(1)
#define MCP_GPA1_PDC_ECO	(1<<1) //   2: A3: PDC NORM/ECO(1)
#define MCP_GPA2_PDC_ON		(1<<2) //   4: A4: PDC OFF/ON(1)
#define MCP_GPA3_INPUT1		(1<<3) //   8: IN1: ESTATE/INVERNO(0)
#define MCP_GPA4_INPUT2		(1<<4) //  16: IN2: EHS(0)
#define MCP_GPA5_INPUT3		(1<<5) //  32: IN3: DEFROST(0)
#define MCP_GPA6_INPUT4		(1<<6) //  64: IN4: PT-ON(0)
#define MCP_GPA7_INPUT5		(1<<7) // 128: IN5: P1-ON(0)

#define MCP_GPB0_PDC_ACS	  (1<<0) //   1: A1:   PDC ACS OFF/ON(1)
#define MCP_GPB1_PUMP_PT	  (1<<1) //   2: ULN7: Pompa collettore PT ON/OFF(0)
#define MCP_GPB2_PUMP_P1	  (1<<2) //   4: ULN6: Pompa collettore P1 ON/OFF(0)
#define MCP_GPB3_BOOST		  (1<<3) //   8: ULN5: booster ON/OFF(0) (INV->caldaia, EST->ECO-PDC=off)
#define MCP_GPB4_R_ACS	  	(1<<4) //  16: ULN4: R elettrica x ACS ON/OFF(0)
#define MCP_GPB5_SOLO_CALD  (1<<5) //  32: ULN3: SOLO-CALD(1) commutatore
#define MCP_GPB6_TBD1		    (1<<6) //  64: ULN2: ON(1)
#define MCP_GPB7_TBD2		    (1<<7) // 128: ULN1: ON(1)

// I2C Addresses B2 B1 B0
#define I2C_MCP1      1 // 0x21: MCP 001
#define I2C_ONEW     24 // 0x18: 0x18 | 0 -> AD1=AD0=0
#define I2C_POT100K  44 // 0x2C: P-1 100 -> CRT + ADDR (first 7 bits): 0101 100 0; R=100 Kohm
#define I2C_POT10K   46 // 0x2E: P-2 110 -> CRT + ADDR (first 7 bits): 0101 110 0; R=10 Kohm

// global error codes
#define ERROR_INVALID_KEY       (1<<0)
#define ERROR_MCP               (1<<1)
#define ERROR_PIO_TAPPA_SET     (1<<2)
#define ERROR_PIO_TAPPA_GET     (1<<3)
#define ERROR_PIO_ALARM_SET     (1<<4)
#define ERROR_PIO_ALARM_GET     (1<<5)
#define ERROR_OW1_GET_TEMP      (1<<6)
#define ERROR_OW2_GET_TEMP      (1<<7)

#define BURDEN_RESISTOR      (220.0-3.5)  // resistor at the sensor output ohm
#define TRANSFORMER_RATIO    (2000.0)
#define CURRENT_CONSTANT     (TRANSFORMER_RATIO/BURDEN_RESISTOR)
#define CURRENT_CONSTANT_PV  (97)
#define CURRENT_CONSTANT_LD  (97)

#define TIME_TAP_SO_MSEC  (20000) // 20 sec
#define TIME_TAP_SE_MSEC  (19000) // 19 sec
#define TIME_TAP_NO_MSEC  (27000) // 27 sec
#define TIME_CUR_AVG_MSEC (60000) // 1 minute

/*-----------------------------------------------------------------------------
// Type definitions (including Enumerations). (external scope)
-----------------------------------------------------------------------------*/
typedef enum {
    O_NULL = 0,
    O_MCP_GPA = 65, // ord('A')
    O_MCP_GPB = 66, // ord('B')
    O_DEFAULT = 67, // ord('C')
    O_ERROR = 69, // ord('E')
    O_PIO_ALARM = 73, // ord('I')
    O_TUNE_PV_SENSOR = 76, // ord('L')
    O_SET_POT100K_A = 80, // ord('P')
    O_SET_POT100K_B = 81, // ord('Q')
    O_SET_POT10K_A = 82, // ord('R')
    O_SET_POT10K_B = 83, // ord('S')
    O_PIO_TAPPA = 84 // ord('T')
} Msg_opc;

typedef struct Mcp_gpa_s {
  byte opcode;
  unsigned int  pdc_fre :1;
  unsigned int  pdc_eco :1;
  unsigned int  pdc_on  :1;
  unsigned int  est_inv :1;
  unsigned int  ehs     :1;
  unsigned int  defrost :1;
  unsigned int  pt_on   :1;
  unsigned int  p1_on   :1;
} Mcp_gpa;

typedef struct Mcp_gpb_s {
  byte opcode;
  unsigned int  pdc_acs :1;
  unsigned int  pump_pt :1;
  unsigned int  pump_p1 :1;
  unsigned int  boost   :1;
  unsigned int  r_acs   :1;
  unsigned int  s_cald  :1;
  unsigned int  tbd1    :1;
  unsigned int  tbd2    :1;
} Mcp_gpb;

typedef struct Msg_fields_s {
  byte   opcode;
  byte   data;
} Msg_fields;

typedef union {
  struct Mcp_gpa_s    mpc_gpa;
  struct Mcp_gpb_s    mpc_gpb;
  struct Msg_fields_s f;
  byte                bw[2];
  char                cw[2];
  uint8_t             uw[2];
} Msg;

/*-----------------------------------------------------------------------------
// Variable definitions. (external scope)
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
// Function prototype declarations (external scope)
-----------------------------------------------------------------------------*/

#endif