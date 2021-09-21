#ifndef __HARTPROTOCOL_H
#define __HARTPROTOCOL_H

#include <stdint.h>

/*           Universal Commands                 */
#define FUNCTION_READ_UNIQUE_ID                  0  
#define FUNCTION_READ_PRIMARY_VALUE				 1
#define FUNCTION_READ_CURRENT_PERCENT_OF_RANGE	 2
#define FUNCTION_READ_DYN_VARIABLES_AND_CURRENT  3
#define FUNCTION_WRITE_POLLING_ADDRESS           6
#define FUNCTION_READ_MESSAGE	                 12
#define FUNCTION_READ_TAG_DISC_DATE              13
#define FUNCTION_WRITE_MESSAGE                   17
#define FUNCTION_WRITE_TAG_DISC_DATE             18

/*         Common Practice Commands              */
#define FUNCTION_WRTIE_PV_UPPER_RANGE            36
#define FUNCTION_WRTIE_PV_LOWER_RANGE            37
#define FUNCTION_ENTEROREXIT_FIXED_CURRENT       40
#define FUNCTION_PERFORM_TRANSMITTER_SELFTEST    41
#define FUNCTION_PERFORM_DEVICE_RESET            42
#define FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO      45
#define FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN      46
#define FUNCTION_WRITE_NUM_RES_PREAMBLES         59

/*         Device-specific commands               */
#define FUNCTION_READ_SPECIFIC_SLOT              128
#define FUNCTION_WRITE_SPECIFIC_SLOT             129
#define FUNCTION_READ_DATE_TIME                  130
#define FUNCTION_WRITE_DATE_TIME                 131
//---------------------------------------------------
/*                  Slot list                      
           Device specific variables slots         */ 
#define SLOT_0_FIRST_THRESHOLD                   0x00
#define SLOT_1_SECOND_THRESHOLD                  0x01
#define SLOT_2_HYSTERESIS                        0x02
#define SLOT_3_UNIT_OF_MEASUREMENT               0x03
#define SLOT_4_ADDRESS_OF_MODBUS                 0x04
#define SLOT_5_BAUDRATE_OF_MODBUS                0x05

#define SLOT_MAX_NUMBER                          0x06 
//---------------------------------------------------
/*          Peambles value and numbers of it       */
#define PREAMBLE                                 0xFF
#define PREAMBLE_MIN_NUM                         5
#define PREAMBLE_MAX_NUM                         20
//---------------------------------------------------
/*           Long & short frame size and code      */ 
#define LONG_FRAME     			                 0x80
#define SHORT_FRAME                              0x00
#define SHORT_ADDR_SIZE                             1
#define LONG_ADDR_SIZE                              5
//---------------------------------------------------
/*             Start Delimiter values              */
#define _START_DELIMITER_MASTER                  0x02
#define _START_DELIMITER_SLAVE                   0x06
#define _START_DELIMITER_SLAVE_BURST_MODE        0x01
//---------------------------------------------------
/*     Master primary & secondary, burst mode      */
#define _MASTER_PRIMARY	                         0x80
#define _MASTER_SECONDARY	                     0x00
#define _BURST_FRAME_ON		                     0x40
#define _BURST_FRAME_OFF                         0x00
//---------------------------------------------------
#define MANUFACTURER_IDENTIFICATION_CODE         250 // = Not used
#define MANUFACTURER_DEVICE_TYPE                 0x01
#define UNIQUE_DEVICE_ID0                        0xAA
//---------------------------------------------------
#define MAX_POLLING_ADDRESS                      0x0F
//---------------------------------------------------
#define MAX_PV_UPPER_RANGE	                      50
#define MIN_PV_LOWER_RANGE  	                   5
//---------------------------------------------------



#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#if defined(FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO) ||	 defined(FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN)

#define SET_ZERO_CURRENT						 10
#define MIN_CONCENTRATION						 10
#define MAX_CONCENTRATION						 45

typedef enum
{
  ZERO    = 0x00U,
  GAIN    = 0x01U,
} E_CalbType;
#endif

typedef enum
{
 NO_ERROR  				                 = 0x0000,//0x00 0x00
 NO_ERROR_CONFIG_CHANGED           = 0x0040,//0x00 0x40
 NO_ERROR_OUTPUT_CURRENT_FIXED     = 0x0008,//0x00 0x08
 /* COMMUNICATION ERROR*/ 
 FRAMING_ERROR_HART                = 0x9000,//0x90 0x00
 CHEKSUM_ERROR_HART                = 0x8800,//0x88 0x00
 /* COMMAND ERROR*/ 
 ERROR_INVALID_SELECTION           = 0x0200,//0x02 0x00
 ERROR_PARAMETER_TOO_LARGE         = 0x0300,//0x03 0x00
 ERROR_PARAMETER_TOO_SMALL         = 0x0400,//0x04 0x00
 ERROR_FEW_DATA_BYTE_RECEIVED      = 0x0500,//0x05 0x00
 ERROR_UPDATE_FAILED               = 0x0800,//0x08 0x00
 ERROR_NOT_IN_FIXED_CURRENT_MODE   = 0x0900,//0x09 0x00
 ERROR_MULIDROP_NOT_SUPPORTED      = 0x0A00,//0x0A 0x00
 ERROR_SPAN_TOO_SMALL              = 0x0E00,//0x0E 0x00
 ERROR_COMMAND_NOT_IMPLEMENTED     = 0x4000,//0x40 0x00

 ERROR_DEVICE_MALFUNCTION          = 0x0080,//0x40 0x00
	
} E_HART_Status;

typedef struct
{
	uint8_t   Manufactuere_Id;
  uint8_t   DeviceType;
	uint8_t   Unique_Device_Id0;
	uint8_t   Unique_Device_Id1;
	uint8_t   Unique_Device_Id2;
	
} s_AddressLong;

typedef enum
{
	ShortFrameAddress       = 0x00U, 
  LongFrameAddress        = 0x01U,
	
} E_AddressType;

typedef enum
{
	BurstFrame     = 0x00U, 
  MasterToSlave  = 0x01U,
	SlaveToMaster  = 0x02U,
	
} E_FrameType;

typedef struct
{
	uint8_t        addrSize;
	E_AddressType  AddressType;
  E_FrameType    FrameType;
	
}S_H_StartDelimiter;

typedef struct
{
	uint8_t  H_Startdelimiter;
	uint8_t  TypeMaster;      //Primary or secondary
	uint8_t  TypeFrame;       //Burst mode or unicast
	uint8_t  H_Address[5];
	uint8_t  H_Command;
	uint8_t  H_BytesCount;
	uint8_t  H_StatusByte1;
	uint8_t  H_StatusByte2;
	uint8_t  H_Data[25];      // max is 25 bytes
}S_PacketHart;


typedef struct
{
	/* 20max preamble + 25data + 11-> (start, add, command, bcnt, status, chs)*/
	uint8_t        pPacketDataOut[56];   // Packet out holding Data to send response
  uint8_t        PacketOutSize;        // Packet out size data																			
	E_HART_Status  eHartStatus;
	uint8_t        commandUsed;
	
} S_packetAnswer_Hart;
//----------------------------------------------------------------------------------------------------------------------------------

/* Self test & master reset*/ 
typedef E_HART_Status (*PerformTransmitterSelfTest)(void);
typedef void (*PerformMasterReset)(void);

/* Clibration (zero, sensitivity)*/
typedef E_HART_Status (*TrimDAC_Value)(E_CalbType, float);

//----------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
	uint8_t ResposePreambleNum;
	
	float PVUpperRange;
	float PVlowerRange;
	
	float PrimaryValue;   
	float SecondaryValue;
	float TertiaryValue ;
	float QuaternaryValue;
	
	float LoopCurrent;    // unit : mA
	float PercnetOfRange; // unit : %
	float FixedCurrent;   // unit : mA
		
	uint8_t PVCode;
	uint8_t SVCode;
	uint8_t TVCode;
	uint8_t QVCode;
	
	uint8_t PollingAddress_Short;
	s_AddressLong PollingAddress_Long;
	
	uint8_t Message[24];          //packed ascii
	uint8_t Tag[6];               //packed ascii
	uint8_t Descriptor[12];       //descriptor , packed ascii
	uint8_t Date[3];              //date : day/month/year
	
	/* Self test & master reset*/ 
	PerformTransmitterSelfTest PTST; 
	PerformMasterReset PMR;
	
	/* Clibration (zero, sensitivity)*/
	TrimDAC_Value CalibrateDevice;
	
	/* for Device specific Commands */
	//Time : hour/minute/sec
	uint8_t Time[3];              
	//first threshold, second threshold, hystirasis, unit, address modbus, baudrate 
  uint16_t *DateSlotCommands;    
	
} S_Parameters_Hart;

//----------------------------------------------------------------------------------------------------------------------------------
/*  ProcesseingRXData_HART 
    Variables :
				1- pointer to structure S_packetAnswer_Hart that holding response packet to send 
				2- pointer to structure S_Parameters_Hart 
				3- pointer to data  pDataIN witch holding the incoming packet from master
				4- sizePData size of pDataIN 
    Output is referenced in sPacketAns  */

void ProcesseingRXData_HART(S_packetAnswer_Hart *sPacketAns, S_Parameters_Hart *HartParameters, uint8_t *pDataIN, uint8_t sizePData);
//----------------------------------------------------------------------------------------------------------------------------------
uint8_t packed_Ascii(uint8_t *Src, uint8_t SrcLen, uint8_t *Dst);
uint8_t unpacked_Ascii(uint8_t *Src, uint8_t SrcLen, uint8_t *Dst);
//----------------------------------------------------------------------------------------------------------------------------------
/*  These function should be implemented in main  */
uint8_t SendAnsweringPacket_HART(uint8_t *data, uint8_t size);
void ReadDataFromHartStructure(S_Parameters_Hart *vParam_Hart, uint8_t command);
void WriteDataToHartStruct(S_Parameters_Hart *vParam_Hart);
//----------------------------------------------------------------------------------------------------------------------------------
#endif

