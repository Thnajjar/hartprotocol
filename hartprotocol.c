
#include "hartprotocol.h"

static uint8_t Calculate_LongitudinalParity(uint8_t *pData, uint8_t Size);
static uint8_t get_polling_addr_short(S_Parameters_Hart *HartParameters) ;
static uint8_t *get_polling_addr_long(S_Parameters_Hart *HartParameters) ;
static uint8_t SplitstartDelimiter(uint8_t _sDlm);
static uint8_t BuildStartDelimiter(E_AddressType eAddType, E_FrameType eframeType);
static uint8_t Is_Address_Matched(S_Parameters_Hart *HartParameters, E_AddressType eA, uint8_t *_address);
static void BuildAddress(S_Parameters_Hart *HartParameters , E_AddressType eAddType);
static uint8_t Is_Command_Supported(uint8_t cmd);
static uint8_t Is_ByteCounts_Correct(uint8_t BCNT, uint8_t cmd);
static uint8_t CombineResponePacket(S_Parameters_Hart *HartParameters, uint8_t *pDataOUT);

static float data_to_floatIEEE754(uint8_t *tmp);
static void floatIEEE754_to_data(uint8_t *data, float *tmp);
static void FillHartPacketOutData(uint8_t *pdataIN, uint8_t lengthD);
static void set_status_Bytes(uint16_t statusHart);

static S_H_StartDelimiter  StartDelimiterOut_Prop;
static S_PacketHart        HartPacketOut_str;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t Calculate_LongitudinalParity(uint8_t *pData, uint8_t Size)
{
	uint8_t cks_xor=0;
	while(Size--)
	{
		cks_xor ^= pData[Size];	
	}
	return cks_xor;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/* polling address */
static uint8_t get_polling_addr_short(S_Parameters_Hart *HartParameters) 
{
   return HartParameters->PollingAddress_Short; 
}
static uint8_t *get_polling_addr_long(S_Parameters_Hart *HartParameters) 
{
	static uint8_t pA_Long[5];
	pA_Long[0] = HartParameters->PollingAddress_Long.Manufactuere_Id;
	pA_Long[1] = HartParameters->PollingAddress_Long.DeviceType;
	pA_Long[2] = HartParameters->PollingAddress_Long.Unique_Device_Id0;
	pA_Long[3] = HartParameters->PollingAddress_Long.Unique_Device_Id1;
	pA_Long[4] = HartParameters->PollingAddress_Long.Unique_Device_Id2;
	
 return pA_Long;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/* Convert from visible string (8 bit format) into packed ascii string (6 bit format) */
uint8_t packed_Ascii(uint8_t *Src, uint8_t SrcLen, uint8_t *Dst)
{ 
	uint8_t DstLen = 0, byDstIdx = 0, bySrcIdx = 0;
 
  while (SrcLen)
  { /* First character */
    Dst[byDstIdx] &= 0x03;
    Dst[byDstIdx] |= (Src[bySrcIdx] & 0x3F) << 2;
    (DstLen)++;
    if (--SrcLen == 0)
      return DstLen;
    /* Second character */
    Dst[byDstIdx] &= 0xFC;
    Dst[byDstIdx] |= (Src[bySrcIdx+1] & 0x30) >> 4;
    Dst[byDstIdx+1] &= 0x0F;
    Dst[byDstIdx+1] |= (Src[bySrcIdx+1] & 0x0F) << 4;
    (DstLen)++;
    if (--SrcLen == 0)
      return DstLen;
    /* Third character */
    Dst[byDstIdx+1] &= 0xF0;
    Dst[byDstIdx+1] |= (Src[bySrcIdx+2] & 0x3C) >> 2;
    Dst[byDstIdx+2] &= 0x3F;
    Dst[byDstIdx+2] |= Src[bySrcIdx+2] << 6;
    (DstLen)++;
    if (--SrcLen == 0)
      return DstLen; 
    /* Fourth character */
    Dst[byDstIdx+2] &= 0xC0;
    Dst[byDstIdx+2] |= Src[bySrcIdx+3] & 0x3F;
    if (--SrcLen == 0)
      return DstLen;
    Dst += 3; Src += 4; 
  }
	return DstLen;
}
/* Convert from packed ascii string (6 bit format) into visible string (8 bit format) */
uint8_t unpacked_Ascii(uint8_t *Src, uint8_t SrcLen, uint8_t *Dst)
{
  uint8_t uc, e, byMyLen;
	uint8_t DstLen = 0;

  DstLen = 0;
  if (SrcLen < 3)
    return DstLen;
  if ((SrcLen % 3)>0)
    return DstLen;
  byMyLen = (uint8_t) (SrcLen - (SrcLen % 3));
  for (e=0;e<(byMyLen-2);e+=3)
  { /* First character */
    uc = (uint8_t)((Src[e] >> 2) & 0x3F);
    if (uc < 0x20) uc += 0x40;
    Dst[(DstLen)++] = uc;
    /* Second character */
    uc = (uint8_t)(((Src[e] << 4 ) & 0x3F) | ((Src[e+1] >> 4) & 0x3F));
    if (uc < 0x20) uc += 0x40;
    Dst[(DstLen)++] = uc;
    /* Third character */
    uc = (uint8_t)(((Src[e+1] << 2 ) & 0x3F) | ((Src[e+2] >> 6) & 0x3F));
    if (uc < 0x20) uc += 0x40;
    Dst[(DstLen)++] = uc;
    /* Fourth character */
    uc = (uint8_t)(Src[e+2] & 0x3F);
    if (uc < 0x20) uc += 0x40;
    Dst[(DstLen)++] = uc;
  }
  return DstLen;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
Bit 7 indicates the Address Type (1 byte or 5 bytes),
Bits 2, 1 and 0 indicate the Frame Type, with values 1 (burst frame), 2 (master to slave), or 6 (slave to master).
In HART Revision 6, in addition to the above:

Bits 6 and 5 indicate the number of Expansion Bytes (0 to 3 bytes, between the Address and Command fields),
Bits 4 and 3 indicate the Physical Layer Type (0 for normal FSK HART).
*/
static uint8_t SplitstartDelimiter(uint8_t _sDlm)
{
	if((_sDlm&0x03) == _START_DELIMITER_MASTER)
	{ 
		StartDelimiterOut_Prop.FrameType   = SlaveToMaster;
	}
	if((_sDlm&0x80) == LONG_FRAME)
	{
		StartDelimiterOut_Prop.AddressType = LongFrameAddress;
		StartDelimiterOut_Prop.addrSize    = LONG_ADDR_SIZE;
		return LONG_ADDR_SIZE;
	}
	else 
	{
		StartDelimiterOut_Prop.AddressType = ShortFrameAddress;
		StartDelimiterOut_Prop.addrSize    = SHORT_ADDR_SIZE;
		return SHORT_ADDR_SIZE;
	}
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t BuildStartDelimiter(E_AddressType eAddType, E_FrameType eframeType)
{
	uint8_t startD = 0x00;
	if(eframeType == LongFrameAddress)   { startD = LONG_FRAME;}
	
	if(eframeType == SlaveToMaster)      { startD |= _START_DELIMITER_SLAVE; }
	else if(eframeType == MasterToSlave) { startD |= _START_DELIMITER_MASTER; }
	else if(eframeType == BurstFrame)    { startD |= _START_DELIMITER_SLAVE_BURST_MODE; }
	
	return startD;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t Is_Address_Matched(S_Parameters_Hart *HartParameters, E_AddressType eA, uint8_t *_address)
{
	uint8_t *Device_addr, DeviceAddress = 0;
	if(eA == ShortFrameAddress)
	{
		DeviceAddress = get_polling_addr_short(HartParameters);
		if(DeviceAddress == (_address[0]&0x0F))
		{
			HartPacketOut_str.TypeMaster = _address[0]&_MASTER_PRIMARY;
			return TRUE;
		}
		else 
		{
			return FALSE;
		}
	}
	else
	{
		Device_addr = get_polling_addr_long(HartParameters);
		
		if((!(_address[0]&0x3F)) && (!_address[1]) && (!_address[2]) && \
       (!_address[3]) && (!_address[4]))
		{
			HartPacketOut_str.TypeMaster = _address[0]&_MASTER_PRIMARY;
			return TRUE;
		}
		else
		{ 
			Device_addr[0] = Device_addr[0]&0x3F;
			if((Device_addr[0] == (_address[0]&0x3F)) && (Device_addr[1] == _address[1])  && \
			   (Device_addr[2] == _address[2]) && (Device_addr[3] == _address[3]) && (Device_addr[4] == _address[4]))
				 
			{
				HartPacketOut_str.TypeMaster = _address[0]&_MASTER_PRIMARY;
				return TRUE;
			}
			else
			{
				return FALSE;
			}
	  }
  }
}
static void BuildAddress(S_Parameters_Hart *HartParameters , E_AddressType eAddType)
{
	uint8_t *tmp, i = 0;
  if(eAddType == ShortFrameAddress)
	{
		HartPacketOut_str.H_Address[0] = get_polling_addr_short(HartParameters)&0x0F;
	}
  else 
	{
		tmp = get_polling_addr_long(HartParameters);
		for(i = 0; i < 5; i++) 
		{
			HartPacketOut_str.H_Address[i] = tmp[i];
		}
		HartPacketOut_str.H_Address[0] &=0x3F;
	}
	HartPacketOut_str.H_Address[0] |= HartPacketOut_str.TypeMaster;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t Is_Command_Supported(uint8_t cmd)
{
//----------------------------------------------------
/*           Universal Commands                 */
//----------------------------------------------------
#ifdef FUNCTION_READ_UNIQUE_ID
if(cmd == FUNCTION_READ_UNIQUE_ID)
	return TRUE;
#endif

#ifdef FUNCTION_READ_PRIMARY_VALUE
if(cmd == FUNCTION_READ_PRIMARY_VALUE)
	return TRUE;
#endif

#ifdef FUNCTION_READ_CURRENT_PERCENT_OF_RANGE
if(cmd == FUNCTION_READ_CURRENT_PERCENT_OF_RANGE)
	return TRUE;
#endif

#ifdef FUNCTION_READ_DYN_VARIABLES_AND_CURRENT
if(cmd == FUNCTION_READ_DYN_VARIABLES_AND_CURRENT)
	return TRUE;
#endif

#ifdef FUNCTION_WRITE_POLLING_ADDRESS
if(cmd == FUNCTION_WRITE_POLLING_ADDRESS)
	return TRUE;
#endif

#ifdef FUNCTION_READ_MESSAGE
if(cmd == FUNCTION_READ_MESSAGE)
	return TRUE;
#endif

#ifdef FUNCTION_READ_TAG_DISC_DATE
if(cmd == FUNCTION_READ_TAG_DISC_DATE)
	return TRUE;
#endif

#ifdef FUNCTION_WRITE_MESSAGE
if(cmd == FUNCTION_WRITE_MESSAGE)
	return TRUE;
#endif
#ifdef FUNCTION_WRITE_TAG_DISC_DATE
if(cmd == FUNCTION_WRITE_TAG_DISC_DATE)
	return TRUE;
#endif

//----------------------------------------------------
/*         Common Practice Commands              */
//----------------------------------------------------
#ifdef FUNCTION_WRTIE_PV_UPPER_RANGE
if(cmd == FUNCTION_WRTIE_PV_UPPER_RANGE)
	return TRUE;
#endif

#ifdef FUNCTION_WRTIE_PV_LOWER_RANGE
if(cmd == FUNCTION_WRTIE_PV_LOWER_RANGE)
	return TRUE;
#endif

#ifdef FUNCTION_ENTEROREXIT_FIXED_CURRENT
if(cmd == FUNCTION_ENTEROREXIT_FIXED_CURRENT)
	return TRUE;
#endif

#ifdef FUNCTION_PERFORM_TRANSMITTER_SELFTEST
if(cmd == FUNCTION_PERFORM_TRANSMITTER_SELFTEST)
	return TRUE;
#endif

#ifdef FUNCTION_PERFORM_DEVICE_RESET
if(cmd == FUNCTION_PERFORM_DEVICE_RESET)
	return TRUE;
#endif

#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO
if(cmd == FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO)
	return TRUE;
#endif

#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN
if(cmd == FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN)
	return TRUE;
#endif

#ifdef FUNCTION_WRITE_NUM_RES_PREAMBLES
if(cmd == FUNCTION_WRITE_NUM_RES_PREAMBLES)
	return TRUE;
#endif
//----------------------------------------------------
/*              Device-specific commands            */
//----------------------------------------------------
#ifdef FUNCTION_READ_SPECIFIC_SLOT
if(cmd == FUNCTION_READ_SPECIFIC_SLOT)
	return TRUE;
#endif

#ifdef FUNCTION_WRITE_SPECIFIC_SLOT
if(cmd == FUNCTION_WRITE_SPECIFIC_SLOT)
	return TRUE;
#endif

#ifdef FUNCTION_READ_DATE_TIME
if(cmd == FUNCTION_READ_DATE_TIME)
	return TRUE;
#endif

#ifdef FUNCTION_WRITE_DATE_TIME
if(cmd == FUNCTION_WRITE_DATE_TIME)
	return TRUE;
#endif

return FALSE;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t Is_ByteCounts_Correct(uint8_t BCNT, uint8_t cmd)
{
	/* 1 byte  - start delimiter 
		 1-> 5 bytes - Address 
		 1 byte  - Command
		 1 byte  - Byte count
		 1 byte  - Check sum */
	
//----------------------------------------------------
/*           Universal Commands                 */
//----------------------------------------------------
	switch(cmd)
	{
		#ifdef FUNCTION_READ_UNIQUE_ID
		case(FUNCTION_READ_UNIQUE_ID):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_PRIMARY_VALUE
		case(FUNCTION_READ_PRIMARY_VALUE):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_CURRENT_PERCENT_OF_RANGE
		case(FUNCTION_READ_CURRENT_PERCENT_OF_RANGE):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_DYN_VARIABLES_AND_CURRENT
		case(FUNCTION_READ_DYN_VARIABLES_AND_CURRENT):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRITE_POLLING_ADDRESS
		case(FUNCTION_WRITE_POLLING_ADDRESS):
			if(BCNT == 1) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_MESSAGE
		case(FUNCTION_READ_MESSAGE):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_TAG_DISC_DATE
		case(FUNCTION_READ_TAG_DISC_DATE):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRITE_MESSAGE
		case(FUNCTION_WRITE_MESSAGE):
			if(BCNT == 25) {return TRUE;}
			break;
		#endif
			
		#ifdef FUNCTION_WRITE_TAG_DISC_DATE
		case(FUNCTION_WRITE_TAG_DISC_DATE):
			if(BCNT == 21) {return TRUE;}
			break;
		#endif
		//----------------------------------------------------
		/*         Common Practice Commands              */
		//----------------------------------------------------
		#ifdef FUNCTION_WRTIE_PV_UPPER_RANGE
		case(FUNCTION_WRTIE_PV_UPPER_RANGE):
//			if(BCNT == 21) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRTIE_PV_LOWER_RANGE
		case(FUNCTION_WRTIE_PV_LOWER_RANGE):
//			if(BCNT == 21) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_ENTEROREXIT_FIXED_CURRENT
		case(FUNCTION_ENTEROREXIT_FIXED_CURRENT):
			if(BCNT == 4) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_PERFORM_TRANSMITTER_SELFTEST
		case(FUNCTION_PERFORM_TRANSMITTER_SELFTEST):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_PERFORM_DEVICE_RESET
		case(FUNCTION_PERFORM_DEVICE_RESET):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO
		case(FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO):
			if(BCNT == 4) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN
		case(FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN):
			if(BCNT == 4) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRITE_NUM_RES_PREAMBLES
		case(FUNCTION_WRITE_NUM_RES_PREAMBLES):
			if(BCNT == 1) {return TRUE;}
			break;
		#endif
		//----------------------------------------------------
		/*              Device-specific commands            */
		//----------------------------------------------------
		#ifdef FUNCTION_READ_SPECIFIC_SLOT
		case(FUNCTION_READ_SPECIFIC_SLOT):
			if(BCNT == 1) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRITE_SPECIFIC_SLOT
		case(FUNCTION_WRITE_SPECIFIC_SLOT):
			if(BCNT == 2) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_READ_DATE_TIME
		case(FUNCTION_READ_DATE_TIME):
			if(BCNT == 0) {return TRUE;}
			break;
		#endif

		#ifdef FUNCTION_WRITE_DATE_TIME
		case(FUNCTION_WRITE_DATE_TIME):
			if(BCNT == 6) {return TRUE;}
			break;
		#endif
}
return FALSE;
}
//----------------------------------------------------------------------------------------------------------------------
static uint8_t CombineResponePacket(S_Parameters_Hart *HartParameters, uint8_t *pDataOUT)
{
	uint8_t preambleNum, i, pDoutOutIndCnt = 0;
	//------------------------------------------------------------------------------------------------------------------
	preambleNum = HartParameters->ResposePreambleNum;
	for(pDoutOutIndCnt = 0; pDoutOutIndCnt <  preambleNum; pDoutOutIndCnt++)
	{
		pDataOUT[pDoutOutIndCnt]   = PREAMBLE;
	}
	//------------------------------------------------------------------------------------------------------------------
	pDataOUT[pDoutOutIndCnt++]   = BuildStartDelimiter(StartDelimiterOut_Prop.AddressType, SlaveToMaster);
	//------------------------------------------------------------------------------------------------------------------
	for(i = 0; i < StartDelimiterOut_Prop.addrSize; i ++)
	{
		pDataOUT[pDoutOutIndCnt++] = HartPacketOut_str.H_Address[i];
	}
	//------------------------------------------------------------------------------------------------------------------
	pDataOUT[pDoutOutIndCnt++]   = HartPacketOut_str.H_Command;
	//------------------------------------------------------------------------------------------------------------------
	pDataOUT[pDoutOutIndCnt++]   = HartPacketOut_str.H_BytesCount;
	//------------------------------------------------------------------------------------------------------------------
	pDataOUT[pDoutOutIndCnt++]   = HartPacketOut_str.H_StatusByte1;
	pDataOUT[pDoutOutIndCnt++]   = HartPacketOut_str.H_StatusByte2;
	//------------------------------------------------------------------------------------------------------------------
	for(i = 0; i < HartPacketOut_str.H_BytesCount - 2; i ++)
	{
		pDataOUT[pDoutOutIndCnt++] = HartPacketOut_str.H_Data[i];
	}
	//------------------------------------------------------------------------------------------------------------------
	pDataOUT[pDoutOutIndCnt]   = Calculate_LongitudinalParity(pDataOUT + preambleNum, pDoutOutIndCnt - preambleNum - 1);
	//------------------------------------------------------------------------------------------------------------------
	return (pDoutOutIndCnt + 1);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static float data_to_floatIEEE754(uint8_t *tmp)
{
	uint8_t i;
	union {
		float tmp_f;
		uint8_t buf[4];
	}U;

	for(i = 0; i < 4; i++)
	{
		U.buf[i] = *((uint8_t *)tmp+3-i);
	}
	return U.tmp_f;
}
static void floatIEEE754_to_data(uint8_t *data, float *tmp)
{
	uint8_t i = 0;
	data[i++] = *((uint8_t *)tmp+3);
	data[i++] = *((uint8_t *)tmp+2);
	data[i++] = *((uint8_t *)tmp+1);
	data[i++] = *((uint8_t *)tmp);
}
static void FillHartPacketOutData(uint8_t *pdataIN, uint8_t lengthD)
{
	uint8_t i = 0;
	for(i = 0; i < lengthD; i++)
	{
		HartPacketOut_str.H_Data[i] = pdataIN[i];
	}
}
static void set_status_Bytes(uint16_t statusHart)
{
	HartPacketOut_str.H_StatusByte1 = (uint8_t)(statusHart>>8);
	HartPacketOut_str.H_StatusByte2 = (uint8_t)(statusHart&0x00FF);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRITE_DATE_TIME                 // 131  
static E_HART_Status C131_WRITE_DATE_TIME(S_Parameters_Hart *HartParameters, uint8_t *pinData)
{
	uint8_t i=0;
	if(pinData[0] > 23 || pinData[1] > 59 || pinData[2] > 59)
		return ERROR_PARAMETER_TOO_LARGE;
  if(pinData[3] > 31 || pinData[4] > 12 || pinData[5] > 50)
		return ERROR_PARAMETER_TOO_LARGE;
	for(i = 0; i < 3; i++)
	{
		HartParameters->Time[i] = pinData[i];
		HartParameters->Date[i] = pinData[i+3];
	}

	HartPacketOut_str.H_BytesCount   = 8;
	
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_DATE_TIME                  // 130  
static void C130_READ_DATE_TIME(S_Parameters_Hart *HartParameters)
{
	uint8_t Icnt=0;
	
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Time[0];
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Time[1];
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Time[2];
	
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Date[0];
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Date[1];
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->Date[2];
	
	HartPacketOut_str.H_BytesCount   = Icnt + 2;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_SPECIFIC_SLOT              // 128   
static E_HART_Status C128_READ_SPECIFIC_SLOT(S_Parameters_Hart *HartParameters, uint8_t slotNum)
{
	if(slotNum > (SLOT_MAX_NUMBER - 1))
		return ERROR_INVALID_SELECTION;

	HartPacketOut_str.H_Data[0] = slotNum;
	HartPacketOut_str.H_Data[1] = HartParameters->DateSlotCommands[slotNum];
	
	HartPacketOut_str.H_BytesCount = 4;
	return NO_ERROR;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRITE_NUM_RES_PREAMBLES         // 59  
static E_HART_Status C59_WRTIE_NUM_RES_PREAMBLES(S_Parameters_Hart *HartParameters, uint8_t numResPreambles)
{
	if(numResPreambles < PREAMBLE_MIN_NUM)
		return ERROR_PARAMETER_TOO_SMALL;
	if(numResPreambles > PREAMBLE_MAX_NUM)
		return ERROR_PARAMETER_TOO_LARGE;
	
	HartParameters->ResposePreambleNum = numResPreambles;
	
	HartPacketOut_str.H_BytesCount   = 3;
	
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN      // 46
static E_HART_Status C46_TRIM_PRIMARY_VARIABLE_GAIN(S_Parameters_Hart *HartParameters, uint8_t *pinData)
{
	/*  Data passed from hart master for setting sensitivity represents the used concentration to 
	calibrate the device and should be between MIN_CONCENTRATION and MAX_CONCENTRATION         */ 
	
	TrimDAC_Value func;
	func = HartParameters->CalibrateDevice; 
	
	float dataValue = data_to_floatIEEE754(pinData);
	if(dataValue < (float)MIN_CONCENTRATION) 
		return ERROR_PARAMETER_TOO_SMALL;
	if(dataValue > (float)MAX_CONCENTRATION)
  	return ERROR_PARAMETER_TOO_LARGE;	
	
	HartPacketOut_str.H_BytesCount   = 6;
  
	return func(GAIN, dataValue);
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO      // 45
static E_HART_Status C45_TRIM_PRIMARY_VARIABLE_ZERO(S_Parameters_Hart *HartParameters, uint8_t *pinData)
{
	/*  Data passed from hart master for setting zero should be fixed and equal to 4       */ 
	
	TrimDAC_Value func;
	
	func = HartParameters->CalibrateDevice; 
	
	float dataValue = data_to_floatIEEE754(pinData);
	if(dataValue != SET_ZERO_CURRENT) 
		return ERROR_INVALID_SELECTION;
	
	HartPacketOut_str.H_BytesCount   = 6;
  
	return func(ZERO, dataValue);
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_PERFORM_DEVICE_RESET            // 42
static E_HART_Status C42_PerformMasterReset(S_Parameters_Hart *HartParameters)
{
	PerformMasterReset func; 
	func = HartParameters->PMR; 
	HartPacketOut_str.H_BytesCount = 2;
	func();
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_PERFORM_TRANSMITTER_SELFTEST    // 41
static E_HART_Status C41_PerformTransmitterSelfTest(S_Parameters_Hart *HartParameters)
{
	PerformTransmitterSelfTest func; 
	func = HartParameters->PTST; 
	HartPacketOut_str.H_BytesCount = 2;
	return func();
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_ENTEROREXIT_FIXED_CURRENT       // 40
static E_HART_Status C40_ENTEROREXIT_FIXED_CURRENT(S_Parameters_Hart *HartParameters, uint8_t *pinData)
{
	/* if value of current is 0 => exit fixed mode current
	  else if current passed bigger of 4mA then enter fixed current mode*/
	float CurrentValue = data_to_floatIEEE754(pinData);
	if(CurrentValue > 20)
		return ERROR_PARAMETER_TOO_LARGE;	
	if(CurrentValue < 4 && CurrentValue !=0)
		return ERROR_PARAMETER_TOO_SMALL;
	
	
	HartParameters->FixedCurrent = CurrentValue;
	
	HartPacketOut_str.H_BytesCount   = 6;
	
	if(CurrentValue == 0)
	{		
	//------------------------------------------------------------------------------------------------
	/*  0->3 Analog output current in mA*/
	  floatIEEE754_to_data(HartPacketOut_str.H_Data , &(HartParameters->LoopCurrent)); 
		return NO_ERROR_CONFIG_CHANGED;
	}
	else 
	{
		floatIEEE754_to_data(HartPacketOut_str.H_Data , &CurrentValue);
		return NO_ERROR_OUTPUT_CURRENT_FIXED;
	}	
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRTIE_PV_LOWER_RANGE            // 37 
static E_HART_Status C37_WRTIE_PV_LOWER_RANGE(S_Parameters_Hart *HartParameters , uint8_t *pdataIN)
{
	float pvLowerRange = data_to_floatIEEE754(pdataIN);
	
	if(pvLowerRange < MIN_PV_LOWER_RANGE)
		return ERROR_PARAMETER_TOO_SMALL;
	if(pvLowerRange > HartParameters->PVUpperRange)
		return ERROR_PARAMETER_TOO_LARGE;
	
	HartParameters->PVlowerRange = pvLowerRange;
	
	HartPacketOut_str.H_BytesCount   = 6;
	
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRTIE_PV_UPPER_RANGE            // 36 
static E_HART_Status C36_WRTIE_PV_UPPER_RANGE(S_Parameters_Hart *HartParameters , uint8_t *pdataIN)
{
	float pvUpperRange = data_to_floatIEEE754(pdataIN);
	
	if(pvUpperRange > MAX_PV_UPPER_RANGE)
		return ERROR_PARAMETER_TOO_LARGE;
	if(pvUpperRange < HartParameters->PVlowerRange)
		return ERROR_PARAMETER_TOO_SMALL;
	
	HartParameters->PVUpperRange = pvUpperRange;
	
	HartPacketOut_str.H_BytesCount   = 6;
	
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRITE_TAG_DISC_DATE             // 18
static E_HART_Status C18_WRITE_TAG_DISC_DATE(S_Parameters_Hart *HartParameters, uint8_t *pdataIN, uint16_t sizeData)
{
	uint8_t i = 0, Icnt = 0;
	//----------------------------------------------------------------------------------------------
	if(sizeData != 21)
	{
		return ERROR_FEW_DATA_BYTE_RECEIVED;
	}
	//----------------------------------------------------------------------------------------------
	if(pdataIN[sizeData - 3] > 31 || pdataIN[sizeData - 2] > 12  || pdataIN[sizeData - 1] > 50)
	{
		return ERROR_PARAMETER_TOO_LARGE;
	}
	//----------------------------------------------------------------------------------------------
	for(i = 0; i < 6; i++)
	{
		HartParameters->Tag[i]        = pdataIN[Icnt++];
	}
	for(i = 0; i < 12; i++)
	{
		HartParameters->Descriptor[i] = pdataIN[Icnt++];
	}
	for(i = 0; i < 3; i++)
	{
		HartParameters->Date[i]       = pdataIN[Icnt++];
	}
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = 23;
	//----------------------------------------------------------------------------------------------
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRITE_MESSAGE                   // 17
static E_HART_Status C17_WRITE_MESSAGE(S_Parameters_Hart *HartParameters, uint8_t *pdataIN, uint16_t sizeData)
{
	uint8_t Icnt = 0;
	//----------------------------------------------------------------------------------------------
	if(sizeData != 24)
	{
		return ERROR_FEW_DATA_BYTE_RECEIVED;
	}
	else
	{
		for(Icnt = 0; Icnt < sizeData; Icnt++)
		{
			HartParameters->Message[Icnt]  = pdataIN[Icnt];
		}
  }
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = sizeData + 2;
	//----------------------------------------------------------------------------------------------
	return NO_ERROR_CONFIG_CHANGED;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_TAG_DISC_DATE              // 13
static void C13_READ_TAG_DISC_DATE(S_Parameters_Hart *HartParameters)
{
	uint8_t i = 0, Icnt = 0;
	//----------------------------------------------------------------------------------------------
	for(i = 0; i < 6; i++)
	{
		HartPacketOut_str.H_Data[Icnt++] = HartParameters->Tag[i];
	}
	for(i = 0; i < 12; i++)
	{
		HartPacketOut_str.H_Data[Icnt++] = HartParameters->Descriptor[i];
	}
	for(i = 0; i < 3; i++)
	{
		HartPacketOut_str.H_Data[Icnt++] = HartParameters->Date[i];
	}
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = 23;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_MESSAGE                    // 12
static void C12_READ_MESSAGE(S_Parameters_Hart *HartParameters)
{
	uint8_t Icnt=0;
	//----------------------------------------------------------------------------------------------
	for(Icnt = 0; Icnt < 24; Icnt++)
	{
		HartPacketOut_str.H_Data[Icnt] = HartParameters->Message[Icnt];
	}
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = Icnt + 2;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_WRITE_POLLING_ADDRESS           // 6
static E_HART_Status C6_WRITE_POLLING_ADDRESS(S_Parameters_Hart *HartParameters ,uint8_t newAddress)
{
	if(newAddress > MAX_POLLING_ADDRESS)
		return ERROR_PARAMETER_TOO_LARGE;
	else 
	{
		HartParameters->PollingAddress_Short = newAddress;
				
		HartPacketOut_str.H_BytesCount   = 3;
	}
	return NO_ERROR_CONFIG_CHANGED;
}
#endif	
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_DYN_VARIABLES_AND_CURRENT  // 3
static void C3_READ_DYN_VARIABLES_AND_CURRENT(S_Parameters_Hart *HartParameters)
{
	uint8_t Icnt=0;
	//------------------------------------------------------------------------------------------------
	/*  0->3 Analog output current in mA*/
	floatIEEE754_to_data(HartPacketOut_str.H_Data    , &(HartParameters->LoopCurrent));      Icnt+=4;
	/*  4 Primary Variable Unit Code */
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->PVCode;
	/*  5->8 Primary Variable, IEEE 754 */
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->PrimaryValue));  Icnt+=4;
	/*  9 Secondary Variable Unit Code */
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->SVCode;
	/*  10->13 Secondary Variable, IEEE 754 */
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->SecondaryValue));Icnt+=4;
	/*  14 Tertiary  Variable Unit Code */
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->TVCode;
	/*  15->18 Tertiary Variable, IEEE 754 */
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->TertiaryValue)); Icnt+=4;
	/*  19 Quaternary  Variable Unit Code */
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->QVCode;
	/*  20->23 Quaternary Variable, IEEE 754 */
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->QuaternaryValue));Icnt+=4;
  //------------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = Icnt + 2;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_CURRENT_PERCENT_OF_RANGE   // 2
static void C2_READ_CURRENT_PERCENT_OF_RANGE(S_Parameters_Hart *HartParameters)
{
  uint8_t Icnt=0;	
	//------------------------------------------------------------------------------------------------
	floatIEEE754_to_data(HartPacketOut_str.H_Data    , &(HartParameters->LoopCurrent));    Icnt+=4;
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->PercnetOfRange)); Icnt+=4;
  //------------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = Icnt + 2;

}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_PRIMARY_VALUE              // 1
static void C1_READ_PRIMARY_VALUE(S_Parameters_Hart *HartParameters)
{
	uint8_t Icnt = 0;
  //----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->PVCode;
	floatIEEE754_to_data(HartPacketOut_str.H_Data + Icnt, &(HartParameters->PrimaryValue)); Icnt+=4;
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = Icnt + 2;
	
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef FUNCTION_READ_UNIQUE_ID                  // 0
static void C0_READ_UNIQUE_ID(S_Parameters_Hart *HartParameters)
{
	uint8_t Icnt=0;
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_Data[Icnt++] = 254;
	HartPacketOut_str.H_Data[Icnt++] = MANUFACTURER_IDENTIFICATION_CODE;
	HartPacketOut_str.H_Data[Icnt++] = MANUFACTURER_DEVICE_TYPE;
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->ResposePreambleNum;
	HartPacketOut_str.H_Data[Icnt++] = 5; //revision level of universal commands
	HartPacketOut_str.H_Data[Icnt++] = 0; //Revision Level of Transmitter Document
	HartPacketOut_str.H_Data[Icnt++] = 1; //Software revision
	HartPacketOut_str.H_Data[Icnt++] = 1; //Hardware revision level
	HartPacketOut_str.H_Data[Icnt++] = 0; //Flag assignment : not defined at this time
	HartPacketOut_str.H_Data[Icnt++] = UNIQUE_DEVICE_ID0;
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->PollingAddress_Long.Unique_Device_Id1;
	HartPacketOut_str.H_Data[Icnt++] = HartParameters->PollingAddress_Long.Unique_Device_Id2;
	//----------------------------------------------------------------------------------------------
	HartPacketOut_str.H_BytesCount   = Icnt + 2;
}
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ProcesseingRXData_HART(S_packetAnswer_Hart *sPacketAns, S_Parameters_Hart *HartParameters, uint8_t *pDataIN, uint8_t sizePData)
{
	uint8_t pDataIndex = 0, CHK = 0, preambleCnt = 0, addSz = 0, cmd = 0, BCNT = 0;
	
	sPacketAns->PacketOutSize   = 0; 
	sPacketAns->eHartStatus     = NO_ERROR;
	sPacketAns->commandUsed     = 0; 
	
	/*  Detecting Preamble bytes */
	while(pDataIN[preambleCnt] == PREAMBLE && preambleCnt < (sizePData - 1)) 
  { 
		preambleCnt++;
	}
	pDataIndex = preambleCnt;

	/*  Detecting Address Type and Frame Mode From Start Delimiter byte */
	if(pDataIndex == sizePData - 1)	{ return ;}
	addSz = SplitstartDelimiter(pDataIN[pDataIndex++]);
	
	/* check if packet is okey */
	if(sizePData < preambleCnt + 4 + addSz)	{ return ;}
	
	/* check if address is matched */
	if(Is_Address_Matched(HartParameters, StartDelimiterOut_Prop.AddressType, (pDataIN + pDataIndex)) == TRUE)
	{
		BuildAddress(HartParameters, StartDelimiterOut_Prop.AddressType);
		/* check if CHK is ok */
		CHK = Calculate_LongitudinalParity((pDataIN + preambleCnt), sizePData - preambleCnt - 1);
		
		if(CHK == pDataIN[sizePData-1])
		{
			pDataIndex = pDataIndex + addSz; 
			
			// index refer to Command
			cmd = pDataIN[pDataIndex++];
      sPacketAns->commandUsed = cmd;			
			
			// index refer to BCNT
			BCNT = pDataIN[pDataIndex++];   
			
			// index refer to DATA
			if(Is_Command_Supported(cmd) == TRUE) 
			{
				if(Is_ByteCounts_Correct(BCNT, cmd) == TRUE) 
				{		
					switch(cmd)
					{
						//---------------------------------------------------------------------------------------------------
						/*                                       Universal Commands                                        */
						//---------------------------------------------------------------------------------------------------
						#ifdef FUNCTION_READ_UNIQUE_ID                                                                  //  0
						case (FUNCTION_READ_UNIQUE_ID):
							C0_READ_UNIQUE_ID(HartParameters);
							break;
						#endif
            //---------------------------------------------------------------------------------------------------
						#ifdef FUNCTION_READ_PRIMARY_VALUE                                                              //  1
						case (FUNCTION_READ_PRIMARY_VALUE):
							C1_READ_PRIMARY_VALUE(HartParameters);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------
						#ifdef FUNCTION_READ_CURRENT_PERCENT_OF_RANGE                                                   //  2
						case (FUNCTION_READ_CURRENT_PERCENT_OF_RANGE):
							C2_READ_CURRENT_PERCENT_OF_RANGE(HartParameters);
							break;
						#endif					
            //---------------------------------------------------------------------------------------------------
            #ifdef FUNCTION_READ_DYN_VARIABLES_AND_CURRENT                                                  //  3
						case (FUNCTION_READ_DYN_VARIABLES_AND_CURRENT):
							C3_READ_DYN_VARIABLES_AND_CURRENT(HartParameters);
							break;
						#endif		
            //---------------------------------------------------------------------------------------------------
            #ifdef FUNCTION_WRITE_POLLING_ADDRESS                                                           //  6
						case (FUNCTION_WRITE_POLLING_ADDRESS):
							sPacketAns->eHartStatus = C6_WRITE_POLLING_ADDRESS(HartParameters, pDataIN[pDataIndex]);
							break;
						#endif							
            //---------------------------------------------------------------------------------------------------		
            #ifdef FUNCTION_READ_MESSAGE                                                                    // 12
						case (FUNCTION_READ_MESSAGE):
							C12_READ_MESSAGE(HartParameters);
							break;
						#endif								
						//--------------------------------------------------------------------------------------------------- 
            #ifdef FUNCTION_READ_TAG_DISC_DATE                                                              // 13
						case (FUNCTION_READ_TAG_DISC_DATE):
							C13_READ_TAG_DISC_DATE(HartParameters);
							break;
						#endif	
						//--------------------------------------------------------------------------------------------------- 
            #ifdef FUNCTION_WRITE_MESSAGE                                                                   // 17
						case (FUNCTION_WRITE_MESSAGE):
							sPacketAns->eHartStatus = \
						  C17_WRITE_MESSAGE(HartParameters, (pDataIN+pDataIndex), sizePData-pDataIndex-1);
							break;
						#endif	
            //---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_WRITE_TAG_DISC_DATE                                                             // 18
						case (FUNCTION_WRITE_TAG_DISC_DATE):
							sPacketAns->eHartStatus = \
						  C18_WRITE_TAG_DISC_DATE(HartParameters, (pDataIN+pDataIndex), sizePData-pDataIndex-1);
							break;
						#endif	
						//---------------------------------------------------------------------------------------------------
						/*                                    Common Practice Commands                                     */
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_WRTIE_PV_UPPER_RANGE                                                            // 36 
						case (FUNCTION_WRTIE_PV_UPPER_RANGE):
              sPacketAns->eHartStatus = C36_WRTIE_PV_UPPER_RANGE(HartParameters, pDataIN+pDataIndex);
							break;
						#endif		
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_WRTIE_PV_LOWER_RANGE                                                            // 37
						case (FUNCTION_WRTIE_PV_LOWER_RANGE):
              sPacketAns->eHartStatus = C37_WRTIE_PV_LOWER_RANGE(HartParameters, pDataIN+pDataIndex);
							break;
						#endif	
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_ENTEROREXIT_FIXED_CURRENT                                                       // 40
						case (FUNCTION_ENTEROREXIT_FIXED_CURRENT):
              sPacketAns->eHartStatus = C40_ENTEROREXIT_FIXED_CURRENT(HartParameters, pDataIN+pDataIndex);
							break;
						#endif	
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_PERFORM_TRANSMITTER_SELFTEST                                                    // 41
						case (FUNCTION_PERFORM_TRANSMITTER_SELFTEST):
              sPacketAns->eHartStatus = C41_PerformTransmitterSelfTest(HartParameters);
							break;
						#endif	
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_PERFORM_DEVICE_RESET                                                            // 42
						case (FUNCTION_PERFORM_DEVICE_RESET):
              sPacketAns->eHartStatus = C42_PerformMasterReset(HartParameters);
							break;
						#endif	
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO                                                      // 45
						case (FUNCTION_TRIM_PRIMARY_VARIABLE_ZERO):
							sPacketAns->eHartStatus = C45_TRIM_PRIMARY_VARIABLE_ZERO(HartParameters, pDataIN+pDataIndex);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN                                                      // 46
						case (FUNCTION_TRIM_PRIMARY_VARIABLE_GAIN):
							sPacketAns->eHartStatus = C46_TRIM_PRIMARY_VARIABLE_GAIN(HartParameters, pDataIN+pDataIndex);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------							
						#ifdef FUNCTION_WRITE_NUM_RES_PREAMBLES                                                         // 59 
						case (FUNCTION_WRITE_NUM_RES_PREAMBLES):
              sPacketAns->eHartStatus = C59_WRTIE_NUM_RES_PREAMBLES(HartParameters, pDataIN[pDataIndex]);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------
						/*                                    Device-specific commands                                     */
						//---------------------------------------------------------------------------------------------------		
						#ifdef FUNCTION_READ_SPECIFIC_SLOT                                                              //128
						case (FUNCTION_READ_SPECIFIC_SLOT):
              sPacketAns->eHartStatus = C128_READ_SPECIFIC_SLOT(HartParameters, pDataIN[pDataIndex]);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------		
						#ifdef FUNCTION_WRITE_SPECIFIC_SLOT                                                             //129
						case (FUNCTION_WRITE_SPECIFIC_SLOT):
//              sPacketAns->eHartStatus = \
// 						  C129_WRITE_SPECIFIC_SLOT(HartParameters, pDataIN[pDataIndex], pDataIN[pDataIndex+1]);
						  HartPacketOut_str.H_BytesCount = 4;
							break;
						#endif
						//---------------------------------------------------------------------------------------------------
						#ifdef FUNCTION_READ_DATE_TIME                                                                  //130
						case (FUNCTION_READ_DATE_TIME):
              C130_READ_DATE_TIME(HartParameters);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------
						#ifdef FUNCTION_WRITE_DATE_TIME                                                                 //131
						case (FUNCTION_WRITE_DATE_TIME):
              sPacketAns->eHartStatus = C131_WRITE_DATE_TIME(HartParameters, pDataIN+pDataIndex);
							break;
						#endif
						//---------------------------------------------------------------------------------------------------								
					}
				}
				else
				{
					/* Error Byte counts not compatible with data: Generate error code */
					sPacketAns->eHartStatus = FRAMING_ERROR_HART;
				}
			}
			else
			{
				/* Error The requested command not supported: Generate error code */
				sPacketAns->eHartStatus = ERROR_COMMAND_NOT_IMPLEMENTED;
			}
		}
		else
		{
			/* Error in Check sum: Generate error code */ 
			sPacketAns->eHartStatus = CHEKSUM_ERROR_HART;
		}
		//------------------------------------------------------------------------------------------------------------------
		if(sPacketAns->eHartStatus != NO_ERROR && sPacketAns->eHartStatus != NO_ERROR_CONFIG_CHANGED && \
				sPacketAns->eHartStatus != NO_ERROR_OUTPUT_CURRENT_FIXED)
		{
			HartPacketOut_str.H_BytesCount = 2; // error occured , Data is empty
		}
		else if(sPacketAns->eHartStatus == NO_ERROR_CONFIG_CHANGED || sPacketAns->eHartStatus == NO_ERROR_OUTPUT_CURRENT_FIXED) 
		{
			FillHartPacketOutData(pDataIN + pDataIndex, HartPacketOut_str.H_BytesCount - 2);
			ReadDataFromHartStructure(HartParameters, cmd);
		}
		//------------------------------------------------------------------------------------------------------------------
		/*  fill structure HartPacketOut with  Command */
		HartPacketOut_str.H_Command = cmd;

		/*  fill structure HartPacketOut with  Status bytes */
		set_status_Bytes((uint16_t)sPacketAns->eHartStatus);

		/*  Combine all hart packet bytes (preamble, SD, Address, Command, BCNT, status, data, CKS) in one array  */
		sPacketAns->PacketOutSize = CombineResponePacket(HartParameters, sPacketAns->pPacketDataOut);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------


