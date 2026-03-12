/**
 ******************************************************************************
 * @file 	tle9012.c
 * @version V0.1.0
 * @date    2024-01-01
 * @brief	To implement the TLE9012 protocol
 ******************************************************************************
 * @attention
 *
 * <h2><center> OnzerOS&trade; SYSTEM LEVEL OPERATIONAL SOFTWARE</center></h2>
 * <h2><center>&copy; COPYRIGHT 2019 Neutron Controls</center></h2>
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * !DO NOT DISTRIBUTE!
 * For evaluation only.
 * Extended license agreement required for distribution and use!
 ******************************************************************************
 */

/**
 ******************************************************************************
 *

 * @date    2024-01-01
 * @brief   Enumerate the TLE9012 network
 *
 * From section 8.2.3 of TLE9012AQU_TDS_Rev0.1.pdf:
 * The enumeration procedure is started by the microcontroller. After a wake-up
 * signal has been sent through the chain, all the devices are numbered as "#0",
 * and in this status the devices will not forward any IBCB messages except for
 * EMM signals.
 * The microcontroller will send (from either the IFH or IFL interfaces) a write
 * command to device ID#0 changing the ID from #0 to #1. With this, the first
 * device in the chain is already numbered as #1. Since it is no longer #0, the
 * first device in the chain will forward the messages further in the chain;
 * therefore the microcontroller can again send a write command to device ID#0
 * changing the ID from #0 to #2. Now the second device is already numbered as
 * #2. This task must be continued until the message is received by the opposite
 * interface on the microcontroller side (if the first message was sent from IFL
 * that will be IFH or vice versa). Issuing a number higher than the number of
 * slaves in the chain will result in no response. The last device in the chain
 * must be marked as final node.
 * Please note that the numbering in the chain must always be consecutive,
 * starting with the lowest number next to the master of the chain. Otherwise
 * there is a potential risk of clash in the communication bus.
 *
 * ============================================================================
 * When battery packs are first attached to the TLE9012s and the MCU has power
 * applied, enumeration executes with no errors (write reply frame to CONFIG
 * writes are 0x00).
 * Enumeration of 4 TLE9012 nodes
 * Tx					Rx
 * 1e 80 36 00 01 78	00
 * 1e 01 36 62			01 36 00 01 b1
 * 1e 80 36 00 02 09	00
 * 1e 02 36 76			02 36 00 02 30
 * 1e 80 36 00 03 26	00
 * 1e 03 36 9f			03 36 00 03 aa
 * 1e 80 36 08 04 6e	00
 * 1e 04 36 5e			04 36 08 04 98
 *
 * Resets of the MCU via SW2 causes errors and enumeration only sometimes
 * passes if more than 2 seconds between resets.
 * If a subsequent reset happens before the 2 second timeout the enumeration
 * usually succeeds.
 * A likely consequence of enumeration while the network is awake is a
 * non-response since nodes have already been enumerated and the zero address
 * is invalid.
 *
 * @param   net_arch - high or low side isoUART topology
 * @retval  -1: 						no node memory locations available
 * 			0 - TLE9012_MAX_DEVICES:	number of nodes enumerated
 *
 *****************************************************************************/

/* C includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* IFX includes */
#include "Ifx_Types.h"
#include "Stm/Std/IfxStm.h"
#include "Scu/Std/IfxScuWdt.h"
#include "IfxSrc_reg.h"
#include "IfxPort_Reg.h"
#include "IfxAsclin_reg.h"
#include "Asclin/Std/IfxAsclin.h"
#include "IfxSrc_regdef.h"
#include "IfxStm_reg.h"

/* Application includes */
#include "comm.h"
#include "tle9012_reg.h"
#include "tle9012.h"
#include "can.h"
#include "board.h"
#include "shell.h"
#include "tools.h"
#include "ecu8tr_cmd.h"
#include "isouart.h"
#include "eeprom.h"

#define TLE9012_SYNCH 					(0x1Eu)
#define TLE9012_BROADCAST_DEVICE		(0x3F)
#define TLE9012_MULTI_READ				(0x31)


#define CRC_8BIT_A      (0x011d7207u)
#define CRC_3BIT_A      (0x000b2002u)

/* Timer delays as per TLE9015 and TLE9012 datasheets */
#define TLE9015_MAX_TWAKE		(0.0005f)	/* tWAKE Max. = 500 usec */
#define TLE9012_MAX_TWAKE		(0.00035f)	/* tWAKE Max. = 350 usec */
#define TLE9012_FRAME_DELAY		(0.000005f)	/* 10 bit 2Mbps frame = 5 usec */
/* The TLE9012 reply delay is 3 usec.  Pass through delays accumulate for each
 * node forward and back through the network of up to 20 nodes:
 * 3 usec + ((20 * .1 usec) * 2) = 7 usec
 * We will double that to 14 usec. */
#define TLE9012_MAX_REQ_DELAY	(0.000014f) /* 14 usec */

#define ENABLE_DEBUG    (1)
#define TLE9012_DBG( ... )                             \
    do {                                                \
        if (ENABLE_DEBUG) PRINTF(__VA_ARGS__);          \
    } while (0)

/*----- constant Member variables -----------------------------------------------------*/
static __far uint8 emm_msg[] = { 'E', 'R', 'R', 'Q' };

/*----- Member variables -----------------------------------------------------*/
static uint8 nodeNr = 0;
static __far CanPkt_t emm_canNode __align(4) =
{
		.data = (uint32_t*)emm_msg,
};
volatile static __private0 __far uint32_t errqCnt = 0x00u;

volatile static TLE9012_BITWIDTH_t tle9012_bitWidth = TLE9012_BITWIDTH_16;
extern boolean tle9012_wakeup_result;

/*----- Global variables -----------------------------------------------------*/
const static uint32 MAX_ENTRIES_TLE9012_TABLE = (0x3c);
static uint8 indirect_rd_dev = 0xFF;
static uint8 indirect_rd_reg = 0xFF;

typedef struct __packed__
{
	uint8 reg;
	uint16 revVal;
} TLE9012_REG_TABLE_t;
////volatile  TLE9012_REG_TABLE_t  tle9012_reg_table[20][MAX_ENTRIES_TLE9012_TABLE] __attribute__ ((section("NC_cpu0_dlmu")));


/*----- Protected Functions -------------------------------------------------*/
/**
 ******************************************************************************
 * CRC calculations using TriCore CRCN instruction.
 * From Tasking ctc/include.cpp/intrinsics.h:
 *	unsigned int __crcn(unsigned int d, unsigned int a, unsigned int b);
 *
 * a[2:0] contains M-1, where M is the input data with of b.
 * a[7:3] must be zero.
 * a[8] if set input data bit order of b is treated as little-endian, otherwise
 * input data bit order is treated as big-endian.
 * a[9] if set a bit-wise logical inversion is applied to both the result and
 * seed values.
 * a[11:10] must be zero.
 * a[15:12] contains N-1, where N is the CRC width in the range[1,16]
 * a[16+N-1:16] encodes the coefficients of the generator polynomial.
 * a[32:16+N-1] must be zero.
 */

static inline uint8 tle9012_Calc8BitCrc(uint8* data_ptr, uint8 length)
{
        uint32_t crc_result = 0u;


        while (length--)
        {
                crc_result = __crcn(crc_result, CRC_8BIT_A, (uint32_t)*data_ptr++);
        }
        return ((uint8)crc_result);
} /* END tle9012_Calc8BitCrc() */

static inline uint8 tle9012_Calc3BitCrc( uint8 data )
{
        uint32_t crc_result = __crcn(0, CRC_3BIT_A, (uint32_t)data);


        return ((uint8)crc_result);
} /* END tle9012_Calc3BitCrc() */

boolean tle9012_isWakeup( void )
{
	uint8 buffer[16];
	uint16 icvid;

	tle9012_rdReg( 1, ICVID, buffer );

	if( buffer[0] == ISOUART_COMM_SUCCESS )
	{
		icvid = (buffer[5] << 8) | buffer[6];
		if( icvid == 0xc140 )
		{
			return TRUE;
		}
	}

	return FALSE;
}

static void tle9012_triggerMeas( uint8 *pRet )
{
	TLE9012_MEAS_CTRL_Register_t meas_control;

	meas_control.U16 = 0;
	meas_control.B.CVM_BIT_WIDTH = 0x06;
	meas_control.B.BVM_BIT_WIDTH = 0x06;
	if( tle9012_bitWidth == TLE9012_BITWIDTH_10 )
	{
		meas_control.B.CVM_BIT_WIDTH = 0x00;
		meas_control.B.BVM_BIT_WIDTH = 0x00;
	}
	meas_control.B.CVM_START = 1;
	meas_control.B.BVM_START = 1;
	tle9012_wrReg( TLE9012_BROADCAST_DEVICE, MEAS_CTRL, meas_control.U16, pRet );		//Enable all 12 cells
}

static boolean tle9012_waitMeasDone( uint8 dev )
{
	TLE9012_MEAS_CTRL_Register_t meas_control;
	uint16_t tle9012TimeOut = 0;
	uint8 ret[16];

	while( tle9012TimeOut++ <= 5 )
	{
		tle9012_rdReg( dev, MEAS_CTRL, ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
			meas_control.U16 = (ret[5]<<8) | ret[6];
			if( meas_control.B.CVM_START == 0 && meas_control.B.BVM_START == 0 )
			{
				return TRUE;
			}
		}
		else
		{
			//comms has crashed try to do a wakeup
			tle9012_wakeup_result = FALSE;

			tle9012_doFullWakeup();
		}
	}

	return FALSE;
}


/**
 ******************************************************************************
 *
 * @author 	RG
 * @date   	May2021
 * @brief	Function: tle9012_faultHanlder
 * 			Handling the TLE9012 EMM fault, by sending it to the upper level.
 *
 * @param	None
 * @retval	None
 *
 *****************************************************************************/
void tle9012_faultHanlder( void )
{
	errqCnt++;
///	xcanCH0_SendBlocking( &emm_canNode );

} /* END of tle9012_faultHanlder */


/*----- Public Functions ----------------------------------------------------*/

/**
 ******************************************************************************
 *
 * @author 	RG
 * @date   	May2021
 * @brief	Function: tle9012_rdReg
 * 			Read a register from TLE9012
 * 			It supports single register read, broad cast read and multi-read.
 * 			It supports reads from either HS network or LS network
 *
 * @param	net_arch: specify the network
 * 			device: specify device
 * 			reg: specify register
 * 			pRet: return read message
 * @retval	the total bytes read
 *
 *****************************************************************************/
static volatile uint8 ray_dbg[32] = {0};
uint8 tle9012_rdReg( uint8 device, uint8 reg, uint8 *pRet )
{
	uint8 cmd[5];
	uint8 rd_buffer[32];
	uint8 rd_len;
	uint8 payloadLen;
	uint8 itemNr;


	vTaskDelay(10);

	cmd[0] = TLE9012_SYNCH;
	cmd[1] = device;
	cmd[2] = reg;
	cmd[3] = tle9012_Calc8BitCrc( cmd, 3 );
	memcpy( ray_dbg, cmd, 4 );


	isouart_write( cmd, 4 );
	delay_ms( 4 );

	isouart_getResult( rd_buffer, &rd_len );

	if( rd_len <= 4 )
	{
		TLE9012_DBG( "TLE9012 Read Timeout: device = 0x%x, reg = 0x%x\r\n", device, reg );
		pRet[0] = ISOUART_COMM_FAILURE;
		return (0x01u);
	}

	payloadLen = rd_len - 0x04;
	itemNr = payloadLen / 5;

	for( int idx = 0; idx < itemNr; idx++ )
	{
		uint8 *pPos = &(rd_buffer[4+idx*5]);
		if( tle9012_Calc8BitCrc( pPos, 4) != pPos[4] )
		{
			TLE9012_DBG( "Reading: A CRC check failed:  device = %d, reg = 0x%x\r\n", device, reg );
			pRet[0] = ISOUART_COMM_FAILURE;
			return (0x01u);
		}
	}

	*pRet++ = ISOUART_COMM_SUCCESS;
	*pRet++ = isouart_isRing();
	*pRet++ = itemNr;
	for( int idx = 0; idx < itemNr; idx++ )
	{
		uint8 *pPos = &(rd_buffer[4+idx*5]);
		memmove( (void*)pRet, pPos, 4 );
		pRet += 4;
	}


	return ( 3u+itemNr * 4u );

} 	/* END of tle9012_RdReg */



/**
 ******************************************************************************
 *
 * @author 	RG
 * @date   	May2021
 * @brief	Function: tle9012_wrReg
 * 			Write a value to TLE9012 register
 * 			It supports single register write, broad cast write
 * 			It supports writes from either HS network or LS network
 *
 * @param	net_arch: specify the network
 * 			device: specify device
 * 			reg: specify register
 * 			val: specify value to write
 * 			pRet: return read message
 * @retval	the total bytes read
 *
 *****************************************************************************/
boolean tle9012_wrReg( uint8 device, uint8 reg, uint16_t val, uint8 *pRet )
{
	uint8 cmd[6];
	uint8 rd_buffer[32];
	uint8 rd_len;
	uint8 reply;


	cmd[0] = TLE9012_SYNCH;
	cmd[1] = 0x80u | device;
	cmd[2] = reg;
	cmd[3] = (uint8)((val >> 8) & 0xFF);
	cmd[4] = (uint8)(val & 0xFF);
	cmd[5] = tle9012_Calc8BitCrc( cmd, 5 );

	isouart_write( cmd, 6 );
	delay_ms( 3 );
	isouart_getResult( rd_buffer, &rd_len );
	if( rd_len != 7 )
	{
		//PRINTF( "tle9012_wrReg failure on reception: device = %d, reg = 0x%x\r\n", device, reg );
		pRet[0] = ISOUART_COMM_FAILURE;
		return false;
	}

	reply = ( rd_buffer[6] & 0x38 ) >> 3;
	if( tle9012_Calc3BitCrc(reply) != (rd_buffer[6] & 0x03) )
	{
		TLE9012_DBG( "tle9012_wrReg failure on crc: device = %d, reg = 0x%x\r\n", device, reg );
		pRet[0] = ISOUART_COMM_FAILURE;
		return false;
	}

	pRet[0] = ISOUART_COMM_SUCCESS;
	pRet[1] = isouart_isRing();
	return true;

} /* END of tle9012_wrReg */


/**
 ******************************************************************************
 *
 * @author 	RG
 * @date   	May2021
 * @brief	Function: tle9012_enumerateNetwork
 * 			Enumerate a TLE9012 network
 * 			User can specify either automatic detection or enumeration with specified
 * 			node number
 * 			It supports enumeration from either HS network or LS network
 *
 * @param	net_arch: specify the network
 * 			adc_flag: flat to enable or disable all the cell CVM( Refer to CONFIG register bit 9 )
 * 			nodeNr: node number on the network, if == 0, automatic detection
 * 			pRet: error message
 * @retval	the total nods on the network
 *
 *****************************************************************************/
uint8 tle9012_enumerateNetwork( uint8 adc_flag, uint8 nodeNr, uint8 *pRet )
{
	uint8 devNum;
	TLE9012_OP_MODE_Register_t op_mode;

	TLE9012_DBG( "Starting enumerating the network.\r\n" );

	tle9012_doPulseWakeup();

	if( nodeNr == 0 )
	{
		devNum = 0;
		while( tle9012_wrReg( 0x00, CONFIG, adc_flag << 0x09 | (devNum+1), pRet ) == TRUE )
		{
			devNum += 1;
			/* Undocumented delay required between node enumerations */
			delay_ms( 10 );
		}


		if( devNum > 0 )
		{
			TLE9012_DBG( "Total %d devices found on the network.\r\n", devNum );
		}
		else
		{
			TLE9012_DBG( "Failed to find any devices on the network.\r\n" );
			return 0;
		}

		/* Set the end of the node */
		/* First to put it into sleep mode, then wake it up again */
		op_mode.U16 = 0x0u;
		op_mode.B.PD = 0x01u;
		tle9012_wrReg( devNum, OP_MODE, op_mode.U16, pRet );
		tle9012_doPulseWakeup();
		tle9012_wrReg( 0x00, CONFIG, 0x01<<11 | adc_flag << 0x09 | devNum, pRet );

		return devNum;
	}

	for( devNum = 0; devNum < nodeNr; devNum++ )
	{
		if( (devNum+1) == nodeNr )
		{
			if( tle9012_wrReg( 0x00, CONFIG, 0x01<<11 | adc_flag << 0x09 | (devNum+1), pRet ) == FALSE )
			{
				return 0;
			}
		}
		else
		{
			if( tle9012_wrReg( 0x00, CONFIG, adc_flag << 0x09 | (devNum+1), pRet ) == FALSE )
			{
				return 0;
			}
		}

		/* Undocumented delay required between node enumerations */
		delay_ms( 10 );
	}

	return nodeNr;

} /* END of tle9012_enumerateNetwork */


void tle9012_tle9015Wakeup( uint8 level )
{
	BMS_DOUT_NSLEEP( level );
}

/* This one just sends the wake up pulses to the tle9012 */
void tle9012_doPulseWakeup( )
{
	uint8 pulse[20] = { 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00 };

	IfxPort_setPinLow( &MODULE_P32, 5 );
	delay_ms( 10 );
	IfxPort_setPinHigh( &MODULE_P32, 5 );
	delay_ms( 10 );

	isouart_write( pulse, 20 );

	delay_ms( 5 );
}

uint8 tle9012_getNodeNr( void )
{
	return nodeNr;
}

boolean tle9012_doFullWakeup( void )
{
	uint8 ret[2];

//	if( tle9012_isWakeup() == TRUE )
	{
		tle9012_sleep();
		delay_ms( 200 );
	}

	/* Here we enable all the ADC */
	nodeNr = tle9012_enumerateNetwork( 1, 0, (uint8*)&ret );
	if( nodeNr == 0 )
	{
		return FALSE;
	}

//	tle9012_wrReg( TLE9012_BROADCAST_DEVICE, OP_MODE, 2, ret ); //placed into extended watchdog mode
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		tle9012_wrReg( TLE9012_BROADCAST_DEVICE, WDOG_CNT, 0x7F, ret );		//Maximum sleep time
	}

	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		tle9012_wrReg( TLE9012_BROADCAST_DEVICE, PART_CONFIG, 0xFFF, ret );		//Enable all 12 cells
	}

	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		tle9012_wrReg( TLE9012_BROADCAST_DEVICE, TEMP_CONF, (5<<12), ret );		//5 external temp sensors
	}


	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}

}

boolean tle9012_sleep( void )
{
	uint8 ret[2];
	tle9012_wrReg( TLE9012_BROADCAST_DEVICE, OP_MODE, 1, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

boolean tle9012_readMetrics( uint8 nodeNr, ECU8TR_DATA_t *pData )
{
	uint8 ret[16];
	uint8 body_idx = 0;
	uint16 temp;


	pData->dev = nodeNr;

	tle9012_triggerMeas( ret );
	tle9012_waitMeasDone( nodeNr );
	/* Read CVM, body_idex == 0 */
	for( uint8 idx = 0; idx < 12; idx++, body_idx++ )
	{
		pData->data_body[body_idx].reg = (CVM_0+idx);
		tle9012_rdReg( nodeNr, CVM_0+idx, ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
			pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
		}
		else
		{
			TLE9012_DBG( "Failed to read value from the CVM register, set it to 0x0000\r\n" );
			pData->data_body[body_idx].reg_value = 0x0000;
			return FALSE;
		}
	}

	/* Read BVM, body_idex == 12 */
	pData->data_body[body_idx].reg = BVM;
	tle9012_rdReg( nodeNr, BVM, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the BVM register, set it to 0x0000\r\n" );
		pData->data_body[body_idx].reg_value = 0x0000;
		return FALSE;
	}
	body_idx++;

	/* Read Internal temp, body_idx == 13 */
	pData->data_body[body_idx].reg = INT_TEMP;
	tle9012_rdReg( nodeNr, INT_TEMP, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		temp = (ret[6]<<8) | ret[5];
		pData->data_body[body_idx].reg_value = temp; // internal_temp.U16&0x3FF;	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the INT_TEMP register, set it to 0x0000\r\n" );
		pData->data_body[body_idx].reg_value = 0x0000;
		return FALSE;
	}
	body_idx++;


	/* Read External temp, body_idx == 14 */
	for( uint8 idx = 0; idx < 5; idx++, body_idx++ )
	{
		pData->data_body[body_idx].reg = (EXT_TEMP_0+idx);
		tle9012_rdReg( nodeNr, EXT_TEMP_0+idx, ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
			temp = (ret[6]<<8) | ret[5];
			pData->data_body[body_idx].reg_value = temp;
		}
		else
		{
			TLE9012_DBG( "Failed to read value from the EXT_TEMP register, set it to 0x0000\r\n" );
			pData->data_body[body_idx].reg_value = 0x0000;
			return FALSE;
		}
	}

	/* Read GEN DIAG, body_idx == 19 */
	pData->data_body[body_idx].reg = GEN_DIAG;
	tle9012_rdReg( nodeNr, GEN_DIAG, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		temp = (ret[6]<<8) | ret[5];
		pData->data_body[body_idx].reg_value = temp;
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the GEN DIAG register, set it to 0x0000\r\n" );
		pData->data_body[body_idx].reg_value = 0x0000;
		return FALSE;
	}
	body_idx++;

	/* Read Indirect read, body_idx == 20 */
	if( indirect_rd_dev != 0xff && nodeNr == indirect_rd_dev )
	{
		pData->data_body[body_idx].reg = indirect_rd_reg;
		tle9012_rdReg( nodeNr, indirect_rd_reg, ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
			temp = (ret[6]<<8) | ret[5];
			pData->data_body[body_idx].reg_value = temp;
		}
		else
		{
			TLE9012_DBG( "Failed to read value from the register, set it to 0x0000\r\n" );
			pData->data_body[body_idx].reg_value = 0x0000;
			return FALSE;
		}
		body_idx++;
		indirect_rd_dev = 0xFF;
	}

	pData->body_cnt = body_idx;				//Here body_idx == 20
	return TRUE;
}

void tle9012_appyBitWidth( TLE9012_BITWIDTH_t bitWidth )
{
	tle9012_bitWidth = bitWidth;
	PRINTF( "The bit width is applied to %d\n", tle9012_bitWidth );
}

boolean tle9012_setBitWidth( int bitWidth )
{
	if( bitWidth == 10 )
	{
		tle9012_appyBitWidth( TLE9012_BITWIDTH_10 );
		eeprom_set_tle9012_resolution( TLE9012_BITWIDTH_10 );
		return TRUE;
	}

	if( bitWidth == 16 )
	{
		tle9012_appyBitWidth( TLE9012_BITWIDTH_16 );
		eeprom_set_tle9012_resolution( TLE9012_BITWIDTH_16 );
		return TRUE;
	}

	return FALSE;
}

boolean tle9012_resetWDT(void)
{
	uint8_t ret[2];
	boolean retVal = FALSE;

	tle9012_wrReg( TLE9012_BROADCAST_DEVICE, WDOG_CNT, 0x7F, ret );		//Maximum sleep time
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		retVal = TRUE;
	}
	return retVal;
}

TLE9012_BITWIDTH_t tle9012_getBitWidth( void )
{
	return tle9012_bitWidth;
}


boolean tle9012_readDiag( uint8 nodeIdx, ECU8TR_DATA_t *pData )
{
	uint8 ret[16];
	uint8 body_idx = 0;
	uint16 temp;
	boolean result = TRUE;

	pData->dev = nodeIdx;

	//1. Read GEN_DIAG( 0x0B )
	tle9012_rdReg( nodeIdx, GEN_DIAG, ret );
	pData->data_body[body_idx].reg = GEN_DIAG;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the GEN_DIAG register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//2. Read CELL_UV( 0x0C )
	tle9012_rdReg( nodeIdx, CELL_UV, ret );
	pData->data_body[body_idx].reg = CELL_UV;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the CELL_UV register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//3. Read EXT_TEMP_DIAG( 0x0 )
	tle9012_rdReg( nodeIdx, EXT_TEMP_DIAG, ret );
	pData->data_body[body_idx].reg = EXT_TEMP_DIAG;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the EXT_TEMP_DIAG register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//4. Read CELL_OV( 0x0D )
	tle9012_rdReg( nodeIdx, CELL_OV, ret );
	pData->data_body[body_idx].reg = CELL_OV;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the CELL_OV register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//5. Read DIAG_OL( 0x10 )
	tle9012_rdReg( nodeIdx, DIAG_OL, ret );
	pData->data_body[body_idx].reg = DIAG_OL;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the DIAG_OL register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//6. Read CELL_UV_DAC_COMP( 0x12 )
	tle9012_rdReg( nodeIdx, CELL_UV_DAC_COMP, ret );
	pData->data_body[body_idx].reg = CELL_UV_DAC_COMP;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the CELL_UV_DAC_COMP register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//7. Read CELL_OV_DAC_COMP( 0x13 )
	tle9012_rdReg( nodeIdx, CELL_OV_DAC_COMP, ret );
	pData->data_body[body_idx].reg = CELL_OV_DAC_COMP;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the CELL_OV_DAC_COMP register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//8. Read BAL_DIAG_OC( 0x33 )
	tle9012_rdReg( nodeIdx, BAL_DIAG_OC, ret );
	pData->data_body[body_idx].reg = BAL_DIAG_OC;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the BAL_DIAG_OC register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;

	//9. Read BAL_DIAG_UC( 0x33 )
	tle9012_rdReg( nodeIdx, BAL_DIAG_UC, ret );
	pData->data_body[body_idx].reg = BAL_DIAG_UC;
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		pData->data_body[body_idx].reg_value = (ret[6]<<8) | ret[5];	//Looks like the the cell volt is in big endiness already.
	}
	else
	{
		TLE9012_DBG( "Failed to read value from the BAL_DIAG_UC register.\n\r" );
		pData->data_body[body_idx].reg_value = 0xFFFF;
		result = FALSE;
	}
	body_idx++;



	pData->body_cnt = body_idx;
	return result;
}


void tle9012_apply_eeprom_conf( void )
{
	uint16 val;
	uint8 ret[2];

	for( uint8 reg_idx = 0; reg_idx < 0x5F; reg_idx++)
	{
		boolean valid;

		valid = eeprom_get_tle9012_reg( reg_idx, &val );
		if( valid )
		{
			PRINTF("Apply tle9012 persist register: reg = %d, val = 0x%x\r\n", (reg_idx+1), val );
			tle9012_wrReg( TLE9012_BROADCAST_DEVICE, (reg_idx+1), val, ret );
		}
	}
}


boolean tle9012_readSARHighLow( uint8 node, uint16 *pSARLow, uint16 *pSARHigh )
{
	uint8 ret[16];
	uint16_t tle9012TimeOut = 0;
	TLE9012_MEAS_CTRL_Register_t meas_control;

	*pSARLow = *pSARHigh = 0x00;
	meas_control.U16 = 0;
	meas_control.B.CVM_BIT_WIDTH = 0x06;
	meas_control.B.BVM_BIT_WIDTH = 0x06;
	if( tle9012_bitWidth == TLE9012_BITWIDTH_10 )
	{
		meas_control.B.CVM_BIT_WIDTH = 0x00;
		meas_control.B.BVM_BIT_WIDTH = 0x00;
	}
	meas_control.B.SAR_START = 1;
	tle9012_wrReg( node, MEAS_CTRL, meas_control.U16, ret );		//Enable all 12 cells
	if( ret[0] != ISOUART_COMM_SUCCESS )
	{
		return FALSE;
	}

	while( tle9012TimeOut++ <= 5 )
	{
		vTaskDelay( 5 );
		tle9012_rdReg( node, MEAS_CTRL, ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
			meas_control.U16 = (ret[5]<<8) | ret[6];
			if( meas_control.B.SAR_START == 0 )
			{
				tle9012_rdReg( node, SAR_LOW, ret );
				if( ret[0] == ISOUART_COMM_SUCCESS )
				{
					*pSARLow = ((ret[6]<<8) | ret[5]);
				}
				tle9012_rdReg( node, SAR_HIGH, ret );
				if( ret[0] == ISOUART_COMM_SUCCESS )
				{
					*pSARHigh = ((ret[6]<<8) | ret[5]);
				}
				return TRUE;
			}
		}
	}

	return FALSE;
}

boolean tle9012_readInternalTemp( uint8 node, uint16 *pTemp_1, uint16 *pTemp_2 )
{
	uint8 ret[16];
	uint16_t tle9012TimeOut = 0;
	TLE9012_MEAS_CTRL_Register_t meas_control;

	*pTemp_1 = *pTemp_2 = 0x00;
	tle9012_rdReg( node, INT_TEMP, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		*pTemp_1 = ((ret[6]<<8) | ret[5]);
	}

	tle9012_rdReg( node, INT_TEMP_2, ret );
	if( ret[0] == ISOUART_COMM_SUCCESS )
	{
		*pTemp_2 = ((ret[6]<<8) | ret[5]);
	}


	return TRUE;
}


boolean tle9012_readAll( uint8 node, ECU8TR_DATA_t* pData )
{
	uint8 ret[16];
	const uint8 reg[] = { PART_CONFIG, OL_OV_THR, TEMP_CONF, INT_OT_WARN_CONF, RR_ERR_CNT, RR_CONFIG, GEN_DIAG, CELL_UV, CELL_OV,
							DIAG_OL, REG_CRC_ERR, OP_MODE, ICVID };

	pData->body_cnt = sizeof( reg );
	pData->dev = node;
	for( int idx = 0; idx < sizeof(reg)/sizeof(uint8); idx++ )
	{
		tle9012_rdReg( node, reg[idx], ret );
		if( ret[0] == ISOUART_COMM_SUCCESS )
		{
#if 0
			tle9012_reg_table[node][idx].reg = reg[idx];
			tle9012_reg_table[node][idx].revVal = ((ret[6]<<8) | ret[5]);
#else
			pData->data_body[idx].reg = reg[idx];
			pData->data_body[idx].reg_value = ((ret[6]<<8) | ret[5]);
#endif
		}
		else
		{
			return FALSE;
		}
	}

	return TRUE;

}

void tle9012_indriect_wrReg( uint8 dev, uint8 reg, uint16 val )
{
	uint8 ret[16];

	if( dev != 0xFF )
	{
		tle9012_wrReg( dev, reg, val, ret );
	}
}

void tle9012_indriect_rdReg( uint8 dev, uint8 reg )
{
	uint8 ret[16];

	if( dev != 0xFF )
	{
		indirect_rd_reg = reg;
		indirect_rd_dev = dev;
	}
}

