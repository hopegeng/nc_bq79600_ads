/*
 * bq797xx.c
 *
 *  Created on: Oct 16, 2025
 *      Author: rgeng
 */


#include <IfxAsclin_Asc.h>
#include <stdio.h>
/* IFX includes */
#include <IfxStm.h>
#include <IfxPort.h>
#include <Ifx_Types.h>
#include <Cpu/Std/Platform_types.h>
/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <Bsp.h>
#include "bq79600_priv.h"
#include "bq79718_priv.h"
#include "bq79600.h"
#include "comm.h"
#include "shell.h"

#define MAX_READ_LENGTH  (36)

#define ASCLIN5_TOS IfxSrc_Tos_cpu2

_Bq79600_RegisterGroup Bq79600_Reg;
_Bq79616_RegisterGroup Bq79718_Reg[TOTALBOARDS-1];

static uint16 CRC16( uint8 *pBuf, int nLen);


#define BQ_UART_BAUDRATE     1000000  // 1 Mbps
#define BQ_WAKE_LOW_US       2500     // 2.5 ms low pulse

#define ISR_PRIORITY_ASCLIN_TX 102 /* Priority of the interrupt ISR Transmit */
#define ISR_PRIORITY_ASCLIN_RX 103 /* Priority of the interrupt ISR Receive */
#define ISR_PRIORITY_ASCLIN_ER 104 /* Priority of the interrupt ISR Errors */
#define BQ_UART_BAUDRATE 1000000


#define ASC_TX_BUFFER_SIZE 64
#define ASC_RX_BUFFER_SIZE 128

static const float Gpio_ratio_value[] = {
                                  0,
                                  88.69793733,
                                  77.24052847,
                                  70.80296352,
                                  66.33135038,
                                  62.9068538,
                                  60.13077929,
                                  57.79457148,
                                  55.77567387,
                                  53.99605717,
                                  52.40303197,
                                  50.95937773,
                                  49.63784418,
                                  48.41789201,
                                  47.28366214,
                                  46.22265738,
                                  45.22485661,
                                  44.28210192,
                                  43.38766378,
                                  42.53592561,
                                  41.72215066,
                                  40.94230649,
                                  40.19293109,
                                  39.47102908,
                                  38.7739904,
                                  38.09952584,
                                  37.44561533,
                                  36.81046624,
                                  36.19247919,
                                  35.5902201,
                                  35.00239692,
                                  34.42784023,
                                  33.86548696,
                                  33.31436659,
                                  32.77358948,
                                  32.24233679,
                                  31.71985186,
                                  31.20543275,
                                  30.69842578,
                                  30.19821976,
                                  29.7042411,
                                  29.2159493,
                                  28.73283306,
                                  28.25440679,
                                  27.78020736,
                                  27.3097913,
                                  26.84273216,
                                  26.37861805,
                                  25.91704947,
                                  25.45763712,
                                  25,
                                  24.,54376346,
                                  24.08855736,
                                  23.63401433,
                                  23.17976793,
                                  22.72545092,
                                  22.27069345,
                                  21.81512124,
                                  21.35835364,
                                  20.90000164,
                                  20.43966574,
                                  19.9769337,
                                  19.51137805,
                                  19.04255339,
                                  18.56999347,
                                  18.09320784,
                                  17.6116782,
                                  17.12485426,
                                  16.63214896,
                                  16.13293314,
                                  15.62652929,
                                  15.11220439,
                                  14.58916155,
                                  14.05653012,
                                  13.51335415,
                                  12.95857855,
                                  12.39103255,
                                  11.80940973,
                                  11.2122437,
                                  10.59787824,
                                  9.964430332,
                                  9.309743786,
                                  8.631330617,
                                  7.926295884,
                                  7.191240088,
                                  6.422130533,
                                  5.614128914,
                                  4.7613559,
                                  3.85656274,
                                  2.890661907,
                                  1.852036794,
                                  0.725492442,
                                  -0.50940446,
                                  -1.881081334,
                                  -3.43081766,
                                  -5.221992751,
                                  -7.359588038,
                                  -10.03770597,
                                  -13.6843322,
                                  -19.62865497
};

static uint8 g_AscTxBuffer[ASC_TX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static uint8 g_AscRxBuffer[ASC_RX_BUFFER_SIZE + sizeof(Ifx_Fifo) + 8];
static IfxAsclin_Asc bqAsc;
static uint8 g_readBuffer[MAX_READ_LENGTH];
static uint8 g_stackReadBuffer[(TOTALBOARDS-1)*(6+MAX_READ_LENGTH)] = {0xFF};
static uint8 g_stackReadSingle[TOTALBOARDS-1][1];
static uint8 g_stackReadVolt[TOTALBOARDS-1][TI_NUM_CELL*2];
static uint8 g_stackReadGPIO[TOTALBOARDS-1][TI_NUM_GPIOADC*2];

// Use P00.6 as RX and P00.7 as TX (ASCLIN5)
#define BQ_TX_PIN IfxAsclin5_TX_P00_7_OUT		//DIO1: J18 Pin5
#define BQ_RX_PIN IfxAsclin5_RXA_P00_6_IN		//DIO0: J18 Pin6

IFX_ALIGN(4) volatile uint8 bq_cmd = 0;
uint32 ticks_per_ms = 0;

static uint8 BQ79600_Single_Wr(uint8 wrLen, uint8 devadr, uint16 regadr, uint8 *txbuf);


static void delay_us(uint32 us)
{
    uint64 ticks = IfxStm_getTicksFromMicroseconds(BSP_DEFAULT_TIMER, us);
    IfxStm_waitTicks( BSP_DEFAULT_TIMER, ticks );
}

static void delay_ms(uint32 ms)
{
    delay_us(ms * 1000);
}


IFX_INTERRUPT(asclin5TxISR, 2, ISR_PRIORITY_ASCLIN_TX);
void asclin5TxISR(void)
{
    IfxAsclin_Asc_isrTransmit(&bqAsc);
}

IFX_INTERRUPT(asclin5RxISR, 2, ISR_PRIORITY_ASCLIN_RX);
void asclin5RxISR(void)
{
    IfxAsclin_Asc_isrReceive(&bqAsc);
}



static void UART_Init(void)
{
	IfxAsclin_Asc_Config bqAscConfig;

	IfxAsclin_Asc_initModuleConfig(&bqAscConfig, &MODULE_ASCLIN5);

	/* Set the desired baud rate */
	/* One stop bit, no parity, data bit = 8bit */
	bqAscConfig.baudrate.prescaler = 1;
	bqAscConfig.baudrate.baudrate = BQ_UART_BAUDRATE;
	bqAscConfig.baudrate.oversampling = IfxAsclin_OversamplingFactor_16; /* Set the oversampling factor */

	/* Configure the sampling mode */
	//ascConf.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_three; /* Set the number of samples per bit*/
	bqAscConfig.bitTiming.medianFilter = IfxAsclin_SamplesPerBit_one; /* Set the number of samples per bit*/
	// ascConf.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_8; /* Set the first sample position */
	bqAscConfig.bitTiming.samplePointPosition = IfxAsclin_SamplePointPosition_3; /* Set the first sample position */

	/* ISR priorities and interrupt target */
	bqAscConfig.interrupt.txPriority = ISR_PRIORITY_ASCLIN_TX;
	bqAscConfig.interrupt.rxPriority = ISR_PRIORITY_ASCLIN_RX;
	//bqAscConfig.interrupt.erPriority = ISR_PRIORITY_ASCLIN_ER;
	bqAscConfig.interrupt.typeOfService = ASCLIN5_TOS;

	/* FIFO configuration */
	bqAscConfig.txBuffer = g_AscTxBuffer;
	bqAscConfig.txBufferSize = ASC_TX_BUFFER_SIZE;
	bqAscConfig.rxBuffer = g_AscRxBuffer;
	bqAscConfig.rxBufferSize = ASC_RX_BUFFER_SIZE;

	/* Pin configuration */
	const IfxAsclin_Asc_Pins pins = {
	NULL, IfxPort_InputMode_pullUp, /* CTS port pin not used */
	&BQ_RX_PIN, IfxPort_InputMode_pullUp, /* RX port pin */
	NULL, IfxPort_OutputMode_pushPull, /* RTS port pin not used */
	&BQ_TX_PIN, IfxPort_OutputMode_pushPull, /* TX port pin */
	IfxPort_PadDriver_cmosAutomotiveSpeed1
	};
	bqAscConfig.pins = &pins;

	/* Initialize the module */
	IfxAsclin_Asc_initModule(&bqAsc, &bqAscConfig);

}

static boolean UART_Send( const uint8 *data, uint32 len )
{
#if 0
    for( uint32 i = 0; i < len; i++ )
        IfxAsclin_Asc_blockingWrite(&bqAsc, data[i]);
#else
    return IfxAsclin_Asc_write( &bqAsc, data, &len, 1000 * ticks_per_ms );
#endif

}

static boolean UART_Receive( const uint8 *buf, uint32 len )
{
#if 0
    for (uint32 i = 0; i < len; i++)
    {
        //buf[i] = IfxAsclin_Asc_blockingRead(&bqAsc);
    }
#else
	return IfxAsclin_Asc_read( &bqAsc, buf, &len, 1000 * ticks_per_ms );
#endif

}


static uint8 BQ79600_Single_Rd( uint8 devadr, uint16 regadr, uint8 readlen, uint8 *readbuf )
{
    uint16 bufcrc;
    uint8 sendbuf[20];

    sendbuf[0] = Single_Device_Rd;
    sendbuf[1] = devadr;
    sendbuf[2] = (uint8)(regadr >> 8);
    sendbuf[3] = (uint8)(regadr);
    sendbuf[4] = readlen - 1;    //The bytes to read, 0x00 = read one byte of data
    bufcrc = CRC16(sendbuf, 5);
    sendbuf[5] = (uint8)(bufcrc);
    sendbuf[6] = (uint8)(bufcrc >> 8);

    if( FALSE == UART_Send( sendbuf, 7 ) )
    {
    	return 1;
    }

    if( FALSE == UART_Receive(g_readBuffer, readlen + 6) )
    {
    	return 1;
    }

	// Verify CRC
	uint16 rx_crc = g_readBuffer[readlen + 4] | (g_readBuffer[readlen + 5] << 8);
	uint16 calc_crc = CRC16(g_readBuffer, readlen + 4);
	if (rx_crc != calc_crc)
	{
		PRINTF("BQ79600_Single_Rd: CRC mismatch: RX=%04X CALC=%04X\r\n", rx_crc, calc_crc);
		return 2;
	}

	// Extract data bytes
	for (uint8 i = 0; i < readlen; i++)
		readbuf[i] = g_readBuffer[4 + i];

    return 0;

}

static uint8 BQ79600_Single_Wr(uint8 wrLen, uint8 devadr, uint16 regadr, uint8 *txbuf)
{
    uint16 i, bufcrc;
    uint8 sendbuf[20];
    sendbuf[0] = Single_Device_Wr | (wrLen-1);
    sendbuf[1] = devadr;
    sendbuf[2] = (uint8)(regadr >> 8);
    sendbuf[3] = (uint8)(regadr);

    for (i = 0; i < wrLen; i++)
        sendbuf[4 + i] = *txbuf++;

    bufcrc = CRC16(sendbuf, wrLen + 4);
    sendbuf[4 + i] = (uint8)(bufcrc & 0xFF);
    sendbuf[5 + i] = (uint8)((bufcrc >> 8) & 0xFF);

    if( FALSE == UART_Send( sendbuf, wrLen + 6 ) )
    {
    	return 1;
    }

    return 0;
}

static uint8 BQ79600_Stack_Rd(uint16 regadr, uint8 readlen, uint8 readbuf[TOTALBOARDS-1][readlen] )
{
	uint16 sendcrc;
	uint8 sendbuf[20];

    sendbuf[0] = Stack_Rd;
    sendbuf[1] = (uint8)(regadr >> 8);
    sendbuf[2] = (uint8)(regadr);
    sendbuf[3] = readlen - 1;
    sendcrc = CRC16(sendbuf, 4);
    sendbuf[4] = (uint8)(sendcrc);
    sendbuf[5] = (uint8)(sendcrc >> 8);


    if( FALSE == UART_Send( sendbuf, 6 ) )
    {
    	return 1;
    }

    /*
    * Conflicting with the datasheet, must delay 250us
    */
    delay_us(250); 	//Move to the if clause
    //Response frame:
    // byte 0 : byte 1: byte 2 : byte 3 : byte 4: byte 5 : byte 6 : repeat
    // 0x00     devaddr  regaddr  regaddr  val     crc low  crc high
	if( FALSE == UART_Receive( g_stackReadBuffer, (6+readlen)*(TOTALBOARDS-1) ) )
	{
		return 1;
	}


	// Verify CRC, for simplicity, only one stack device on the daisy chain
	for( int board = 0; board < (TOTALBOARDS-1); board++ )
	{
		volatile uint8 *p;
		uint16 rx_crc;
		uint16 calc_crc;

		p = &(g_stackReadBuffer[board*(6+readlen)]);
		rx_crc = p[readlen + 4] | (p[readlen + 5] << 8);
		calc_crc = CRC16(p, readlen+4);
		if (rx_crc != calc_crc)
		{
			PRINTF("BQ79600_Stack_Rd: CRC mismatch: RX=%04X CALC=%04X\r\n", rx_crc, calc_crc);
			return 2;
		}
		// Extract data bytes
		for (uint8 i = 0; i < readlen; i++)
			readbuf[p[1]-1][i] = p[4 + i];

	}

    return 0;

}

static uint8 BQ79600_Stack_Wr (uint8 wrLen, uint16 regadr, uint8 *txbuf)
{
	uint16 i, bufcrc;
    uint8 sendbuf[20];

    sendbuf[0] = Stack_Wr | (wrLen-1);
    sendbuf[1] = (uint8)(regadr >> 8);
    sendbuf[2] = (uint8)(regadr);

    for (i = 0; i < wrLen; i++)
        sendbuf[3 + i] = *txbuf++;

    bufcrc = CRC16(sendbuf, wrLen + 3);
    sendbuf[3 + i] = (uint8)(bufcrc);
    sendbuf[4 + i] = (uint8)(bufcrc >> 8);

    if( FALSE == UART_Send( sendbuf, wrLen + 5 ) )
    {
    	return 1;
    }

    return 0;
}

static uint8 BQ79600_Broadcast_Wr( uint8 datanum, uint16 regadr, uint8 *txbuf )
{
	uint16 i, bufcrc;
    uint8 sendbuf[20];

    sendbuf[0] = Broadcast_Wr | (datanum-1);
    sendbuf[1] = (uint8)(regadr >> 8);
    sendbuf[2] = (uint8)(regadr);

    for (i = 0; i < datanum; i++)
        sendbuf[3 + i] = *txbuf++;

    bufcrc = CRC16(sendbuf, datanum + 3);
    sendbuf[3 + i] = (uint8)(bufcrc);
    sendbuf[4 + i] = (uint8)(bufcrc >> 8);


    if( FALSE == UART_Send( sendbuf, 5+datanum ) )
    {
    	return 1;
    }

    return 0;
}

static uint8 BQ79600_Broadcast_Wr_Rev(uint8 datanum, uint16 regadr, uint8 *txbuf)
{
    uint16 i, bufcrc;
    uint8 count = 0;
    uint8 sendbuf[20];
    sendbuf[0] = Broadcast_Wr_Reverse | (datanum-1 );
    sendbuf[1] = (uint8)(regadr >> 8);
    sendbuf[2] = (uint8)(regadr);

    for (i = 0; i < datanum; i++)
        sendbuf[3 + i] = *txbuf++;

    bufcrc = CRC16(sendbuf, datanum + 3);
    sendbuf[3 + i] = (uint8)(bufcrc);
    sendbuf[4 + i] = (uint8)(bufcrc >> 8);

    if( FALSE == UART_Send( sendbuf,5 + datanum ) )
    {
    	return 1;
    }

    return 0;

}


static void bq79600_task( void )
{
    vTaskDelay( pdMS_TO_TICKS(3000) ); // Yield to other tasks
    while( 1 )
    {
    	uint8 rxbuf[1];

    	switch( bq_cmd )
    	{
    	case 1:
    		PRINTF( "********************** bq sleep*******************\r\n" );
    		rxbuf[0] = 0x04;
    		//BQ79600_Single_Wr( 1, 0x00, 0x309, rxbuf );
    		BQ79600_Stack_Wr( 1, 0x309, rxbuf );
    		bq_cmd = 0;
    		break;
    	case 2:
    		rxbuf[0] = 0x00;
    		BQ79600_Single_Wr( 1, 0x00, 0x309, rxbuf );
    		break;
    	default:
    		break;
    	}

    	vTaskDelay( pdMS_TO_TICKS(100) );
    }
}


/**************************** Public functions *******************************/

static uint8 BQ79600_Wake(void)
{

    IfxPort_setPinModeOutput(&MODULE_P00, 7, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinHigh(&MODULE_P00, 7 );
    delay_ms( 100 );

    //First wake ping
    {
		IfxPort_setPinLow(&MODULE_P00, 7 );
		// Drive TX low for 2.75 ms
		delay_us(2750);
		// Return TX high
		IfxPort_setPinHigh(&MODULE_P00, 7 );
		// Wait at least 3.5 ms (tSU(WAKE_SHUT)) to allow BQ79600-Q1 device to enter ACTIVE mode
		delay_us( 3500 );
    }


#if 0 //Second wake ping
    {
		IfxPort_setPinLow(&MODULE_P00, 7 );
		// Drive TX low for 2.75 ms
		delay_us(2750);
		// Return TX high
		IfxPort_setPinHigh(&MODULE_P00, 7 );
		delay_us( 3500 );
    }
#endif

    //Mux the P00.7 to the ASCLIN5 TX pin
	IfxAsclin_initTxPin( &IfxAsclin5_TX_P00_7_OUT, IfxPort_OutputMode_pushPull, IfxPort_PadDriver_cmosAutomotiveSpeed4 );

	{
		uint8 rxbuf[1];
		rxbuf[0] = 0xFF;
		delay_ms( 2 );
		return BQ79600_Single_Wr( 0x01, 0x00, 0x2020, rxbuf );
	}

}


static uint8 Bq79600_Auto_Adr()
{
    uint8 txbuf[8];
    uint8 comm_ret;
	uint8 response_buf[1] = { 0x0 };

    txbuf[0] = 0;

    //Step 1: Dummy stack write registers OPT_ECC_DATAIN1 to OPT_ECC_DATAIN8 to sync the DLL
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN1, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN2, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN3, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN4, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN5, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN6, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN7, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN8, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    delay_ms(1);

    //Step 2: Auto Addressing mode enable
    Bq79600_Reg.CONTROL1.Byte = 0x00;
    Bq79600_Reg.CONTROL1.Bit.ADDR_WR = 0x01;
    comm_ret = BQ79600_Broadcast_Wr(0x01, Z_CONTROL1, &Bq79600_Reg.CONTROL1.Byte);
    if( comm_ret != 0 )
    {
    	PRINTF( "Failed to auto address the BQ stack\r\n" );
    	return 1;
    }
    delay_ms(1);				//

    //Step 3: SET ADDRESSES FOR EVERY BOARD
    for( uint8 currentBoard=0; currentBoard<TOTALBOARDS; currentBoard++ )
	{
    	txbuf[0] = currentBoard;
        BQ79600_Broadcast_Wr( 0x01, (uint16)S_DIR0_ADDR, txbuf );
	}

    //Step 4: SET THE HIGHEST DEVICE IN THE STACK AS BOTH STACK AND TOP OF STACK
    txbuf[0] = 0x01;
    BQ79600_Single_Wr( 0x01, TOTALBOARDS-1, (uint16)S_COMM_CTRL, txbuf );

    //Step 6:SYNCRHONIZE THE DLL WITH A THROW-AWAY READ
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN1, 0x01, g_stackReadSingle );
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN2, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN3, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN4, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN5, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN6, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN7, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN8, 0x01, g_stackReadSingle );   // Read address

    //OPTIONAL: read back all device addresses
    for( uint8 currentBoard=0; currentBoard<TOTALBOARDS; currentBoard++ )
    {
    	if( 0 == BQ79600_Single_Rd( currentBoard, S_DIR0_ADDR, 1, response_buf ) )
    	{
    		PRINTF( "The board address: %d\r\n", response_buf[0] );
    	}
    	else
    	{
    		return 1;
    	}
    }

    //OPTIONAL: read register address 0x2001 and verify that the value is 0x14
	if( 0 != BQ79600_Single_Rd( 0, 0x2001, 1, response_buf ) )
	{
		return 1;
	}

    return 0;

}

uint8 Bq79600_Auto_Adr_Rev()
{
    uint8 response_buf[20];
    uint8 txbuf[8];
    uint8 comm_ret;

    txbuf[0] = 0;

    //Step 1: Dummy stack write registers OPT_ECC_DATAIN1 to OPT_ECC_DATAIN8 to sync the DLL
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN1, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN2, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN3, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN4, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN5, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN6, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN7, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    comm_ret = BQ79600_Stack_Wr( 0x01, S_OTP_ECC_DATAIN8, txbuf );
    if( comm_ret != 0 )
    {
    	return 1;
    }
    delay_ms(1);

    //Step 2: Change the addressing direction on stack devices
    Bq79600_Reg.CONTROL1.Byte = 0x80;
    comm_ret = BQ79600_Broadcast_Wr_Rev(0x01, Z_CONTROL1, &Bq79600_Reg.CONTROL1.Byte);
    if( comm_ret != 0 )
    {
    	PRINTF( "Failed to auto address the BQ stack\r\n" );
    	return 1;
    }
    delay_ms(1);				//

    //Step 3: Broadcast write to clear top stack flag
    txbuf[0] = 0x00;
    BQ79600_Broadcast_Wr(0x01, (uint16)S_COMM_CTRL, txbuf );
    delay_ms(1);

    //Step 4: Enable auto addressing mode
    Bq79600_Reg.CONTROL1.Byte = 0x81;
    txbuf[0] = 0x81;
    BQ79600_Broadcast_Wr(0x01, (uint16)Z_CONTROL1, txbuf );


    //Step 5: SET ADDRESSES FOR EVERY BOARD( Reverse direction )
    for( uint8 currentBoard=0; currentBoard<TOTALBOARDS; currentBoard++ )
	{
    	txbuf[0] = currentBoard;
        BQ79600_Broadcast_Wr( 0x01, (uint16)S_DIR1_ADDR, txbuf );
        delay_us( 50 );
	}

    //Step 6: Set everything as a stack device first
    txbuf[0] = 0x02;
    BQ79600_Broadcast_Wr( 0x01, S_COMM_CTRL, txbuf );

    //Step 7: SET THE HIGHEST DEVICE IN THE STACK AS BOTH STACK AND TOP OF STACK
    if( TOTALBOARDS == 1 )
    {
    	txbuf[0] = 0x01;		//Not a stack, it is a base device
    	BQ79600_Single_Wr( 0x01, TOTALBOARDS-1, (uint16)S_COMM_CTRL, txbuf );
    }
    else
    {
    	txbuf[0] = 0x03;
    	BQ79600_Single_Wr( 0x01, TOTALBOARDS-1, (uint16)S_COMM_CTRL, txbuf );
    }

    //Step 6:SYNCRHONIZE THE DLL WITH A THROW-AWAY READ
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN1, 0x01, g_stackReadSingle );
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN2, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN3, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN4, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN5, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN6, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN7, 0x01, g_stackReadSingle );   // Read address
    BQ79600_Stack_Rd( S_OTP_ECC_DATAIN8, 0x01, g_stackReadSingle );   // Read address

    //OPTIONAL: read back all device addresses
    for( uint8 currentBoard=0; currentBoard<TOTALBOARDS; currentBoard++ )
    {
    	response_buf[0] = 0xFF;
    	if( 0 == BQ79600_Single_Rd( currentBoard, S_DIR1_ADDR, 1, response_buf ) )
    	{
    		PRINTF( "The board address: %d\r\n", response_buf[0] );
    	}
    	else
    	{
            PRINTF( "The board address: Failed to find the AFE board\r\n" );
    		return 1;
    	}
    }

    //OPTIONAL: read register address 0x2001 and verify that the value is 0x14
	if( 0 != BQ79600_Single_Rd( 0, 0x2001, 1, response_buf ) )
	{
		return 1;
	}

    return 0;

}

static uint8 BQ79718_Wake_Rev( void )
{
	uint8 comm_ret;
	uint8 txbuf[1];

    delay_ms(5);// Wakeup delay

	//Below: https://www.ti.com/lit/an/sluaa17a/sluaa17a.pdf?ts=1760665032542&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ79600-Q1
	//Send a single device write to BQ79600-Q1 to set CONTROL1[SEND_WAKE]=1, which wakes up all stacked BQ79718-Q1 devices.
    /********************** Change addressing direction ***********************/
#if 1
    // Change addressing direction, this step is mandatory, the command frame tranvels from MCU to COML
    Bq79600_Reg.CONTROL1.Bit.DIR_SEL = 1;
    Bq79600_Reg.CONTROL1.Bit.GOTO_SHUTDOWN = 0;
    Bq79600_Reg.CONTROL1.Bit.SEND_SHUTDOWN = 0;
    Bq79600_Reg.CONTROL1.Bit.GOTO_SLEEP = 0;
    txbuf[0] = 0x80;
    comm_ret = BQ79600_Single_Wr( 0x01, BQ79600_ADR, Z_CONTROL1, txbuf );// &Bq79600_Reg.CONTROL1.Byte);
    if( comm_ret != 0 )
	{
		PRINTF( "Failed to wake up the BQ79718\r\n" );
		return 1;
	}
    delay_ms(4);
#endif

    Bq79600_Reg.CONTROL1.Bit.SEND_WAKE = 1;
    txbuf[0] = 0xa0;
    comm_ret = BQ79600_Single_Wr( 0x01, BQ79600_ADR, Z_CONTROL1, txbuf ); //&Bq79600_Reg.CONTROL1.Byte);
	if( comm_ret != 0 )
	{
    	PRINTF( "Failed to wake up the BQ79718\r\n" );
    	return 1;
	}

	//Allow all the devices to receive the wake tone and enter active mode before starting Auto-Address sequence
    delay_us( 50 );
	delay_ms( 80 );


	return 0;

}

static uint8 BQ79718_Wake( void )
{
	uint8 comm_ret;


    delay_ms(5);// Wakeup delay

	//Below: https://www.ti.com/lit/an/sluaa17a/sluaa17a.pdf?ts=1760665032542&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FBQ79600-Q1
	//Send a single device write to BQ79600-Q1 to set CONTROL1[SEND_WAKE]=1, which wakes up all stacked BQ79718-Q1 devices.
    /********************** Change addressing direction ***********************/
#if 0
    // Change addressing direction, this step is mandatory, the command frame tranvels from MCU to COML
    Bq79600_Reg.CONTROL1.Bit.DIR_SEL = 1;
    Bq79600_Reg.CONTROL1.Bit.GOTO_SHUTDOWN = 0;
    Bq79600_Reg.CONTROL1.Bit.SEND_SHUTDOWN = 0;
    Bq79600_Reg.CONTROL1.Bit.GOTO_SLEEP = 0;
    BQ79600_Single_Wr( 0x01, BQ79600_ADR, Z_CONTROL1, &Bq79600_Reg.CONTROL1.Byte);
    delay_ms(4);
#endif

    Bq79600_Reg.CONTROL1.Byte = 0x20;
    comm_ret = BQ79600_Single_Wr( 0x01, BQ79600_ADR, Z_CONTROL1, &Bq79600_Reg.CONTROL1.Byte);
	if( comm_ret != 0 )
	{
    	PRINTF( "Failed to wake up the BQ79718\r\n" );
    	return 1;
	}

	//Allow all the devices to receive the wake tone and enter active mode before starting Auto-Address sequence
    delay_us( 50 );
	delay_ms( 80 );


	return 0;

}

static void BQ79718_setGPIO( void )
{
	uint8 txbuf[1] = { 0x09 };

	BQ79600_Stack_Wr( 1, 0x01E, txbuf );
	BQ79600_Stack_Wr( 1, 0x01F, txbuf );
	BQ79600_Stack_Wr( 1, 0x020, txbuf );
	BQ79600_Stack_Wr( 1, 0x021, txbuf );
	BQ79600_Stack_Wr( 1, 0x022, txbuf );

	txbuf[0] = 0x01;
	BQ79600_Stack_Wr( 1, 0x023, txbuf );



}

static void debug_print( void )
{
	uint8 rxbuf[5] = { 0xff, 0xff, 0xff, 0xff, 0xff };

	BQ79600_Single_Rd( 0x00, 0x2020, 0x01, rxbuf );
	PRINTF( "The 0x306 = 0x%x\r\n", rxbuf[0] );

	memset( rxbuf, 0xff, 5 );
	rxbuf[0] = 0xFF;
	delay_ms( 2 );
	BQ79600_Single_Wr( 0x01, 0x00, 0x2020, rxbuf );

	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2020, 0x01, rxbuf );
	PRINTF( "The 0x306 = 0x%x\r\n\r\n", rxbuf[0] );

	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x309, 0x01, rxbuf );
	PRINTF( "The control 1 = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x30a, 0x01, rxbuf );
	PRINTF( "The control 2 = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2100, 0x01, rxbuf );
	PRINTF( "The fault summary = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2101, 0x01, rxbuf );
	PRINTF( "The fault reg = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2102, 0x01, rxbuf );
	PRINTF( "The fault sys = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2103, 0x01, rxbuf );
	PRINTF( "The fault pwr = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	BQ79600_Single_Rd( 0x00, 0x2104, 0x01, rxbuf );
	PRINTF( "The fault comm 1 = 0x%x\r\n", rxbuf[0] );
	memset( rxbuf, 0xff, 5 );
	delay_ms( 2 );
	BQ79600_Single_Rd( 0x00, 0x2102, 0x01, rxbuf );
	PRINTF( "The fault comm 2 = 0x%x\r\n", rxbuf[0] );

}


void bq79xxx_comInit( void )
{
	ticks_per_ms = IfxStm_getTicksFromMilliseconds( BSP_DEFAULT_TIMER, 1 );
	UART_Init();
}

uint8 bq79xxx_doInit( void )
{

	//Step 1: wakeup the 79BQ600
	{
		if( 0 != BQ79600_Wake() )
		{
			return 1;
		}

	}

	//Step 2: Wakes up all stacked BQ79718 devices
	{
		if( 0 != BQ79718_Wake() )
		{
			return 1;
		}
	}

	{//Step 3: Auto Addressing
		if( 0 != Bq79600_Auto_Adr() )
		{
			return 1;
		}
		//test single read:
		volatile uint8 rxbuf[1] = { 0xff };
#if 0	//Single read bq79718 passed.
		BQ79600_Single_Rd( 0x01, 0x001, 0x01, rxbuf );
		PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
		BQ79600_Stack_Rd( 0x001, 0x01, rxbuf );
		PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
#endif

	}

	BQ79718_setGPIO();

	{//Step 4:
		//disable comm error
		uint8 txbuf[1] = { 0x00 };
		BQ79600_Single_Wr( 1, 0x00, 0x2005, txbuf );

		txbuf[0] = 0xa5;
		BQ79600_Broadcast_Wr( 1, 0x700, txbuf );
		delay_ms( 20 );

		txbuf[0] = 0x20;
		BQ79600_Broadcast_Wr( 1, 0x701, txbuf );
		delay_us( 4000 );	//4ms total required after shutdown to wake transition for AFE settling time, this is for top device only

		txbuf[0] = 0x40;
		BQ79600_Stack_Wr( 1, 0x10, txbuf );	//MASK CUST_CRC SO CONFIG CHANGES DON'T FLAG A FAULT

	}


	{ //Step 5: main loop
		uint8 txbuf[1] = { 0x0c };
#if 1
		BQ79600_Stack_Wr( 1, 0x03, txbuf );
#else
		//For test purpose
		txbuf[0] = 0x12;

		BQ79600_Single_Wr( 1, 0x01, 0x03, txbuf );
		BQ79600_Single_Wr( 1, 0x02, 0x03, txbuf );
#endif

		delay_us( 50 );

		txbuf[0] = 0x05;
		BQ79600_Stack_Wr( 1, 0x311, txbuf );
		delay_us( 38000 + 5 * TOTALBOARDS );

		txbuf[0] = 0x02;
		BQ79600_Stack_Wr( 1, 0x313, txbuf );
		delay_us( 4000 );

		txbuf[0] = 0x03;
		BQ79600_Stack_Wr( 1, 0x313, txbuf );
		delay_us( 2000 );

		txbuf[0] = 0x16;
		BQ79600_Stack_Wr( 1, 0x316, txbuf );
		txbuf[0] = 0x17;
		BQ79600_Stack_Wr( 1, 0x316, txbuf );
		delay_ms( 70 );
	}

	return 0;
}

uint8 bq79xxx_doInit_Rev( void )
{

	//Step 1: wakeup the 79BQ600
	{
		if( 0 != BQ79600_Wake() )
		{
			return 1;
		}

	}

	//Step 2: Wakes up all stacked BQ79718 devices
	{
		if( 0 != BQ79718_Wake_Rev() )
		{
			return 1;
		}
	}

	{//Step 3: Auto Addressing
		if( 0 != Bq79600_Auto_Adr_Rev() )
		{
			return 1;
		}
		//test single read:
		volatile uint8 rxbuf[1] = { 0xff };
#if 0	//Single read bq79718 passed.
		BQ79600_Single_Rd( 0x01, 0x001, 0x01, rxbuf );
		PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
		BQ79600_Stack_Rd( 0x001, 0x01, rxbuf );
		PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
#endif

	}

	BQ79718_setGPIO();

	{//Step 4:
		//disable comm error
		uint8 txbuf[1] = { 0x00 };
		BQ79600_Single_Wr( 1, 0x00, 0x2005, txbuf );

		txbuf[0] = 0xa5;
		BQ79600_Broadcast_Wr( 1, 0x700, txbuf );
		delay_ms( 20 );

		txbuf[0] = 0x20;
		BQ79600_Broadcast_Wr( 1, 0x701, txbuf );
		delay_us( 4000 );	//4ms total required after shutdown to wake transition for AFE settling time, this is for top device only

		txbuf[0] = 0x40;
		BQ79600_Stack_Wr( 1, 0x10, txbuf );	//MASK CUST_CRC SO CONFIG CHANGES DON'T FLAG A FAULT

	}


	{ //Step 5: main loop
		uint8 txbuf[1] = { 0x0c };
		BQ79600_Stack_Wr( 1, 0x03, txbuf );
		delay_us( 50 );

		txbuf[0] = 0x05;
		BQ79600_Stack_Wr( 1, 0x311, txbuf );
		delay_us( 38000 + 5 * TOTALBOARDS );

		txbuf[0] = 0x02;
		BQ79600_Stack_Wr( 1, 0x313, txbuf );
		delay_us( 4000 );

		txbuf[0] = 0x03;
		BQ79600_Stack_Wr( 1, 0x313, txbuf );
		delay_us( 2000 );

		txbuf[0] = 0x16;
		BQ79600_Stack_Wr( 1, 0x316, txbuf );
		txbuf[0] = 0x17;
		BQ79600_Stack_Wr( 1, 0x316, txbuf );
		delay_ms( 70 );
	}

	return 0;
}


void bq79xxx_sampleVolt( uint16 pVolt[TOTALBOARDS-1][TI_NUM_CELL] )
{
	uint8 rxbuf[18*2];
	uint16 raw;

	{
		sint16 signed_val;
		#define ADC_RESOLUTION 0.1f  // 100 �V

		raw = 0x00;

		//Both stack read and single read volts are OK.
#if 1
		if( 1 == BQ79600_Stack_Rd( 0x574, 18*2, g_stackReadVolt ) )
		{
		    memset( (void*)pVolt, 0, sizeof(uint16) * (TOTALBOARDS-1) * TI_NUM_CELL );
		}
		else
		{
            for( int board = 0; board < TOTALBOARDS-1; board++ )
            {
                //PRINTF( "\r\nThe board = %d\r\n", board+1 );

                for( int cell = 0; cell < TI_NUM_CELL; cell++ )
                {
                    raw = g_stackReadVolt[board][2*cell];
                    raw = (raw<<8) | g_stackReadVolt[board][2*cell+1];
                    pVolt[board][cell] = raw;
                    //PRINTF( "\tThe board = %d,  cell %d = 0x%x, %d\r\n", board+1, (18-cell), pVolt[board][cell], (int)((float)pVolt[board][cell] * (ADC_RESOLUTION)) );

                }
            }
		}

#else
		for( int dev = 1; dev < TOTALBOARDS; dev++ )
		{
			PRINTF( "\r\nThe device = %d\r\n", dev );
			BQ79600_Single_Rd( dev, 0x574, 18 * 2, rxbuf );
			for( int idx = 0; idx < 18; idx++ )
			{
				raw = rxbuf[2*idx];
				raw = (raw<<8) | rxbuf[2*idx+1];
				pVolt[17-idx] = raw;

				if (raw & 0x8000)
					signed_val = (sint16)(raw - 0x10000);
				else
					signed_val = (sint16)raw;
				PRINTF( "The cell %d: voltage = 0x%x, %d\r\n",
						(18-idx), raw, (int)((float)signed_val * (ADC_RESOLUTION)) );
			}

		}
#endif

		delay_ms( 500 );


	}

}
void bq79xxx_sampleGPIO( uint16 pGPIO[TOTALBOARDS-1][TI_NUM_GPIOADC] )
{
	uint8 rxbuf[18*2];
	uint16 raw;

	{
		sint16 signed_val;
		#define ADC_RESOLUTION 0.1f  // 100 �V
		#define VLSB_GPIO_RATIO	0.005f
		raw = 0x00;

		//Both stack read and single read volts are OK.
		BQ79600_Stack_Rd( 0x5AC, TI_NUM_GPIOADC*2, g_stackReadGPIO );
		for( int board = 0; board < TOTALBOARDS-1; board++ )
		{
			//PRINTF( "\r\nThe board = %d\r\n", board+1 );
			for( int gpio = 0; gpio < TI_NUM_GPIOADC; gpio++ )
			{
				uint8 tempVolt;
				raw = g_stackReadGPIO[board][2*gpio];
				raw = (raw<<8) | g_stackReadGPIO[board][2*gpio+1];
				pGPIO[board][gpio] = raw;

                tempVolt = abs((uint8) ((raw) * VLSB_GPIO_RATIO));//VLSB_GPIO_RATIO;
                if ( (tempVolt > 0) && ( tempVolt < 100))
				{
				   //PRINTF("GPIO %d, raw = 0x%x, tempVolt = %d, temp = %f\r\n", gpio+1, raw, tempVolt, Gpio_ratio_value[tempVolt] );

				}
				else
				{
					   //PRINTF("GPIO %d = Error, raw = 0x%x, tempVolt = %d\r\n", gpio+1, raw, tempVolt );
				}
				////PRINTF( "\tThe cell %d = 0x%x, %d\r\n", (18-cell), raw, (int)((float)raw * (ADC_RESOLUTION)) );
			}
		}

	}

}


void bq79600_doCmd( void )
{

}

void bq79600_taskInit( void )
{
    xTaskCreate( bq79600_task, "bq79600", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}
#if 0
void core0_main(void)
{

	IfxCpu_enableInterrupts();

	delay_ms( 1000 );

	PRINTF( "The BQ79xx device driver is ready\r\n" );


	{

		//Initializing the UART(ASCLIN5)
		UART_Init();

		{
#if 0
			//Go to sleep
			uint8 txbuf[1] = { 0x04 };
			BQ79600_Single_Wr( 0x01, 0x00, 0x309, txbuf );
			delay_ms( 2000 );

			while( 1 )
				;
#endif

		}

		//Step 1: wakeup the 79BQ600
		{
			BQ79600_Wake();

		}

		//Step 2: Wakes up all stacked BQ79718 devices
		{
			BQ79718_Wake();
		}

		{//Step 3: Auto Addressing
			Bq79600_Auto_Adr();
			//test single read:
			volatile uint8 rxbuf[1] = { 0xff };
#if 0	//Single read bq79718 passed.
			BQ79600_Single_Rd( 0x01, 0x001, 0x01, rxbuf );
			PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
			BQ79600_Stack_Rd( 0x001, 0x01, rxbuf );
			PRINTF( "The bq79718 register 0x01 = 0x%x\r\n", rxbuf[0] );
#endif

		}

		{//Step 4:
			//disable comm error
			uint8 txbuf[1] = { 0x00 };
			BQ79600_Single_Wr( 1, 0x00, 0x2005, txbuf );

			txbuf[0] = 0xa5;
			BQ79600_Broadcast_Wr( 1, 0x700, txbuf );
			delay_ms( 20 );

			txbuf[0] = 0x20;
			BQ79600_Broadcast_Wr( 1, 0x701, txbuf );
			delay_us( 4000 );	//4ms total required after shutdown to wake transition for AFE settling time, this is for top device only

			txbuf[0] = 0x40;
			BQ79600_Stack_Wr( 1, 0x10, txbuf );	//MASK CUST_CRC SO CONFIG CHANGES DON'T FLAG A FAULT

		}

		{ //Step 5: main loop
			uint8 txbuf[1] = { 0x0c };
			BQ79600_Stack_Wr( 1, 0x03, txbuf );
			delay_us( 50 );

			txbuf[0] = 0x05;
			BQ79600_Stack_Wr( 1, 0x311, txbuf );
			delay_us( 38000 + 5 * TOTALBOARDS );

			txbuf[0] = 0x02;
			BQ79600_Stack_Wr( 1, 0x313, txbuf );
			delay_us( 4000 );

			txbuf[0] = 0x03;
			BQ79600_Stack_Wr( 1, 0x313, txbuf );
			delay_us( 2000 );

			txbuf[0] = 0x16;
			BQ79600_Stack_Wr( 1, 0x316, txbuf );
			txbuf[0] = 0x17;
			BQ79600_Stack_Wr( 1, 0x316, txbuf );
			delay_ms( 70 );
			do
			{
				uint8 rxbuf[18*2];
				uint16 raw;
				static int cnt = 0;

#if 0
				BQ79600_Single_Rd( 0x01, 0x574, 1, rxbuf );
				vol = rxbuf[0];

				BQ79600_Single_Rd( 0x01, 0x575, 1, rxbuf );
				vol = (vol<<8) | rxbuf[0];
#endif
				for( int idx = 0; idx < 18; idx++ )
				{
					sint16 signed_val;
					#define ADC_RESOLUTION 0.1f  // 100 �V

					raw = 0x00;
					BQ79600_Stack_Rd( 0x574 + 2*idx, 2, rxbuf );
					raw = rxbuf[0];
					raw = (raw<<8) | rxbuf[1];
					if (raw & 0x8000)
						signed_val = (sint16)(raw - 0x10000);
					else
						signed_val = (sint16)raw;
					PRINTF( "The cell %d: voltage = 0x%x, %d, %d\r\n",
							(18-idx), raw, signed_val, (int)((float)signed_val * (ADC_RESOLUTION)) );

					delay_ms( 500 );
				}

				{

				}
				PRINTF( "\r\n" );

			} while( 1 );
		}

		{
			//debug_print();
		}
	}

	while (1)
	{
		delay_ms(100);
	}

}

#endif


/***************************************** Help Function *******************************/
// CRC16 TABLE
// ITU_T polynomial: x^16 + x^15 + x^2 + 1
const uint16 crc16_table[256] = { 0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301,
        0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
        0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81,
        0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00,
        0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1,
        0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380,
        0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141,
        0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501,
        0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0,
        0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881,
        0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401,
        0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1,
        0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180,
        0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
        0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01,
        0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1,
        0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80,
        0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200,
        0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1,
        0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780,
        0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41,
        0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901,
        0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1,
        0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80,
        0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

static uint16 CRC16(uint8 *pBuf, int nLen)
{
    uint16 wCRC = 0xFFFF;
    int i;

    for (i = 0; i < nLen; i++)
    {
        wCRC ^= (*pBuf++) & 0x00FF;
        wCRC = crc16_table[wCRC & 0x00FF] ^ (wCRC >> 8);
    }

    return wCRC;
}
