/*
 * ecu8tr_can.c
 *
 *  Created on: Mar 24, 2025
 *      Author: rgeng
 */

#include <Ifx_Types.h>
#include <freeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <IfxCan_Can.h>
#include <IfxCan.h>
#include "tle9012.h"
#include "isouart.h"
#include "can.h"
#include "ecu8tr_cmd.h"
#include "ecu8tr_can.h"
#include "comm.h"
#include "led.h"
#include "bq_shared.h"
#include "shell.h"
#include "tools.h"

// Updated structure for CAN message
typedef struct {
    uint32_t id;                // CAN message ID (e.g., 257)
    uint32_t low;               // First 4 bytes (Cell_V_01 and Cell_V_02)
    uint32_t high;              // Next 4 bytes (Cell_V_03 and Cell_V_04)
    uint8_t dlc;                // Data length (fixed at 8)
} ECU8TR_CAN_Message_t;


const uint32 ECU8TR_CAN_ID[] = {
		0x0000,			//place hold, because the TLE9012 index is started from 1
		257,
		258,
		259,
		260,
		261,
		262,
		263,
		264,
		265,
		266,
		267
};

const float CELL_MEASUREMENT_FACTOR = 5000.0/65536.0;
const float	BLOCK_MEASUREMENT_FACTOR = 60000.0/65536.0;
const float SCVM_MEASUREMENT_FACTOR = 5000.0/2048.0;	//Please refer to: Page 58 SCVM Formula D:\NeutronControls\BMS\Doc\ds_TLE9012DQU_TLE9015DQU-UM-v01_00-EN.pdf
const float INT_TEMP_BASE = 547.3;						//D:\NeutronControls\Ecu8TR\ds_TLE9012DQU-DataSheet-v02_00_Downloaded_on_2025_04_04.pdf, page 39
const float INT_TEMP_LSB = 0.66;
const float DBC_TEMP_FACTOR = (1/0.1);
SemaphoreHandle_t canTxSemaphore = NULL;
extern ECU8TR_CAN_STATUS_e ecu8tr_can_status;

static inline uint16 toLittleEndian(uint16 bigEndianValue)
{
    return ((bigEndianValue & 0xFF00) >> 8) | ((bigEndianValue & 0x00FF) << 8);
}

/**
 * Encode cell voltages into a CAN message based on DBC file.
 * @param msg Pointer to CAN message structure to fill
 * @param id CAN message ID (e.g., 257 for Cell_V_01 to Cell_V_04)
 * @param adc_values Array of 4 ADC physical values from TLE9012
 */
static IfxCan_Status ecu8tr_can_encodeCellVolt( uint32 id, uint16 adc_values[4] )
{
    // Set message properties
	volatile uint32 low;
	volatile uint32 high;



    // Pack into low and high (little-endian, per DBC @1+)
    // low:  Cell_V_01 (bits 0�15), Cell_V_02 (bits 16�31)
    low = ((uint32_t)adc_values[1] << 16) | (uint32_t)adc_values[0];

    // high: Cell_V_03 (bits 0�15), Cell_V_04 (bits 16�31)
    high = ((uint32_t)adc_values[3] << 16) | (uint32_t)adc_values[2];

    return can_transmitCanMessage( id, low, high );

}

static IfxCan_Status ecu8tr_can_encodeSarLowHigh( uint16 adc_values[4] )
{
    // Set message properties
	volatile uint32 low;
	volatile uint32 high;
	uint32 id = 267;		//Fixed CAN ID based on SK's DBC



    // Pack into low and high (little-endian, per DBC @1+)
    // low:  Cell_V_01 (bits 0�15), Cell_V_02 (bits 16�31)
    low = ((uint32_t)adc_values[1] << 16) | (uint32_t)adc_values[0];

    // high: Cell_V_03 (bits 0�15), Cell_V_04 (bits 16�31)
    high = ((uint32_t)adc_values[3] << 16) | (uint32_t)adc_values[2];

    return can_transmitCanMessage( id, low, high );

}

static void ecu8tr_TITask_CAN(void)
{
    uint16_t cell_volt[4];
    IfxCan_Status tx_status;

    for (;;)
    {
        if (g_bqSharedData.dataReady)
        {
            __dsync();

            for (uint8_t board = 0; board < (TOTALBOARDS - 1U); board++)
            {
                for (uint8_t cell = 0; cell < TI_NUM_CELL; cell += 3U)
                {
                    cell_volt[0] = (cell+1)<<8 | (board+1);
                    for (uint8_t idx = 1; idx < 4U; idx++)
                    {
                        uint8_t src_cell = (uint8_t)(TI_NUM_CELL - (cell + idx));
                        cell_volt[idx] = swap_u16(g_bqSharedData.cellVolt[board][src_cell]);

                        PRINTF("\tCAN: The board = %u, cell %u = 0x%04X\r\n",
                               (unsigned int)(board + 1U),
                               (unsigned int)(cell + idx),
                               (unsigned int)cell_volt[idx]);
                    }

                    tx_status = ecu8tr_can_encodeCellVolt( 0x88, cell_volt );
                    if( pdFALSE == xSemaphoreTake(canTxSemaphore, pdMS_TO_TICKS(400)) || tx_status != IfxCan_Status_ok )
                    {
                        ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
                    }
                    else
                    {
                        ecu8tr_can_status = ECU8TR_CAN_DATA_STREAMING;
                    }
                }
            }

            g_bqSharedData.dataReady = 0U;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void ecu8tr_canServerTask( void *arg )
{
	boolean is_tle9012_wakeup = FALSE;
	uint8 node_num;
	ECU8TR_DATA_t data;
	ISOUART_NetArch_t iso_network;



	if( comm_get_interface() == ECU8TR_COMM_INTERFACE_NET )
	{
		while( TRUE )
		{
			//vTaskDelay( 1000 );
			PRINTF( "This is the CAN TASK\r\n" );
			ecu8tr_TITask_CAN();
		}
	}

	ecu8tr_can_status = ECU8TR_CAN_TLE9012_NOT_CONNECT;
    vTaskDelay( pdMS_TO_TICKS(2000) ); // Yield to other tasks


	do
	{
		vTaskDelay( pdMS_TO_TICKS(1000) );
		is_tle9012_wakeup = tle9012_doFullWakeup();
		if( is_tle9012_wakeup == FALSE )
		{
			iso_network = isouart_get_network();
			if( iso_network == ISOUART_LowSide )
			{
				isouart_setNetwork( ISOUART_HighSide );
			}
			else
			{
				isouart_setNetwork( ISOUART_LowSide );
			}
		}
	} while( is_tle9012_wakeup == FALSE );

	node_num = tle9012_getNodeNr();
	node_num = (node_num>3)? 3: node_num;
	ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
	vTaskDelay( pdMS_TO_TICKS(1000) );

    while( 1 )
    {
    	if( FALSE == tle9012_isWakeup() )
    	{
    		vTaskDelay( pdMS_TO_TICKS(300) );
        	if( FALSE == tle9012_isWakeup() )		//Double check
        	{
				ecu8tr_can_status = ECU8TR_CAN_TLE9012_NOT_CONNECT;
				do
				{
					vTaskDelay( pdMS_TO_TICKS(2000) );
					is_tle9012_wakeup = tle9012_doFullWakeup();
					if( is_tle9012_wakeup == FALSE )
					{
						iso_network = isouart_get_network();
						if( iso_network == ISOUART_LowSide )
						{
							isouart_setNetwork( ISOUART_HighSide );
						}
						else
						{
							isouart_setNetwork( ISOUART_LowSide );
						}
					}
				} while( is_tle9012_wakeup == FALSE );

				node_num = tle9012_getNodeNr();
				node_num = (node_num>3)? 3: node_num;
				ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
        	}
    	}

    	for( uint8 node_idx = 1; node_idx <= node_num; node_idx++ )
    	{
        	tle9012_resetWDT();
    		tle9012_readMetrics( node_idx, &data );
    		{
    			uint16 adc_value[4];
    			for( int id_idx = 0; id_idx < 3; id_idx++ )
    			{
    				IfxCan_Status tx_status;


    				adc_value[0] = (uint16)( CELL_MEASUREMENT_FACTOR * (float)toLittleEndian( data.data_body[0 + id_idx*4].reg_value ) );
    				adc_value[1] = (uint16)( CELL_MEASUREMENT_FACTOR * (float)toLittleEndian( data.data_body[1 + id_idx*4].reg_value ) );
    				adc_value[2] = (uint16)( CELL_MEASUREMENT_FACTOR * (float)toLittleEndian( data.data_body[2 + id_idx*4].reg_value ) );
    				adc_value[3] = (uint16)( CELL_MEASUREMENT_FACTOR * (float)toLittleEndian( data.data_body[3 + id_idx*4].reg_value ) );
    				tx_status = ecu8tr_can_encodeCellVolt( ECU8TR_CAN_ID[(node_idx-1)*3+id_idx+1], adc_value );
    				if( pdFALSE == xSemaphoreTake(canTxSemaphore, pdMS_TO_TICKS(400)) || tx_status != IfxCan_Status_ok )
    				{
    					ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
    				}
    				else
    				{
    					ecu8tr_can_status = ECU8TR_CAN_DATA_STREAMING;
    				}
    			}
    		}
    	}

    	for( uint8 node_idx = 1; node_idx < 2; node_idx++ )
    	{
    		uint16 sarLow;
    		uint16 sarHigh;
    		uint16 int_temp;
    		uint16 int_temp_2;
			uint16 adc_value[4];
			IfxCan_Status tx_status;

			tle9012_resetWDT();
    		tle9012_readSARHighLow( node_idx, &sarLow, &sarHigh );
    		adc_value[0] = (uint16)( SCVM_MEASUREMENT_FACTOR * (float)(toLittleEndian(sarLow)>>5) );
    		adc_value[1] = (uint16)( SCVM_MEASUREMENT_FACTOR * (float)(toLittleEndian(sarHigh)>>5) );

    		tle9012_readInternalTemp( node_idx, &int_temp, &int_temp_2 );
    		int_temp = toLittleEndian(int_temp) & 0x3FF;
    		int_temp_2 = toLittleEndian(int_temp_2) & 0x3FF;
    		adc_value[2] = (uint16)( DBC_TEMP_FACTOR * (INT_TEMP_BASE - INT_TEMP_LSB * (float)int_temp) );
    		adc_value[3] = (uint16)( DBC_TEMP_FACTOR * (INT_TEMP_BASE - INT_TEMP_LSB * (float)int_temp_2) );
    		tx_status = ecu8tr_can_encodeSarLowHigh( adc_value );
			if ( pdFALSE == xSemaphoreTake(canTxSemaphore, pdMS_TO_TICKS(400)) || tx_status != IfxCan_Status_ok )
			{
				ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
			}
			else
			{
				ecu8tr_can_status = ECU8TR_CAN_DATA_STREAMING;
			}
    	}


        vTaskDelay( pdMS_TO_TICKS(100) ); // Yield to other tasks
    }
}


void ecu8tr_canServerInit( void )
{
	canTxSemaphore = xSemaphoreCreateBinary();
	if (canTxSemaphore == NULL)
	{
	   // Handle semaphore creation failure
		__debug();
    }

 ////   xTaskCreate( ecu8tr_canServerTask, "CANServer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
}

