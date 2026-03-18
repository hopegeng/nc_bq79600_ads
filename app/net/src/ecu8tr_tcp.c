/*
 * ecu8_tcp.c
 *
 *  Created on: Jan 26, 2024
 *      Author: rgeng
 */

#include <bsp/inc/ecu8tr_global.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
/* IFX includes */
#include <Ifx_Types.h>

/* FreeRTOS includes */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <tcp.h>
#include <tcpip.h>
#include <udp.h>

#include "comm.h"
#include "shell.h"
#include "tools.h"
#include "eeprom.h"
#include "ecu8tr_cmd.h"
#include "tle9012.h"
#include "isouart.h"
#include "ecu8tr_version.h"
#include "isouart.h"
#include "board.h"
#include "bq_shared.h"
#include "led.h"

#define ECU8_TCP_CMD_SERVER_PORT  		8000
#define ECU8_TCP_DATA_SERVER_PORT  		8001

#define CMD_DELIMITER ( 0x2424 )			//"$$"

typedef enum
{
	ECU8TR_CMD_SUCCESS = 0,
	ECU8TR_CMD_FAILURE = 1,
	ECU8TR_CMD_NONE = 2,
} ECU8TR_CMD_RET_e;

#define DATA_PACKET_SIZE  sizeof(ECU8TR_DATA_t)

extern uint8_t network_def_config[5][8];
extern ECU8TR_CAN_STATUS_e ecu8tr_can_status;


static ECU8TR_CMD_t ecu8tr_cmd = {0};
static struct tcp_pcb *pcb_cmd_server = NULL;
static struct tcp_pcb *pcb_data_server = NULL;
static struct tcp_pcb *pcb_data_client = NULL;
static struct tcp_pcb *pcb_cmd_client = NULL;

static ip_addr_t remote_ip;

static volatile boolean is_data_sent = TRUE;
static volatile ECU8TR_TLE9012_State_e tle9012_state __attribute__ ((section("NC_cpu0_dlmu"))) = ECU8TR_TLE9012_IDLE;
static volatile ECU8TR_TLE9012_State_e tle9012_pre_state __attribute__ ((section("NC_cpu0_dlmu"))) = ECU8TR_TLE9012_IDLE;
static volatile boolean tle9012_diag_result __attribute__ ((section("NC_cpu0_dlmu"))) = TRUE;
static volatile ECU8TR_DIAG_t tle9012_diag_data __attribute__ ((section("NC_cpu0_dlmu")));

static boolean tle9012_sleep_result = FALSE;
boolean tle9012_wakeup_result = FALSE;
static uint8 tle9012_node_for_streaming = 1;
static boolean tle9012_is_sleep = TRUE;

static uint16 streamingTimeOut = 0;
static uint8 streamingTimeOutLimit = 0;
static uint8 partial_data_buff[DATA_PACKET_SIZE];
static uint32 partial_data_cnt = 0;
static SemaphoreHandle_t xDataSocketMutex;
static SemaphoreHandle_t xGetDiagSemaphore;
static volatile uint32  snd_wnd_cnt = DATA_PACKET_SIZE;

static void ecu8tr_closeDataSocket( void );

struct WRITE_TABLE_s
{
	uint8 write_dev;
	uint8 write_reg;
	uint16 write_value;
} write_table =
{
		0xFF, 0xFF, 0xFFFF
};


//static void ecu8tr_tcpDataClientTask( void *arg );

static void  ecu8tr_dataErr(void *arg, err_t err)
{
	PRINTF( "The TCP/IP error on data connection: %d\r\n", err );
	ecu8tr_closeDataSocket();
}

static err_t ecu8tr_dataSent( void *arg, struct tcp_pcb *tpcb, uint16 len )
{
	////PRINTF( "Data connection done sending\r\n" );
	//PRINTF( "ecu8tr_dataSent: The win size = %d\n", tpcb->snd_wnd );
	snd_wnd_cnt = tpcb->snd_wnd;
	is_data_sent = TRUE;
	return ERR_OK;
}

static err_t ecu8tr_dataRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    // Echo data back
    if( p != NULL )
    {
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
    }
    else //NULL when the connection has been closed!
    {
    	PRINTF( "Data connection closed\r\n" );
        // Close connection
        ecu8tr_closeDataSocket();
        //tcp_close(tpcb);
    }

    return ERR_OK;

}


static err_t ecu8tr_dataAccept( void *arg, struct tcp_pcb *new_pcb, err_t err )
{
	//tcp_write( new_pcb, ECU8_LOGO, strlen(ECU8_LOGO), 1 );
	ecu8tr_closeDataSocket();

	PRINTF( "Received a new data connection\r\n" );
    tcp_recv( new_pcb, ecu8tr_dataRecv );
    tcp_err( new_pcb, ecu8tr_dataErr );
    tcp_sent( new_pcb, ecu8tr_dataSent );

    ////remote_ip = new_pcb->remote_ip;

    pcb_data_client = new_pcb;

    if( tle9012_state == ECU8TR_TLE9012_WAKEUP_DONE )
    {
    	tle9012_state = ECU8TR_TLE9012_DATA_STREAMING;
    }

    return ERR_OK;
}

static uint16 build_cmd_return( ECU8TR_CMD_e cmd, ECU8TR_CMD_RET_e ret, void *buffer )
{
	uint32 i;
	size_t len;

	len = strlen( &((const char*)(buffer))[8] );
	i = htonl( cmd );
	memmove( buffer, (const void*)&i, 4 );
	i = htonl( ret );
	memmove( &((uint8*)buffer)[4], (const void*)&i, 4 );

	return( len+8 );
}

static void process_tcp_request( struct tcp_pcb *tpcb, uint8 *p_data, uint16 data_len )
{
	static uint8 pos = 0;
	char buffer[256];
	int buffer_pos = 0;
	ECU8TR_CMD_RET_e ret;
	uint16 ret_len = 0;
	uint8 *p_req;

	p_req = (uint8*)( ((uint8*)&ecu8tr_cmd) + pos );

	if( pos == 0 )
	{
		memset(ecu8tr_cmd.cmd_body, 0, CMD_BODY_SIZE );
	}

	//sanity check
	if( (pos+data_len) > sizeof(ECU8TR_CMD_t) )
	{
		pos = 0;
		return;
	}

	memcpy( p_req, p_data, data_len );
	pos += data_len;

	if( pos != sizeof(ECU8TR_CMD_t) )
	{
		return;
	}

	pos = 0;

	if( ecu8tr_cmd.delimiter != CMD_DELIMITER )
	{
		return;
	}
	ecu8tr_cmd.cmd_code = ntohl( ecu8tr_cmd.cmd_code );

	switch( ecu8tr_cmd.cmd_code )
	{
		case CMD_NET_SET_MAC:
		{
			if( TRUE == eeprom_set_mac( ecu8tr_cmd.cmd_body ) )
			{
				sprintf( buffer+8, "set mac to %s successfully\r\n", ecu8tr_cmd.cmd_body  );
				ret = ECU8TR_CMD_SUCCESS;
			}
			else
			{
				sprintf( buffer+8, "failed to set mac address\r\n" );
				ret = ECU8TR_CMD_SUCCESS;
			}

			ret_len = build_cmd_return( CMD_NET_SET_MAC, ret, buffer );

			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_NET_SHOW_NETWORK:

			if( network_def_config[1][4] == 1 )
			{
				buffer_pos = sprintf( buffer+8, "DHCP is enabled\r\n" );
			}
			else
			{
				buffer_pos = sprintf( buffer+8, "DHCP is disabled\r\n" );
			}

			buffer_pos += sprintf( buffer+buffer_pos+8, "mac address = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
						network_def_config[0][0],network_def_config[0][1], network_def_config[0][2], network_def_config[0][3], network_def_config[0][4], network_def_config[0][5] );

			buffer_pos += sprintf( buffer+buffer_pos+8, "ip address = %02d.%02d.%02d.%02d\r\n",
						network_def_config[1][0],network_def_config[1][1], network_def_config[1][2], network_def_config[1][3] );

			buffer_pos += sprintf( buffer+buffer_pos+8, "netmask = %02d.%02d.%02d.%02d\r\n",
						network_def_config[2][0],network_def_config[2][1], network_def_config[2][2], network_def_config[2][3] );

			sprintf( buffer+buffer_pos+8, "gateway = %02d.%02d.%02d.%02d\r\n$$",
						network_def_config[3][0],network_def_config[3][1], network_def_config[3][2], network_def_config[3][3] );
			ret_len = build_cmd_return( CMD_NET_SHOW_NETWORK, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;

		case CMD_NET_SET_IP:
			if( eeprom_set_ip( ecu8tr_cmd.cmd_body ) == TRUE )
			{
				sprintf( buffer+8, "set ip to %s successfully\r\n", ecu8tr_cmd.cmd_body );
				ret = ECU8TR_CMD_SUCCESS;
			}
			else
			{
				sprintf( buffer+8, "Failed to set the ip address\r\n" );
				ret = ECU8TR_CMD_FAILURE;
			}
			ret_len = build_cmd_return( CMD_NET_SET_IP, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );

			break;

		case CMD_NET_SET_NETMASK:
			if( eeprom_set_netmask( ecu8tr_cmd.cmd_body ) == TRUE )
			{
				sprintf( buffer+8, "set netmask to %s successfully\r\n", ecu8tr_cmd.cmd_body );
				ret = ECU8TR_CMD_SUCCESS;
			}
			else
			{
				sprintf( buffer+8, "Failed to set the ip address\r\n" );
				ret = ECU8TR_CMD_FAILURE;
			}
			ret_len = build_cmd_return( CMD_NET_SET_NETMASK, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );

			break;

		case CMD_NET_SET_GATEWAY:
			if( eeprom_set_gateway( ecu8tr_cmd.cmd_body ) == TRUE )
			{
				sprintf( buffer+8, "set gateway to %s successfully\r\n", ecu8tr_cmd.cmd_body );
				ret = ECU8TR_CMD_SUCCESS;
			}
			else
			{
				sprintf( buffer+8, "Failed to set the ip address\r\n" );
				ret = ECU8TR_CMD_FAILURE;
			}
			ret_len = build_cmd_return( CMD_NET_SET_GATEWAY, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );

			break;

		case CMD_TLE9012_SLEEP:
			//Here we don't do anything, because if the data is streaming,then we have to wait it to exit from the streaming state7

			if( comm_get_interface() == ECU8TR_COMM_INTERFACE_CAN )
			{
				sprintf( buffer+8, "Failed to perform the sleep command, the CAN Interface is in use\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_SLEEP, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}
			else if (tle9012_state == ECU8TR_TLE9012_IDLE)
			{
				sprintf( buffer+8, "The TLE9012 nodes are already in sleep mode\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_SLEEP, ECU8TR_CMD_SUCCESS, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}
			else
			{
				tle9012_state = ECU8TR_TLE9012_SLEEP;
			}

			break;


		case CMD_TLE9012_WAKEUP:
		{
			int pos;

			if( comm_get_interface() == ECU8TR_COMM_INTERFACE_CAN )
			{
				ret = ECU8TR_CMD_FAILURE;
				sprintf( buffer+8, "Failed to perform the wakeup command, the CAN Interface is in use\r\n" );

			}
			else if (tle9012_state != ECU8TR_TLE9012_DATA_STREAMING)
			{
				if( tle9012_wakeup_result != TRUE )
				{
					tle9012_state = ECU8TR_TLE9012_WAKEUP;
					while( (tle9012_state != ECU8TR_TLE9012_WAKEUP_DONE) && (tle9012_state != ECU8TR_TLE9012_IDLE) && (tle9012_state == ECU8TR_TLE9012_WAKEUP))
					{
						vTaskDelay(pdMS_TO_TICKS(5) );
					}
					if( TRUE == tle9012_wakeup_result )
					{
						tle9012_is_sleep = FALSE;
						pos = sprintf( buffer+8, "The tle9012 was put in the operation mode successfully\r\n" );
						sprintf( buffer + pos+8, "Total %d tle9012 nodes found on the network with ring structure = %s\r\n", tle9012_getNodeNr(), (isouart_getRing())?"Yes" : "No" );
						ret = ECU8TR_CMD_SUCCESS;
						if( pcb_data_client != NULL )
						{
							tle9012_state = ECU8TR_TLE9012_DATA_STREAMING;
						}
					}
					else
					{
						sprintf( buffer+8, "Failed to put the tle9012 into the operation mode\r\n" );
						ret = ECU8TR_CMD_FAILURE;
					}
				}
				else if (TRUE == tle9012_isWakeup())// Need to read ID from chip to verify that it is in wakeup and that comms have not failed
				{
					pos = sprintf( buffer+8, "The tle9012 is already in operation mode \r\n" );
					sprintf( buffer + pos+8, "Total %d tle9012 nodes found on the network with ring structure = %s\r\n$$", tle9012_getNodeNr(), (isouart_getRing())?"Yes" : "No" );
					ret = ECU8TR_CMD_SUCCESS;
				}
				else // comms have failed
				{
					pos = sprintf( buffer+8, "isoUart communication has failed \r\n" );
					sprintf( buffer+ pos+8, "Attempting to put the tle9012 into the operation mode\r\n$$" );
					ret = ECU8TR_CMD_FAILURE;
					tle9012_wakeup_result = FALSE;
					tle9012_state = ECU8TR_TLE9012_WAKEUP;
				}
			}
			else
			{
				pos = sprintf( buffer+8, "The tle9012 is already in operation mode and streaming data \r\n" );
				ret = ECU8TR_CMD_SUCCESS;
			}

			ret_len = build_cmd_return( CMD_TLE9012_WAKEUP, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;
		}

		case CMD_TLE9012_SET_NETWORK:
		{
			boolean parse_result;
			int network;

			sprintf( buffer+8, "Failed to set the network\r\n" );
			ret = ECU8TR_CMD_FAILURE;

			parse_result = parse_tle9012_network_params( ecu8tr_cmd.cmd_body, &network );
			if( parse_result == TRUE )
			{
				if( isouart_setNetwork( network ) == TRUE )
				{
					tle9012_state = ECU8TR_TLE9012_IDLE;
					sprintf( buffer+8, "Network set successfully\r\n" );
					ret = ECU8TR_CMD_SUCCESS;

				}
			}
			ret_len = build_cmd_return( CMD_TLE9012_SET_NETWORK, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_ECU8TR_VERSION:
			sprintf( buffer+8, "v%d.%d.%d\r\n", ECU8TR_MAJOR, ECU8TR_MINIOR, ECU8TR_PATCH );
			ret_len = build_cmd_return( CMD_ECU8TR_VERSION, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;

		case CMD_TLE9012_GET_NETWORK:
			sprintf( buffer+8, (isouart_get_network())? "HighSide\r\n" : "LowSide\r\n" );
			ret_len = build_cmd_return( CMD_TLE9012_GET_NETWORK, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;

		case CMD_TLE9012_RUNNING_MODE:
			sprintf( buffer+8, "%s\r\n", (tle9012_is_sleep==FALSE)? "Wakeup" : "Sleep" );
			ret_len = build_cmd_return( CMD_TLE9012_RUNNING_MODE, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;

		case CMD_TLE9012_SET_BITWIDTH:
		{
			boolean parse_result;
			int bitWidth;

			sprintf( buffer+8, "Failed to set the TLE9012 bit width\r\n" );
			ret = ECU8TR_CMD_FAILURE;

			parse_result = parse_tle9012_bitwidth_params( ecu8tr_cmd.cmd_body, &bitWidth );
			if( parse_result == TRUE )
			{
				if( tle9012_setBitWidth( bitWidth ) == TRUE )
				{
					//tle9012_state = ECU8TR_TLE9012_IDLE;
					sprintf( buffer+8, "TLE9012 bit width set successfully\r\n" );
					ret = ECU8TR_CMD_SUCCESS;
				}
			}
			ret_len = build_cmd_return( CMD_TLE9012_SET_BITWIDTH, ret, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_TLE9012_GET_BITWIDTH:
			sprintf( buffer+8, (tle9012_getBitWidth()==TLE9012_BITWIDTH_16)? "Bit Width 16bit\r\n" : "Bit Width 10bit\r\n$$" );//1 = 10 bits, 0 = 16 bits
			ret_len = build_cmd_return( CMD_TLE9012_GET_BITWIDTH, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
			break;

		case CMD_BOARD_RESET:
		{
			board_reset( eSysSystem );
			break;
		}

		case CMD_TLE9012_GET_DIAG:
		{
			tle9012_pre_state = tle9012_state;
			tle9012_state = ECU8TR_TLE9012_GET_DIAG;

			if (xSemaphoreTake(xGetDiagSemaphore, (TickType_t)portMAX_DELAY) == pdTRUE)
			{
				if( tle9012_diag_result == FALSE )
				{
					sprintf( buffer+8, "Failed to wakeup the TLE9012(s)\r\n" );
					ret_len = build_cmd_return( CMD_TLE9012_GET_DIAG, ECU8TR_CMD_FAILURE, buffer );
					tcp_write( tpcb, buffer, ret_len, 1 );
					tcp_output( tpcb );
				}
				else
				{
					uint8 diag_buffer[sizeof(ECU8TR_DIAG_t)+8];

					diag_buffer[8] = 0x00;
					tle9012_diag_data.delimiter = CMD_DELIMITER;
					tle9012_diag_data.crc = crc32( (const uint8*)&tle9012_diag_data, sizeof(ECU8TR_DIAG_t) - 6 );
					tle9012_diag_data.crc = htonl( tle9012_diag_data.crc );

					build_cmd_return( CMD_TLE9012_GET_DIAG, ECU8TR_CMD_SUCCESS, diag_buffer );
					memcpy( diag_buffer + 8, (void*)(&tle9012_diag_data), sizeof(diag_buffer) );
					ret_len = 8 + sizeof(ECU8TR_DIAG_t);
					tcp_write( tpcb, diag_buffer, ret_len, 1 );
					tcp_output( tpcb );
				}
			}
			else
			{
				sprintf( buffer+8, "Failed to retreive TLE9012 diagnostics results\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_GET_DIAG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}

			break;
		}

		case CMD_TLE9012_READ_REG:
		{
			int dev, reg;
			boolean parse_result;
			uint8 rd_buffer[16];
			uint16 val;

			if( tle9012_state != ECU8TR_TLE9012_WAKEUP && tle9012_state != ECU8TR_TLE9012_WAKEUP_DONE )
			{
				sprintf( buffer+8, "TLE9012 is not in wakeup mode. Reading failed\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_READ_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
				break;
			}
			parse_result = parse_tle9012_read_params( ecu8tr_cmd.cmd_body, &dev, &reg );
			PRINTF( "tle9012_read_reg: %d, 0x%x\r\n", dev, reg );
			if( parse_result == FALSE )
			{
				sprintf( buffer+8, "Failed to parse the parameters\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_READ_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}


			ret = tle9012_rdReg( dev, reg, (uint8*)rd_buffer );
			if( rd_buffer[0] == ISOUART_COMM_SUCCESS )
			{
				buffer[8] = 0x00;
				ret_len = build_cmd_return( CMD_TLE9012_READ_REG, ECU8TR_CMD_SUCCESS, buffer );
				buffer[8] = dev;
				buffer[9] = reg;
				buffer[10] = rd_buffer[5];
				buffer[11] = rd_buffer[6];
				ret_len = 8 + 4;
			}
			else
			{
				sprintf( buffer+8, "Failed to read the register values\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_READ_REG, ECU8TR_CMD_FAILURE, buffer );
			}

			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_TLE9012_WRITE_REG:
		{
			int dev, reg, val;
			boolean parse_result;
			uint8 wr_buffer[16];

			if( tle9012_state != ECU8TR_TLE9012_WAKEUP && tle9012_state != ECU8TR_TLE9012_WAKEUP_DONE )
			{
				sprintf( buffer+8, "TLE9012 is not in wakeup mode. Reading failed\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_WRITE_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
				break;
			}

			parse_result = parse_tle9012_write_params( ecu8tr_cmd.cmd_body, &dev, &reg , &val );
			if( parse_result == FALSE )
			{
				sprintf( buffer+8, "Failed to parse the parameters\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_WRITE_REG, ECU8TR_CMD_FAILURE, buffer );
			}
			else
			{
				PRINTF( "CMD_TLE9012_WRITE_REG dev = %d, reg = 0x%x, val = 0x%x\r\n", dev, reg, val );
				ret = tle9012_wrReg( dev, reg, val, (uint8*)wr_buffer );
				if( wr_buffer[0] == ISOUART_COMM_SUCCESS )
				{
					sprintf( buffer+8, "Write successfully\r\n" );
					ret_len = build_cmd_return( CMD_TLE9012_WRITE_REG, ECU8TR_CMD_SUCCESS, buffer );
				}
				else
				{
					sprintf( buffer+8, "Writing failed\r\n" );
					ret_len = build_cmd_return( CMD_TLE9012_WRITE_REG, ECU8TR_CMD_FAILURE, buffer );
				}

			}
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );

		}
			break;

		case CMD_TLE9012_REG_PERSIST:
		{
			int reg, val;
			boolean parse_result;

			parse_result = parse_tle9012_read_params( ecu8tr_cmd.cmd_body, &reg, &val );
			PRINTF( "tle9012_read_reg: %d, 0x%x\r\n", reg, val );
			if( parse_result == FALSE || reg <=0 )
			{
				sprintf( buffer + 8, "Failed to parse the parameters \r\n" );
				build_cmd_return( CMD_TLE9012_REG_PERSIST, ECU8TR_CMD_FAILURE, buffer );
			}
			else
			{
				eeprom_persist_reg( (uint8)(reg-1), (uint16)val ); //Reg address is from 1, but index into the EEPROM is from 0
				sprintf( buffer+8, "The TLE9012 register is successfully persisted to EEPROM.\r\n");
				ret_len = build_cmd_return( CMD_TLE9012_REG_PERSIST, ECU8TR_CMD_SUCCESS, buffer );
			}

			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_TLE9012_ERASE_CONFIG:
		{
			eeprom_erase_config();
			sprintf( buffer + 8, "TLE9012 Configuration in the EEPROM is erased successfully.\r\n" );
			ret_len = build_cmd_return( CMD_TLE9012_ERASE_CONFIG, ECU8TR_CMD_SUCCESS, buffer );
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );
		}
			break;

		case CMD_COMM_SET_INTERFACE:
		{
			boolean parse_result;
			int  interface;

			parse_result = parse_tle9012_network_params( ecu8tr_cmd.cmd_body, &interface );
			if( parse_result == FALSE || (interface !=0 && interface !=1) )
			{
				sprintf( buffer + 8, "The incorrect parameter: 0 = CAN interface; 1=Network interface.\r\n" );
				ret_len = build_cmd_return( CMD_COMM_SET_INTERFACE, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );

			}
			else
			{
				sprintf( buffer + 8, "The communication interface is applied successfully. A reset is in progress\r\n" );
				ret_len = build_cmd_return( CMD_COMM_SET_INTERFACE, ECU8TR_CMD_SUCCESS, buffer );
				eeprom_set_comm_interface( (ECU8TR_COMM_INTERFACE_e)interface );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
				vTaskDelay( pdMS_TO_TICKS(500) );
				board_reset( eSysSystem );
			}
		}
			break;

		case CMD_TLE9012_INDIRECT_WRITE_REG:
		{
			int dev, reg, val;
			boolean parse_result;

			if( tle9012_state != ECU8TR_TLE9012_WAKEUP && tle9012_state != ECU8TR_TLE9012_WAKEUP_DONE && tle9012_state != ECU8TR_TLE9012_DATA_STREAMING )
			{
				sprintf( buffer+8, "TLE9012 is not in wakeup mode. Reading failed\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_WRITE_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
				break;
			}

			parse_result = parse_tle9012_write_params( ecu8tr_cmd.cmd_body, &dev, &reg , &val );
			if( parse_result == FALSE )
			{
				sprintf( buffer+8, "Failed to parse the parameters\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_WRITE_REG, ECU8TR_CMD_FAILURE, buffer );
			}
			else
			{
				write_table.write_value = (uint16)val;
				write_table.write_reg = (uint8)reg;
				write_table.write_dev = (uint8)dev;
				sprintf( buffer+8, "Write Setting Done\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_WRITE_REG, ECU8TR_CMD_SUCCESS, buffer );
			}
			tcp_write( tpcb, buffer, ret_len, 1 );
			tcp_output( tpcb );

		}
			break;

		case CMD_TLE9012_INDIRECT_READ_REG:
		{
			int dev, reg;
			boolean parse_result;
			uint8 rd_buffer[16];
			uint16 val;

			if( tle9012_state != ECU8TR_TLE9012_DATA_STREAMING )
			{
				sprintf( buffer+8, "TLE9012 is not in wakeup mode. Reading failed\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_READ_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
				break;
			}

			parse_result = parse_tle9012_read_params( ecu8tr_cmd.cmd_body, &dev, &reg );
			PRINTF( "tle9012_read_reg: %d, 0x%x\r\n", dev, reg );
			if( parse_result == FALSE )
			{
				sprintf( buffer+8, "Failed to parse the parameters\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_READ_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}
			else
			{
				tle9012_indriect_rdReg( dev, reg );
				sprintf( buffer+8, "Reading setting done\r\n" );
				ret_len = build_cmd_return( CMD_TLE9012_INDIRECT_READ_REG, ECU8TR_CMD_FAILURE, buffer );
				tcp_write( tpcb, buffer, ret_len, 1 );
				tcp_output( tpcb );
			}
		}
			break;

		default:
			PRINTF( "Received unknown command\r\n" );
			break;
	}

}

static err_t ecu8tr_cmdSent( void *arg, struct tcp_pcb *tpcb, uint16 len )
{
	//PRINTF( "Command connection done sending\r\n" );

	return ERR_OK;
}

static err_t ecu8tr_cmdRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    // Echo data back
    if( p != NULL )
    {
        tcp_recved( tpcb, p->tot_len );
        tcp_send_empty_ack( tpcb );
    	process_tcp_request( tpcb, p->payload, p->len );
        pbuf_free(p);
    }
    else
    {
    	pcb_cmd_client = NULL;
    	PRINTF( "Command connection closed\r\n" );
        // Close connection
        tcp_close(tpcb);
    }


    return ERR_OK;

}

static void  ecu8tr_cmdErr(void *arg, err_t err)
{
	PRINTF( "The TCP/IP error on command connection: %d\r\n", err );
}

static err_t ecu8tr_cmdAccept( void *arg, struct tcp_pcb *new_pcb, err_t err )
{
	//tcp_write( new_pcb, ECU8_LOGO, strlen(ECU8_LOGO), 1 );
	PRINTF( "Received a new command connection\r\n" );
	pcb_cmd_client = new_pcb;
    tcp_recv( new_pcb, ecu8tr_cmdRecv );
    tcp_err( new_pcb, ecu8tr_cmdErr );
    tcp_sent( new_pcb, ecu8tr_cmdSent );
    remote_ip = new_pcb->remote_ip;

    return ERR_OK;
}


static void ecu8tr_startCmdServer( void )
{
	xGetDiagSemaphore = xSemaphoreCreateBinary();
	if( xGetDiagSemaphore == NULL )
	{
		__debug();
	}

    pcb_cmd_server = tcp_new();
    tcp_bind( pcb_cmd_server, IP_ADDR_ANY, ECU8_TCP_CMD_SERVER_PORT );
    pcb_cmd_server = tcp_listen( pcb_cmd_server );
    tcp_accept( pcb_cmd_server, ecu8tr_cmdAccept );
}

static void ecu8tr_startDataServer( void )
{
    pcb_data_server = tcp_new();
    tcp_bind( pcb_data_server, IP_ADDR_ANY, ECU8_TCP_DATA_SERVER_PORT );
    pcb_data_server = tcp_listen( pcb_data_server );
    tcp_accept( pcb_data_server, ecu8tr_dataAccept );
}


static void ecu8tr_tcpServerTask( void *arg )
{
    vTaskDelay( pdMS_TO_TICKS(3000) ); // Yield to other tasks

    ecu8tr_startCmdServer();
    ecu8tr_startDataServer();

    while (1)
    {
        vTaskDelay( pdMS_TO_TICKS(1000) ); // Yield to other tasks
    }
}

static void ecu8tr_closeDataSocket( void )
{
	if( pcb_data_client != NULL )
	{
		xSemaphoreTake( xDataSocketMutex, portMAX_DELAY );
	    tcp_recv( pcb_data_client, (tcp_recv_fn)NULL );
	    tcp_err( pcb_data_client, (tcp_err_fn)NULL );
	    tcp_sent( pcb_data_client, (tcp_sent_fn)NULL );
	    tcp_close( pcb_data_client );
	    pcb_data_client = NULL;
	    if( tle9012_state == ECU8TR_TLE9012_DATA_STREAMING )
	    {
	    	tle9012_state = ECU8TR_TLE9012_WAKEUP_DONE;
	    }
	    xSemaphoreGive( xDataSocketMutex );
	}
}
static void ecu8tr_TITask( void *arg )
{
	uint32 crc;
	ECU8TR_TI_DATA_t data;


	 while( 1 )
	 {
		data.delimiter = CMD_DELIMITER;
		if( g_bqSharedData.dataReady )
		{
			__dsync();
			for( uint8 board = 0; board < TOTALBOARDS-1; board++ )
			{
				data.dev = board+1;
				for( uint8 idx = 0; idx < TI_NUM_CELL; idx++ )
				{
					data.cell_volt[TI_NUM_CELL-1-idx] = ( (g_bqSharedData.cellVolt[board][idx] & 0xFF) << 8 ) | ((g_bqSharedData.cellVolt[board][idx] >> 8) & 0xFF);
	                PRINTF( "\tThe board = %d,  cell %d = 0x%x\r\n", board+1, (18-idx), data.cell_volt[TI_NUM_CELL-1-idx] );
				}
				for( uint8 idx = 0; idx < TI_NUM_GPIOADC; idx++ )
				{
					data.cell_temp[TI_NUM_GPIOADC-1-idx] = ( (g_bqSharedData.cellTemp[board][idx] & 0xFF) << 8 ) | ((g_bqSharedData.cellTemp[board][idx] >> 8) & 0xFF);
				}
				crc = crc32( (const uint8*)&data, sizeof(ECU8TR_TI_DATA_t) - 6 );

				data.crc = htonl( crc );

				if( pcb_data_client != NULL )
				{
					ecu8tr_can_status = ECU8TR_CAN_DATA_STREAMING;
					tle9012_state = ECU8TR_TLE9012_DATA_STREAMING;
					tcp_write( pcb_data_client, (const void*)&data, sizeof(ECU8TR_TI_DATA_t), 1 );
					tcp_output( pcb_data_client );
					vTaskDelay( pdMS_TO_TICKS(500) );
					//PRINTF( "Send out a device cell data\r\n" );
				}
				else
				{
					tle9012_state = ECU8TR_TLE9012_WAKEUP_DONE;
					ecu8tr_can_status = ECU8TR_CAN_TLE9012_WAKEUP;
				}
			}
		 }

        g_bqSharedData.dataReady = 0;

		vTaskDelay( pdMS_TO_TICKS(500) ); // Yield to other tasks
	}
}

static void ecu8tr_TLE9012Task( void *arg )
{
	ECU8TR_DATA_t data;
	uint8  nodeNr = 0;
	uint32 crc;
	uint32 global_node_cnt = 1;
	uint32 global_chance = 0;
//	static uint16_t streamingTimeOut = 0;


#if !__TI__
	while( 1 )
	{
		tle9012_doPulseWakeup();
		vTaskDelay( 1000 );

	}
#endif
	if( comm_get_interface() == ECU8TR_COMM_INTERFACE_CAN )
	{
		while( TRUE )
		{
			vTaskDelay( 1000 );
		}
	}

	data.delimiter = CMD_DELIMITER;
	vTaskDelay( pdMS_TO_TICKS(100) );
	tle9012_sleep();		//Put the tle9012 into sleep mode on the initial state
	vTaskDelay( pdMS_TO_TICKS(100) );
	tle9012_sleep();



	while( 1 )
	{
		switch( tle9012_state )
		{
		case ECU8TR_TLE9012_IDLE:
			tle9012_is_sleep = TRUE;
			tle9012_wakeup_result = FALSE;
			vTaskDelay( pdMS_TO_TICKS(10) );
			break;

		case ECU8TR_TLE9012_SLEEP:		//ECU8TR_TLE9012_IDLE, ECU8TR_TLE9012_WAKEUP
		{
			char buffer[128];
			ECU8TR_CMD_RET_e ret;
			uint16 ret_len;

			sprintf( buffer+8, "Failed to put the TLE9012 nodes into sleep mode\r\n$$" );
			ret = ECU8TR_CMD_FAILURE;

			tle9012_sleep_result = tle9012_sleep();
//			tle9012_state = ECU8TR_TLE9012_IDLE;
			if( tle9012_sleep_result == TRUE )
			{
				tle9012_state = ECU8TR_TLE9012_IDLE;
				tle9012_is_sleep = TRUE;
				ret = ECU8TR_CMD_SUCCESS;
				sprintf( buffer+8, "The TLE9012 nodes are in sleep mode\r\n$$" );
			}
			else
			{
				tle9012_is_sleep = FALSE;
			}

			ret_len = build_cmd_return( CMD_TLE9012_SLEEP, ret, buffer );
			if( pcb_cmd_client != NULL )
			{
				tcp_write( pcb_cmd_client, buffer, ret_len, 1u );
				tcp_output( pcb_cmd_client );
			}
		}
			break;

		case ECU8TR_TLE9012_WAKEUP:
			if (tle9012_state != ECU8TR_TLE9012_DATA_STREAMING)
			{
				tle9012_wakeup_result = tle9012_doFullWakeup();
				if( tle9012_wakeup_result == TRUE )
				{
					nodeNr = tle9012_getNodeNr();
					tle9012_apply_eeprom_conf();
					tle9012_state = ECU8TR_TLE9012_WAKEUP_DONE;
					tle9012_is_sleep = FALSE;

				}
				else
				{
					tle9012_state = ECU8TR_TLE9012_IDLE;
					nodeNr = 0;
				}
			}
			break;

		case ECU8TR_TLE9012_WAKEUP_DONE:
			if( pcb_data_client != NULL )
			{
				is_data_sent = TRUE;
				tle9012_node_for_streaming = 1u;
			}

			//need to verify that there was no comms interruption
			if (TRUE == tle9012_isWakeup())
			{
				tle9012_resetWDT();
			}
			else
			{
				tle9012_state = ECU8TR_TLE9012_IDLE;
				nodeNr = 0;
			}

			if( write_table.write_dev != 0xFF )
			{
				tle9012_indriect_wrReg( write_table.write_dev, write_table.write_reg, write_table.write_value );
				write_table.write_dev = 0xFF;
			}

			vTaskDelay( pdMS_TO_TICKS(500) );
			break;


		case ECU8TR_TLE9012_DATA_STREAMING:

			tle9012_resetWDT();	//reset watch dog timer

			xSemaphoreTake( xDataSocketMutex, portMAX_DELAY );
			if( pcb_data_client == NULL )
			{
			    xSemaphoreGive( xDataSocketMutex );
			    tle9012_state = ECU8TR_TLE9012_WAKEUP_DONE;
			    break;
			}

			if( (partial_data_cnt > 0) ) // Here snd_wnd = 0
			{
				is_data_sent = FALSE;
				tcp_write( pcb_data_client, (const void*)partial_data_buff, partial_data_cnt, 1 );
				partial_data_cnt = 0;
				tcp_output( pcb_data_client );
			    xSemaphoreGive( xDataSocketMutex );
				break;
			}
			else if( snd_wnd_cnt == 0 )
			{
				static uint8 zero_window_to = 0;
				if( zero_window_to == 200 )
				{
					tcp_zero_window_probe(  pcb_data_client );
					zero_window_to = 0;
				}
				else
				{
					zero_window_to++;
				}
				vTaskDelay( pdMS_TO_TICKS(10) );
				snd_wnd_cnt = pcb_data_client->snd_wnd;
				xSemaphoreGive( xDataSocketMutex );
				break;
			}


			/*if( global_chance++ == 30 )
			{
				if( FALSE == tle9012_readAll( global_node_cnt, &data ) )		//TLE9012 Node starts from 1
				{
					xSemaphoreGive( xDataSocketMutex );
					break;
				}
				if( global_node_cnt < nodeNr )
				{
					global_node_cnt++;
				}
				else
				{
					global_node_cnt = 1;
				}
				global_chance = 0;
			}
			else*/
			{
				if( FALSE == tle9012_readMetrics( tle9012_node_for_streaming, &data ) )		//TLE9012 Node starts from 1
				{
					xSemaphoreGive( xDataSocketMutex );
					break;
				}
				if( tle9012_node_for_streaming < nodeNr )
				{
					tle9012_node_for_streaming++;
				}
				else
				{
					tle9012_node_for_streaming = 1;
				}
			}


			crc = crc32( (const uint8*)&data, sizeof(ECU8TR_DATA_t) - 6 );

			data.crc = htonl( crc );

			if( pcb_data_client != NULL )
			{
				is_data_sent = FALSE;
				if( snd_wnd_cnt >= DATA_PACKET_SIZE )
				{
					tcp_write( pcb_data_client, (const void*)&data, sizeof(ECU8TR_DATA_t), 1 );
				}
				else if( snd_wnd_cnt > 0 )
				{
					partial_data_cnt = DATA_PACKET_SIZE - snd_wnd_cnt;
					tcp_write( pcb_data_client, (const void*)&data, snd_wnd_cnt, 1 );
					memcpy( (void*)partial_data_buff, ((unsigned char*)&data)+snd_wnd_cnt, partial_data_cnt );
				}
				else
				{
					xSemaphoreGive( xDataSocketMutex );
					break;
				}
				tcp_output( pcb_data_client );

				while( (is_data_sent == FALSE) && (streamingTimeOut <= 1000))
				{
					if (streamingTimeOut++ == 1000)
						{
							tle9012_is_sleep = TRUE;
							tle9012_wakeup_result = tle9012_doFullWakeup();
							if (streamingTimeOutLimit++ >= 3)
							{
								pcb_data_client = NULL;
								streamingTimeOutLimit = 0;
							}
						}
					vTaskDelay( pdMS_TO_TICKS(1) );
				}
			}
			else
			{
				tle9012_state = ECU8TR_TLE9012_WAKEUP_DONE;
			}

			if(is_data_sent == TRUE)
			{
				streamingTimeOutLimit = 0;
			}

			streamingTimeOut = 0;
			xSemaphoreGive( xDataSocketMutex );

			if( write_table.write_dev != 0xFF )
			{
				tle9012_indriect_wrReg( write_table.write_dev, write_table.write_reg, write_table.write_value );
				write_table.write_dev = 0xFF;
			}

			break;

		case ECU8TR_TLE9012_GET_DIAG:
		{
			if( tle9012_pre_state == ECU8TR_TLE9012_IDLE || tle9012_pre_state == ECU8TR_TLE9012_SLEEP )
			{
				if( FALSE == tle9012_doFullWakeup() )
				{
					tle9012_state = tle9012_pre_state;
					tle9012_diag_result = FALSE;
					xSemaphoreGive(xGetDiagSemaphore);
					break;
				}
			}

			nodeNr = tle9012_getNodeNr();
			if( nodeNr > MAX_TLE9012_NODES )
			{
				tle9012_state = tle9012_pre_state;
				tle9012_diag_result = FALSE;
				xSemaphoreGive(xGetDiagSemaphore);
				break;
			}

			tle9012_diag_data.node_cnt = 0x00;
			xSemaphoreTake( xDataSocketMutex, portMAX_DELAY );
			for( int nodeIdx = 0; nodeIdx < nodeNr; nodeIdx++ )
			{
				boolean result;


				tle9012_resetWDT();
				result = tle9012_readDiag( nodeIdx+1, &data );

				crc = crc32( (const uint8*)&data, sizeof(ECU8TR_DATA_t) - 6 );
				data.crc = htonl( crc );

				if( pcb_data_client != NULL )
				{
					is_data_sent = FALSE;
					tcp_write( pcb_data_client, (const void*)&data, sizeof(ECU8TR_DATA_t), 1 );
					tcp_output( pcb_data_client );

					while( (is_data_sent == FALSE) && (streamingTimeOut <= 1000))
					{
						if (streamingTimeOut++ == 1000)
						{
							tle9012_is_sleep = TRUE;
							tle9012_wakeup_result = tle9012_doFullWakeup();
							if (streamingTimeOutLimit++ >= 3)
							{
								pcb_data_client = NULL;
								streamingTimeOutLimit = 0;
							}
						}
						vTaskDelay( pdMS_TO_TICKS(1) );
					}
					if(is_data_sent == TRUE)
					{
						streamingTimeOutLimit = 0;
					}
					streamingTimeOut = 0;
				}

				PRINTF( "The diag data:\r\n");
				for( int reg_idx = 0; reg_idx < data.body_cnt; reg_idx++ )
				{
					tle9012_diag_data.node[nodeIdx][reg_idx] = data.data_body[reg_idx];
					PRINTF( "reg = 0%02x, value = 0x%02x\r\n", data.data_body[reg_idx].reg, data.data_body[reg_idx].reg_value );
				}

			}	//For

			xSemaphoreGive( xDataSocketMutex );
			data.dev = 0x00; //Mark as end of the nodes
			tle9012_diag_data.node_cnt = nodeNr;
			tle9012_state = tle9012_pre_state;
			tle9012_diag_result = TRUE;
			xSemaphoreGive(xGetDiagSemaphore);

			break;
		}	//case ECU8TR_TLE9012_GET_DIAG:


		default:
			vTaskDelay( pdMS_TO_TICKS(50) );
			break;
		}
	}
}


ECU8TR_TLE9012_State_e ecu8tr_getTLE9012State( void )
{
	return tle9012_state;
}

void ecu8_tcpServerInit( void )
{
	xDataSocketMutex = xSemaphoreCreateMutex();
	if( xDataSocketMutex == NULL )
	{
		__debug();
	}

    xTaskCreate( ecu8tr_tcpServerTask, "TCPServer", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
#if __TI__
    xTaskCreate( ecu8tr_TITask, "TCPClient", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
#else
    xTaskCreate( ecu8tr_TLE9012Task, "TCPClient", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
#endif
}


/*********************** Test Version *******************************/
#if 0

#define BUFFER_SIZE 256
#define PAYLOAD (BUFFER_SIZE-4)
uint8 buffer[BUFFER_SIZE];
static void ecu8tr_tcpDataClientTask( void *arg )
{
	uint32 crc;
	uint32 *p_crc;

	p_crc = (uint32*)(&buffer[PAYLOAD]);

	for( int i = 0; i < PAYLOAD; i++ )
	{
		buffer[i] = 'a';
	}

	crc = crc32( buffer, PAYLOAD );
	*p_crc = htonl( crc );


    vTaskDelay( pdMS_TO_TICKS(3000) ); // Yield to other tasks

    while (1)
    {
    	if( is_tle9012_wakeup == TRUE && pcb_data_client != NULL && is_data_sent == TRUE )
    	{
    		is_data_sent = FALSE;
    		tcp_write( pcb_data_client, buffer, BUFFER_SIZE, 1 );
    		tcp_output( pcb_data_client );
    	}
        vTaskDelay( pdMS_TO_TICKS(4) ); // Yield to other tasks
    }
}
#endif



