/*
 * ecu8_cmd.h
 *
 *  Created on: Feb 2, 2024
 *      Author: rgeng
 */

#ifndef APP_NET_INC_ECU8TR_CMD_H_
#define APP_NET_INC_ECU8TR_CMD_H_

#include <Ifx_Types.h>
#include "bq79600.h"

#define CMD_BODY_SIZE 32
#define MAX_TLE9012_DATA_ELEMENT  32
#define MAX_TLE9012_NODES	20
#define COUNT_OF_DIAG_REG	9
typedef enum
{
	UPGRADE_STATUS_IDLE = 0,
	UPGRADE_STATUS_IN_PROGRESS = 1,
	UPGRADE_STATUS_SUCCESS = 2,
	UPGRADE_STATUS_FAILURE = 3,
	UPGRADE_STATUS_NONE = 4,
} ECU8TR_UPGRADE_STATUS_e;

typedef enum
{
	CMD_NET_SET_MAC = 0,
	CMD_NET_SET_IP = 1,
	CMD_NET_SET_NETMASK = 2,
	CMD_NET_SET_GATEWAY = 3,
	CMD_NET_SHOW_NETWORK = 4,
	CMD_TLE9012_SLEEP = 5,
	CMD_TLE9012_WAKEUP = 6,
	CMD_TLE9012_SET_NETWORK = 7,
	CMD_ECU8TR_VERSION = 8,
	CMD_TLE9012_GET_NETWORK = 9,
	CMD_TLE9012_RUNNING_MODE = 10,
	CMD_TLE9012_READ_REG = 11,
	CMD_TLE9012_WRITE_REG = 12,
	CMD_TLE9012_SET_BITWIDTH = 13,
	CMD_TLE9012_GET_BITWIDTH = 14,
	CMD_BOARD_RESET = 15,
	//2025-03-07: add new command for diagnotics messages
	CMD_TLE9012_GET_DIAG = 16,
	CMD_TLE9012_REG_PERSIST = 17,			//The configurable registers which will be written into the EEPROM, and each wakeup will apply these settings to TLE9012
	CMD_TLE9012_ERASE_CONFIG = 18,
	CMD_COMM_SET_INTERFACE = 19,
	CMD_TLE9012_INDIRECT_WRITE_REG = 20,
	CMD_TLE9012_INDIRECT_READ_REG = 21,
	CMD_NONE
} 	ECU8TR_CMD_e;

typedef enum
{
	ECU8TR_TLE9012_IDLE = 0,
	ECU8TR_TLE9012_SLEEP = 1,
	ECU8TR_TLE9012_WAKEUP = 2,
	ECU8TR_TLE9012_WAKEUP_DONE = 3,
	ECU8TR_TLE9012_DATA_STREAMING = 4,
	ECU8TR_TLE9012_GET_DIAG = 5,
	ECU8TR_TLE9012_NONE = 6,
} ECU8TR_TLE9012_State_e;

#define __packed__              //The ADS tasking compiler doesn't support __packed__ data
typedef struct  __packed__
{

	ECU8TR_CMD_e cmd_code;
	char cmd_body[ CMD_BODY_SIZE ];
	uint16_t delimiter;	/* "$$" */
} ECU8TR_CMD_t;


typedef struct __packed__
{
	uint8 reg;
	uint16 reg_value;
} ECU8TR_DATA_BODY_t;

typedef struct __packed__
{
	uint8 dev;
	uint8 body_cnt;
	ECU8TR_DATA_BODY_t data_body[ MAX_TLE9012_DATA_ELEMENT ];
	uint32 crc;
	uint16 delimiter;	/* "$$" */
} ECU8TR_DATA_t;


typedef struct __packed__
{
	uint8 dev;
	uint16 cell_volt[ TI_NUM_CELL ];
	uint16 cell_temp[12];
	uint16 die_temp[2];
	uint32 crc;
	uint16 delimiter;	/* "$$" */
} ECU8TR_TI_DATA_t;

typedef struct __packed__
{
	uint8 node_cnt;
	ECU8TR_DATA_BODY_t node[MAX_TLE9012_NODES][COUNT_OF_DIAG_REG];
	uint32 crc;
	uint16 delimiter;	/* "$$" */
} ECU8TR_DIAG_t;



extern ECU8TR_TLE9012_State_e ecu8tr_getTLE9012State( void );

#endif /* APP_NET_INC_ECU8TR_CMD_H_ */
