/*
 * comm.h
 *
 *  Created on: Mar 26, 2025
 *      Author: rgeng
 */

#ifndef APP_COMM_INC_COMM_H_
#define APP_COMM_INC_COMM_H_


typedef enum
{
	ECU8TR_COMM_INTERFACE_CAN = 0,
	ECU8TR_COMM_INTERFACE_NET = 1,
	ECU8TR_INTERFACE_NONE = 2,
} ECU8TR_COMM_INTERFACE_e;

extern void comm_apply_interface( ECU8TR_COMM_INTERFACE_e configured_interface );
extern ECU8TR_COMM_INTERFACE_e comm_get_interface( void );

#endif /* APP_COMM_INC_COMM_H_ */
