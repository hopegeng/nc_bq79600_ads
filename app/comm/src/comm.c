/*
 * comm.c
 *
 *  Created on: Mar 26, 2025
 *      Author: rgeng
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ecu8tr_cmd.h"
#include "comm.h"



volatile ECU8TR_COMM_INTERFACE_e comm_interface = ECU8TR_COMM_INTERFACE_CAN; //Default is CAN
void comm_apply_interface( ECU8TR_COMM_INTERFACE_e configured_interface )
{
	comm_interface = configured_interface;
}

ECU8TR_COMM_INTERFACE_e comm_get_interface( void )
{
#if __TI__
	return ECU8TR_COMM_INTERFACE_NET;
#else
	return comm_interface;
#endif

}
