/*
 * led.h
 *
 *  Created on: Jan 30, 2024
 *      Author: rgeng
 */

#ifndef BSP_LED_INC_LED_H_
#define BSP_LED_INC_LED_H_

extern void start_led( void );

typedef enum
{
	ECU8TR_CAN_TLE9012_NOT_CONNECT = 0,
	ECU8TR_CAN_TLE9012_WAKEUP = 1,
	ECU8TR_CAN_DATA_STREAMING = 2,
	ECU8TR_CAN_NONE = 3,
} ECU8TR_CAN_STATUS_e;

#endif /* BSP_LED_INC_LED_H_ */
