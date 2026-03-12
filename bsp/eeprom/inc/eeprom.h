/*
 * eeprom.h
 *
 *  Created on: Jan 24, 2024
 *      Author: rgeng
 */

#ifndef BSP_EEPROM_INC_EEPROM_H_
#define BSP_EEPROM_INC_EEPROM_H_

#include "isouart.h"
#include "tle9012.h"
/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

extern boolean eeprom_set_ip( const char *ip_str );
extern boolean eeprom_set_gateway( const char *gw_str );
extern boolean eeprom_set_netmask( const char *netmask_str );
extern boolean eeprom_set_mac( const char *mac_str );
extern boolean eeprom_set_dhcp( const char *dhcp_str );
extern boolean eeprom_read_net_config( void );
extern boolean eeprom_set_isouartNetwork( ISOUART_NetArch_t network );
extern boolean eeprom_set_tle9012_resolution( TLE9012_BITWIDTH_t bitWidth );
extern void eeprom_read_tle9012_conf( void );
extern void eeprom_persist_reg( uint8 reg, uint16 val );
extern boolean eeprom_get_tle9012_reg( uint8 reg_idx, uint16 *pVal );
extern void eeprom_erase_config( void );
extern boolean eeprom_set_comm_interface( ECU8TR_COMM_INTERFACE_e interface );

#endif /* BSP_EEPROM_INC_EEPROM_H_ */
