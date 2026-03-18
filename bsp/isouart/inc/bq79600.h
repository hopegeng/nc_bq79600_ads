/*
 * bq79600.h
 *
 *  Created on: Oct 18, 2025
 *      Author: rgeng
 */

#ifndef BSP_ISOUART_INC_BQ79600_H_
#define BSP_ISOUART_INC_BQ79600_H_

#define TOTALBOARDS 3
#define TI_NUM_CELL 18
#define TI_NUM_GPIOADC	8

void bq79xxx_comInit( void );
uint8 bq79xxx_doInit( void );
void bq79xxx_sampleVolt( uint16 pVolt[TOTALBOARDS-1][TI_NUM_CELL] );
void bq79xxx_sampleGPIO( uint16 pGPIO[TOTALBOARDS-1][TI_NUM_GPIOADC] );
void bq79600_doCmd( void );
void bq79600_taskInit( void );
uint8 bq79xxx_doInit_Rev( void );


#endif /* BSP_ISOUART_INC_BQ79600_H_ */
