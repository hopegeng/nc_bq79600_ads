/*
 * bq_shared.h
 *
 *  Created on: Nov 4, 2025
 *      Author: rgeng
 */

#ifndef BQ_SHARED_H
#define BQ_SHARED_H

#include "Ifx_Types.h"

#include "bq79600.h"

/* Structure shared between Core2 and Core0 */
typedef struct
{
    uint16 cellVolt[TOTALBOARDS-1][TI_NUM_CELL];  // each cell voltage in raw ADC LSB (2 bytes each)
    uint16 cellTemp[TOTALBOARDS-1][TI_NUM_GPIOADC];
    uint16 dieTemp[TOTALBOARDS-1][2];
    uint32 timestamp;            // optional: time of last update
    uint8  dataReady;            // flag = 1 means new data ready for UDP send
    uint8  reserved[3];          // padding for 32-bit alignment
} BQ_ShareData_t;

/* Shared memory section declaration */
extern volatile BQ_ShareData_t g_bqSharedData;

#endif

